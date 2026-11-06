
from pathlib import Path
import csv,json,hashlib,os
import numpy as np
import pandas as pd
import rawpy,cv2
from scipy import sparse
from skimage.restoration import denoise_tv_chambolle
from skimage.metrics import structural_similarity

HERE=Path(__file__).resolve().parent
WORK=HERE.parents[1]
DATA=Path(os.environ.get('MECH5720_DATA_ROOT',str(WORK/'MECH5720_Lab/mech5720/code/supplement_20261031')))
QC=(HERE/'acquisition_qc') if (HERE/'acquisition_qc').exists() else WORK/'output/supplement_QC_20261031'
CACHE=HERE/'cache';CACHE.mkdir(parents=True,exist_ok=True)
WIDTH,HEIGHT=240,180
cv2.setNumThreads(1);cv2.setRNGSeed(20261031)

def dump(path,obj):
    def convert(o):
        if isinstance(o,np.ndarray):return o.tolist()
        if isinstance(o,np.generic):return o.item()
        if isinstance(o,Path):return str(o)
        raise TypeError(type(o).__name__)
    Path(path).write_text(json.dumps(obj,indent=2,default=convert,allow_nan=False),encoding='utf-8')

def read_run(run):
    c=pd.read_csv(run/'camera_frames.csv');l=pd.read_csv(run/'lidar_frames.csv')
    m=json.loads((run/'manifest.json').read_text());origin=int(m['started_monotonic_ns'])
    l=l.iloc[1:].copy() # retain raw file, omit first known startup sample in analysis
    vals=[];spread=[];counts=[]
    for _,r in l.iterrows():
        grid=np.asarray(json.loads(r.distances_8x8_json),float)
        a=grid[3:5,3:5].ravel();a=a[(a>0)&np.isfinite(a)]
        counts.append(len(a));vals.append(float(np.median(a)) if len(a) else np.nan)
        spread.append(float(np.ptp(a)) if len(a) else np.inf)
    l['recomputed_mm']=vals;l['zone_spread_mm']=spread;l['zones_positive']=counts
    valid=(l.zones_positive>=3)&(l.zone_spread_mm<=50)&l.recomputed_mm.between(200,2000)
    good=l.loc[valid].copy()
    tl=(good.receive_monotonic_ns.to_numpy(np.int64)-origin)/1e9
    z=good.recomputed_mm.to_numpy(float)
    events=pd.read_csv(run/'illumination_events.csv') if (run/'illumination_events.csv').exists() else None
    c['end_s']=(c.sensor_timestamp_ns.to_numpy(np.int64)-origin)/1e9
    c['start_s']=c.end_s-c.exposure_time_us/1e6
    return dict(run=run,c=c,l=l,m=m,origin=origin,t=tl,z=z,events=events,excluded_depth_samples=int((~valid).sum()))

def linear_green(path,return_qc=False):
    # Equal average of the two GREEN sensels in every2x2 Bayer cell.

    with rawpy.imread(str(path)) as raw:
        a=raw.raw_image_visible.astype(np.float32)
        black=np.asarray(raw.black_level_per_channel,dtype=np.float32)[raw.raw_colors_visible]
        norm=(a-black)/(raw.white_level-black)
        colours=raw.raw_colors_visible
        positions=[(y,x) for y in range(2) for x in range(2) if raw.color_desc[colours[y,x]]==ord('G')]
        assert len(positions)==2
        green=sum(norm[y::2,x::2] for y,x in positions)/2
        g=cv2.resize(green,(WIDTH,HEIGHT),interpolation=cv2.INTER_AREA).astype(np.float32)
        stats=dict(saturated_pct=float((a>=raw.white_level).mean()*100),raw_max=float(a.max()),white_level=int(raw.white_level),green_positions=positions,black_levels=list(map(int,raw.black_level_per_channel)))
    return (g,stats) if return_qc else g

def frame_image(run,frame):return linear_green(run/'camera_dng'/f'frame_{int(frame):06d}.dng')

def norm_image(im):
    lo,hi=np.percentile(im,[3,97])
    return np.clip((im-lo)/max(hi-lo,1e-6),0,1).astype(np.float32),float(lo),float(hi)

def similarity(a,b):
    a8=(norm_image(a)[0]*255).astype('uint8');b8=(norm_image(b)[0]*255).astype('uint8')
    sift=cv2.SIFT_create(nfeatures=2500,contrastThreshold=.015,edgeThreshold=12)
    ka,da=sift.detectAndCompute(a8,None);kb,db=sift.detectAndCompute(b8,None)
    if da is None or db is None:raise ValueError('No SIFT features')
    pairs=cv2.BFMatcher().knnMatch(da,db,k=2)
    matches=[x[0] for x in pairs if len(x)==2 and x[0].distance<.7*x[1].distance]
    if len(matches)<6:raise ValueError('Too few unique SIFT matches')
    p=np.float32([ka[m.queryIdx].pt for m in matches]);q=np.float32([kb[m.trainIdx].pt for m in matches])
    M,mask=cv2.estimateAffinePartial2D(p,q,method=cv2.RANSAC,ransacReprojThreshold=1.3,maxIters=7000,confidence=.999)
    if M is None or mask.sum()<6:raise ValueError('Endpoint similarity unreliable')
    err=np.linalg.norm(p@M[:,:2].T+M[:,2]-q,axis=1)
    return M,dict(inliers=int(mask.sum()),matches=len(matches),median_inlier_error_px=float(np.median(err[mask.ravel()>0])),scale=float(np.linalg.norm(M[:,0])),rotation_deg=float(np.degrees(np.arctan2(M[1,0],M[0,0]))))

def fraction(z,ze,zl):return (1/np.asarray(z)-1/ze)/(1/zl-1/ze)

def geometric_matrix(z,zref,g,rotate=False):
    db=fraction(z,g['ze'],g['zl'])-fraction(zref,g['ze'],g['zl'])
    sc=g['scale']**db;ang=np.radians(g['rotation_deg'])*db if rotate else 0.
    co=np.cos(ang);si=np.sin(ang);pivot=np.asarray(g['pivot'])
    R=np.array([[co,-si],[si,co]])*sc
    T=np.eye(3);T[:2,:2]=R;T[:2,2]=pivot-R@pivot
    return T

def fit_geometry(d,reference,zref):
    c=d['c'];tl=d['t'];z=d['z']
    rows=c[(c.start_s>=tl[0])&(c.end_s<=tl[-1])].copy()
    for idx,r in rows.iterrows():
        a=z[(tl>=r.start_s)&(tl<=r.end_s)]
        rows.loc[idx,'range_mm']=np.ptp(a) if len(a) else np.inf
        rows.loc[idx,'depth_mm']=np.median(a) if len(a) else np.nan

    early=rows[(rows.end_s<3)&(rows.range_mm<15)].iloc[0]
    late=rows[(rows.start_s>13)&(rows.range_mm<15)].iloc[-1]
    A=frame_image(d['run'],early.frame);B=frame_image(d['run'],late.frame)
    M,info=similarity(A,B)
    ze,zl=float(early.depth_mm),float(late.depth_mm)
    pivot=np.linalg.solve(np.eye(2)-M[:,:2],M[:,2])
    if abs(ze-zl)<50 or not .95<info['scale']<1.4 or not np.isfinite(pivot).all() or np.max(abs(pivot))>2000:
        raise ValueError('Endpoint geometry outside declared diagnostic bounds')
    g=dict(**info,ze=ze,zl=zl,pivot=pivot.tolist(),early_frame=int(early.frame),late_frame=int(late.frame),expected_pinhole_scale=ze/zl)
    T,reg=similarity(reference,A)
    T=np.vstack([T,[0,0,1]])

    # Registration is reference-assisted and is shared by every reconstruction method.
    G=geometric_matrix(zref,ze,g,rotate=True)
    mapping=G@T
    R=cv2.warpAffine(reference,mapping[:2].astype(np.float32),(WIDTH,HEIGHT),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    valid=cv2.warpAffine(np.ones_like(reference),mapping[:2].astype(np.float32),(WIDTH,HEIGHT),flags=cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    g.update(reference_registration=reg,reference_to_reconstruction=mapping.tolist(),zref=zref)
    return g,R,valid

def timeline(d,row,lag=.026,depth_shift=0,steps=48,light_floor=.19,camera_shift=0,depth_time_shift=0):
    start=float(row.start_s)+camera_shift;end=float(row.end_s)+camera_shift
    t=start+(np.arange(steps)+.5)*(end-start)/steps
    z=np.interp(t,d['t']+depth_time_shift,d['z'])+depth_shift
    edgez=np.interp([start,end],d['t']+depth_time_shift,d['z'])+depth_shift
    constant=edgez[0]+(edgez[1]-edgez[0])*(t-start)/(end-start)
    ev=d['events'];state=np.ones(steps)
    covered=True;irregular=False
    if ev is not None:
        et=(ev.pi_receive_monotonic_ns.to_numpy(np.int64)-d['origin'])/1e9+lag
        covered=bool(start>=et[0] and end<=et[-1])
        idx=np.clip(np.searchsorted(et,t,side='right')-1,0,len(et)-1)
        state=ev.state_white_1_black_0.to_numpy(int)[idx]
        intervals=np.diff(et)
        irregular=bool(np.any(((intervals<.05)|(intervals>.35))&(et[:-1]<end+.05)&(et[1:]>start-.05)))
    w=light_floor+(1-light_floor)*state;w=w/w.sum()
    inside=(d['t']>=start)&(d['t']<=end)
    ts=np.r_[start,d['t'][inside],end];zs=np.interp(ts,d['t'],d['z'])
    baseline=zs[0]+(zs[-1]-zs[0])*(ts-start)/(end-start)
    return dict(t=t,z=z,constant=constant,w=w,state=state,start=start,end=end,
        measured_n=int(inside.sum()),depth_covered=bool(start>=d['t'][0] and end<=d['t'][-1]),light_covered=covered,irregular_events=irregular,
        approach_mm=float(edgez[0]-edgez[1]),depth_range_mm=float(np.ptp(zs)),nonlinearity_mm=float(np.max(np.abs(zs-baseline))))

def build_operator(z,w,zref,g,rotate=False):
    yy,xx=np.indices((HEIGHT,WIDTH),dtype=np.float32);xy=np.c_[xx.ravel(),yy.ravel(),np.ones(WIDTH*HEIGHT)]
    rr=np.arange(len(xy),dtype=np.int32);rows=[];cols=[];vals=[]
    for depth,weight in zip(z,w):
        inv=np.linalg.inv(geometric_matrix(depth,zref,g,rotate))
        p=xy@inv[:2].T
        # Replicated edge in operator, then exclude an eroded interior ROI in evaluation.
        u=np.clip(p[:,0],0,WIDTH-1.00001);v=np.clip(p[:,1],0,HEIGHT-1.00001)
        ui=u.astype(np.int32);vi=v.astype(np.int32);dx=u-ui;dy=v-vi
        for ox,oy,term in ((0,0,(1-dx)*(1-dy)),(1,0,dx*(1-dy)),(0,1,(1-dx)*dy),(1,1,dx*dy)):
            rows.append(rr);cols.append((vi+oy)*WIDTH+ui+ox);vals.append((weight*term).astype(np.float32))
    A=sparse.coo_matrix((np.concatenate(vals),(np.concatenate(rows),np.concatenate(cols))),shape=(WIDTH*HEIGHT,WIDTH*HEIGHT),dtype=np.float32).tocsr()
    A.sum_duplicates();A.eliminate_zeros();return A

def tv_inverse(A,B,lam=.002,niter=80):
    atb=A.T@B.ravel();rng=np.random.default_rng(20261031);v=rng.normal(size=B.size).astype(np.float32);v/=np.linalg.norm(v)
    for _ in range(12):
        v=A.T@(A@v);v/=np.linalg.norm(v)+1e-30
    L=max(float(v@(A.T@(A@v)))*1.08,1e-6)
    x=B.copy();y=x.copy();acc=1.;history=[]
    for i in range(niter):
        a=y-(A.T@(A@y.ravel())-atb).reshape(B.shape)/L
        if lam:a=denoise_tv_chambolle(a,weight=lam/L,eps=1e-5,max_num_iter=35)
        xn=np.clip(a,0,1).astype(np.float32);an=(1+np.sqrt(1+4*acc*acc))/2
        change=float(np.linalg.norm(xn-x)/(np.linalg.norm(x)+1e-20))
        y=xn+(acc-1)/an*(xn-x);x=xn;acc=an
        if i in (0,19,39,79,niter-1):history.append(dict(iteration=i+1,relative_change=change,residual_RMSE=float(np.sqrt(np.mean((A@x.ravel()-B.ravel())**2)))))
    return x,(A@x.ravel()).reshape(B.shape),history

def masks(reference,valid):
    mask=cv2.erode((valid>.99).astype('uint8'),np.ones((17,17),'uint8'))>0
    mask[:12]=False;mask[-12:]=False;mask[:,:12]=False;mask[:,-12:]=False
    grad=np.hypot(cv2.Sobel(reference,cv2.CV_32F,1,0),cv2.Sobel(reference,cv2.CV_32F,0,1))
    feature=(cv2.dilate((grad>.35).astype('uint8'),np.ones((9,9),'uint8'))>0)&mask
    return mask,feature

def metrics(reference,im,mask):
    mse=float(np.mean((reference[mask]-im[mask])**2))
    _,ss=structural_similarity(reference,im,data_range=1.,gaussian_weights=True,sigma=1.5,use_sample_covariance=False,full=True)
    return dict(PSNR_dB=float(-10*np.log10(max(mse,1e-15))),SSIM=float(ss[mask].mean()),MSE=mse,pixels=int(mask.sum()))
