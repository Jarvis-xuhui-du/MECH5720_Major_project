
import argparse,time,gc
from analysis_core import *

RESULTS=HERE/'results';RESULTS.mkdir(exist_ok=True)

def prepare_trial(sel):
    run=DATA/sel.condition/sel.run;d=read_run(run)
    row=d['c'][d['c'].frame==int(sel.frame)].iloc[0]
    g=json.loads((HERE/'endpoint_geometry.json').read_text())[sel.condition+'__'+sel.run]
    cache=np.load(CACHE/(sel.condition+'__'+sel.run+'_geometry.npz'))
    R=np.clip(cache['reference'],0,1).astype(np.float32);valid=cache['valid']
    B,low,high=norm_image(frame_image(run,sel.frame))
    full,feature=masks(R,valid)
    return d,row,g,R,B,full,feature,dict(input_low=low,input_high=high)

def analyse_one(sel,lam=.002,niter=320,lag=.026,floor=.19,depth_shift=0,camera_shift=0,depth_time_shift=0,rotate=False,steps=48,tag='primary',save=True):
    tic=time.time();d,row,g,R,B,full,feature,norm=prepare_trial(sel)
    t=timeline(d,row,lag=lag,depth_shift=depth_shift,light_floor=floor,camera_shift=camera_shift,depth_time_shift=depth_time_shift,steps=steps)
    g=g.copy();g['ze']+=depth_shift;g['zl']+=depth_shift;zref=g['zref']+depth_shift
    images={'input':B,'denoise_only':denoise_tv_chambolle(B,weight=lam,eps=1e-5,max_num_iter=100).astype(np.float32)}
    forward={};history={};records=[]
    for method,z in (('constant',t['constant']),('lidar',t['z'])):
        A=build_operator(z,t['w'],zref,g,rotate=rotate)
        X,pred,trace=tv_inverse(A,B,lam,niter)
        images[method]=X;forward[method]=pred
        forward[method+'_from_reference']=(A@R.ravel()).reshape(R.shape)
        history[method]=trace
        del A;gc.collect()
    canon=np.load(CACHE/'canonical_evaluation.npz')
    inverse=np.linalg.inv(np.asarray(g['reference_to_reconstruction']))
    canonical_images={name:cv2.warpAffine(im,inverse[:2].astype('float32'),(WIDTH,HEIGHT),flags=cv2.INTER_LINEAR) for name,im in images.items()}
    for name,im in images.items():
        for roi,mask in (('common_interior',full),('features',feature)):
            records.append(dict(condition=sel.condition,run=sel.run,frame=int(sel.frame),status=sel.status,
                analysis_tag=tag,method=name,ROI=roi,**metrics(R,im,mask),
                tv_lambda=lam,lag_ms=lag*1000,light_floor=floor,depth_shift_mm=depth_shift,camera_shift_ms=camera_shift*1000,depth_time_shift_ms=depth_time_shift*1000,rotation_model=rotate,quadrature_bins=steps,iterations=niter))
        for roi,mask in (('canonical_common',canon['full_mask']),('canonical_features',canon['feature_mask'])):
            records.append(dict(condition=sel.condition,run=sel.run,frame=int(sel.frame),status=sel.status,
                analysis_tag=tag,method=name,ROI=roi,**metrics(canon['reference'],canonical_images[name],mask),
                tv_lambda=lam,lag_ms=lag*1000,light_floor=floor,depth_shift_mm=depth_shift,camera_shift_ms=camera_shift*1000,depth_time_shift_ms=depth_time_shift*1000,rotation_model=rotate,quadrature_bins=steps,iterations=niter))
    for name,pred in forward.items():
        records.append(dict(condition=sel.condition,run=sel.run,frame=int(sel.frame),status=sel.status,analysis_tag=tag,
            method=name,ROI='forward_residual_to_observation',PSNR_dB=float(-10*np.log10(max(np.mean((pred[full]-B[full])**2),1e-15))),
            SSIM=float('nan'),MSE=float(np.mean((pred[full]-B[full])**2)),pixels=int(full.sum()),tv_lambda=lam,lag_ms=lag*1000,
            light_floor=floor,depth_shift_mm=depth_shift,camera_shift_ms=camera_shift*1000,depth_time_shift_ms=depth_time_shift*1000,rotation_model=rotate))
    if save:
        key=sel.condition+'__'+sel.run
        np.savez_compressed(RESULTS/(key+'.npz'),reference=R,full_mask=full,feature_mask=feature,**images,
            pred_constant=forward['constant'],pred_lidar=forward['lidar'],reference_pred_constant=forward['constant_from_reference'],reference_pred_lidar=forward['lidar_from_reference'],
            t=t['t']-t['start'],z=t['z'],z_constant=t['constant'],weights=t['w'],state=t['state'])
        np.savez_compressed(RESULTS/(key+'_canonical.npz'),reference=canon['reference'],full_mask=canon['full_mask'],feature_mask=canon['feature_mask'],**canonical_images)
        pd.DataFrame(dict(exposure_time_s=t['t']-t['start'],depth_lidar_mm=t['z'],depth_endpoint_linear_mm=t['constant'],illumination_weight=t['w'],state=t['state'])).to_csv(RESULTS/(key+'_timeline.csv'),index=False)
        dump(RESULTS/(key+'_solver.json'),dict(convergence=history,shared_normalization=norm,geometry=g,
            boundary='Reference-assisted endpoint registration; numerical convergence is not physical model validation. First-row exposure convention; row timing unmeasured.'))
    print(tag,sel.run,int(sel.frame),'seconds',round(time.time()-tic,2),flush=True)
    return records

def static_controls():
    refdata=np.load(CACHE/'static_references_linear_green.npz');reference=refdata['70']
    rows=[];checks=[]
    for run in sorted((DATA/'D_static_coded70_1s').glob('motion_*')):
        d=read_run(run);covered=[]
        for _,r in d['c'].iterrows():
            line=timeline(d,r)
            if line['depth_covered'] and line['light_covered'] and not line['irregular_events']:
                covered.append((r,line))
        row,line=covered[len(covered)//2]
        # Use a separate stationary frame for registration, never the evaluation frame.
        regrow=next(r for r,t in covered if int(r.frame)!=int(row.frame))
        T,reg=similarity(reference,frame_image(run,regrow.frame))
        R=cv2.warpAffine(reference,T.astype(np.float32),(WIDTH,HEIGHT))
        valid=cv2.warpAffine(np.ones_like(reference),T.astype(np.float32),(WIDTH,HEIGHT),flags=cv2.INTER_NEAREST)
        R=norm_image(R)[0];B=norm_image(frame_image(run,row.frame))[0];full,feat=masks(R,valid)
        np.savez_compressed(CACHE/(run.name+'_static.npz'),reference=R,input=B,full_mask=full,feature_mask=feat)
        for lam in (0,.0005,.002,.005,.02):
            X=denoise_tv_chambolle(B,weight=lam,eps=1e-5,max_num_iter=100).astype(np.float32) if lam else B.copy()
            for name,mask in (('common_interior',full),('features',feat)):
                rows.append(dict(run=run.name,frame=int(row.frame),registration_frame=int(regrow.frame),tv_lambda=lam,ROI=name,**metrics(R,X,mask),identity_max_abs_error=float(np.max(abs(X-B))) if lam==0 else None))
        checks.append(dict(run=run.name,selected_frame=int(row.frame),registration_frame=int(regrow.frame),registration=reg,
            depth_range_in_exposure_mm=line['depth_range_mm'],in_exposure_depth_samples=line['measured_n']))
    pd.DataFrame(rows).to_csv(HERE/'static_no_motion_control.csv',index=False);dump(HERE/'static_registration.json',checks)
    return rows

def synthetic_validation():

    g=dict(ze=770.,zl=670.,scale=770/670,rotation_deg=0.,pivot=[119.5,89.5])
    zz=np.linspace(770,670,48);w=np.ones(48)/48
    A=build_operator(zz,w,720.,g);I=build_operator(np.full(48,720.),w,720.,g)
    yy,xx=np.indices((HEIGHT,WIDTH));X=((xx//12+yy//12)%2).astype(np.float32)*.7+.15
    rng=np.random.default_rng(5);u=rng.normal(size=X.size).astype(np.float32);v=rng.normal(size=X.size).astype(np.float32)
    B=(A@X.ravel()).reshape(X.shape);rest,pred,history=tv_inverse(A,B,.0005,100)
    mask=np.zeros_like(X,dtype=bool);mask[15:-15,15:-15]=True
    result=dict(identity_error=float(np.max(abs(I@X.ravel()-X.ravel()))),row_sum_error=float(np.max(abs(A.sum(axis=1).A.ravel()-1))),
        adjoint_relative_error=float(abs(u@(A@v)-(A.T@u)@v)/(abs(u@(A@v))+1e-20)),
        synthetic_input=metrics(X,B,mask),synthetic_reconstruction=metrics(X,rest,mask),solver_history=history,
        interpretation='Synthetic validation checks implementation consistency only; it is not experimental evidence for the apparatus.')
    assert result['identity_error']<1e-4 and result['row_sum_error']<1e-4 and result['adjoint_relative_error']<1e-4
    assert result['synthetic_reconstruction']['PSNR_dB']>result['synthetic_input']['PSNR_dB']
    dump(HERE/'implementation_validation.json',result)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--sensitivity',action='store_true');args=ap.parse_args()
    selected=pd.read_csv(HERE/'frozen_trial_selection.csv')
    if not args.sensitivity:
        synthetic_validation();static_controls();records=[]
        for _,s in selected.iterrows():
            if s.status=='GEOMETRY_FAILURE':continue
            records+=analyse_one(s)
            pd.DataFrame(records).to_csv(HERE/'motion_metrics_all.csv',index=False)
    else:
        # One-at-a-time fixed sensitivity set; no per-trial best-value selection.
        variants=[('lag_minus20ms',dict(lag=-.020)),('lag_zero',dict(lag=0)),('lag_plus50ms',dict(lag=.050)),
            ('depth_time_minus50ms',dict(depth_time_shift=-.050)),('depth_time_plus50ms',dict(depth_time_shift=.050)),
            ('TV_0p0005',dict(lam=.0005)),('TV_0p005',dict(lam=.005)),('TV_0p02',dict(lam=.02)),
            ('floor_0p10',dict(floor=.10)),('floor_0p30',dict(floor=.30)),
            ('depth_minus22mm',dict(depth_shift=-22)),('depth_plus22mm',dict(depth_shift=22)),
            ('rotation_on',dict(rotate=True))]
        records=[]
        coded=selected[(selected.status=='PRIMARY_ELIGIBLE')&selected.condition.str.contains('coded')]
        for tag,kwargs in variants:
            for _,s in coded.iterrows():
                records+=analyse_one(s,tag=tag,save=False,**kwargs)
                pd.DataFrame(records).to_csv(HERE/'sensitivity_metrics.csv',index=False)

if __name__=='__main__':main()
