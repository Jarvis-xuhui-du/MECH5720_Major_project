
import csv,json,hashlib,platform,time
from pathlib import Path
import numpy as np,pandas as pd,cv2,rawpy,scipy,skimage
from analysis_core import *

def main():
    inv=[];rawrows=[]
    for manifest in sorted(DATA.rglob('manifest.json')):
        run=manifest.parent;d=read_run(run);c=d['c'];clip=[]
        assert len(list((run/'camera_dng').glob('*.dng')))==len(c)
        for _,r in c.iterrows():
            image,stats=linear_green(run/'camera_dng'/r['file'],True)
            clip.append(stats['saturated_pct'])
            rawrows.append(dict(condition=run.parent.name,run=run.name,frame=int(r.frame),**stats))
        inv.append(dict(condition=run.parent.name,run=run.name,camera_frames=len(c),lidar_frames=len(d['l'])+1,
            actual_exposures_us=';'.join(map(str,sorted(set(c.exposure_time_us)))),max_RAW_saturation_percent=max(clip),
            depth_samples_excluded_after_startup=d['excluded_depth_samples'],depth_median_mm=float(np.median(d['z'])),
            actual_receive_span_s=float((c.receive_monotonic_ns.iloc[-1]-c.receive_monotonic_ns.iloc[0])/1e9),
            saved_interval_median_s=float(np.median(np.diff(c.sensor_timestamp_ns))/1e9),
            event_count=0 if d['events'] is None else len(d['events']),failures=json.dumps(d['m']['failures'])))
        print('Audited',run.parent.name,run.name,len(c),'DNGs',flush=True)
    pd.DataFrame(inv).to_csv(HERE/'run_inventory.csv',index=False)
    pd.DataFrame(rawrows).to_csv(HERE/'raw_frame_audit.csv',index=False)

    # these are technical subsamples, never treated as ten independent experiments.
    references={};depths={};sources={}
    for cm in (50,60,65,70,75,80,90):
        run=sorted((DATA/f'B_sharp{cm}').glob('motion_*'))[-1]
        d=read_run(run);numbers=d['c'].frame.iloc[2:12].tolist()
        images=[frame_image(run,f) for f in numbers]
        ref=np.median(images,axis=0).astype(np.float32)
        references[str(cm)]=ref;depths[str(cm)]=float(np.median(d['z']))
        sources[str(cm)]=dict(run=str(run.relative_to(DATA)),frames=list(map(int,numbers)),depth_mm=depths[str(cm)])
    np.savez_compressed(CACHE/'static_references_linear_green.npz',**references)
    dump(HERE/'static_reference_sources.json',sources)
    R,lo,hi=norm_image(references['70']);np.save(CACHE/'reference70_normalized.npy',R)
    zref=depths['70']
    selection=[];geometry={};allwindows=[]
    motiondirs=sorted([p for p in DATA.iterdir() if p.is_dir() and p.name.startswith('E_')])
    for group in motiondirs:
        for run in sorted(group.glob('motion_*')):
            d=read_run(run);c=d['c'];eligible=[]
            for _,r in c.iterrows():
                t=timeline(d,r)
                item=dict(condition=group.name,run=run.name,frame=int(r.frame),
                    **{k:t[k] for k in ('start','end','measured_n','depth_covered','light_covered','irregular_events','approach_mm','depth_range_mm','nonlinearity_mm')})
                allwindows.append(item)
                if t['depth_covered'] and t['light_covered'] and not t['irregular_events'] and t['approach_mm']>0:
                    eligible.append((r,t))
            if not eligible:
                selection.append(dict(condition=group.name,run=run.name,frame=None,status='NO_COVERED_APPROACH_WINDOW'));continue
            strong=[x for x in eligible if x[1]['approach_mm']>=15]
            pool=strong or eligible
            criterion='nonlinearity_mm' if 'variable' in group.name else 'approach_mm'
            row,t=max(pool,key=lambda x:x[1][criterion])
            status='PRIMARY_ELIGIBLE' if strong else 'WEAK_MOTION_EXPLORATORY'
            if 'pilot' in group.name:status='PILOT_EXPLORATORY'
            key=group.name+'__'+run.name
            try:
                g,registered,valid=fit_geometry(d,references['70'],zref)

                registered=(registered-lo)/(hi-lo)
                geometry[key]=g
                np.savez_compressed(CACHE/(key+'_geometry.npz'),reference=registered.astype(np.float32),valid=valid)
                err=''
            except Exception as ex:
                err=str(ex);status='GEOMETRY_FAILURE'
            selection.append(dict(condition=group.name,run=run.name,frame=int(row.frame),status=status,
                approach_mm=t['approach_mm'],nonlinearity_mm=t['nonlinearity_mm'],lidar_samples=t['measured_n'],
                geometry_error=err,source_frame_sha256=hashlib.sha256((run/'camera_dng'/row['file']).read_bytes()).hexdigest()))
            print('Selected',group.name,run.name,int(row.frame),status,err,flush=True)
    pd.DataFrame(allwindows).to_csv(HERE/'all_exposure_windows.csv',index=False)
    pd.DataFrame(selection).to_csv(HERE/'frozen_trial_selection.csv',index=False)
    dump(HERE/'endpoint_geometry.json',geometry)
    common=np.ones((HEIGHT,WIDTH),dtype='uint8')
    for key,g in geometry.items():
        # Geometric support only; this mask never depends on reconstruction scores.
        valid=np.load(CACHE/(key+'_geometry.npz'))['valid']
        interior=cv2.erode((valid>.99).astype('uint8'),np.ones((17,17),'uint8'))
        interior[:12]=0;interior[-12:]=0;interior[:,:12]=0;interior[:,-12:]=0
        inverse_map=np.linalg.inv(np.asarray(g['reference_to_reconstruction']))
        back=cv2.warpAffine(interior,inverse_map[:2].astype('float32'),(WIDTH,HEIGHT),flags=cv2.INTER_NEAREST)
        common &= back
    common=cv2.erode(common,np.ones((5,5),'uint8'))>0
    grad=np.hypot(cv2.Sobel(R,cv2.CV_32F,1,0),cv2.Sobel(R,cv2.CV_32F,0,1))
    feature=(cv2.dilate((grad>.35).astype('uint8'),np.ones((9,9),'uint8'))>0)&common
    np.savez_compressed(CACHE/'canonical_evaluation.npz',reference=R,full_mask=common,feature_mask=feature)
    assert common.sum()>10000 and feature.sum()>2000
    config=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,opencv=cv2.__version__,rawpy=rawpy.__version__,skimage=skimage.__version__,
        resolution=[WIDTH,HEIGHT],reference_depth_raw_lidar_mm=zref,primary_tv_lambda=.002,primary_iterations=320,quadrature_bins=48,
        primary_event_effective_lag_s=.026,primary_light_floor=.19,primary_depth_offset_mm=0,
        reference_used_for_evaluation_registration_and_shared_photometric_transfer=True,
        selection='Exploratory trajectory-only QC frozen before supplement scoring: covered approach>=15mm. Variable ranked by departure from endpoint-linear depth; smooth by displacement. Weak originals and pilots still scored separately.',
        timestamp='SensorTimestamp minus exposure for first-row exposure start; rolling-shutter row skew unmeasured. Pi implementation supported by official Picamera2 discussion; alignment sensitivity remains required.',
        timing_source='https://github.com/raspberrypi/picamera2/issues/821',
        raw='Two green sensels averaged per2x2 Bayer cell; black-level/white-level normalized; resized240x180; no JPEG, gamma, AWB, or demosaicing',
        evaluation='Inverse-warp all methods to one fixed sharp-reference coordinate frame; intersect geometric support over all16 attempts before scoring. Primary identical canonical mask; native-coordinate scores secondary.',
        iteration_revision='Initial80-iteration pilot retained separately; increase to320 based on >1e-3 iterate changes, then check640. This is a numerical stopping-budget change, not a per-trial score optimization.',
        non_preregistered=True)
    dump(HERE/'analysis_configuration.json',config)
    print('Inventory',len(inv),'runs;',len(rawrows),'DNGs;',len(selection),'motion attempts',flush=True)

if __name__=='__main__':main()
