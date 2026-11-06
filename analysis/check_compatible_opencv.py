
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
runtime=HERE/'compatibility_check/runtime'
if runtime.exists():sys.path.insert(0,str(runtime))
from analysis_core import *
from run_reconstruction import analyse_one

def main():
    assert cv2.__version__=='4.10.0'
    refs=np.load(CACHE/'static_references_linear_green.npz')
    R=np.load(CACHE/'reference70_normalized.npy')
    _,lo,hi=norm_image(refs['70'])
    source=json.loads((HERE/'endpoint_geometry.json').read_text())
    rows=[];replayed=[]
    for _,r in pd.read_csv(HERE/'frozen_trial_selection.csv').iterrows():
        key=r.condition+'__'+r.run
        d=read_run(DATA/r.condition/r.run)
        g,registered,valid=fit_geometry(d,refs['70'],source[key]['zref'])
        old=np.load(CACHE/(key+'_geometry.npz'))
        normed=(registered-lo)/(hi-lo)
        prior=np.load(HERE/'results'/(key+'.npz'))
        canonical=np.load(HERE/'results'/(key+'_canonical.npz'))
        back=np.linalg.inv(np.asarray(g['reference_to_reconstruction']))
        warp_differences=[]
        for name in ('input','denoise_only','constant','lidar'):
            warped=cv2.warpAffine(prior[name],back[:2].astype('float32'),(WIDTH,HEIGHT),flags=cv2.INTER_LINEAR)
            warp_differences.append(float(np.max(abs(warped-canonical[name]))))
        rows.append(dict(condition=r.condition,run=r.run,
            reference_mapping_max_abs_difference=float(np.max(abs(np.asarray(g['reference_to_reconstruction'])-source[key]['reference_to_reconstruction']))),
            registered_reference_max_abs_difference=float(np.max(abs(normed-old['reference']))),
            valid_mask_identical=bool(np.array_equal(valid,old['valid'])),
            evaluated_image_warp_max_abs_difference=max(warp_differences)))
        replayed+=analyse_one(r,save=False)
    out=HERE/'compatibility_check';out.mkdir(exist_ok=True)
    pd.DataFrame(rows).to_csv(out/'opencv_4p10_geometry_check.csv',index=False)
    replay=pd.DataFrame(replayed)
    replay.to_csv(out/'opencv_4p10_frozen_replay_metrics.csv',index=False)
    keys=['condition','run','frame','status','analysis_tag','method','ROI']
    old_metrics=pd.read_csv(HERE/'motion_metrics_all.csv')
    joined=old_metrics.merge(replay,on=keys,suffixes=('_delivered','_replay'),validate='one_to_one')
    roi=joined[joined.ROI=='canonical_common']
    result=dict(opencv=cv2.__version__,numpy=np.__version__,tested_motion_attempts=len(rows),
        max_mapping_difference=max(x['reference_mapping_max_abs_difference'] for x in rows),
        max_registered_reference_difference=max(x['registered_reference_max_abs_difference'] for x in rows),
        max_evaluated_warp_difference=max(x['evaluated_image_warp_max_abs_difference'] for x in rows),
        all_valid_masks_identical=all(x['valid_mask_identical'] for x in rows),
        frozen_cache_replay_max_PSNR_difference=float(abs(roi.PSNR_dB_replay-roi.PSNR_dB_delivered).max()),
        frozen_cache_replay_max_SSIM_difference=float(abs(roi.SSIM_replay-roi.SSIM_delivered).max()),
        scope='All16 primary inversions replayed under4.10 using delivered frozen geometry/masks. Separately recomputed endpoint registrations can differ at subpixel/border rounding and do not replace the delivered geometry or scores.')
    dump(out/'opencv_4p10_compatibility.json',result)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
