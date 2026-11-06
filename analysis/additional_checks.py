
import argparse
from analysis_core import *

def spatial():
    refs=np.load(CACHE/'static_references_linear_green.npz')
    sources=json.loads((HERE/'static_reference_sources.json').read_text())
    R=norm_image(refs['70'])[0];rows=[]
    for cm in (50,60,65,70,75,80,90):
        img=norm_image(refs[str(cm)])[0]
        expected=sources['70']['depth_mm']/sources[str(cm)]['depth_mm']
        if cm==70:T=np.array([[1.,0,0],[0,1.,0]]);info=dict(scale=1.,rotation_deg=0.,inliers=0,matches=0,median_inlier_error_px=0.)
        else:
            try:T,info=similarity(R,img)
            except ValueError as ex:
                rows.append(dict(base_front_cm=cm,**sources[str(cm)],status=str(ex)));continue
        aligned=cv2.warpAffine(img,cv2.invertAffineTransform(T).astype(np.float32),(WIDTH,HEIGHT))
        good=cv2.warpAffine(np.ones_like(img),cv2.invertAffineTransform(T).astype(np.float32),(WIDTH,HEIGHT),flags=cv2.INTER_NEAREST)>.99
        good[:20]=False;good[-20:]=False;good[:,:20]=False;good[:,-20:]=False
        grad=np.hypot(cv2.Sobel(aligned,cv2.CV_32F,1,0),cv2.Sobel(aligned,cv2.CV_32F,0,1))
        edge=cv2.dilate((np.hypot(cv2.Sobel(R,cv2.CV_32F,1,0),cv2.Sobel(R,cv2.CV_32F,0,1))>.4).astype('uint8'),np.ones((5,5),'uint8'))>0
        mask=good&edge
        rows.append(dict(base_front_cm=cm,**sources[str(cm)],status='REGISTERED',**info,expected_pinhole_scale=expected,
            scale_ratio_measured_over_pinhole=info['scale']/expected,aligned_gradient_energy=float(np.mean(grad[mask]**2)),
            aligned_laplacian_variance=float(np.var(cv2.Laplacian(aligned,cv2.CV_32F)[mask])),pixels=int(mask.sum())))
        np.save(CACHE/f'static{cm}_registered_normalized.npy',aligned)
    table=pd.DataFrame(rows)
    baseline=table.loc[table.base_front_cm==70,'aligned_gradient_energy'].iloc[0]
    table['relative_gradient_energy_to70']=table.aligned_gradient_energy/baseline
    table.to_csv(HERE/'static_spatial_calibration.csv',index=False)

    cal=pd.read_csv(QC/'B_lidar_near_terminal_records.csv')
    cal.to_csv(HERE/'near_calibration_source_records.csv',index=False)
    print(table[['base_front_cm','scale','expected_pinhole_scale','relative_gradient_energy_to70']].to_string(index=False),flush=True)

def numerical():
    from run_reconstruction import analyse_one
    selected=pd.read_csv(HERE/'frozen_trial_selection.csv')
    rows=[]
    for _,s in selected[(selected.status=='PRIMARY_ELIGIBLE')&selected.condition.str.contains('coded')].iterrows():
        rows+=analyse_one(s,tag='iterations640',niter=640,save=False)
        rows+=analyse_one(s,tag='quadrature96',steps=96,save=False)
        pd.DataFrame(rows).to_csv(HERE/'numerical_stability_metrics.csv',index=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--numerical',action='store_true');a=p.parse_args()
    numerical() if a.numerical else spatial()
