
from analysis_core import *

def main():
    geometries=json.loads((HERE/'endpoint_geometry.json').read_text())
    sources=json.loads((HERE/'static_registration.json').read_text())
    ref=np.load(CACHE/'static_references_linear_green.npz')['70']
    canonical=np.load(CACHE/'canonical_evaluation.npz')
    g=dict(ze=float(np.median([x['ze'] for x in geometries.values()])),zl=float(np.median([x['zl'] for x in geometries.values()])),
        scale=float(np.median([x['scale'] for x in geometries.values()])),rotation_deg=0.,pivot=np.median([x['pivot'] for x in geometries.values()],axis=0).tolist())
    rows=[]
    for source in sources:
        run=DATA/'D_static_coded70_1s'/source['run'];d=read_run(run)
        row=d['c'][d['c'].frame==source['selected_frame']].iloc[0]
        registration=frame_image(run,source['registration_frame']);T,info=similarity(ref,registration)
        inverse=cv2.invertAffineTransform(T)
        B=norm_image(frame_image(run,row.frame))[0]
        line=timeline(d,row);zref=float(np.mean(line['z']))
        for model,z in [('constant',line['constant']),('lidar',line['z'])]:
            A=build_operator(z,line['w'],zref,g)
            X,pred,trace=tv_inverse(A,B,.002,320)
            for name,im in [('input',B),(model,X)]:
                estimate=cv2.warpAffine(im,inverse.astype('float32'),(WIDTH,HEIGHT))
                rows.append(dict(run=run.name,frame=int(row.frame),method=name,ROI='canonical_common',**metrics(canonical['reference'],estimate,canonical['full_mask']),depth_range_mm=line['depth_range_mm']))
        print(run.name,'completed',flush=True)
    table=pd.DataFrame(rows).drop_duplicates(['run','method','ROI'])
    table.to_csv(HERE/'static_depth_driven_control.csv',index=False)
    wide=table.pivot(index='run',columns='method',values=['PSNR_dB','SSIM']);wide.columns=['_'.join(x) for x in wide.columns]
    for m in ('PSNR_dB','SSIM'):wide[m+'_lidar_minus_input']=wide[m+'_lidar']-wide[m+'_input']
    wide.reset_index().to_csv(HERE/'static_depth_driven_summary.csv',index=False)
    dump(HERE/'static_depth_driven_model.json',dict(geometry=g,source='Median of endpoint-calibrated motion geometry, not fitted to static reconstruction scores.',
        interpretation='Stationary targets processed with their measured small depth fluctuations; shared static reference and canonical mask. This is a practical no-motion pipeline control, not proof of exact optical synchronization.'))
    print(wide.to_string(),flush=True)

if __name__=='__main__':main()
