"""Recompute saved-array metrics; optionally replay all inversions from RAW."""
from pathlib import Path
import argparse,hashlib,json,os,sys,platform,time
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from skimage.metrics import structural_similarity

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'reproduced';OUT.mkdir(exist_ok=True)
AN=ROOT/'analysis'

def metric(ref,im,mask):
    mse=float(np.mean((ref[mask]-im[mask])**2))
    _,ss=structural_similarity(ref,im,data_range=1.,gaussian_weights=True,sigma=1.5,
                              use_sample_covariance=False,full=True)
    return float(-10*np.log10(max(mse,1e-15))),float(ss[mask].mean())

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reconstruct',action='store_true',help='also replay all 16 RAW inversions with frozen geometry')
    ap.add_argument('--trial',choices=['S1','S2','S3','V1','V2','V3'],help='replay one coded RAW inversion only')
    args=ap.parse_args()
    rows=pd.read_csv(AN/'all_attempts_PSNR_SSIM.csv')
    six=pd.read_csv(AN/'coded_6_trials_PSNR_SSIM.csv')
    computed=[];maxp=maxs=0.
    for _,r in rows.iterrows():
        key=r.condition+'__'+r.run
        arrays=np.load(AN/'results'/(key+'_canonical.npz'))
        for method in ('input','denoise_only','constant','lidar'):
            p,s=metric(arrays['reference'],arrays[method],arrays['full_mask'].astype(bool))
            maxp=max(maxp,abs(p-r['PSNR_dB_'+method]));maxs=max(maxs,abs(s-r['SSIM_'+method]))
            computed.append(dict(condition=r.condition,run=r.run,frame=int(r.frame),status=r.status,
                                 method=method,PSNR_dB=p,SSIM=s))
    assert maxp<1e-5 and maxs<1e-6,(maxp,maxs)
    pd.DataFrame(computed).to_csv(OUT/'all_16_recomputed_metrics.csv',index=False)
    six.to_csv(OUT/'six_coded_trials_PSNR_SSIM.csv',index=False)
    source_manifest=json.loads((AN/'source_checksums.json').read_text(encoding='utf8'))
    for name,expected in source_manifest.items():
        original=ROOT/'data/supplement_20261031'/name
        assert original.stat().st_size==expected['bytes'],name
        assert hashlib.sha256(original.read_bytes()).hexdigest()==expected['sha256'],name
    hashes=[]
    for _,r in pd.read_csv(AN/'frozen_trial_selection.csv').iterrows():
        if pd.isna(r.frame):continue
        p=ROOT/'data/supplement_20261031'/r.condition/r.run/'camera_dng'/f'frame_{int(r.frame):06d}.dng'
        digest=hashlib.sha256(p.read_bytes()).hexdigest()
        assert digest==r.source_frame_sha256,str(p)
        hashes.append({'run':r.run,'frame':int(r.frame),'sha256':digest})
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42})
    fig,axs=plt.subplots(2,1,figsize=(3.5,3.2),layout='constrained')
    labels=['S1','S2','S3','V1','V2','V3']
    for ax,field,ylabel in zip(axs,['PSNR_dB_gain_lidar_vs_constant','SSIM_gain_lidar_vs_constant'],['PSNR difference (dB)','SSIM difference']):
        g=six[field].to_numpy();ax.axhline(0,color='.5',lw=.8)
        ax.scatter(range(6),g,c=['#286DA8']*3+['#B25E20']*3,zorder=3)
        ax.errorbar(6.25,g.mean(),yerr=g.std(ddof=1),fmt='D',color='black',capsize=3)
        ax.set_xticks(list(range(6))+[6.25],labels+['Mean\n+/- SD']);ax.set_ylabel(ylabel)
        ax.grid(axis='y',alpha=.2)
    fig.savefig(OUT/'paired_effects.pdf');fig.savefig(OUT/'paired_effects.png',dpi=300);plt.close(fig)
    a=np.load(AN/'cache/canonical_evaluation.npz');mask=a['full_mask'].astype(bool)
    fig,axs=plt.subplots(1,2,figsize=(3.5,1.65),layout='constrained')
    for ax in axs:ax.imshow(a['reference'],cmap='gray',vmin=0,vmax=1);ax.axis('off')
    red=np.zeros((*mask.shape,4));red[~mask]=[.85,.2,.12,.65];axs[1].imshow(red)
    axs[0].set_title('(a) Canonical reference',fontsize=8);axs[1].set_title('(b) Excluded pixels in red',fontsize=8)
    fig.savefig(OUT/'evaluation_mask.pdf');fig.savefig(OUT/'evaluation_mask.png',dpi=300);plt.close(fig)
    gains=six.PSNR_dB_gain_lidar_vs_constant.to_numpy()
    loo=[np.delete(gains,i).mean() for i in range(6)]
    pd.DataFrame({'omitted_trial':labels,'remaining_mean_gain_dB':loo}).to_csv(OUT/'leave_one_out.csv',index=False)
    qa={'python':platform.python_version(),'numpy':np.__version__,'saved_array_PSNR_max_error_dB':maxp,
        'saved_array_SSIM_max_error':maxs,'saved_attempts':len(rows),'verified_selected_RAW_hashes':len(hashes),
        'verified_source_files':len(source_manifest),
        'median_coded_gain_dB':float(np.median(gains)),'leave_one_out_mean_range_dB':[float(min(loo)),float(max(loo))],
        'scored_pixels':int(mask.sum()),'mode':'saved-array metric/figure verification'}
    if args.reconstruct or args.trial:
        os.environ['MECH5720_DATA_ROOT']=str(ROOT/'data/supplement_20261031')
        sys.path.insert(0,str(AN))
        from run_reconstruction import analyse_one
        import cv2
        selected=pd.read_csv(AN/'frozen_trial_selection.csv')
        if args.trial:
            r=six.iloc[labels.index(args.trial)]
            selected=selected[(selected.condition==r.condition)&(selected.run==r.run)]
        replay=[];start=time.time()
        for _,sel in selected.iterrows():
            if sel.status=='GEOMETRY_FAILURE':continue
            records=analyse_one(sel,save=False,tag='portable_RAW_replay')
            truth=rows[(rows.condition==sel.condition)&(rows.run==sel.run)].iloc[0]
            for r in records:
                if r['ROI']!='canonical_common':continue
                r['PSNR_error_dB']=r['PSNR_dB']-truth['PSNR_dB_'+r['method']]
                r['SSIM_error']=r['SSIM']-truth['SSIM_'+r['method']]
                replay.append(r)
        df=pd.DataFrame(replay);df.to_csv(OUT/'RAW_replay_metrics.csv',index=False)
        qa.update(RAW_replay_attempts=int(df.run.nunique()),RAW_replay_seconds=time.time()-start,
          opencv=cv2.__version__,RAW_PSNR_max_error_dB=float(df.PSNR_error_dB.abs().max()),
          RAW_SSIM_max_error=float(df.SSIM_error.abs().max()))
        assert qa['RAW_PSNR_max_error_dB']<.01 and qa['RAW_SSIM_max_error']<.0002,qa
        qa['mode']='saved-array verification plus frozen-geometry RAW replay'
    (OUT/'verification.json').write_text(json.dumps(qa,indent=2),encoding='utf8')
    print(json.dumps(qa,indent=2));print('PASS: outputs written to '+str(OUT))

if __name__=='__main__':main()
