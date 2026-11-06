from analysis_core import *

def comparison_table(df,roi='canonical_common'):
    d=df[df.ROI==roi].copy()
    key=['condition','run','frame','status','analysis_tag']
    wide=d.pivot(index=key,columns='method',values=['PSNR_dB','SSIM'])
    wide.columns=['_'.join(x) for x in wide.columns];wide=wide.reset_index()
    wide['profile']=np.where(wide.condition.str.contains('variable'),'variable','smooth')
    wide['illumination']=np.where(wide.condition.str.contains('coded'),'coded','white')
    for metric in ('PSNR_dB','SSIM'):
        for base in ('constant','input','denoise_only'):
            wide[f'{metric}_gain_lidar_vs_{base}']=wide[f'{metric}_lidar']-wide[f'{metric}_{base}']
    return wide

def aggregate(wide):
    records=[]
    for (light,profile),d in wide.groupby(['illumination','profile'],sort=True):
        r=dict(illumination=light,profile=profile,n_independent_pushes=len(d))
        for c in d.select_dtypes(include='number').columns:
            if c=='frame':continue
            r[c+'_mean']=float(d[c].mean());r[c+'_sample_SD']=float(d[c].std(ddof=1)) if len(d)>1 else None
            if 'gain_' in c:r[c+'_positive_trials']=int((d[c]>0).sum())
        records.append(r)
    return pd.DataFrame(records)

def main():
    raw=pd.read_csv(HERE/'motion_metrics_all.csv');wide=comparison_table(raw)
    wide.to_csv(HERE/'all_attempts_PSNR_SSIM.csv',index=False)
    primary=wide[wide.status=='PRIMARY_ELIGIBLE'].copy();primary.to_csv(HERE/'primary_12_trials_PSNR_SSIM.csv',index=False)
    group=aggregate(primary);group.to_csv(HERE/'group_mean_sample_SD.csv',index=False)
    coded=primary[primary.illumination=='coded'];coded.to_csv(HERE/'coded_6_trials_PSNR_SSIM.csv',index=False)
    feature=comparison_table(raw,'canonical_features')
    feature[feature.status=='PRIMARY_ELIGIBLE'].to_csv(HERE/'primary_12_feature_region_PSNR_SSIM.csv',index=False)
    roi_rows=[]
    for roi in ('canonical_common','canonical_features','common_interior','features'):
        alt=comparison_table(raw,roi)
        alt=alt[(alt.status=='PRIMARY_ELIGIBLE')&(alt.illumination=='coded')]
        for metric in ('PSNR_dB','SSIM'):
            delta=alt[f'{metric}_gain_lidar_vs_constant']
            roi_rows.append(dict(ROI=roi,metric=metric,n_independent_pushes=len(alt),mean_gain_lidar_vs_constant=float(delta.mean()),sample_SD=float(delta.std(ddof=1)),positive_trials=int((delta>0).sum())))
    pd.DataFrame(roi_rows).to_csv(HERE/'evaluation_region_sensitivity.csv',index=False)
    sens=comparison_table(pd.read_csv(HERE/'sensitivity_metrics.csv'))
    sens.to_csv(HERE/'sensitivity_trial_comparisons.csv',index=False)
    srows=[]
    base=coded.set_index('run')
    for tag,d in sens.groupby('analysis_tag',sort=False):
        srows.append(dict(analysis_tag=tag,n=len(d),mean_PSNR_gain_lidar_vs_constant=d.PSNR_dB_gain_lidar_vs_constant.mean(),
            SD_PSNR_gain_lidar_vs_constant=d.PSNR_dB_gain_lidar_vs_constant.std(ddof=1),
            mean_SSIM_gain_lidar_vs_constant=d.SSIM_gain_lidar_vs_constant.mean(),
            PSNR_positive_trials=int((d.PSNR_dB_gain_lidar_vs_constant>0).sum()),SSIM_positive_trials=int((d.SSIM_gain_lidar_vs_constant>0).sum()),
            mean_PSNR_gain_lidar_vs_input=d.PSNR_dB_gain_lidar_vs_input.mean(),
            max_abs_PSNR_change_vs_primary=max(abs(float(r.PSNR_dB_lidar)-float(base.loc[r.run,'PSNR_dB_lidar'])) for _,r in d.iterrows())))
    pd.DataFrame(srows).to_csv(HERE/'sensitivity_summary.csv',index=False)
    num=comparison_table(pd.read_csv(HERE/'numerical_stability_metrics.csv'))
    num.to_csv(HERE/'numerical_stability_comparisons.csv',index=False)
    numchecks=[]
    for tag,d in num.groupby('analysis_tag'):
        numchecks.append(dict(tag=tag,max_abs_lidar_PSNR_change=max(abs(float(r.PSNR_dB_lidar)-float(base.loc[r.run,'PSNR_dB_lidar'])) for _,r in d.iterrows()),
            mean_lidar_vs_constant_PSNR=float(d.PSNR_dB_gain_lidar_vs_constant.mean())))
    inv=pd.read_csv(HERE/'run_inventory.csv')
    stats={}
    for metric in ('PSNR_dB','SSIM'):
        for b in ('constant','input','denoise_only'):
            a=coded[f'{metric}_gain_lidar_vs_{b}']
            stats[f'{metric}_gain_lidar_vs_{b}']=dict(mean=float(a.mean()),sample_SD=float(a.std(ddof=1)),min=float(a.min()),max=float(a.max()),positive_trials=int((a>0).sum()),n=6)
    static=pd.read_csv(HERE/'static_no_motion_control.csv')
    st=static[static.ROI=='common_interior'].pivot(index='run',columns='tv_lambda',values=['PSNR_dB','SSIM'])
    staticdelta={m:dict(mean=float((st[m][.002]-st[m][0]).mean()),min=float((st[m][.002]-st[m][0]).min()),max=float((st[m][.002]-st[m][0]).max())) for m in ('PSNR_dB','SSIM')}
    selection=pd.read_csv(HERE/'frozen_trial_selection.csv')
    summary=dict(raw_runs=int(len(inv)),raw_DNGs=int(inv.camera_frames.sum()),all_motion_attempts=len(wide),
        primary_independent_pushes=len(primary),primary_coded_pushes=len(coded),weak_attempts_retained=int((wide.status=='WEAK_MOTION_EXPLORATORY').sum()),pilots_retained=int((wide.status=='PILOT_EXPLORATORY').sum()),
        coded_primary_paired_effects=stats,static_identity_control_lambda0_max_abs_error=float(static[static.tv_lambda==0].identity_max_abs_error.max()),
        static_TV0p002_effect_vs_input=staticdelta,numerical_stability=numchecks,
        sensitivity_experiments=len(srows),
        inference='Descriptive exploratory means and sample SD across independent hand pushes. No p-values or randomized/matched coded-versus-white causal claim. Per-frame samples are technical observations. Scores are reference-assisted and registration-sensitive.',
        no_motion_claim='An identity forward model with zero regularization reproduces its input exactly. Static TV is checked separately; this does not validate all dynamic-model assumptions.')
    dump(HERE/'analysis_summary.json',summary)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
