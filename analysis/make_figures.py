
import sys,json
from pathlib import Path
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analysis_core import HERE,QC,DATA,CACHE,read_run
sys.path.insert(0,str(HERE/'figure_support'))
from audit_panel_alignment import require_matplotlib_panel_alignment

FIG=HERE/'figures';FIG.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Arial','DejaVu Sans'],'font.size':8,'axes.titlesize':8,'axes.labelsize':8,
    'xtick.labelsize':7,'ytick.labelsize':7,'axes.spines.top':False,'axes.spines.right':False,
    'svg.fonttype':'none','pdf.fonttype':42,'legend.fontsize':7,'legend.frameon':False})
BLUE='#286291';ORANGE='#BD6A36';GRAY='#666666'

def save(fig,name,**alignment_options):
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,json_out=FIG/(name+'.alignment.json'),tolerance_pt=1.5,gutter_tolerance_pt=1.5,strict=True,**alignment_options)
    fig.savefig(FIG/(name+'.pdf'))
    fig.savefig(FIG/(name+'.svg'))
    fig.savefig(FIG/(name+'.png'),dpi=300)
    fig.savefig(FIG/(name+'.tiff'),dpi=300,pil_kwargs={'compression':'tiff_lzw'})
    plt.close(fig)

def coded_trials():
    d=pd.read_csv(HERE/'coded_6_trials_PSNR_SSIM.csv').sort_values(['profile','run']).reset_index(drop=True)
    d['label']=['S1','S2','S3','V1','V2','V3']
    d.to_csv(FIG/'coded_figure_trial_labels.csv',index=False)
    return d

def primary(d):
    fig,axs=plt.subplots(1,2,figsize=(7.2,2.9));fig.subplots_adjust(left=.09,right=.98,bottom=.22,top=.84,wspace=.36)
    for ax,metric,title,unit in zip(axs,['PSNR_dB','SSIM'],['a  PSNR improvement','b  SSIM improvement'],['PSNR difference (dB)','SSIM difference']):
        values=d[metric+'_gain_lidar_vs_constant'].to_numpy()
        ax.axhline(0,color=GRAY,lw=.8,ls='--',zorder=1)
        ax.scatter(np.arange(6),values,c=[BLUE]*3+[ORANGE]*3,s=30,edgecolor='white',linewidth=.5,zorder=3)
        ax.set_xticks(range(6),d.label);ax.set_ylabel(unit);ax.set_title(title,loc='left',weight='bold',pad=11)
        ax.grid(axis='y',alpha=.2)
    fig.text(.5,.025,'LiDAR minus endpoint-linear model on the same image; n = 6 independent coded pushes.',ha='center',fontsize=7)
    save(fig,'fig02_paired_coded_effects')

def diagnostics():
    fig,axs=plt.subplots(1,3,figsize=(7.2,2.8));fig.subplots_adjust(left=.085,right=.98,bottom=.23,top=.81,wspace=.58)
    cal=json.loads((QC/'B_lidar_near_terminal_summary.json').read_text())['position_summaries']
    x=[r['paper_reference_mm'] for r in cal];y=[r['mean_residual_mm'] for r in cal];sd=[r['sample_SD_across_run_summaries_mm'] for r in cal]
    axs[0].errorbar(x,y,yerr=sd,fmt='o-',color=BLUE,capsize=3,lw=1)
    axs[0].axhline(0,ls='--',color=GRAY,lw=.7);axs[0].set(xlabel='Paper reference (mm)',ylabel='LiDAR minus ruler (mm)',xticks=x,ylim=(-9,1))
    sharp=pd.read_csv(HERE/'static_spatial_calibration.csv')
    axs[1].plot(sharp.base_front_cm,sharp.relative_gradient_energy_to70,'o-',color=BLUE,lw=1)
    axs[1].axvspan(65,75,color=BLUE,alpha=.08);axs[1].set(xlabel='Base front edge (cm)',ylabel='Relative gradient energy',xticks=[50,70,90],ylim=(0,1.12))
    c=json.loads((QC/'C_timing_QC_summary.json').read_text())['runs']
    f=json.loads((QC/'F_timing_end_QC_summary.json').read_text())['runs'][0]
    vals=[r['conditional_optical_alignment']['timestamp_as_exposure_end']['effective_offset_ms'] for r in c]+[f['conditional_optical_alignment']['timestamp_as_exposure_end']['effective_offset_ms']]
    axs[2].plot(range(4),vals,'o-',color=ORANGE,lw=1);axs[2].set(xticks=range(4),xticklabels=['Start1','Start2','Start3','End'],ylabel='Effective fitted offset (ms)',xlabel='Stationary 20 ms control',ylim=(15,40))
    for ax,title in zip(axs,['a  Distance reference','b  Fixed-focus limitation','c  Conditional timing']):ax.set_title(title,loc='left',weight='bold',pad=12);ax.grid(axis='y',alpha=.15)
    fig.text(.5,.025,'Distance bars: SD of 3 technical run summaries. Timing offsets are model fits, not optical synchronization.',ha='center',fontsize=6.6)
    save(fig,'fig01_acquisition_bounds')

def sensitivity():
    d=pd.read_csv(HERE/'sensitivity_summary.csv')
    primary=json.loads((HERE/'analysis_summary.json').read_text())['coded_primary_paired_effects']['PSNR_dB_gain_lidar_vs_constant']['mean']
    labels={'TV_0p0005':'TV 0.0005','TV_0p005':'TV 0.005','TV_0p02':'TV 0.02','depth_minus22mm':'Depth offset -22 mm','depth_plus22mm':'Depth offset +22 mm',
        'depth_time_minus50ms':'Depth timing -50 ms','depth_time_plus50ms':'Depth timing +50 ms','floor_0p10':'Black/white floor 0.10','floor_0p30':'Black/white floor 0.30',
        'lag_minus20ms':'Light offset -20 ms','lag_zero':'Light offset 0 ms','lag_plus50ms':'Light offset +50 ms','rotation_on':'Endpoint rotation included'}
    fig,ax=plt.subplots(figsize=(7.2,4.3));fig.subplots_adjust(left=.34,right=.96,bottom=.18,top=.88)
    yy=np.arange(len(d));ax.errorbar(d.mean_PSNR_gain_lidar_vs_constant,yy,xerr=d.SD_PSNR_gain_lidar_vs_constant,fmt='o',color=BLUE,ecolor='#AABAC8',capsize=2,ms=4,lw=.8)
    ax.axvline(0,ls='--',color=GRAY,lw=.8);ax.axvline(primary,color=ORANGE,lw=1)
    ax.set(yticks=yy,yticklabels=[labels[t] for t in d.analysis_tag],xlabel='LiDAR minus endpoint-linear PSNR (dB)')
    ax.invert_yaxis();ax.set_title('One factor changed at a time',loc='left',weight='bold',pad=12)
    fig.text(.5,.045,'Points: mean across the same 6 coded pushes. Bars: sample SD. Orange line: primary mean.',ha='center',fontsize=7)
    save(fig,'fig03_sensitivity')

def image_plate(d,light='coded'):

    fig,axs=plt.subplots(6,4,figsize=(7.2,9.2));fig.subplots_adjust(left=.035,right=.99,bottom=.035,top=.94,wspace=.06,hspace=.28)
    canonical=np.load(CACHE/'canonical_evaluation.npz');mask=canonical['full_mask'];yy,xx=np.where(mask)
    x0,x1=xx.min(),xx.max()+1;y0,y1=yy.min(),yy.max()+1
    for i,(_,row) in enumerate(d.iterrows()):
        a=np.load(HERE/'results'/(row.condition+'__'+row.run+'_canonical.npz'))
        for j,method in enumerate(('reference','input','constant','lidar')):
            ax=axs[i,j];ax.imshow(a[method][y0:y1,x0:x1],cmap='gray',vmin=0,vmax=1,interpolation='nearest');ax.set_xticks([]);ax.set_yticks([])
            for spine in ax.spines.values():spine.set_visible(False)
            title=row.label+' reference' if method=='reference' else f"{row[method+'_label'] if method+'_label' in row else method}: {row['PSNR_dB_'+method]:.2f} dB"
            ax.set_title(title,fontsize=7,pad=3)
    fig.suptitle('All six coded trials: shared physical target coordinates and grayscale',fontsize=10,y=.985)
    fig.text(.5,.008,'Columns: static reference | input | endpoint-linear reconstruction | LiDAR reconstruction',ha='center',fontsize=7)
    save(fig,'fig04_all_coded_reconstructions')
    return [int(x0),int(y0),int(x1),int(y1)]

def trajectories(d):
    fig,axs=plt.subplots(2,3,figsize=(7.2,4.6),sharex=True,sharey=True);fig.subplots_adjust(left=.09,right=.98,bottom=.17,top=.88,wspace=.22,hspace=.42)
    for ax,(_,row) in zip(axs.flat,d.iterrows()):
        a=np.load(HERE/'results'/(row.condition+'__'+row.run+'.npz'))
        data=read_run(DATA/row.condition/row.run);c=data['c'];r=c[c.frame==row.frame].iloc[0]
        inside=(data['t']>=r.start_s)&(data['t']<=r.end_s)
        ax.plot(a['t'],a['z'],color=BLUE,lw=1,label='LiDAR interpolation')
        ax.scatter(data['t'][inside]-r.start_s,data['z'][inside],s=10,color=BLUE,zorder=3,label='Received depth samples')
        ax.plot(a['t'],a['z_constant'],color=ORANGE,ls='--',lw=1,label='Endpoint-linear')
        ax.set(xlim=(0,1),ylim=(660,780),title=row.label+' | frame '+str(int(row.frame)),xticks=[0,.5,1],yticks=[660,700,740,780]);ax.grid(alpha=.15)
    for ax in axs[-1]:ax.set_xlabel('Time within exposure (s)')
    for ax in axs[:,0]:ax.set_ylabel('Raw LiDAR depth (mm)')
    handles,labels=axs[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=3,bbox_to_anchor=(.5,.01),fontsize=7)
    save(fig,'fig05_measured_trajectories')

def main():
    d=coded_trials();diagnostics();primary(d);sensitivity();crop=image_plate(d);trajectories(d)
    (FIG/'image_integrity.json').write_text(json.dumps(dict(backend='Python Matplotlib',grayscale_range=[0,1],gamma=1,shared_crop_xyxy=crop,
        images='Canonical linear-green arrays saved in results. No local retouching. Global normalization estimated once per input and shared across reconstruction methods.',
        source_data=['coded_6_trials_PSNR_SSIM.csv','near_calibration_source_records.csv','static_spatial_calibration.csv','sensitivity_summary.csv'],
        n='6 independent coded pushes; three smooth and three variable; no multiple-frame pseudoreplication'),indent=2),encoding='utf-8')

if __name__=='__main__':main()
