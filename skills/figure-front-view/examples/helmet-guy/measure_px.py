"""Measure the artwork's pixel size, effective resolution (edge spread) and edge-position noise."""
import cv2, numpy as np, json, pickle
from scipy.ndimage import map_coordinates
from shapely.geometry import LineString
im=cv2.imread('/root/.claude/uploads/5391fe7b-2194-5842-9b9d-7c568b21fbc3/0fe88611-image.webp').astype(np.float32)
spec=json.load(open('parts_mm.json'))['spec']; mm_px=spec['scale_mm_per_artwork_px']
G=pickle.load(open('smooth.pkl','rb'))
BG=np.array([251,254,219],np.float32); BLUE=np.array([212,159,40],np.float32); BLACK=np.array([45,45,45],np.float32)
def profiles(ring,A,B,step=1.5,half=4.0,ns=81):
    """sample colour across the edge along normals; return alpha(t) profiles (0=B side, 1=A side)"""
    L=LineString(ring); out=[]
    for d in np.arange(0,L.length,step):
        p=np.array(L.interpolate(d).coords[0]); q=np.array(L.interpolate(min(d+0.5,L.length)).coords[0])
        tvec=q-p; n=np.linalg.norm(tvec)
        if n==0: continue
        nrm=np.array([-tvec[1],tvec[0]])/n
        t=np.linspace(-half,half,ns); pts=p[None]+t[:,None]*nrm[None]
        col=np.stack([map_coordinates(im[:,:,c],[pts[:,1],pts[:,0]],order=1) for c in range(3)],1)
        v=A-B; a=((col-B)@v)/(v@v)
        out.append((t,a))
    return out
def esf_width(profs):
    w=[]; pos=[]
    for t,a in profs:
        a=np.clip(a,-.2,1.2)
        if a[0]>a[-1]: a=a[::-1]
        if a[:10].mean()>0.15 or a[-10:].mean()<0.85: continue      # skip edges not between these two colours
        lo=t[np.argmax(a>=0.1)]; hi=t[np.argmax(a>=0.9)]; mid=t[np.argmax(a>=0.5)]
        w.append(hi-lo); pos.append(mid)
    return np.array(w),np.array(pos)
res={}
# helmet dome (blue | background) : top half of helmet exterior
helm=max(getattr(G['HELMET'],'geoms',[G['HELMET']]),key=lambda p:p.area)
ring=np.array(helm.exterior.coords); ring=ring[ring[:,1]<170]
w,pos=esf_width(profiles(ring,BLUE,BG)); res['helmet_vs_bg']=dict(n=len(w),edge_10_90_px=float(np.median(w)),edge_pos_offset_px_std=float(np.std(pos)))
# leg outer edges (black | background)
pants=max(getattr(G['PANTS'],'geoms',[G['PANTS']]),key=lambda p:p.area)
ring=np.array(pants.exterior.coords); ring=ring[(ring[:,1]>330)&(ring[:,1]<400)]
w2,pos2=esf_width(profiles(ring,BLACK,BG)); res['legs_vs_bg']=dict(n=len(w2),edge_10_90_px=float(np.median(w2)),edge_pos_offset_px_std=float(np.std(pos2)))
# a perfectly sampled ideal step edge through a box-filter pixel has 10-90% width = 0.8 px
ideal=0.8
eff=np.median(np.r_[w,w2])/ideal
res['nominal_mm_per_px']=mm_px
res['effective_pixel_px']=float(eff)
res['effective_mm_per_px']=float(eff*mm_px)
# positional noise of the sub-pixel edge = scatter of the 50% crossing about the smooth curve
noise=float(np.sqrt(np.mean(np.r_[pos-np.median(pos),pos2-np.median(pos2)]**2)))
res['edge_noise_px_rms']=noise
# defect budget: a raster can misplace an edge by up to half an effective pixel; compression adds the measured noise
tol_px=float(np.hypot(0.5*eff,2*noise)) if False else float(max(0.5*eff,3*noise))
res['defect_tolerance_px']=tol_px; res['defect_tolerance_mm']=tol_px*mm_px
res['smoothing_sigma_px']=noise
print(json.dumps(res,indent=1)); json.dump(res,open('pixel_metrics.json','w'),indent=1)

# ---- 1. calibrate: ideal box-filtered (anti-aliased) step, sampled the same way (bilinear along a normal) ----
rng=np.random.default_rng(0); widths={}
for scale in (1.0,1.5,2.0,2.5,3.0):
    ws=[]
    for _ in range(200):
        off=rng.uniform(0,1); ang=rng.uniform(0,np.pi/2)
        # render native step at (scale x finer) then box-downsample by `scale`, i.e. an image enlarged from a coarser original
        N=64; yy,xx=np.mgrid[0:N,0:N].astype(float)
        nx,ny=np.cos(ang),np.sin(ang)
        ss=8; fine=((((np.arange(N*ss)+.5)/ss)[None,:]*nx+((np.arange(N*ss)+.5)/ss)[:,None]*ny)>(N/2*(nx+ny)+off)).astype(float)
        native=fine.reshape(N,ss,N,ss).mean((1,3))
        if scale>1:   # coarse original (pixel = scale) then bicubic enlarge back to grid
            k=int(round(scale*ss)); M=(N*ss)//k
            coarse=fine[:M*k,:M*k].reshape(M,k,M,k).mean((1,3))
            native=cv2.resize(coarse.astype(np.float32),(N,N),interpolation=cv2.INTER_CUBIC)
        c0=np.array([N/2,N/2])+0*off; t=np.linspace(-4,4,81)
        p0=np.array([N/2*(1)+0.0,N/2]); base=np.array([(N/2*(nx+ny)+off)*nx,(N/2*(nx+ny)+off)*ny])
        pts=base[None]+t[:,None]*np.array([nx,ny])[None]
        a=map_coordinates(native,[pts[:,1],pts[:,0]],order=1)
        lo=t[np.argmax(a>=0.1)]; hi=t[np.argmax(a>=0.9)]; ws.append(hi-lo)
    widths[scale]=float(np.median(ws))
print('simulated 10-90% edge width by source pixel size:',widths)
meas=float(np.median(np.r_[w,w2]))
sc=np.array(list(widths)); wv=np.array(list(widths.values()))
eff_px=float(np.interp(meas,wv,sc))
# ---- 2. edge noise: raw 50% crossings vs a local quadratic over +-3 effective px (keeps real shape, removes jitter) ----
Z=np.load('full_masks.npz')
from skimage import measure
from scipy import ndimage as ndi
def raw_ring(mask):
    f=ndi.gaussian_filter(mask.astype(float),1.4); cs=measure.find_contours(np.pad(f,2),0.5)
    c=max(cs,key=len); return np.c_[c[:,1]-2,c[:,0]-2]/int(Z['UP'])+[float(Z['X0']),float(Z['Y0'])]
def local_noise(r,win):
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(r,axis=0),axis=1))]; t=np.arange(0,d[-1],0.25)
    P=np.c_[np.interp(t,d,r[:,0]),np.interp(t,d,r[:,1])]; n=len(P); h=int(win/0.25); res_=[]
    for i in range(h,n-h,4):
        seg=P[i-h:i+h+1]; tt=np.linspace(-1,1,len(seg))
        fx=np.polyval(np.polyfit(tt,seg[:,0],2),tt); fy=np.polyval(np.polyfit(tt,seg[:,1],2),tt)
        tan=np.gradient(np.c_[fx,fy],axis=0)[h]; tan/=np.linalg.norm(tan)+1e-12; nrm=np.array([-tan[1],tan[0]])
        res_.append((seg[h]-[fx[h],fy[h]])@nrm)
    return np.array(res_)
hr=raw_ring(Z['HELMET']); hr=hr[hr[:,1]<170]
pr=raw_ring(Z['PANTS']); pr=pr[(pr[:,1]>330)&(pr[:,1]<400)]
nz=np.r_[local_noise(hr,1.5*eff_px),local_noise(pr,1.5*eff_px)]
noise_rms=float(np.sqrt(np.mean(nz**2)))
out=dict(nominal_mm_per_px=mm_px,measured_edge_width_px=meas,simulated_widths=widths,
         effective_source_pixel_px=round(eff_px,2),effective_source_pixel_mm=round(eff_px*mm_px,3),
         edge_noise_rms_px=round(noise_rms,3),edge_noise_rms_mm=round(noise_rms*mm_px,3))
# raster-defect budget derived from the measurements
out['smoothing_sigma_px']=round(noise_rms,3)                       # what smoothing is allowed to remove (1 sigma of jitter)
out['max_shift_px']=round(max(2.5*noise_rms,0.25*eff_px),3)          # hard cap: jitter outliers, never a whole source pixel
out['corner_scale_px']=round(eff_px,2)                               # corners are judged over one source pixel
out['max_shift_mm']=round(out['max_shift_px']*mm_px,3)
print(json.dumps(out,indent=1)); json.dump(out,open('pixel_metrics.json','w'),indent=1)
