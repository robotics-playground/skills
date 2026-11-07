"""Remove raster defects only: sub-pixel contour -> corner detection -> per-segment smoothing spline,
bounded so the curve never leaves the drawing by more than TOL artwork pixels. Real corners stay sharp."""
import numpy as np, pickle, json
from scipy import ndimage as ndi
from scipy.interpolate import splprep, splev
from skimage import measure
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
PM=json.load(open('pixel_metrics.json'))   # measured from the image itself (measure_px.py)
Z=np.load('full_masks.npz'); UP=int(Z['UP']); X0=float(Z['X0']); Y0=float(Z['Y0'])
SMALL={'EYEBROWS','MOUTH','NOSE','GLASSES'}
def resample(p,step):
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    t=np.arange(0,d[-1],step); return np.c_[np.interp(t,d,p[:,0]),np.interp(t,d,p[:,1])]
def turning(p,k):
    a=np.roll(p,k,0); b=np.roll(p,-k,0); v1=p-a; v2=b-p
    ang=np.arctan2(v1[:,0]*v2[:,1]-v1[:,1]*v2[:,0],(v1*v2).sum(1)); return np.abs(np.degrees(ang))
def corners(p,k,thr):
    t=turning(p,k); idx=[]
    for i in np.argsort(-t):
        if t[i]<thr: break
        if all(min(abs(i-j),len(p)-abs(i-j))>2*k for j in idx): idx.append(i)
    return sorted(idx)
def fit_seg(seg,closed,sig,tol):
    n=len(seg)
    if n<5: return seg
    w=np.ones(n)
    if not closed: w[[0,-1]]=1e3        # pin the corner points
    for s_mult in (1.0,0.5,0.25,0.1,0.0):
        try:
            tck,_=splprep([seg[:,0],seg[:,1]],w=w,s=n*(sig*s_mult)**2,per=int(closed),k=3 if n>8 else 1)
        except Exception: continue
        u=np.linspace(0,1,max(int(n*3),20)); c=np.column_stack(splev(u,tck))
        if not closed: c[0],c[-1]=seg[0],seg[-1]
        dev=LineString(seg).hausdorff_distance(LineString(c))   # vs corner-rebuilt trace
        if dev<=tol: return c
    return seg
def smooth_ring(ring,small):
    step=0.25                              # artwork px
    p=resample(np.vstack([ring,ring[:1]]),step)[:-1] if len(ring)>3 else ring
    if len(p)<8: return p,0
    k=max(3,int(PM['corner_scale_px']/step))      # corners judged over one source pixel
    thr=40 if small else 32
    cs=corners(p,k,thr) if len(p)>4*k else []
    sig=PM['smoothing_sigma_px']           # jitter level derived from the image
    tol=0.35 if small else 0.45              # max allowed departure from the drawing (artwork px)
    # --- a corner is real only if the edge is straight on both sides (a round tip turns too, but is curved) ---
    if cs:
        r_=int(PM['effective_source_pixel_px']/step); n_=len(p); keep=[]
        for c in cs:
            A=p[[(c-r_-j)%n_ for j in range(1,2*r_)]]; B=p[[(c+r_+j)%n_ for j in range(1,2*r_)]]
            ra=np.linalg.svd(A-A.mean(0))[1][1]/np.sqrt(len(A)); rb=np.linalg.svd(B-B.mean(0))[1][1]/np.sqrt(len(B))
            def dirn(Q): v=Q[-1]-Q[0]; return np.arctan2(v[1],v[0])
            def bend(Q): h=len(Q)//2; a=dirn(Q[:h+1])-dirn(Q[h:]); return abs((a+np.pi)%(2*np.pi)-np.pi)
            turn=abs((dirn(A)-dirn(B)+np.pi)%(2*np.pi)-np.pi)
            # real corner: straight sides that keep their direction; an arc keeps turning along its sides
            if max(ra,rb)<=PM['effective_source_pixel_px']/8 and max(bend(A),bend(B))<0.25*turn: keep.append(c)
        cs=keep
    # --- rebuild true corners: the image's softness rounds them over ~1 source pixel ---
    if cs:
        r=int(PM['effective_source_pixel_px']/step)        # rounding zone = one source pixel
        n=len(p); p=p.copy()
        for c in cs:
            ia=[(c-r-j)%n for j in range(1,2*r)]; ib=[(c+r+j)%n for j in range(1,2*r)]
            A=p[ia]; B=p[ib]
            sa=np.linalg.svd(A-A.mean(0)); sb=np.linalg.svd(B-B.mean(0)); da=sa[2][0]; db=sb[2][0]
            # both arms must be straight (rms off-line < 1/8 source px); a round tip is not a corner
            if max(sa[1][1],sb[1][1])/np.sqrt(len(A))>PM['effective_source_pixel_px']/8: continue
            M_=np.c_[da,-db]
            if abs(np.linalg.det(M_))<0.15: continue          # nearly parallel: not a real corner
            tt=np.linalg.solve(M_,B.mean(0)-A.mean(0)); v=A.mean(0)+tt[0]*da
            if np.linalg.norm(v-p[c])>0.9*PM['effective_source_pixel_px']: continue   # implausible: keep as traced
            for j in range(-r,r+1):                            # straight runs into the rebuilt vertex
                i=(c+j)%n; a_=abs(j)/r
                end=p[(c-r)%n] if j<0 else p[(c+r)%n]
                p[i]=v*(1-a_)+end*a_
    if not cs: out=fit_seg(p,True,sig,tol)
    else:
        parts=[]
        for a,b in zip(cs,cs[1:]+[cs[0]+len(p)]):
            seg=np.vstack([p,p])[a:b+1]; parts.append(fit_seg(seg,False,sig,tol)[:-1])
        out=np.vstack(parts)
    return out,len(cs)
def mask_rings(m,small):
    f=ndi.gaussian_filter(m.astype(float),0.9 if small else 1.4)
    cs=measure.find_contours(np.pad(f,2),0.5)
    rings=[np.c_[c[:,1]-2,c[:,0]-2]/UP+[X0,Y0] for c in cs]
    return [r for r in rings if len(r)>6 and Polygon(r).area>(0.3 if small else 2.0)]
out={}; report={}
import model as M_
_fit=json.load(open('fit.json'))
for k in Z.files:
    if k in ('UP','X0','Y0'): continue
    if k in ('CRAVAT','JACKET'):   # exact straight-line geometry: shirt V and the jacket panel with that V cut out
        cq=json.load(open('cravat_poly.json')); Pp=_fit['P']; Xb=_fit['X']['body']
        y_top=Pp['n_cy']                              # suit top edge extends up to the nose (hidden behind face/helmet)
        kk=(cq['w_at_v']-cq['w_top_at_chin'])/(cq['y_v']-cq['y_chin']); wt=cq['w_top_at_chin']+kk*(y_top-cq['y_chin'])
        v=Polygon([(cq['xc']-wt,y_top),(cq['xc']+wt,y_top),(cq['xc']+cq['w_at_v'],cq['y_v']),(cq['xc'],cq['y_tip']),(cq['xc']-cq['w_at_v'],cq['y_v'])])
        jacket=__import__('shapely.geometry',fromlist=['box']).box(Xb-Pp['t_w'],y_top,Xb+Pp['t_w'],Pp['t_bot'])
        out[k]=v if k=='CRAVAT' else jacket.difference(v)
        report[k]=dict(corners_kept=5,max_dev_px=0.0,source='exact straight-line geometry'); continue
    if k=='ARMS':     # exact analytic ovals, no tracing
        out[k]=M_.arms(_fit['P'],_fit['X']['body']).intersection(Polygon([(0,0),(999,0),(999,435.8),(0,435.8)]))
        report[k]=dict(corners_kept=0,max_dev_px=0.0,source='analytic two-oval model'); continue
    small=k in SMALL; geom=Polygon(); ncorner=0; devs=[]
    for r in mask_rings(Z[k],small):
        sr,nc=smooth_ring(r,small); ncorner+=nc
        if len(sr)<3: continue
        devs.append(LineString(np.vstack([r,r[:1]])).hausdorff_distance(LineString(np.vstack([sr,sr[:1]]))))
        geom=geom.symmetric_difference(Polygon(sr).buffer(0))      # even-odd -> holes
    if small and geom.geom_type=='MultiPolygon':            # drop empty specks (< 0.5 px^2)
        geom=unary_union([p for p in geom.geoms if p.area>=0.5])
    if not small and geom.geom_type=='MultiPolygon':        # drop stray specks (raster leftovers), keep real pieces
        big=max(p.area for p in geom.geoms); geom=unary_union([p for p in geom.geoms if p.area>=0.05*big])
    out[k]=geom; report[k]=dict(corners_kept=ncorner,max_dev_px=round(max(devs),3) if devs else 0)
# --- face: VISIBLE outline = traced artwork (above); HIDDEN part under the helmet = a normal face shape
#     (rounded forehead, temples, cheeks), so the face piece is bigger and the helmet sits on top ---
def _face_shape(A,CY,BT,BB,XC,e=2.47,n=480):
    t=np.linspace(0,2*np.pi,n,endpoint=False); c,s_=np.cos(t),np.sin(t); up=s_<0
    x=np.where(up, A*np.sign(c)*np.abs(c)**(2/2.6), A*np.sign(c)*np.abs(c)**(2/e))
    y=np.where(up, CY+BT*np.sign(s_)*np.abs(s_)**(2/2.6), CY+BB*s_)
    return Polygon(np.c_[x+XC,y]).buffer(0)
_vis_face=out['FACE']; _helm=out['HELMET']
_gl=out['GLASSES']; _CY=_gl.centroid.y                                  # eye line = glasses
_fx0,_fy0,_fx1,_fy1=_vis_face.bounds
_row=LineString([(-1e4,_CY),(1e4,_CY)]).intersection(_vis_face)        # visible face width at the eye line
_rx0,_,_rx1,_=_row.bounds
_hw=_helm.bounds[2]-_helm.bounds[0]; _tuck=0.06*_hw                    # how far the face runs under the helmet
_XC=(_rx0+_rx1)/2; _A=(_rx1-_rx0)/2+_tuck
_BT=_CY-(_fy0-_tuck)                                                      # forehead top: under the helmet opening
_BB=_fy1-_CY                                                              # chin: the traced chin
_shape=_face_shape(_A,_CY,_BT,_BB,_XC)
from shapely.geometry import box as _box
# visible bottom border = one circular arc, fitted to the traced jaw/chin between the flap tips
_hb=_helm.buffer(0.6)
_fp=np.array(_vis_face.exterior.coords)
_jaw=np.array([q for q in _fp if q[1]>_CY+0.35*_BB and not _hb.contains(Point(q))])
# previous (approved) head: upper face shape + circle fitted to the traced jaw -- kept unchanged above the join
_Am=np.c_[2*_jaw,np.ones(len(_jaw))]; _ox,_oy,_ok=np.linalg.lstsq(_Am,(_jaw**2).sum(1),rcond=None)[0]; _oR=float(np.sqrt(_ok+_ox**2+_oy**2))
_top=_shape.bounds[1]
_prev=unary_union([_face_shape(_oR,_oy,_oy-_top,_oR,_ox).intersection(_box(-1e4,-1e4,1e4,_oy)),
                   Point(_ox,_oy).buffer(_oR,quad_segs=256).intersection(_box(-1e4,_oy,1e4,1e4))])
# chin only: circle measured on the artwork's face/jacket edge (shirt columns excluded)
_ce=json.load(open('chin_edge.json')); _cx,_cy,_R=_ce['cx'],_ce['cy'],float(_ce['R'])
_yj=(_R**2-_oR**2+_oy**2-_cy**2)/(2*(_oy-_cy))                  # height where both circles have the same width
_chin=Point(_cx,_cy).buffer(_R,quad_segs=256).intersection(_box(-1e4,_yj,1e4,1e4))
out['FACE']=unary_union([_prev.intersection(_box(-1e4,-1e4,1e4,_yj)),_chin]).buffer(0.05).buffer(-0.05)
report['FACE_chin_circle']=dict(cx=round(float(_cx),2),cy=round(float(_cy),2),R=round(_R,2),join_y=round(float(_yj),2))
if out['FACE'].geom_type=='MultiPolygon': out['FACE']=max(out['FACE'].geoms,key=lambda q:q.area)
report['FACE_hidden_shape']=dict(eye_line_px=round(_CY,2),half_width_px=round(_A,2),forehead_px=round(_BT,2),chin_px=round(_BB,2),tuck_px=round(_tuck,2))
# jacket + pants delivered as one piece (the suit); seam between them disappears
out['SUIT']=unary_union([out['JACKET'],out['PANTS']]).buffer(0.01).buffer(-0.01)
# --- suit lower body: fill the waist step and make hips/legs one smooth curve (print shows smooth edges) ---
from shapely.geometry import box as _box
from scipy.ndimage import gaussian_filter1d
_P=_fit['P']; Y_S=_P['t_bot']-25; Y_E=_P['p_ank']-2          # waist .. ankle (boots cover below)
_low=_box(-1e4,Y_S,1e4,1e4)
out['SUIT']=unary_union([out['SUIT'],out['SUIT'].buffer(4,join_style='round').buffer(-4,join_style='round').intersection(_low)])
def _smooth_lower(poly,sig_px=2.5,step=0.25):
    def ring(r):
        r=np.array(r.coords)[:-1]
        d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([r,r[:1]]),axis=0),axis=1))]
        t=np.arange(0,d[-1],step); rr=np.vstack([r,r[:1]])
        p=np.c_[np.interp(t,d,rr[:,0]),np.interp(t,d,rr[:,1])]
        sm=np.c_[gaussian_filter1d(p[:,0],sig_px/step,mode='wrap'),gaussian_filter1d(p[:,1],sig_px/step,mode='wrap')]
        w=np.clip((p[:,1]-Y_S)/6,0,1)*np.clip((Y_E-p[:,1])/6,0,1)   # blend in/out at the ends of the zone
        return p*(1-w[:,None])+sm*w[:,None]
    ps=list(getattr(poly,'geoms',[poly])); res=[]
    for q in ps:
        res.append(Polygon(ring(q.exterior),[ring(h) for h in q.interiors]).buffer(0))
    return unary_union(res)
_before=out['SUIT']; out['SUIT']=_smooth_lower(out['SUIT'])
# --- sharp waist corners: where the straight jacket side meets the hip curve ---
def _sharp_waist(poly):
    Xb=_fit['X']['body']; tw=_P['t_w']
    r=np.array(poly.exterior.coords)[:-1]
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([r,r[:1]]),axis=0),axis=1))]
    t=np.arange(0,d[-1],0.2); rr=np.vstack([r,r[:1]])
    p=np.c_[np.interp(t,d,rr[:,0]),np.interp(t,d,rr[:,1])]; n=len(p)
    for xe in (Xb-tw,Xb+tw):
        on=np.nonzero((np.abs(p[:,0]-xe)<0.35)&(p[:,1]>Y_S-25)&(p[:,1]<Y_S+25))[0]
        if not len(on): continue
        iA=on[np.argmax(p[on,1])]                                        # lowest point still on the jacket side line
        out_dir=np.sign(xe-Xb)                                            # hip goes outward from here
        stepi=1 if p[(iA+25)%n,1]>p[(iA-25)%n,1] else -1                # walk along the contour toward the hip (downward)
        idx=[(iA+stepi*j)%n for j in range(1,int(14/0.2))]
        arc=np.cumsum(np.r_[0,np.linalg.norm(np.diff(p[idx],axis=0),axis=1)])
        seg=p[[i for i,a in zip(idx,arc) if 3.5<=a<=8.0]]               # hip curve beyond the rounded zone
        if len(seg)<5: continue
        dirv=np.linalg.svd(seg-seg.mean(0))[2][0]
        if abs(dirv[0])<1e-6: continue
        tt=(xe-seg.mean(0)[0])/dirv[0]; V=seg.mean(0)+tt*dirv              # extend hip tangent to the jacket line
        if abs(V[1]-p[iA,1])>12: continue
        cut=[i for i,a in zip(idx,arc) if a<3.5]+[iA]
        p[cut]=np.nan; p[iA]=V
        snap=(np.abs(p[:,0]-xe)<0.8)&(p[:,1]>Y_S-30)&(p[:,1]<V[1])      # jacket side stays a straight line to the corner
        p[snap,0]=xe
    p=p[~np.isnan(p[:,0])]
    return Polygon(p,[h for h in poly.interiors]).buffer(0)
if out['SUIT'].geom_type=='Polygon': out['SUIT']=_sharp_waist(out['SUIT'])
# --- legs: crotch gap = 3 fitted curves (inner edges x(y), top y(x)) meeting at sharp corners;
#     outer leg edges = one continuous fitted curve each (waist corner -> ankle) ---
from scipy.interpolate import LSQUnivariateSpline
def _legs(poly):
    Xb=_fit['X']['body']; yc=_P['p_crotch']
    r=np.array(poly.exterior.coords)[:-1]
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([r,r[:1]]),axis=0),axis=1))]
    t=np.arange(0,d[-1],0.2); rr=np.vstack([r,r[:1]])
    p=np.c_[np.interp(t,d,rr[:,0]),np.interp(t,d,rr[:,1])]
    y_bot=p[:,1].max()
    # ---------- crotch gap ----------
    gp=p[(np.abs(p[:,0]-Xb)<50)&(p[:,1]>yc-8)&(p[:,1]<y_bot-1.5)]
    top=gp[(gp[:,1]<yc+5)&(np.abs(gp[:,0]-Xb)<_P['p_gt']-4)]
    L=gp[(gp[:,0]<Xb)&(gp[:,1]>yc+6)&(gp[:,1]<y_bot-3)]; Rr=gp[(gp[:,0]>Xb)&(gp[:,1]>yc+6)&(gp[:,1]<y_bot-3)]
    cl=np.polyfit(L[:,1],L[:,0],2); cr=np.polyfit(Rr[:,1],Rr[:,0],2)
    # top of the gap: flat line with a smooth downward bulge in the middle (crotch), as drawn
    from scipy.optimize import least_squares
    def bump(q,x):
        y0,dep,xc,w=q; u=np.clip((x-xc)/max(w,1e-3),-1,1); return y0+dep*(1-u**4)**2   # broad, round-bottomed bulge
    q=least_squares(lambda q:bump(q,top[:,0])-top[:,1],[np.median(top[np.abs(top[:,0]-Xb)>20,1]),1.3,Xb-3,14.0],loss='soft_l1',f_scale=0.3,
                    bounds=([yc-8,0,Xb-15,4],[yc+8,4,Xb+15,30])).x
    global GAP_TOP; GAP_TOP=dict(y_flat=float(q[0]),bulge_depth_px=float(q[1]),bulge_center_px=float(q[2]-Xb),bulge_half_width_px=float(q[3]))
    top_y=lambda x: bump(q,np.asarray(x,float))
    def corner(cs):                     # intersect side x(y) with top y(x)
        y=yc
        for _ in range(50): x=np.polyval(cs,y); y=float(top_y(x))
        return np.array([np.polyval(cs,y),y])
    cL,cR=corner(cl),corner(cr)
    ys_l=np.linspace(cL[1],y_bot+20,120); ys_r=np.linspace(cR[1],y_bot+20,120); xs_t=np.linspace(cL[0],cR[0],240)
    gap=Polygon(list(zip(np.polyval(cl,ys_l[::-1]),ys_l[::-1]))+list(zip(xs_t,top_y(xs_t)))+list(zip(np.polyval(cr,ys_r),ys_r)))
    fill=_box(Xb-45,yc-10,Xb+45,y_bot)
    poly=poly.union(fill.intersection(poly.convex_hull)).difference(gap).buffer(0)
    if poly.geom_type!='Polygon': poly=max(poly.geoms,key=lambda q:q.area)
    # ---------- outer leg edges ----------
    r=np.array(poly.exterior.coords)[:-1]
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([r,r[:1]]),axis=0),axis=1))]
    t=np.arange(0,d[-1],0.2); rr=np.vstack([r,r[:1]])
    p=np.c_[np.interp(t,d,rr[:,0]),np.interp(t,d,rr[:,1])]; n=len(p)
    for side in (-1,1):
        xe=Xb+side*_P['t_w']
        cand=np.nonzero((np.abs(p[:,0]-xe)<0.05)&(p[:,1]>Y_S-30)&(p[:,1]<Y_S+30))[0]
        if not len(cand): continue
        i0=cand[np.argmax(p[cand,1])]                                  # waist corner
        bot=np.nonzero((side*(p[:,0]-Xb)>45)&(p[:,1]>y_bot-3.0)&(p[:,1]<y_bot-2.0))[0]   # stop just above the ankle corner
        if not len(bot): continue
        i1=bot[np.argmin(side*(p[bot,0]-Xb))] if False else bot[np.argmax(side*(p[bot,0]-Xb))]   # outer ankle corner
        idx=np.arange(i0,i1+1) if i1>i0 else np.arange(i0,i1+1+n)%n
        if len(idx)>n/2: idx=np.arange(i1,i0+1) if i0>i1 else np.arange(i1,i0+1+n)%n
        seg=p[idx]; a=np.r_[0,np.cumsum(np.linalg.norm(np.diff(seg,axis=0),axis=1))]
        knots=np.linspace(a[0],a[-1],6)[1:-1]
        w=np.ones(len(a)); w[[0,-1]]=1e4
        sx=LSQUnivariateSpline(a,seg[:,0],knots,w=w,k=3); sy=LSQUnivariateSpline(a,seg[:,1],knots,w=w,k=3)
        new=np.c_[sx(a),sy(a)]; new[0]=seg[0]; new[-1]=seg[-1]
        p[idx]=new
    return Polygon(p).buffer(0)
if out['SUIT'].geom_type=='Polygon':
    _b=out['SUIT']; out['SUIT']=_legs(out['SUIT'])
    report['SUIT_gap_top']=GAP_TOP
    report['SUIT_legs_max_shift_px']=round(LineString(np.array(_b.exterior.coords)).hausdorff_distance(LineString(np.array(out['SUIT'].exterior.coords))),3)
report['SUIT_lower_smoothing_max_shift_px']=round(LineString(np.array(_before.exterior.coords)).hausdorff_distance(LineString(np.array(out['SUIT'].exterior.coords))),3) if out['SUIT'].geom_type=='Polygon' else None
report['SUIT']=dict(corners_kept=report['JACKET']['corners_kept']+report['PANTS']['corners_kept'],max_dev_px=report['PANTS']['max_dev_px'],source='jacket (exact) + pants (traced), merged')
pickle.dump(out,open('smooth.pkl','wb')); json.dump(report,open('smooth_report.json','w'),indent=1)
print(json.dumps(report))
