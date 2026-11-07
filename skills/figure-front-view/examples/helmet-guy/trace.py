"""Detail-preserving parts: visible edges traced from the artwork; fitted primitives only
(1) decide which part each artwork pixel belongs to and (2) complete the edges hidden behind front parts."""
import cv2, numpy as np, json, pickle
from scipy import ndimage as ndi
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
exec(open('fit.py').read().split('P={')[0])          # artwork masks: fg, blue, white, dark, face_t, det, cravat_t, hands_t, body_t, R, ox, oy
import model as M
# ---- round (disk) morphology: a diamond kernel would cut facets into smooth curves ----
def disk_op(m,op,r):
    r=max(1,int(round(r))); k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*r+1,2*r+1)); u=m.astype(np.uint8)
    if op=='dil': return cv2.dilate(u,k).astype(bool)
    if op=='close': return cv2.morphologyEx(u,cv2.MORPH_CLOSE,k).astype(bool)
    if op=='open': return cv2.morphologyEx(u,cv2.MORPH_OPEN,k).astype(bool)
d=json.load(open('fit.json')); P,X=d['P'],d['X']
grp=lambda k:X['head'] if k in('HELMET','FACE','GLASSES','EYEBROWS','NOSE','MOUTH') else (X['legs'] if k in('PANTS','BOOTS') else X['body'])
prim={k:R(f(P,grp(k))) for k,f in M.PARTS.items()}
sole=oy<435.8
vis={}
# ---- head ----
lb_,_=ndi.label(blue&sole); szb=np.bincount(lb_.ravel()); szb[0]=0
hm=lb_==szb.argmax()                                     # mixed-colour pixels elsewhere are not helmet
hh,_=ndi.label(ndi.binary_fill_holes(hm)&~hm); szh=np.bincount(hh.ravel()); szh[0]=0
vis['HELMET']=hm|np.isin(hh,np.nonzero((szh>0)&(szh<(3*UP)**2))[0])      # fill only pinholes, never the face opening
vis['FACE']=face_t&~det
for k in ['GLASSES','EYEBROWS','NOSE','MOUTH']: pass
# face graphics: each dark component goes to the detail primitive it overlaps most (keeps every stroke, incl. the lip line)
lab,n=ndi.label(det)
for k in ['GLASSES','EYEBROWS','NOSE','MOUTH']: vis[k]=np.zeros_like(det)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<2: continue
    dm=ndi.binary_dilation(m,iterations=3*UP)
    sc={k:(dm&prim[k]).sum() for k in ['GLASSES','EYEBROWS','NOSE','MOUTH']}
    k=max(sc,key=sc.get) if max(sc.values())>0 else min(['GLASSES','EYEBROWS','NOSE','MOUTH'],key=lambda k:ndi.distance_transform_edt(~prim[k])[m].mean())
    cyc=np.nonzero(m)[0].mean()/UP+Y0
    if k=='MOUTH' and cyc>P['m_cy']+6: vis['FACE']=vis.get('FACE',face_t&~det)|m; continue   # chin-corner specks, not mouth
    vis[k]|=m
# ---- body ----
c_=disk_op(cravat_t,'open',UP); lc,_=ndi.label(c_); szc=np.bincount(lc.ravel()); szc[0]=0; vis['CRAVAT']=lc==szc.argmax()
vis['HANDS']=hands_t
core=disk_op(dark&body_t&sole&(oy>110),'open',UP//2)          # true black ink only
black=ndi.binary_fill_holes(core|hands_t|cravat_t)
black=disk_op(black,'dil',UP//2)&body_t&sole&(oy>110)&~hands_t&~cravat_t
bluish=(H>90)&(H<125)&(S>40)                                                   # helmet anti-alias rim
black&=~(bluish&~ndi.binary_erosion(core,iterations=1))
black&=disk_op(fg&sole,'open',UP//2)
lab_b,_=ndi.label(black); sz=np.bincount(lab_b.ravel()); sz[0]=0; black=lab_b==sz.argmax()
FRONT2BACK=['BOOTS','ARMS','PANTS','JACKET']     # among black parts (photo seams: ring in front of jacket)
lbl=np.full(black.shape,-1)
for i,k in reversed(list(enumerate(FRONT2BACK))): lbl[black&prim[k]]=i   # front-most wins
# pixels outside every primitive -> nearest primitive (keeps the artwork's real outline)
miss=black&(lbl<0)
dist=np.stack([ndi.distance_transform_edt(~prim[k]) for k in FRONT2BACK])
lbl[miss]=dist[:,miss].argmin(0)
for i,k in enumerate(FRONT2BACK): vis[k]=black&(lbl==i)
# ---- leftover anti-aliased pixels (between two colours) -> nearest visible part ----
keys=list(vis); lab_all=np.zeros(black.shape,np.int16)
for i,k in enumerate(keys): lab_all[vis[k]&(lab_all==0)]=i+1
fg_clean=disk_op(fg&sole&(oy>110),'open',UP//2)      # drop compression speckle / 1-px lines
lf,_=ndi.label(fg_clean); szf=np.bincount(lf.ravel()); szf[0]=0; fg_clean=lf==szf.argmax()
left=fg_clean&(lab_all==0)&~disk_op(~fg,'dil',UP)   # never grow a part into the background
if left.any():
    HEADP=['HELMET','FACE','GLASSES','EYEBROWS','NOSE','MOUTH']
    headzone=ndi.binary_fill_holes(ndi.binary_closing(vis['HELMET']|vis['FACE'],iterations=2*UP))
    # body parts may not reach into the head: remove their pixels there, they become leftovers for head parts
    for k in keys:
        if k not in HEADP:
            bad=vis[k]&headzone&~ndi.binary_dilation(~headzone,iterations=UP); vis[k]&=~bad
            left|=bad
    lab_all=np.zeros(black.shape,np.int16)
    for i,k in enumerate(keys): lab_all[vis[k]&(lab_all==0)]=i+1
    lab_head=np.where(np.isin(lab_all,[keys.index(k)+1 for k in HEADP]),lab_all,0)
    for L,zone in ((lab_head,headzone),(lab_all,~headzone)):
        _,(iy,ix)=ndi.distance_transform_edt(L==0,return_indices=True); near=L[iy,ix]
        for i,k in enumerate(keys): vis[k]|=left&zone&(near==i+1)
# remove slivers created by the partition on large black parts
for k in ('JACKET','ARMS','PANTS'):
    vis[k]=disk_op(vis[k],'open',UP//2)
# ---- arms are exact geometry: two oval loops (confirmed smooth on the real print); artwork edge bumps are drawing jitter ----
arm_g=prim['ARMS']&sole
in_front_of_arms=np.zeros_like(black)
for j in M.ORDER[M.ORDER.index('ARMS')+1:]: in_front_of_arms|=vis[j]
vis['ARMS']=arm_g&~in_front_of_arms
for j in M.ORDER[:M.ORDER.index('ARMS')]: vis[j]&=~arm_g          # parts behind the arms are covered there
# ---- cravat (shirt): straight-line V, fitted to the visible white pixels ----
from shapely.geometry import Polygon as _Poly
from scipy import optimize as _opt
Y_REF=232.0                                                          # chin line: first visible row of the shirt
def cravat_poly(q,top):
    xc,wt,wv,yv,yt=q                                                  # wt: half-width at the chin line, wv: at the V start
    k=(wv-wt)/(yv-Y_REF); wtop=wt+k*(top-Y_REF)                       # straight sides, extended up under the chin
    return _Poly([(xc-wtop,top),(xc+wtop,top),(xc+wv,yv),(xc,yt),(xc-wv,yv)])
_tgt=vis['CRAVAT']; _win=(oy>232)&(oy<270)&(np.abs(ox-257)<14)          # visible below the chin, above the hands
def _loss(q):
    if q[1]<=0.5 or q[2]<=0.5 or q[3]<=Y_REF+2 or q[4]<=q[3]: return 2.0
    m=R(cravat_poly(q,P['t_top'])); a=m&_win; b=_tgt&_win; return 1-(a&b).sum()/max((a|b).sum(),1)
_q0=np.array([grp('CRAVAT'),5.5,6.8,252.5,265.7])
_st=np.vstack([np.zeros(5),np.eye(5)*[0.5,0.6,0.6,1.5,1.0]])
_r=min((_opt.minimize(_loss,_q0+d,method='Nelder-Mead',options=dict(maxiter=900,xatol=0.01,fatol=1e-6,initial_simplex=_q0+d+_st))
        for d in (np.zeros(5),np.array([0,0.8,0,0,0]),np.array([0,-0.8,0.4,-1,0]))),key=lambda r:r.fun)
CRAVAT_Q=[float(v) for v in _r.x]; print('cravat V fit IoU',round(1-_r.fun,3),[round(v,2) for v in CRAVAT_Q])
_cp=cravat_poly(CRAVAT_Q,P['t_top'])
json.dump(dict(poly=[list(map(float,c)) for c in _cp.exterior.coords],xc=CRAVAT_Q[0],w_top_at_chin=CRAVAT_Q[1],w_at_v=CRAVAT_Q[2],y_chin=Y_REF,y_v=CRAVAT_Q[3],y_tip=CRAVAT_Q[4],top=P['t_top']),open('cravat_poly.json','w'))
cravat_g=R(cravat_poly(CRAVAT_Q,P['t_top']))&sole
in_front_of_c=np.zeros_like(black)
for j in M.ORDER[M.ORDER.index('CRAVAT')+1:]:
    if j!='JACKET': in_front_of_c|=vis[j]
vis['CRAVAT']=cravat_g&~in_front_of_c
# ---- jacket: straight-sided panel (straight on artwork and print); V-neck = traced shirt shape, opened up to the collar ----
box_t=R(M.torso(P,grp('JACKET')))
notch=cravat_g.copy()                                             # V-neck = exact cravat polygon
jacket_g=box_t&~notch&sole
in_front_of_j=np.zeros_like(black)
for j in M.ORDER[M.ORDER.index('JACKET')+1:]: in_front_of_j|=vis[j]
vis['JACKET']=jacket_g&~in_front_of_j
# ---- complete hidden portions from primitives ----
ORDER=M.ORDER    # back -> front
full={}
for i,k in enumerate(ORDER):
    front=np.zeros_like(black)
    for j in ORDER[i+1:]: front|=vis[j]
    hidden=(prim[k]|disk_op(vis[k],'dil',3*UP))&disk_op(front,'dil',UP//2)   # tuck under front parts: no seams
    full0=vis[k]|hidden
    if k not in ('EYEBROWS','MOUTH','NOSE','GLASSES'):
        near=disk_op(front,'dil',2*UP) if front.any() else np.zeros_like(front)
        full1=full0|(disk_op(full0,'close',1.5*UP)&near)                     # fill seams, only next to front parts
        full[k]=(disk_op(full1,'open',0.75*UP)&near)|(full1&~near)            # remove spurs, only next to front parts
        full[k]|=vis[k]&~near
    else:
        full[k]=full0
    if k=='FACE': full[k]=ndi.binary_fill_holes(full[k])        # face continues under glasses/mouth
full['JACKET']=jacket_g; full['ARMS']=arm_g; full['CRAVAT']=cravat_g        # exact geometric parts: no seam growth on their edges
# ---- fidelity check: rebuild the coloured artwork from visible parts ----
cls=lambda ks: np.any([vis[k] for k in ks],axis=0)
chk={'blue':(cls(['HELMET']),blue&sole),
     'white':(cls(['FACE','CRAVAT','HANDS']),white&sole&~blue),
     'black_face_graphics':(cls(['GLASSES','EYEBROWS','NOSE','MOUTH']),det),
     'silhouette':(cls(list(vis)),fg&sole)}
rep={k:round(float((a&b).sum()/max((a|b).sum(),1)),3) for k,(a,b) in chk.items()}
print(json.dumps(rep,indent=1))
# ---- vectorise (sub-pixel, light smoothing) to artwork px ----
def vec(mask,sig=2.0,eps=0.5,minA=4*UP*UP):
    f=cv2.GaussianBlur(mask.astype(np.float32),(0,0),sig)
    m=(f>=0.5).astype(np.uint8)
    cs,h=cv2.findContours(m,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
    if h is None: return Polygon()
    polys=[]
    for i,c in enumerate(cs):
        if h[0][i][3]!=-1 or cv2.contourArea(c)<minA: continue
        ext=cv2.approxPolyDP(c,eps,True).reshape(-1,2)/UP+[X0,Y0]
        holes=[cv2.approxPolyDP(cs[j],eps,True).reshape(-1,2)/UP+[X0,Y0] for j in range(len(cs)) if h[0][j][3]==i and cv2.contourArea(cs[j])>=minA/4]
        if len(ext)>=3: polys.append(Polygon(ext,[x for x in holes if len(x)>=3]).buffer(0))
    return unary_union(polys)
small={'EYEBROWS','MOUTH','NOSE','GLASSES'}
G={k:vec(full[k],sig=1.2 if k in small else 3.0,minA=(UP*UP if k in small else 4*UP*UP)) for k in ORDER}
GV={k:vec(vis[k],sig=1.2 if k in small else 3.0,minA=(UP*UP if k in small else 4*UP*UP)) for k in ORDER}
pickle.dump(dict(full=G,visible=GV,check=rep,axis=X['single']),open('traced.pkl','wb'))
np.savez_compressed('full_masks.npz',**{k:full[k] for k in ORDER},UP=UP,X0=X0,Y0=Y0)
# overlay for review
S=4; im2=cv2.resize(im[Y0:Y1,X0:X1],None,fx=S,fy=S,interpolation=cv2.INTER_CUBIC); im2=(im2*0.5+127).astype(np.uint8)
pal=[(255,0,0),(0,170,0),(0,0,255),(0,120,255),(200,0,200),(0,200,200),(120,60,0),(0,0,0),(255,0,255),(0,90,180),(60,60,200),(200,120,0)]
for (k,g),cc in zip(G.items(),pal):
    for p in getattr(g,'geoms',[g]):
        if p.is_empty: continue
        for r in [p.exterior,*p.interiors]: cv2.polylines(im2,[((np.array(r.coords)-[X0,Y0])*S).round().astype(np.int32)],True,cc,2,cv2.LINE_AA)
cv2.imwrite('trace_overlay.png',im2)
