import cv2, numpy as np, json
from scipy import ndimage as ndi, optimize
from shapely.ops import unary_union
import model as M
UP=6; X0,Y0,X1,Y1=120,105,380,445          # artwork crop (orig px)
im=cv2.imread('/root/.claude/uploads/5391fe7b-2194-5842-9b9d-7c568b21fbc3/0fe88611-image.webp')
c=cv2.resize(im[Y0:Y1,X0:X1],None,fx=UP,fy=UP,interpolation=cv2.INTER_CUBIC).astype(np.float32)
hsv=cv2.cvtColor(np.clip(c,0,255).astype(np.uint8),cv2.COLOR_BGR2HSV); H,S,V=[a.astype(int) for a in cv2.split(hsv)]
# --- raster-defect-free colour separation: light blur (kills compression ringing), then each pixel
# takes its nearest flat paint colour in Lab -> edges land at the 50 % anti-alias mix = true edge
PAL={'bg':(251,254,219),'blue':(212,159,40),'white':(254,250,253),'grey':(221,221,221),'black':(45,45,45)}
cb=cv2.GaussianBlur(np.clip(c,0,255).astype(np.uint8),(0,0),0.3*UP)
# anti-alias unmixing: a pixel is either a flat paint colour or a linear blend of two neighbouring paints.
# Find the best (paint i, paint j, alpha) explaining it; label = the paint with alpha >= 0.5.
img=cb.astype(np.float32).reshape(-1,3); pal=np.array(list(PAL.values()),np.float32); nP=len(pal)
best_d=np.full(len(img),np.inf,np.float32); cls_=np.zeros(len(img),np.int32)
for i in range(nP):
    d=((img-pal[i])**2).sum(1); m=d<best_d; best_d[m]=d[m]; cls_[m]=i
for i in range(nP):
    for j in range(i+1,nP):
        v=pal[j]-pal[i]; a=np.clip(((img-pal[i])@v)/(v@v),0,1)
        d=((img-(pal[i]+a[:,None]*v))**2).sum(1)
        m=d<best_d*0.999; best_d[m]=d[m]; cls_[m]=np.where(a[m]<0.5,i,j)
cls_=cls_.reshape(cb.shape[:2])
names=list(PAL)
fg=cls_!=names.index('bg')
blue=cls_==names.index('blue')
white=(cls_==names.index('white'))|(cls_==names.index('grey'))
dark=cls_==names.index('black')
gy,gx=np.mgrid[0:c.shape[0],0:c.shape[1]]; ox=gx/UP+X0; oy=gy/UP+Y0     # orig coords per pixel
def R(g):
    m=np.zeros(c.shape[:2],np.uint8)
    for p in getattr(g,'geoms',[g]):
        if p.is_empty: continue
        cv2.fillPoly(m,[((np.array(p.exterior.coords)-[X0,Y0])*UP).round().astype(np.int32)],1)
        for h in p.interiors: cv2.fillPoly(m,[((np.array(h.coords)-[X0,Y0])*UP).round().astype(np.int32)],0)
    return m.astype(bool)
def iou(a,b,w=None):
    if w is not None: a,b=a&w,b&w
    return (a&b).sum()/max((a|b).sum(),1)
# ---------- semantic targets from the artwork ----------
head_t=ndi.binary_fill_holes(ndi.binary_closing(blue,iterations=2*UP))
face_t=white&(oy<236)&(oy>150)
face_t=ndi.binary_opening(face_t,iterations=UP); lab,_=ndi.label(face_t); face_t=lab==lab[int((200-Y0)*UP),int((256-X0)*UP)]
face_t=ndi.binary_fill_holes(ndi.binary_closing(face_t,iterations=6*UP))&~blue
face_t=ndi.binary_opening(face_t,iterations=UP)
helmet_t=blue
from skimage.morphology import convex_hull_image
facehull=convex_hull_image(face_t)&~blue
det=dark&facehull
cravat_t=white&(oy>226)&(oy<268)&(np.abs(ox-257)<9)&~face_t
hands_t=cv2.morphologyEx((white&(oy>250)&(oy<300)&~cravat_t).astype(np.uint8),cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))).astype(bool)
body_t=fg&(oy<436.5)&~blue&~face_t     # everything that is not head
body_t=ndi.binary_opening(body_t,iterations=2)
P={ # initial guesses read from the gridded artwork (orig px)
 'h_cy':178,'h_rx':64.5,'h_ry':58,'h_cut':222,'h_fr':8,
 'f_cy':201,'f_w':39,'f_h':32,'f_n':3.0,
 'g_dx':18,'g_cy':194.5,'g_w':15,'g_h':6.5,'g_n':2.6,'g_tilt':-4,'g_bt':2.5,
 'b_dx':23,'b_cy':182,'b_r':14,'b_span':60,'b_t':2.2,
 'n_dx':1,'n_cy':205.5,'n_rx':1.8,'n_ry':1.5,
 'm_dx':-1,'m_cy':213,'m_hl':8,'m_sag':1.2,'m_t':2,
 'c_w':7.5,'c_top':228,'c_v':248,'c_tip':265,
 'k_w':24.5,'k_top':270,'k_bot':292,'k_r':3,'k_tdx':12.5,'k_tw':3,'k_ttop':256.5,
 'a_cy':235,'a_rx':95,'a_ry':60,'a_icy':241,'a_irx':59,'a_iry':40,
 't_w':46.5,'t_top':215,'t_bot':310,
 'p_top':300,'p_hw':71,'p_aw':70,'p_ank':408,'p_r':22,'p_crotch':338,'p_gt':33.5,'p_gb':37,'p_gr':4,
 'o_ix':43,'o_aw':23,'o_top':406,'o_sole':436.5,'o_tx':76,'o_ttop':418,'o_tr':7,
}
X={'head':256.5,'body':256.5,'legs':253.5}
def fitgroup(keys,loss,xkey=None,iters=600):
    names=list(keys)+([xkey] if xkey else [])
    v0=np.array([P[k] for k in keys]+([X[xkey]] if xkey else []),float)
    step=np.maximum(np.abs(v0)*0.04,1.0); simplex=np.vstack([v0]+[v0+np.eye(len(v0))[i]*step[i] for i in range(len(v0))])
    def f(v):
        for k,val in zip(keys,v): P[k]=val
        if xkey: X[xkey]=v[-1]
        try: return loss()
        except Exception: return 2.0
    r=optimize.minimize(f,v0,method='Nelder-Mead',options=dict(maxiter=iters,initial_simplex=simplex,xatol=0.02,fatol=1e-6))
    f(r.x); return 1-r.fun
log={}
xh=lambda: X['head']; xb=lambda: X['body']
log['head_outline']=fitgroup(['h_cy','h_rx','h_ry','h_cut','h_fr'],lambda:1-iou(R(M.helmet(P,xh()).union(M.face(P,xh()))),head_t|face_t,oy<240),'head')
log['FACE']=fitgroup(['f_cy','f_w','f_h','f_n'],lambda:1-iou(R(M.face(P,xh())),face_t))
log['HELMET']=fitgroup(['h_cy','h_rx','h_ry','h_cut','h_fr'],lambda:1-iou(R(M.helmet(P,xh())),helmet_t,oy<240))
win=lambda a,b:(oy>a)&(oy<b)
log['GLASSES']=fitgroup(['g_dx','g_cy','g_w','g_h','g_n','g_tilt','g_bt'],lambda:1-iou(R(M.glasses(P,xh())),det,win(184,203)))
log['EYEBROWS']=fitgroup(['b_dx','b_cy','b_r','b_span','b_t'],lambda:1-iou(R(M.brows(P,xh())),det,win(172,189)))
log['NOSE']=fitgroup(['n_dx','n_cy','n_rx','n_ry'],lambda:1-iou(R(M.nose(P,xh())),det,win(201,209)&(np.abs(ox-258)<6)))
log['MOUTH']=fitgroup(['m_dx','m_cy','m_hl','m_sag','m_t'],lambda:1-iou(R(M.mouth(P,xh())),det,win(209,220)))
log['CRAVAT']=fitgroup(['c_w','c_top','c_v','c_tip'],lambda:1-iou(R(M.cravat(P,xb())),cravat_t,win(222,270)))
log['HANDS']=fitgroup(['k_w','k_top','k_bot','k_r','k_tdx','k_tw','k_ttop'],lambda:1-iou(R(M.hands(P,xb())),hands_t,win(250,300)))
headg=lambda:R(M.helmet(P,xh()).union(M.face(P,xh())))
upper=lambda:R(unary_union([M.arms(P,xb()),M.torso(P,xb()),M.cravat(P,xb()),M.hands(P,xb())]))&~headg()
for rnd in range(3):
    log['JACKET+ARMS']=fitgroup(['a_cy','a_rx','a_ry','a_icy','a_irx','a_iry','t_w','t_top'],lambda:1-iou(upper(),body_t,win(150,300)),'body')
    xl=lambda: X['legs']
    log['PANTS']=fitgroup(['t_bot','p_top','p_hw','p_aw','p_ank','p_r','p_crotch','p_gt','p_gb','p_gr'],
        lambda:1-iou(R(unary_union([M.torso(P,xb()),M.pants(P,xl()),M.boots(P,xl())])),body_t,win(290,406)),'legs')
    log['BOOTS']=fitgroup(['o_ix','o_aw','o_top','o_tx','o_ttop','o_tr'],
        lambda:1-iou(R(unary_union([M.pants(P,xl()),M.boots(P,xl())])),body_t,win(400,436.5)))
# whole-figure check
GRP=lambda k:'head' if k in('HELMET','FACE','GLASSES','EYEBROWS','NOSE','MOUTH') else ('legs' if k in('PANTS','BOOTS') else 'body')
full=lambda ax:R(unary_union([M.PARTS[k](P,ax(k)) for k in M.PARTS]))
log['WHOLE_SILHOUETTE_artwork_axes']=iou(full(lambda k:X[GRP(k)]),fg&(oy<436.5))
# design rule: one symmetry axis for the whole figure (area-weighted mean of group axes)
XS=float(np.average([X['head'],X['body'],X['legs']],weights=[1,1.2,1.5]))
log['WHOLE_SILHOUETTE_single_axis']=iou(full(lambda k:XS),fg&(oy<436.5))
X['single']=XS
print(json.dumps({k:round(v,3) for k,v in log.items()},indent=1)); print('axes',X)
json.dump(dict(P=P,X=X,iou=log),open('fit.json','w'),indent=1)
np.savez_compressed('targets.npz',fg=fg&(oy<436.5),blue=blue,white=white,dark=dark)
