"""Parametric part model (artwork pixel coords, y DOWN). Each part -> shapely geometry."""
import numpy as np
from shapely.geometry import Polygon, box, Point, LineString
from shapely import affinity
from shapely.ops import unary_union
def ellipse(cx,cy,rx,ry,n=128):
    t=np.linspace(0,2*np.pi,n,endpoint=False); return Polygon(np.c_[cx+rx*np.cos(t),cy+ry*np.sin(t)])
def superellipse(cx,cy,a,b,n=2.5,k=160):
    t=np.linspace(0,2*np.pi,k,endpoint=False); c,s=np.cos(t),np.sin(t)
    return Polygon(np.c_[cx+a*np.sign(c)*np.abs(c)**(2/n),cy+b*np.sign(s)*np.abs(s)**(2/n)])
def rrect(x0,y0,x1,y1,r):
    r=max(0.01,min(r,(x1-x0)/2-0.01,(y1-y0)/2-0.01)); return box(x0+r,y0+r,x1-r,y1-r).buffer(r,quad_segs=12)
def arc_stroke(cx,cy,r,a0,a1,t):
    a=np.linspace(a0,a1,40); return LineString(np.c_[cx+r*np.cos(a),cy+r*np.sin(a)]).buffer(t/2,cap_style='round')
def mirror(g,X): return affinity.scale(g,-1,1,origin=(X,0))
def sym(g,X): return unary_union([g,mirror(g,X)])
def soft_open(g,r):  # fillet convex corners
    return g.buffer(-r,join_style='round').buffer(r,join_style='round') if r>0.01 else g

def face(P,X):  return superellipse(X,P['f_cy'],P['f_w'],P['f_h'],P['f_n'])
def helmet(P,X):
    outer=ellipse(X,P['h_cy'],P['h_rx'],P['h_ry']).intersection(box(X-500,0,X+500,P['h_cut']))
    outer=soft_open(outer,P['h_fr'])
    return outer.difference(face(P,X))
def glasses(P,X):
    L=superellipse(X-P['g_dx'],P['g_cy'],P['g_w'],P['g_h'],P['g_n']); L=affinity.rotate(L,P['g_tilt'],origin=(X-P['g_dx'],P['g_cy']))
    bridge=box(X-P['g_dx'],P['g_cy']-P['g_bt']/2,X+P['g_dx'],P['g_cy']+P['g_bt']/2)
    return unary_union([sym(L,X),bridge])
def brows(P,X):
    a=np.deg2rad(P['b_span'])/2
    b=arc_stroke(X-P['b_dx'],P['b_cy']+P['b_r'],P['b_r'],-np.pi/2-a,-np.pi/2+a,P['b_t'])
    return sym(b,X)
def nose(P,X): return ellipse(X+P['n_dx'],P['n_cy'],P['n_rx'],P['n_ry'])
def mouth(P,X):
    hl,s=P['m_hl'],max(P['m_sag'],0.05); r=(hl**2+s**2)/(2*s); a=np.arcsin(min(hl/r,1))
    return arc_stroke(X+P['m_dx'],P['m_cy']-r+s,r,np.pi/2-a,np.pi/2+a,P['m_t'])
def cravat(P,X):
    # visible shape fitted from c_top down; it continues up (hidden under the chin) to the jacket's top edge
    w=P['c_w']; t=min(P['c_top'],P['t_top'])
    return Polygon([(X-w,t),(X+w,t),(X+w,P['c_v']),(X,P['c_tip']),(X-w,P['c_v'])])
def hands(P,X):
    palm=rrect(X-P['k_w'],P['k_top'],X+P['k_w'],P['k_bot'],P['k_r'])
    th=rrect(X-P['k_tdx']-P['k_tw'],P['k_ttop'],X-P['k_tdx']+P['k_tw'],P['k_top']+3,P['k_tw']*0.9)
    return unary_union([palm,sym(th,X)])
def arms(P,X):
    """Arms = ONE oval ring: outer oval minus inner oval, centred on the axis. Upper part passes behind the
    head; the lower band crosses in front of the jacket to the hands (the inner oval's lower arc is the seam)."""
    return ellipse(X,P['a_cy'],P['a_rx'],P['a_ry']).difference(ellipse(X,P['a_icy'],P['a_irx'],P['a_iry']))
def torso(P,X): return box(X-P['t_w'],P['t_top'],X+P['t_w'],P['t_bot'])
def jacket(P,X):   # torso panel with the V-neck opening through which the shirt/cravat shows
    return torso(P,X).difference(cravat(P,X))
def pants(P,X):
    hip=Polygon([(X-P['p_hw'],P['p_top']),(X+P['p_hw'],P['p_top']),(X+P['p_aw'],P['p_ank']),(X-P['p_aw'],P['p_ank'])])
    # fillet only the two top outer corners: round the polygon then restore the bottom
    hipr=soft_open(hip,P['p_r']).union(box(X-P['p_aw']-50,P['p_top']+P['p_r'],X+P['p_aw']+50,P['p_ank'])).intersection(hip)
    gap=Polygon([(X-P['p_gt'],P['p_crotch']),(X+P['p_gt'],P['p_crotch']),(X+P['p_gb'],P['p_ank']+5),(X-P['p_gb'],P['p_ank']+5)])
    gap=soft_open(gap.union(box(X-P['p_gb'],P['p_ank'],X+P['p_gb'],P['p_ank']+40)),P['p_gr'])
    return hipr.difference(gap)
def boots(P,X):
    ank=box(X-P['o_ix']-P['o_aw'],P['o_top'],X-P['o_ix'],P['o_sole'])
    toe=rrect(X-P['o_tx'],P['o_ttop'],X-P['o_ix'],P['o_sole']+P['o_tr'],P['o_tr']).intersection(box(0,0,9999,P['o_sole']))
    return sym(unary_union([ank,toe]),X)
PARTS={'HELMET':helmet,'FACE':face,'GLASSES':glasses,'EYEBROWS':brows,'NOSE':nose,'MOUTH':mouth,
       'CRAVAT':cravat,'HANDS':hands,'ARMS':arms,'JACKET':jacket,'PANTS':pants,'BOOTS':boots}
COLOR={'HELMET':'blue','FACE':'white','GLASSES':'black','EYEBROWS':'black','NOSE':'black','MOUTH':'black',
       'CRAVAT':'white','HANDS':'white','ARMS':'black','JACKET':'black','PANTS':'black','BOOTS':'black'}
# draw order (back -> front) used for the visible colour map
# back -> front, from the seams visible on the print: shirt behind jacket; arm ring lies IN FRONT of the
# jacket (its lower band crosses the chest) with the hands on it; upper ring passes BEHIND the head;
# the helmet sits in front of the head (face shows through its opening).
ORDER=['CRAVAT','JACKET','PANTS','BOOTS','ARMS','HANDS','FACE','HELMET','GLASSES','EYEBROWS','NOSE','MOUTH']

# ---- output parts (what is delivered): jacket + pants are ONE piece, the SUIT ----
OUT_ORDER=['CRAVAT','SUIT','BOOTS','ARMS','HANDS','FACE','HELMET','GLASSES','EYEBROWS','NOSE','MOUTH']
OUT_COLOR=dict(COLOR,SUIT='black')
