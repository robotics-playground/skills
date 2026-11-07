import json, numpy as np, ezdxf, os
from shapely import affinity
from shapely.geometry import box
import model as M, pickle
TR={'full':pickle.load(open('smooth.pkl','rb'))}   # raster defects removed (smooth.py)
d=json.load(open('fit.json')); P,X=d['P'],d['X']; XS=X['single']
PLATE_H=99.9; BASE_W,BASE_H=70.0,20.3; SOLE=436.5
top=TR['full']['HELMET'].bounds[1]; s=PLATE_H/(SOLE-top)          # mm per artwork px
def to_mm(g):   # artwork px (y down) -> mm (origin base bottom-left, y up)
    g=affinity.translate(g,-XS,-SOLE); g=affinity.scale(g,s,-s,origin=(0,0)); return affinity.translate(g,BASE_W/2,BASE_H)
G={k:to_mm(TR['full'][k]) for k in M.OUT_ORDER}            # detail-preserving traced parts (hidden edges completed)
# what you see: each part minus every part stacked in front of it -> exact, gap-free colour regions
from shapely.ops import unary_union as _uu
GV={}
for i,k in enumerate(M.OUT_ORDER):
    GV[k]=G[k].difference(_uu([G[j] for j in M.OUT_ORDER[i+1:]])).buffer(0) if i<len(M.OUT_ORDER)-1 else G[k]
G['BASE']=box(0,0,BASE_W,BASE_H)
FILL={'blue':'#1f45d6','white':'#f2f2ee','black':'#222'}; COL=dict(M.OUT_COLOR,BASE='black')
def path(g):
    out=''
    for p in getattr(g,'geoms',[g]):
        for r in [p.exterior,*p.interiors]:
            out+='M'+' L'.join(f'{x:.3f},{-y:.3f}' for x,y in r.coords)+'Z'
    return out
# ---------- key dimensions (mm) ----------
L=lambda k:round(P[k]*s,2)
spec={'_note':'bbox_* = traced detailed part; other values = fitted reference primitive (structure/pose), not used for the outline',
 'scale_mm_per_artwork_px':round(s,4),
 'BASE':{'width':BASE_W,'height':BASE_H},
 'HELMET':{'dome_rx':L('h_rx'),'dome_ry':L('h_ry'),'flap_bottom_above_base':round(BASE_H+(SOLE-P['h_cut'])*s,2),'flap_fillet_r':L('h_fr')},
 'FACE':{'half_w':L('f_w'),'half_h':L('f_h'),'squircle_n':round(P['f_n'],2),'center_above_base':round(BASE_H+(SOLE-P['f_cy'])*s,2)},
 'GLASSES':{'lens_spacing':round(2*P['g_dx']*s,2),'lens_half_w':L('g_w'),'lens_half_h':L('g_h'),'lens_n':round(P['g_n'],2),'tilt_deg':round(P['g_tilt'],1),'bridge_t':L('g_bt')},
 'EYEBROWS':{'spacing':round(2*P['b_dx']*s,2),'arc_r':L('b_r'),'span_deg':round(P['b_span'],1),'stroke_t':L('b_t')},
 'NOSE':{'rx':L('n_rx'),'ry':L('n_ry')},
 'MOUTH':{'half_len':L('m_hl'),'sag':L('m_sag'),'stroke_t':L('m_t'),'offset_x':L('m_dx')},
 'CRAVAT':{'note':'shirt/tie behind JACKET, visible through V-neck','width':round(2*P['c_w']*s,2),'length':round((P['c_tip']-P['c_top'])*s,2),'v_depth':round((P['c_tip']-P['c_v'])*s,2)},
 'HANDS':{'note':'two fists on the front of the ARMS','width':round(2*P['k_w']*s,2),'height':round((P['k_bot']-P['k_top'])*s,2),'corner_r':L('k_r'),'thumb_spacing':round(2*P['k_tdx']*s,2),'thumb_w':round(2*P['k_tw']*s,2),'thumb_h':round((P['k_top']-P['k_ttop'])*s,2)},
 'ARMS':{'note':'one oval ring: outer oval minus inner oval, centred on the axis','outer_rx':L('a_rx'),'outer_ry':L('a_ry'),'inner_rx':L('a_irx'),'inner_ry':L('a_iry'),'inner_center_offset_down':round((P['a_icy']-P['a_cy'])*s,2),'center_above_base':round(BASE_H+(SOLE-P['a_cy'])*s,2),'band_width_at_sides':round((P['a_rx']-P['a_irx'])*s,2)},
 'SUIT':{'note':'jacket + pants as one piece; V-neck = CRAVAT V; ARMS lie in front','jacket':{'width':round(2*P['t_w']*s,2),'height':round((P['t_bot']-P['t_top'])*s,2),'v_neck_width':round(2*P['c_w']*s,2),'v_neck_depth':round((P['c_tip']-P['t_top'])*s,2),},'pants':{'hip_width':round(2*P['p_hw']*s,2),'ankle_width':round(2*P['p_aw']*s,2),'hip_fillet_r':L('p_r'),'crotch_above_base':round(BASE_H+(SOLE-P['p_crotch'])*s,2),'gap_top_w':round(2*P['p_gt']*s,2),'gap_bottom_w':round(2*P['p_gb']*s,2)}},
 'BOOTS':{'height':round((SOLE-P['o_top'])*s,2),'ankle_w':L('o_aw'),'toe_reach_from_axis':L('o_tx'),'inner_from_axis':L('o_ix'),'toe_fillet_r':L('o_tr')},
}
for k,g in G.items():
    x0,y0,x1,y1=g.bounds; spec.setdefault(k,{}).update(bbox_w=round(x1-x0,2),bbox_h=round(y1-y0,2),bbox_bottom=round(y0,2))
json.dump(dict(spec=spec,fit_iou=d['iou'],axes_px=X),open('parts_mm.json','w'),indent=1)
def spline_segments(ring,step=0.35,corner_deg=35):
    ring=ring[:-1] if np.allclose(ring[0],ring[-1]) else ring
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([ring,ring[:1]]),axis=0),axis=1))]
    t=np.arange(0,d[-1],0.05); P2=np.c_[np.interp(t,d,np.r_[ring[:,0],ring[0,0]]),np.interp(t,d,np.r_[ring[:,1],ring[0,1]])]
    k=6; a=np.roll(P2,k,0); b=np.roll(P2,-k,0); v1=P2-a; v2=b-P2
    ang=np.abs(np.degrees(np.arctan2(v1[:,0]*v2[:,1]-v1[:,1]*v2[:,0],(v1*v2).sum(1))))
    cs=[]
    for i in np.argsort(-ang):
        if ang[i]<corner_deg: break
        if all(min(abs(i-j),len(P2)-abs(i-j))>3*k for j in cs): cs.append(i)
    cs=sorted(cs); n=len(P2); stride=max(1,int(step/0.05))
    if not cs:
        pts=P2[::stride]; return [np.vstack([pts,pts[:1]])]
    segs=[]
    for a_,b_ in zip(cs,cs[1:]+[cs[0]+n]):
        idx=np.r_[np.arange(a_,b_,stride),b_]%n; seg=P2[idx]
        segs.append(seg if len(seg)>2 and np.abs((seg[-1]-seg[0])[0]*(seg[1:-1]-seg[0])[:,1]-(seg[-1]-seg[0])[1]*(seg[1:-1]-seg[0])[:,0]).max()/max(np.linalg.norm(seg[-1]-seg[0]),1e-9)>0.02 else seg[[0,-1]])
    return segs
# ---------- DXF: one layer per part (full part shapes, mm) ----------
doc=ezdxf.new(units=4); msp=doc.modelspace(); aci={'blue':5,'white':8,'black':7}
os.makedirs('parts',exist_ok=True)
for k,g in G.items():
    doc.layers.add(k,color=aci[COL[k]])
    one=ezdxf.new(units=4); om=one.modelspace()
    for p in getattr(g,'geoms',[g]):
        for r in [p.exterior,*p.interiors]:
            for seg in spline_segments(np.array(r.coords)):
                for mm_ in (msp,om):
                    if len(seg)==2: mm_.add_line(tuple(seg[0]),tuple(seg[1]),dxfattribs={'layer':k})
                    else: mm_.add_spline(fit_points=[(x,y,0) for x,y in seg],dxfattribs={'layer':k})
    one.saveas(f'parts/{k}.dxf')
doc.layers.add('AXIS',color=1); msp.add_line((BASE_W/2,-3),(BASE_W/2,BASE_H+PLATE_H+3),dxfattribs={'layer':'AXIS'})
doc.saveas('toy_front_view_parts.dxf')
pr=ezdxf.new(units=4); pm=pr.modelspace()
for k,g in GV.items():
    pr.layers.add(k,color=aci[COL[k]])
    for p in getattr(g,'geoms',[g]):
        for r in [p.exterior,*p.interiors]: pm.add_lwpolyline([(round(x,4),round(y,4)) for x,y in r.coords],close=True,dxfattribs={'layer':k})
pr.layers.add('BASE',color=7); pm.add_lwpolyline([(0,0),(BASE_W,0),(BASE_W,BASE_H),(0,BASE_H)],close=True,dxfattribs={'layer':'BASE'})
pr.saveas('toy_print_regions_visible.dxf')
# ---------- SVG 1: assembled front view ----------
order=['BASE']+M.OUT_ORDER
yt=BASE_H+PLATE_H; pad=22
def dimh(x1,x2,y,t): return f'<line x1="{x1:.2f}" y1="{-y:.2f}" x2="{x2:.2f}" y2="{-y:.2f}" class="d"/><text x="{(x1+x2)/2:.2f}" y="{-y-1:.2f}" class="t" text-anchor="middle">{t}</text>'
def dimv(x,y1,y2,t): return f'<line x1="{x:.2f}" y1="{-y1:.2f}" x2="{x:.2f}" y2="{-y2:.2f}" class="d"/><text class="t" text-anchor="middle" transform="translate({x-1:.2f} {-(y1+y2)/2:.2f}) rotate(-90)">{t}</text>'
STY='<defs><marker id="a" viewBox="0 0 4 4" refX="2" refY="2" markerWidth="3" markerHeight="3" orient="auto-start-reverse"><path d="M0,0 L4,2 L0,4z" fill="#0a5bd6"/></marker><style>.d{stroke:#0a5bd6;stroke-width:.2;marker-start:url(#a);marker-end:url(#a)}.t{font:3px Helvetica,Arial,sans-serif;fill:#0a5bd6}.l{font:2.4px Helvetica,Arial,sans-serif;fill:#333}</style></defs>'
pl=G['ARMS'].bounds
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{-pad} {-yt-pad} {BASE_W+2*pad} {yt+2*pad}" width="{(BASE_W+2*pad)*6:.0f}" height="{(yt+2*pad)*6:.0f}">',STY,
     f'<rect x="{-pad}" y="{-yt-pad}" width="{BASE_W+2*pad}" height="{yt+2*pad}" fill="#fff"/>']
for k in order: svg.append(f'<path d="{path(G[k])}" fill="{FILL[COL[k]]}" stroke="#000" stroke-width=".12" fill-rule="evenodd"/>')
hb=G['HELMET'].bounds
svg+=[f'<line x1="{BASE_W/2}" y1="3" x2="{BASE_W/2}" y2="{-yt-4:.2f}" stroke="#d22" stroke-width=".15" stroke-dasharray="3 1 .5 1"/>',
 dimh(0,BASE_W,-7,f'{BASE_W:.0f}'),dimh(pl[0],pl[2],yt+8,f'{pl[2]-pl[0]:.1f}'),dimh(hb[0],hb[2],yt+3,f'{hb[2]-hb[0]:.1f}'),
 dimv(-6,0,BASE_H,f'{BASE_H}'),dimv(-13,0,yt,f'{yt:.1f}'),dimv(BASE_W+6,BASE_H,yt,f'{PLATE_H}'),
 f'<text x="0" y="{pad-3}" class="l">FRONT VIEW · mm · detail traced from Vinc artwork · scale from measured print</text></svg>']
open('toy_front_view_v2.svg','w').write('\n'.join(svg))
# ---------- SVG 2: exploded parts sheet ----------
cells=['HELMET','FACE','GLASSES','EYEBROWS','NOSE','MOUTH','CRAVAT','SUIT','HANDS','ARMS','BOOTS','BASE']
CW,CH,NC=78,62,4; rows=(len(cells)+NC-1)//NC
out=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {CW*NC} {CH*rows}" width="{CW*NC*5}" height="{CH*rows*5}">',STY,f'<rect width="{CW*NC}" height="{CH*rows}" fill="#fff"/>']
for i,k in enumerate(cells):
    g=G[k]; x0,y0,x1,y1=g.bounds; w,h=x1-x0,y1-y0
    sc=min((CW-22)/w,(CH-24)/h,6.0)
    cx,cy=(i%NC)*CW,(i//NC)*CH
    tr=f'translate({cx+CW/2-(x0+w/2)*sc:.2f} {cy+CH/2+3+(y0+h/2)*sc:.2f}) scale({sc:.4f})'
    out.append(f'<rect x="{cx+1}" y="{cy+1}" width="{CW-2}" height="{CH-2}" fill="none" stroke="#ccc" stroke-width=".3"/>')
    out.append(f'<text x="{cx+3}" y="{cy+5}" style="font:bold 3.2px Helvetica,Arial,sans-serif">{i+1}. {k}</text>')
    out.append(f'<text x="{cx+3}" y="{cy+CH-3}" class="l">{w:.1f} × {h:.1f} mm · {COL[k]}{"  ×2 mirrored" if k in ("BOOTS","EYEBROWS") else ""}{" · oval ring, in front of jacket" if k=="ARMS" else ""}{" · V-neck" if k=="JACKET" else ""}{" · on the arms" if k=="HANDS" else ""}</text>')
    out.append(f'<path transform="{tr}" d="{path(g)}" fill="{FILL[COL[k]]}" stroke="#000" stroke-width="{0.25/sc:.3f}" fill-rule="evenodd"/>')
out.append('</svg>'); open('parts_sheet.svg','w').write('\n'.join(out))
print(json.dumps(spec['ARMS']),json.dumps(spec['HELMET']))
