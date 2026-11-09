import pickle, json, base64, cv2, numpy as np
from shapely.ops import unary_union
from shapely import affinity
import model as M
G=pickle.load(open('smooth.pkl','rb'))
spec=json.load(open('parts_mm.json'))['spec']; dev=json.load(open('smooth_report.json')); s=spec['scale_mm_per_artwork_px']
X0,Y0,X1,Y1=120,105,380,445
ORDER=M.OUT_ORDER
VIS={}
for i,k in enumerate(ORDER):
    VIS[k]=G[k].difference(unary_union([G[j] for j in ORDER[i+1:]])).buffer(0) if i<len(ORDER)-1 else G[k]
def d(g,tx=lambda x,y:(x,y),tol=0.08):
    g=g.simplify(tol,preserve_topology=True); out=''
    for p in getattr(g,'geoms',[g]):
        if p.is_empty: continue
        for r in [p.exterior,*p.interiors]:
            out+='M'+' '.join('%.2f %.2f'%tx(x,y) for x,y in r.coords)+'Z'
    return out
COL={k:M.OUT_COLOR[k] for k in ORDER}
# --- artwork crop as background ---
im=cv2.imread('/root/.claude/uploads/5391fe7b-2194-5842-9b9d-7c568b21fbc3/0fe88611-image.webp')[Y0:Y1,X0:X1]
im=cv2.resize(im,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
ok,buf=cv2.imencode('.png',im); art='data:image/png;base64,'+base64.b64encode(buf).decode()
W,H=X1-X0,Y1-Y0
cmp_=[f'<svg class="cmp" viewBox="{X0} {Y0} {W} {H}" role="img" aria-label="Reconstructed parts drawn over the artwork">',
      f'<image href="{art}" x="{X0}" y="{Y0}" width="{W}" height="{H}" class="art" preserveAspectRatio="none" style="image-rendering:pixelated"/>','<g class="recon">']
for k in ORDER:
    cmp_.append(f'<path data-part="{k}" class="p c-{COL[k]}" d="{d(VIS[k],tol=0.05)}" fill-rule="evenodd"/>')
cmp_.append('</g></svg>')
# --- dimensioned front view (mm) ---
BW,BH=70.0,20.3; PH=99.9; top=G['HELMET'].bounds[1]; XS=json.load(open('fit.json'))['X']['single']
mm=lambda x,y:((x-XS)*s+BW/2,(436.5-y)*s+BH)
fv=lambda x,y:(mm(x,y)[0],-mm(x,y)[1])
yt=BH+PH; pad=16
FV=[f'<svg class="fv" viewBox="{-pad-6} {-yt-pad} {BW+2*pad+12} {yt+2*pad+4}" role="img" aria-label="Front view with dimensions in millimetres">',
    f'<rect x="0" y="{-BH}" width="{BW}" height="{BH}" class="c-black edge"/>']
for k in ORDER: FV.append(f'<path data-part="{k}" class="p c-{COL[k]} edge" d="{d(VIS[k],fv,0.05)}" fill-rule="evenodd"/>')
def bounds_mm(k):
    x0,y0,x1,y1=G[k].bounds; a=mm(x0,y1); b=mm(x1,y0); return a[0],a[1],b[0],b[1]
ax0,_,ax1,_=bounds_mm('ARMS'); hx0,_,hx1,_=bounds_mm('HELMET')
def dh(x1,x2,y,t): return f'<g class="dim"><line x1="{x1:.2f}" y1="{-y:.2f}" x2="{x2:.2f}" y2="{-y:.2f}"/><line class="ext" x1="{x1:.2f}" y1="{-y+1.5:.2f}" x2="{x1:.2f}" y2="{-y-1.5:.2f}"/><line class="ext" x1="{x2:.2f}" y1="{-y+1.5:.2f}" x2="{x2:.2f}" y2="{-y-1.5:.2f}"/><text x="{(x1+x2)/2:.2f}" y="{-y-1.3:.2f}">{t}</text></g>'
def dv(x,y1,y2,t): return f'<g class="dim"><line x1="{x:.2f}" y1="{-y1:.2f}" x2="{x:.2f}" y2="{-y2:.2f}"/><line class="ext" x1="{x-1.5:.2f}" y1="{-y1:.2f}" x2="{x+1.5:.2f}" y2="{-y1:.2f}"/><line class="ext" x1="{x-1.5:.2f}" y1="{-y2:.2f}" x2="{x+1.5:.2f}" y2="{-y2:.2f}"/><text transform="translate({x-1.3:.2f} {-(y1+y2)/2:.2f}) rotate(-90)">{t}</text></g>'
FV+=[f'<line class="axis" x1="{BW/2}" y1="4" x2="{BW/2}" y2="{-yt-5:.2f}"/>',
     dh(0,BW,-8,'70.0'),dh(ax0,ax1,yt+9,f'{ax1-ax0:.1f}'),dh(hx0,hx1,yt+3.5,f'{hx1-hx0:.1f}'),
     dv(-5,0,BH,f'{BH}'),dv(-12,0,yt,f'{yt:.1f}'),dv(BW+7,BH,yt,f'{PH}'),'</svg>']
# --- part thumbnails ---
names={'SUIT':'Suit (jacket + pants)','HELMET':'Helmet','FACE':'Face','GLASSES':'Glasses','EYEBROWS':'Eyebrows','NOSE':'Nose','MOUTH':'Mouth','CRAVAT':'Cravat (shirt)','JACKET':'Jacket','HANDS':'Hands','ARMS':'Arms (oval ring)','PANTS':'Pants','BOOTS':'Boots'}
notes={'SUIT':'One piece: straight-sided jacket with the cravat V cut out, flowing into the hips and legs as one smooth curve','HELMET':'Dome with side flaps; face shows through its opening','FACE':'Chin is one circular arc on the drawn chin line; under the helmet a normal face shape, helmet on top','GLASSES':'Two tilted lenses joined by an S-shaped bridge','EYEBROWS':'Two curved strokes','NOSE':'Small oval','MOUTH':'Straight line plus the lower-lip curve','CRAVAT':'Straight sides widening downward to a sharp V; seen through the jacket V-neck','JACKET':'Straight-sided panel with the cravat V cut out; behind the arms','HANDS':'Two fists touching, thumbs up, on the front of the arms','ARMS':'One ring: outer oval minus inner oval. Behind the head, in front of the jacket, carrying the hands','PANTS':'Squat stance with rounded hips','BOOTS':'Left and right, toes turned outward'}
EXACT={'ARMS':'exact oval ring','CRAVAT':'exact straight V'}
cards=[]
for k in ['HELMET','FACE','GLASSES','EYEBROWS','NOSE','MOUTH','CRAVAT','SUIT','HANDS','ARMS','BOOTS']:
    x0,y0,x1,y1=G[k].bounds; w,h=x1-x0,y1-y0; m=max(w,h)*0.08+0.5
    th=f'<svg viewBox="{x0-m:.2f} {y0-m:.2f} {w+2*m:.2f} {h+2*m:.2f}" class="thumb" aria-hidden="true"><path class="c-{COL[k]} edge thin" d="{d(G[k],tol=0.04)}" fill-rule="evenodd"/></svg>'
    sp=spec[k]; dv_=(dev[k]['max_dev_px'] or 0)*s
    thin=' <span class="flag">thinner than 0.8 mm</span>' if k in ('EYEBROWS','MOUTH') else ''
    cards.append(f'''<article class="part" data-part="{k}" tabindex="0"><div class="tw">{th}</div><div class="meta">
<h3>{names[k]}</h3><p class="note">{notes[k]}{thin}</p>
<dl><div><dt>Size</dt><dd>{sp['bbox_w']:.1f} × {sp['bbox_h']:.1f} mm</dd></div><div><dt>Colour</dt><dd><i class="sw c-{COL[k]}"></i>{COL[k]}</dd></div><div><dt>{'Geometry' if k in EXACT else 'Max shift'}</dt><dd>{EXACT[k] if k in EXACT else f'{dv_:.2f} mm'}</dd></div></dl></div></article>''')
json.dump(dict(cmp=''.join(cmp_),fv=''.join(FV),cards=''.join(cards)),open('page_parts.json','w'))
print(len(''.join(cmp_))//1024,'KB cmp',len(''.join(FV))//1024,'KB fv',len(''.join(cards))//1024,'KB cards')
