"""Face outline: forehead/cheeks (hidden under the helmet) + jaw narrowing to a rounded chin (fitted to the print)."""
import json, numpy as np, cv2
from scipy.optimize import least_squares
pf=json.load(open('photo_face.json')); m=json.load(open('photo_face_meas.json')); J=np.array(pf['jaw_rel'])
TUCK=0.03                                   # face edge runs this far under the helmet (helmet widths)
A=m['opening_half_width_rel']+TUCK; CY=m['eye_y_rel']; BT=CY-(m['face_top_rel']-TUCK); XC=m['opening_center_rel']
def face_curve(q,n=480):
    bb,k,e=q                                 # chin depth below eye line, jaw taper, jaw/chin roundness
    t=np.linspace(0,2*np.pi,n,endpoint=False); c,s=np.cos(t),np.sin(t)
    up=s<0
    x=np.where(up, A*np.sign(c)*np.abs(c)**(2/2.6), A*np.sign(c)*np.abs(c)**(2/e)*(1-k*s**2))   # forehead: rounded-square; jaw: tapers
    y=np.where(up, CY+BT*np.sign(s)*np.abs(s)**(2/2.6), CY+bb*s)
    return np.c_[x+XC,y]
def resid(q):
    C=face_curve(q,1200); return np.sqrt(((J[:,None,:]-C[None])**2).sum(-1)).min(1)
r=min((least_squares(resid,q0,bounds=([0.1,0.0,1.4],[0.6,0.7,4.0])) for q0 in ([0.3,0.2,2.0],[0.35,0.35,2.4],[0.28,0.1,1.8])),key=lambda r:r.cost)
q=r.x; rms=float(np.sqrt(np.mean(r.fun**2))); print('chin depth %.3f taper %.3f roundness %.2f  rms %.4f helmet widths'%(*q,rms))
json.dump(dict(A=A,CY=CY,BT=BT,XC=XC,q=list(map(float,q)),rms=rms),open('face_fit.json','w'))
im=cv2.imread('/root/.claude/uploads/5391fe7b-2194-5842-9b9d-7c568b21fbc3/8a0bb7c6-image.jpg')
h=pf['helmet']; C=face_curve(q); P=np.c_[C[:,0]*h['w']+h['cx'],C[:,1]*h['w']+h['y0']]
cv2.polylines(im,[P.round().astype(np.int32)],True,(0,0,255),2,cv2.LINE_AA)
for x,y in J: cv2.circle(im,(int(x*h['w']+h['cx']),int(y*h['w']+h['y0'])),2,(0,255,0),-1)
cv2.imwrite('photo_face.png',im[200:800,330:860])
