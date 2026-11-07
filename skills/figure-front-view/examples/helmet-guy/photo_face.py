"""Measure the head (white face disc) relative to the helmet on the real print photo."""
import cv2, numpy as np, json
from scipy import ndimage as ndi
from skimage import measure
im=cv2.imread('/root/.claude/uploads/5391fe7b-2194-5842-9b9d-7c568b21fbc3/8a0bb7c6-image.jpg')
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV); H,S,V=[a.astype(int) for a in cv2.split(hsv)]
blue=(H>100)&(H<130)&(S>120)&(V>60)
lb,_=ndi.label(blue); sz=np.bincount(lb.ravel()); sz[0]=0; helm=lb==sz.argmax()
helm=ndi.binary_fill_holes(ndi.binary_closing(helm,iterations=3))&helm|helm
ys,xs=np.nonzero(helm)
hx0,hx1,hy0,hy1=xs.min(),xs.max(),ys.min(),ys.max(); hcx=(hx0+hx1)/2; hw=hx1-hx0
white=(S<45)&(V>150)
# face = white region connected to the area between the helmet flaps, below the helmet top
seed=(int((hy0+hy1)/2+0.15*(hy1-hy0)),int(hcx))
wl,_=ndi.label(ndi.binary_opening(white,iterations=2)); face=wl==wl[seed]
face=ndi.binary_fill_holes(face)
k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(41,41))
face=cv2.morphologyEx(face.astype(np.uint8),cv2.MORPH_OPEN,k).astype(bool)      # drop the narrow shirt strip below the chin
wl2,_=ndi.label(face); face=wl2==wl2[seed]
# boundary points of the face that are NOT against the helmet (those are the true head outline)
cs=measure.find_contours(face.astype(float),0.5); c=max(cs,key=len); pts=c[:,::-1]
near_helm=ndi.binary_dilation(helm,iterations=6)
free=np.array([not near_helm[int(round(y)),int(round(x))] for x,y in pts])
# also exclude the neck/shirt below the chin: keep points above the jacket top (where face is widest below the flaps)
fp=pts[free]
fy_max=np.percentile(fp[:,1],99)
# chin arc + lower sides: points below the flap bottoms
flap_bottom=hy1
arc=fp[(fp[:,1]>flap_bottom-0.12*(flap_bottom-hy0))]
# the shirt strip continues below the chin: drop points in a narrow column at the centre below the chin curve
A=np.c_[2*arc,np.ones(len(arc))]; b=(arc**2).sum(1)
for _ in range(3):
    cx,cy,k=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(k+cx**2+cy**2)
    res=np.abs(np.hypot(arc[:,0]-cx,arc[:,1]-cy)-R); keep=res<np.percentile(res,80)+1
    A,b,arc=A[keep],b[keep],arc[keep]
jaw=fp[fp[:,1]>hy1-0.30*(hy1-hy0)]
out=dict(jaw_rel=[[float((x-hcx)/hw),float((y-hy0)/hw)] for x,y in jaw[::3]],helmet=dict(x0=float(hx0),x1=float(hx1),y0=float(hy0),y1=float(hy1),cx=float(hcx),w=float(hw)),
         head_circle=dict(cx=float(cx),cy=float(cy),R=float(R)),
         rel=dict(dx=float((cx-hcx)/hw),cy_from_top=float((cy-hy0)/hw),R=float(R/hw)))
print(json.dumps(out,indent=1)); json.dump(out,open('photo_face.json','w'),indent=1)
vis=im.copy(); cv2.circle(vis,(int(cx),int(cy)),int(R),(0,0,255),2); cv2.rectangle(vis,(hx0,hy0),(hx1,hy1),(255,0,0),2)
for x,y in arc[::5]: cv2.circle(vis,(int(x),int(y)),2,(0,255,0),-1)
cv2.imwrite('photo_face.png',vis[200:800,330:860])
