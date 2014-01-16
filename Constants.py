# some constants .
CHASEE=0
CHASER=1
DISTRACTOR=2
DISTRACTOR2=3
X=0;Y=1;PHI=2;T=2
I=0;J=1
MIN=0;MAX=1

import numpy as np
def pointInTriangle(t1,t2,t3,pt):
    def sign(p1,p2,p3):
        return (p1[X]-p3[X])*(p2[Y]-p3[Y])-(p2[X]-p3[X])*(p1[Y]-p3[Y])
    b1=sign(pt,t1,t2)<0
    b2=sign(pt,t2,t3)<0
    b3=sign(pt,t3,t1)<0
    return b1==b2 and b2==b3

def drawCircle(M,pos,radius,value=1):
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            if np.sqrt((i-pos[X])**2+(j-pos[Y])**2)<radius:
                M[i,j]=value
    return M
    
# ring mask
N=128
CIRCLE=np.ones((N,N))*-1
for i in range(N):
    for j in range(N):
        if np.sqrt((i-N/2+0.5)**2+(j-N/2+0.5)**2)<N/2:
            CIRCLE[i,j]= 1
RING=np.ones((N,N))*-1
for i in range(N):
    for j in range(N):
        if np.sqrt((i-N/2+0.5)**2+(j-N/2+0.5)**2)>2*N/5 and np.sqrt((i-N/2+0.5)**2+(j-N/2+0.5)**2)<N/2:
            RING[i,j]= 1
DART=np.ones((N,N))*-1
a=np.cos(np.pi/3.0)*N/2.0#(N**2-(N/2)**2)**0.5-N/2
b=np.sin(np.pi/3.0)*N/2.0#((N/2)**2-( a**2))**0.5
c=N/2-0.5
t1=(0,c);t2=(c+a,c-b); t3=(c+a,c+b)

for i in range(N):
    for j in range(N):
        DART[i,j]+=2*(pointInTriangle(t1,t2,(c,c),(i,j)) or pointInTriangle(t1,t3,(c,c),(i,j)))
        
del a,b,c,t1,t2,t3
DART=np.rot90(DART,-1)

EYES=np.ones((N,N,3))
mid=N/2-0.5
#for k in range(3): EYES[:,:,k]=drawCircle(EYES[:,:,k],(mid,mid), N/2,value=1)
for k in [1,2]:
    EYES[:,:,k]=drawCircle(EYES[:,:,k],(mid-0.48/1.9*N/2,mid+0.67/0.95*N/2), N/10.0,value=-1)
    EYES[:,:,k]=drawCircle(EYES[:,:,k],(mid+0.48/1.9*N/2,mid+0.67/0.95*N/2), N/10.0,value=-1)
EYES=np.uint8((EYES+1)/2.0*255)

del np