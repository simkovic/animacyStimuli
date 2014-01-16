from Settings import *
from Constants import *
from psychopy.event import xydist
import numpy as np
import random, os, pickle

class RandomAgent():
    def __init__(self,nrframes,dispSize,pos,pdc,sd,moveRange):
        self.offset=pos
        self.ds=dispSize
        self.nrframes=nrframes
        self.traj=np.zeros((nrframes,3))
        self.reset()
        self.pdc=pdc
        self.sd=sd
        self.nrcrashes=np.zeros((nrframes))
        self.ndc=np.zeros((self.nrframes))
        self.moveRange=moveRange/2.0
    
    def reset(self):
        """ choose Random position within arena """
        self.ndc=np.zeros((self.nrframes))
        self.nrcrashes=np.zeros((self.nrframes))
        self.f=0
        self.i=0
        self.traj[self.f,:]=np.array((random.random()*self.ds[X]-self.ds[X]/2.0+self.offset[X],
            random.random()*self.ds[Y]-self.ds[Y]/2.0+self.offset[Y],random.random()*360))
        
    def backtrack(self):
        self.f-=51#31
        return self.f<0 or self.i>100000#10000
    def getPosition(self,dec=0):
        return self.traj[self.f+dec,[X,Y]]
    def getTrajectory(self):
        return self.traj
    def move(self):
        self.f+=1
        self.i+=1
        f=self.f
        self.nrcrashes[f]=0
        rnd = random.random()<self.pdc
        self.ndc[f]=float(rnd)
        if rnd: # change direction chasee
            self.traj[f,PHI]=(self.traj[f-1,PHI]
                    +random.uniform(-self.moveRange,self.moveRange))%360
        else:
            self.traj[f,PHI]= self.traj[f-1,PHI]
        adjust =np.array((np.cos(self.traj[f,PHI]/180.0*np.pi) 
                *self.sd,np.sin(self.traj[f,PHI]/180.0*np.pi)*self.sd))
        self.traj[f,[X,Y]]=self.traj[f-1,[X,Y]]+adjust
        return (f+1)==self.nrframes
    def crashed(self,newD):
        self.nrcrashes[self.f]=1
        self.traj[self.f,PHI]=newD[1]
        self.traj[self.f,[X,Y]]=newD[0]

class HeatSeekingChaser(RandomAgent):
    def __init__(self,*args,**kwargs):
        isGreedy=args[-1]        
        RandomAgent.__init__(self,*args[:-1],**kwargs)
        self.isGreedy=isGreedy
    def move(self,targetPos,crash=False):
        if not crash:
            self.f+=1
            self.i+=1
        f=self.f
        rnd = random.random()<self.pdc
        self.ndc[f]=int(rnd)
        if rnd or crash:
            self.traj[f,PHI]=np.arctan2(targetPos[Y],
                            targetPos[X])/np.pi * 180
            self.traj[f,PHI]=(360+self.traj[f,PHI]
                +random.uniform(-self.moveRange,self.moveRange))%360
        else:
            self.traj[f,PHI]= self.traj[f-1,PHI]   
        adjust =np.array((np.cos(self.traj[f,PHI]/180.0*np.pi) 
                *self.sd,np.sin(self.traj[f,PHI]/180.0*np.pi)*self.sd))
        self.traj[f,[X,Y]]=self.traj[f-1,[X,Y]]+adjust
    def crashed(self,newD=None,targetPos=(0,0)):
        #print 'crashed', self.f
        if not self.isGreedy:
            RandomAgent.crashed(self,newD)
        else:
            #print 'before', self.getPosition(), targetPos
            self.move(targetPos,crash=True)
            self.nrcrashes[self.f]=1
            #print 'after', self.getPosition()

            
def generateTrial(STATISTICS=False):
    if STATISTICS: nrbacktracks=0
    # init chaser chasee
    chasee=RandomAgent(Q.nrframes,Q.maze.dispSize,Q.maze.pos,
            Q.pDirChange[CHASEE],Q.aSpeed,Q.phiRange[0])
    chaser=HeatSeekingChaser(Q.nrframes,Q.maze.dispSize,Q.maze.pos,
            Q.pDirChange[CHASER],Q.aSpeed,Q.phiRange[CHASER],True)
    while (xydist(chaser.getPosition(),chasee.getPosition())<Q.initDistCC[MIN]
        and xydist(chaser.getPosition(),chasee.getPosition())>Q.initDistCC[MAX]):
        # resample until valid distance between chaser and chasee is obtained
        chasee.reset(); chaser.reset()
    agents=[chasee,chaser]
    # init distractors
    for d in range(Q.nragents-2):
        distractor=RandomAgent(Q.nrframes,Q.maze.dispSize,Q.maze.pos,
            Q.pDirChange[DISTRACTOR],Q.aSpeed,Q.phiRange[CHASEE])
        agents.append(distractor)
    # check for wall collisions
    for a in range(Q.nragents):
        d,edge=Q.maze.shortestDistanceFromWall(agents[a].getPosition())
        while d<=Q.agentRadius:
            agents[a].reset()
            d,edge=Q.maze.shortestDistanceFromWall(agents[a].getPosition()) 
    # generate the movement of chasee and chaser
    finished=False
    while not finished:
        # check the distance
        (dx,dy)=chasee.getPosition() - chaser.getPosition()
        if np.sqrt(dx**2+dy**2)<Q.rejDist:
            if STATISTICS: nrbacktracks+=1
            deadend=chaser.backtrack()
            chasee.backtrack()
            if deadend:
                print 'dead end', chasee.f
                if STATISTICS: return None, None, None,None
                else: return None
            (dx,dy)=chasee.getPosition() - chaser.getPosition()
        # move chaser and avoid walls
        chaser.move((dx,dy))
        d,edge=Q.maze.shortestDistanceFromWall(chaser.getPosition())
        if d<=Q.agentRadius:
            newD=Q.maze.bounceOff(chaser.getPosition(),
                chaser.getPosition(-1),edge,Q.agentRadius)
            chaser.crashed(newD=newD,targetPos=(dx,dy))
        # move chasee and avoid walls
        finished=chasee.move()
        d,edge=Q.maze.shortestDistanceFromWall(chasee.getPosition())
        if d<=Q.agentRadius:
            newD=Q.maze.bounceOff(chasee.getPosition(),
                chasee.getPosition(-1),edge,Q.agentRadius)
            chasee.crashed(newD)
        #if chaser.f>401:
        #    raise NameError('stop')
    # generate distractor movement
    finished=False
    while not finished and Q.nragents>2:
        for a in range(2,Q.nragents):
            finished=agents[a].move()
            d,edge=Q.maze.shortestDistanceFromWall(agents[a].getPosition())
            if d<=Q.agentRadius:
                newD=Q.maze.bounceOff(agents[a].getPosition(),
                    agents[a].getPosition(-1),edge,Q.agentRadius)
                agents[a].crashed(newD)
    trajectories=np.zeros((Q.nrframes,Q.nragents,3))
    for a in range(Q.nragents):
        tt=agents[a].getTrajectory()
        trajectories[:,a,X]=tt[:,X]
        trajectories[:,a,Y]=tt[:,Y]
        trajectories[:,a,PHI]=tt[:,PHI]
    if STATISTICS:
        statistics=[trajectories,np.zeros((3)),nrbacktracks,
            [chasee.ndc.sum(),chaser.ndc.sum(),agents[2].ndc.sum()]]
        for a in range(3):
            statistics[1][a]=agents[a].nrcrashes.sum()
        return statistics
    else: return trajectories
    
def generateShortTrial(nrdirch=4):
    Q.trialDur=5; Q.rejDist=0.0
    Q.nrframes=Q.trialDur*Q.refreshRate+1
    traj=generateTrial(2)
    ende=np.nonzero(np.cumsum(np.diff(traj[:,1,PHI])>1)==nrdirch)[0]
    print len(ende)
    ende=np.array(ende).min()
    return traj[:ende,:,:]

def generateExperiment(vpn,nrtrials,conditions=None,dispSizes=None):
    #os.chdir('..')
    Q=initQ(myChase)
    os.chdir('input/')
    conditions=np.repeat(conditions,nrtrials)
    dispSizes=np.repeat(dispSizes,nrtrials)
    mazes=[]
    for d in dispSizes:
        mazes.append(EmptyMaze((1,1),dispSize=(d,d)))
    print 'Generating Trajectories'
    for vp in vpn:
        vpname='vp%03d' % vp
        os.mkdir(vpname)
        os.chdir(vpname)
        order=np.random.permutation(conditions.size)
        for trial in range(conditions.size):
            Q.nragents= conditions[order[trial]]
            Q.maze=mazes[order[trial]]
            trajectories=None
            while trajectories is None:
                trajectories=generateTrial()
            fn = 'trial%03d' % trial
            print fn
            np.save(fn,trajectories)  
        os.chdir('..')
    os.chdir('..')
    
def generateMixedExperiment(vpn,trialstotal,blocks=4,probeTrials=False):
    Q=initQ(myChase)
    os.chdir(Q.inputPath)
    mazes=[]
    if probeTrials: bs=range(0,blocks+1)
    else: bs=range(25,blocks+1)
    print 'Generating Trajectories'
    for vp in vpn:
        vpname='vp%03d' % vp
        #os.mkdir(vpname)
        os.chdir(vpname)
        Q.save('SettingsTraj.pkl')
        
        for block in bs:
            if block ==0: nrtrials=10
            else: nrtrials=trialstotal
            for trial in range(nrtrials):
                if vp>1 and vp<10: continue
                if trial >= nrtrials*0.9: Q.rejDist=0.0
                else: Q.rejDist=3.0
                trajectories=None
                while trajectories ==None:
                    trajectories=generateTrial()
                #fn='%str%03dcond%02d'% (vpname,trial,conditions[order[trial]])
                #fn = 'trial%03d' % trial
                fn='%sb%dtrial%03d'% (vpname,block,trial)
                print fn
                np.save(fn,trajectories)
            while True:# check that more than 1 consecutive control trials do not occur
                r=np.random.permutation(nrtrials)
                r2=np.roll(np.random.permutation(nrtrials)>=nrtrials-0.1*nrtrials,1)
                #r3=np.roll(np.random.permutation(50)>=45,2)
                if not np.any(np.bitwise_and(r,r2)):
                    break
            np.save('order%sb%d'% (vpname,block),r)
        os.chdir('..')
    os.chdir('..')
    
def generateBabyExperiment(vpn,nrtrials=10,blocks=1,conditions=[6,8],
        dispSize=29,maze=None):
    #os.chdir('..')
    Q=initQ(mychase)
    os.chdir(Q.inputPath)
    mazes=[]
    Q.nrframes+= Q.refreshRate *5
    Q.rejDist=3.0
    Q.maze=EmptyMaze((1,1),dispSize=(dispSize,dispSize))
    print 'Generating Trajectories'
    for vp in vpn:
        vpname='vp%03d' % vp
        os.mkdir(vpname)
        os.chdir(vpname)
        r=[]
        phase=[0,1,1,2]
        for i in range((len(conditions)*nrtrials-len(phase))/2):
            if np.random.rand()>0.5: phase.extend([1,2])
            else: phase.extend([2,1])
        print 'phase', phase
        for block in range(blocks):
            i=0
            for condition in conditions:
                for trial in range(nrtrials):
                    if condition==conditions[0]: 
                        if np.random.rand()>0.5: r.extend([trial, trial+nrtrials])
                        else: r.extend([trial+nrtrials,trial])
                    Q.nragents=condition
                    trajectories=None
                    while trajectories ==None:
                        trajectories=generateTrial()
                    #fn='%str%03dcond%02d'% (vpname,trial,conditions[order[trial]])
                    #fn = 'trial%03d' % trial
                    trajectories=trajectories[(Q.refreshRate*5):]
                    #print trajectories.shape
                    fn='%sb%dtrial%03d'% (vpname,block,i)
                    i+=1
                    print fn
                    np.save(fn,trajectories)
            #r=np.random.permutation(nrtrials*len(conditions))
            r=np.array(r)
            print r
            np.save('order%sb%d'% (vpname,block),r)
            np.save('phase%sb%d'% (vpname,block),phase)
            Q.save('SettingsTraj.pkl')
        os.chdir('..')
    os.chdir('..')

def generateTremouletTrial(phi=0, lam=1):
    refreshRate=60 # Hz
    speed=3.25/refreshRate
    angle=0
    duration=0.75 # in seconds
    N=np.int32(duration*refreshRate) # nr of frames
    traj = np.zeros((N,1,2))
    traj[0,0,X] = -speed*(N/2-0.5); traj[0,0,Y]=0
    for i in range(1,N):
        traj[i,0,X]=np.cos(angle/180.0*np.pi)*speed+traj[i-1,0,X]
        traj[i,0,Y]=np.sin(angle/180.0*np.pi)*speed+traj[i-1,0,Y]
        if i==((N-1)/2):
            speed=speed*lam
            angle=angle+phi
    return traj
def generateGao09e1(vpn):
    # gao09e1 settings
    Q=initQ(gao09)
    nrtrials=15
    chs=[0,60,120,180,240,300]
    block=0
    #os.chdir('..')
    os.chdir('..')
    os.chdir('input/')
    for vp in vpn:
        vpname='vp%03d' % vp
        os.mkdir(vpname)
        os.chdir(vpname)
        i=0
        r=np.zeros((2*6*nrtrials,2))
        r[:,0]=np.random.permutation(2*6*nrtrials)
        for cond in range(6):
            for trial in range(nrtrials):
                Q.phiRange=(Q.phiRange[0],chs[cond])
                trajectories=None
                while trajectories ==None: trajectories=generateTrial()
                #target present trial
                r[i,1]=cond 
                fn='gao09e1%sb%dtrial%03d'% (vpname,block,i); 
                np.save(fn,trajectories[:,:-1,:]);i+=1
                #target absent trial
                r[i,1]=cond+6
                fn='gao09e1%sb%dtrial%03d'% (vpname,block,i); 
                np.save(fn,trajectories[:,1:,:]);i+=1

        np.save('gao09e1order%sb%d'% (vpname,block),r)
        Q.save('SettingsTraj.pkl')
        os.chdir('..')
    os.chdir('..')
    
def generateGao10e4(vpn):
    Q=initQ(gao10e4)
    nrtrials=90; 
    os.chdir('..');os.chdir('input/')
    for vp in vpn:
        vpname='vp%03d' % vp;os.mkdir(vpname);os.chdir(vpname)
        for trial in range(nrtrials):
            if vp>400 and vp<500: continue
            trajectories=generateTrial(12,maze=maze)
            fn='%strial%03d'% (vpname,trial); 
            np.save(fn,trajectories[:,2:,:])
        np.save('order%sb0'% (vpname),np.random.permutation(nrtrials))
        np.save('order%sb1'% (vpname),np.random.permutation(nrtrials))
        np.save('order%sb2'% (vpname),np.random.permutation(nrtrials))
        Q.save('SettingsTraj.pkl')
        os.chdir('..')

def generateGao10e3(vpn):
    Q=initQ(gao10e3)
    offs=5.875; sz=(2*offs+Q.agentSize,2*offs+Q.agentSize)
    quadrants=[EmptyMaze((1,1),dispSize=sz,pos=(offs,offs),lw2cwRatio=0),
        EmptyMaze((1,1),dispSize=sz,pos=(-offs,offs),lw2cwRatio=0),
        EmptyMaze((1,1),dispSize=sz,pos=(offs,-offs),lw2cwRatio=0),
        EmptyMaze((1,1),dispSize=sz,pos=(-offs,-offs),lw2cwRatio=0)]
    nrtrials=42; 
    os.chdir('..');os.chdir('input/')
    for vp in vpn:
        vpname='vp%03d' % vp;os.mkdir(vpname);os.chdir(vpname)
        for trial in range(nrtrials):
            if vp>300 and vp<400 and vp!=350: continue
            trajectories=[]
            for k in range(len(quadrants)):
                Q.maze=quadrants[k]
                traj=generateTrial()
                trajectories.append(traj[:,2:,:])
            fn='%strial%03d'% (vpname,trial); 
            np.save(fn,np.concatenate(trajectories,axis=1))
        np.save('order%sb%d'% (vpname,0),np.random.permutation(nrtrials))
        np.save('order%sb%d'% (vpname,1),np.random.permutation(nrtrials))
        np.save('order%sb%d'% (vpname,2),np.random.permutation(nrtrials))
        Q.save('SettingsTraj.pkl')
        os.chdir('..')

        
#############################
#
#       DIAGNOSIS TOOLS
#
#############################

def exportSvmGao09(nrtrials=10000):
    def saveTraj(fout,traj,label):
        sample=5
        fout.write('%d '%label)
        i=1
        for f in range(traj.shape[0]/sample):
            for a in range(2):
                fout.write('%d:%f '%(i,traj[f*sample,a,0]));i+=1
                fout.write('%d:%f '%(i,traj[f*sample,a,1]));i+=1
        fout.write('\n')            
    Q=initQ(gao09)
    chs=[300,240,180,120,60,0]
    block=1
    os.chdir('input/')
##    fout=open('svmGao2.train','w')
##    for trial in range(nrtrials):
##        print trial
##        for cond in range(6):
##            trajectories=generateTrial(5,maze=maze,rejectionDistance=5.0,
##                moveSubtlety=(chs[cond],120),trialDur=10)
##            trajectories[:,:,0]/= 32.0
##            trajectories[:,:,1]/= 24.0
##            saveTraj(fout,trajectories[:,[0,1],:],1);
##            saveTraj(fout,trajectories[:,[4,1],:],-1);
##    fout.close()

    nrtrials=10000
        
    for cond in range(5):
        Q.phiRange=(chs[cond],120)
        print cond
        fout1=open('svmGaoCond%03dT.train'%chs[cond],'w')
        fout2=open('svmGaoCond%03dF.train'%chs[cond],'w')
        for trial in range(nrtrials):
            trajectories=generateTrial()
            trajectories[:,:,0]/= 32.0
            trajectories[:,:,1]/= 24.0
            saveTraj(fout1,trajectories[:,[0,1],:],1);
            saveTraj(fout2,trajectories[:,[4,1],:],-1);
        fout1.close()
        fout2.close()
    os.chdir('..')

def createSample(N=1000,prefix='gao09'):
    path='trajectoryData'+os.path.sep+prefix+os.path.sep
    ncr=np.zeros((N,3))
    ndc=np.zeros((N,3))
    nbt=np.zeros(N)
    for i in range(N):
        pos=None
        while pos is None: pos,crashes,bts,dc=generateTrial(STATISTICS=True)
        np.save(path+'t%03d'%i,pos)
        ncr[i,:]=crashes/float(Q.trialDur) # nr crashes per second
        nbt[i]=bts/float(Q.trialDur) # nr backtracks per second
        ndc[i,:]=np.array(dc)/float(Q.trialDur)  # nr dir changes per second
    np.save(path+'nrcrashes',ncr)
    np.save(path+'nrbacktracking',nbt)
    np.save(path+'nrdirchanges',ndc)

def radialDensity(N=100,prefix='gao09',plot=True):
    ''' density per effective surface'''
    import pylab as plt
    subsample=range(0,600,10)
    radius=np.linspace(0,30,61)
    surface=np.diff(np.pi*radius**2)
    mindist=np.zeros((N,3))
    R=np.zeros((3,N,len(radius)-1))*np.nan
    for i in range(N):
        t=np.load('trajectoryData'+os.path.sep+
                prefix+os.path.sep+'t%03d.npy'%i)
        if prefix =='gao09': t=t[subsample,:-1,:2]
        else: t=t[subsample,:,:2]
        for a in [CHASER,CHASEE,DISTRACTOR]:
            D=np.swapaxes(t,0,1)-np.array(t[:,a,:],ndmin=3)
            sel=range(D.shape[0]);sel.pop(a)
            D=D[sel,:,:]
            d=np.sqrt(np.power(D,2).sum(axis=2))
            mindist[i,a]=np.min(d,axis=1).mean(0)
            R[a,i,:]=np.histogram(d.flatten(),bins=radius)[0]/surface
    #R[:,:,0]-=len(subsample)
    R/=len(subsample)*6
    if plot:
        plt.figure()
        for a in range(3):
            plt.plot(radius[:-1],R[a,:,:].mean(0))
        plt.legend(['Chasee','Chaser','Distractor'])
        plt.title('Agent Density')
        plt.ylabel('Nr of Agents per Second per Deg^2')
        plt.xlabel('Radial Distance')
    print mindist.mean(0)
    return R,mindist
    
def agDistance(N=100,prefix='gao09'):
    dif=[[]]*4
    avgdist=np.zeros((N,4))
    for i in range(N):
        t=np.load('trajectoryData'+os.path.sep+
                prefix+os.path.sep+'t%03d.npy'%i)
        t=t[:,:,:2]
        # average distance
        dif[0]=t[:,CHASEE,:]-t[:,CHASER,:]
        dif[1]=t[:,CHASEE,:]-t[:,DISTRACTOR,:]
        dif[2]=t[:,DISTRACTOR,:]-t[:,CHASER,:]
        dif[3]=t[:,DISTRACTOR,:]-t[:,DISTRACTOR2,:]
        for k in range(4):
            avgdist[i,k]=np.sqrt(np.power(dif[k],2).sum(axis=1)).mean()
    print avgdist.mean(0)
    return avgdist
# todo nr agent crashes?

def nrDirChanges(prefix='gao09'):
    ndc=np.load('trajectoryData'+os.path.sep+
        prefix+os.path.sep+'nrdirchanges.npy')
    print ndc.mean(0)

def phiDistribution(N=1000,prefix='gao09',plot=True):
    inc=10
    edges=np.arange(0,180+inc,inc)
    adirs=np.zeros((N,edges.size-1,3))
    ndc=np.load('trajectoryData'+os.path.sep+
        prefix+os.path.sep+'nrdirchanges.npy')
    for i in range(N):
        t=np.load('trajectoryData'+os.path.sep+
                prefix+os.path.sep+'t%03d.npy'%i)
        phi=t[:,:,2]
        for a in [CHASEE,CHASER,DISTRACTOR]:
            df=np.mod(360+np.diff(phi[:,a]),360)
            df[df>180]=360-df[df>180]
            df=df[df>0]
            adirs[i,:,a]= np.histogram(df,bins=edges)[0]
    if plot:
        plt.figure()
        x=edges[:-1]+inc
        for a in range(3):
            plt.plot(x,adirs[:,:,a].mean(0)/float(Q.trialDur),'o-')
        plt.legend(['Chasee','Chaser','Distractor'])
        plt.xlabel('Abs Change in Direction in Degrees')
        plt.ylabel('Number of direction Changes per Second')

def radialDensity(N=1000,prefix='gao09'):
    ''' density per effective surface'''
    import pylab as plt
    inc=1
    
    R=np.zeros(np.int32(Q.maze.dispSize/float(inc)))*np.nan
    for i in range(N):
        t=np.load('trajectoryData'+os.path.sep+
                prefix+os.path.sep+'t%03d.npy'%i)
        for a in [CHASER,CHASEE,DISTRACTOR]:
            R=np.histogram2d(t[:,0,0])
def posDistribution(N=1000,prefix='gao09'):
    import pylab as plt
    inc=1
    x=np.arange(-Q.maze.dispSize[X]/2, Q.maze.dispSize[X]/2+inc,inc)
    y=np.arange(-Q.maze.dispSize[Y]/2, Q.maze.dispSize[Y]/2+inc,inc)
    R=np.zeros((x.size-1,y.size-1,3))
    for i in range(N):
        t=np.load('trajectoryData'+os.path.sep+
                prefix+os.path.sep+'t%03d.npy'%i)
        for a in [CHASER,CHASEE,DISTRACTOR]:
            R[:,:,a]+=np.histogram2d(t[:,a,X],t[:,a,Y] ,bins=[x,y])[0]
    R/=float(N)
    x=x[:-1]+inc/2.
    y=y[:-1]+inc/2.
    aa,bb=np.meshgrid(x,y)
    for a in range(3):
        plt.subplot(1,3,a+1)
        ax=plt.gca()
        ax.set_axis_off()
        plt.imshow(R[:,:,a].T,extent=[x[0],x[-1],y[0],y[-1]],cmap='gray',
                   origin='lower',interpolation='bicubic',aspect='equal')
   
if __name__ == '__main__':
    import pylab as plt
    plt.ion()
    #d=28
    #random.seed(3)
    
    #generateMixedExperiment([2],40,blocks=25,probeTrials=True)
    Q=initQ(gao09)
    Q.phiRange=(Q.phiRange[0],0)
    createSample()
    #radialDensity()
    #agDistance()

    #phiDistribution()


        #plt.xlim([x[0],x[-1]])
        #plt.ylim([y[0],y[-1]])
    #plt.colorbar()
##    pos=None
##    while pos is None: pos=generateTrial()
##    from Tools import showTrial
##    showTrial(t,qsettings=Q)

        
        
    
    #t=np.load('input/vp023/gao09e1vp023b1trial003.npy')
    #TD=TrajectoryData(t,trajRefresh=60.0,highlightChase=True)
    #TD.replay(tlag=0)
    #TD.showFrame(t[333,:,:])
    #generateExperiment([105],1000,conditions=[20],dispSizes=[32],rejectionDistance=3.0)
    #generateExperiment([0],1,conditions=[8,11,14,17,20],
    #                   dispSizes=[18,22,26,29,32],rejectionDistance=3.0)
        
    #generateBabyExperiment([201])
    
    #t=generateShortTrial(maze)
    #print t.shape
    #np.save('t.npy',t)
    #exportSvmGao09()
    
    #generateGao10e4(308)
    
    #generateGao10e3(range(350,370))


        

    
