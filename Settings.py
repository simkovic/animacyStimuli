from psychopy import visual,monitors
from psychopy.misc import pix2deg,deg2pix
from Constants import *
import os
import pickle
from Maze import EmptyMaze

__all__ = ['initQ','mychase','gao09','gao10e3','gao10e4','meyerhoff13']
class Settings():
    def __init__(self,monitor,trialDur,refreshRate,agentSize,phiRange,
        pDirChange,initDistCC,bckgCLR,agentCLR,mouseoverCLR,selectedCLR,aSpeed,
        winPos,fullscr,rejDist,nragents,maze,mask):
        ''' phiRange        - in degrees [0-360]
            agentSize       - in degrees of visial angle
            initDistCC      - list of len==2 (min,max), values in degrees of visial angle
            pDirChange      - len 3 list, avg number of direction changes per second
            agentCLR        - range [1 -1]
            mouseoverCLR    - range [1 -1]
            selectedCLR     - range [1 -1]
            trialDur        - in seconds
            aSpeed          - in degrees of visual angle per second
            rejDist         - rejection Distance in degress of visual angle
            nragents        - number of agents
            mask            - 2d array giving mask for the objects shape
        '''
        self.refreshRate=float(refreshRate)
        self.monitor=monitor
        self.fullscr=fullscr
        self.setTrialDur(trialDur)
        self.agentSize=agentSize
        self.phiRange=phiRange
        self.setpDirChange(pDirChange)
        self.initDistCC=initDistCC
        self.bckgCLR=bckgCLR
        self.agentCLR=agentCLR
        self.mouseoverCLR=mouseoverCLR
        self.selectedCLR=selectedCLR
        self.setAspeed(aSpeed)
        self.winPos=winPos
        self.rejDist=rejDist
        path = os.getcwd()
        path = path.rstrip('code')
        self.inputPath=path+"input"+os.path.sep
        self.outputPath=path+"output"+os.path.sep
        self.stimPath=path+"stimuli"+os.path.sep
        self.agentRadius=self.agentSize/2.0
        self.fullscr=fullscr
        self.nragents=nragents
        self.maze=maze
        self.mask=mask
    def setTrialDur(self,td):
        self.trialDur=td
        self.nrframes=self.trialDur*self.refreshRate+1
    def setpDirChange(self,pDirChange):
        self.pDirChange=[pDirChange[CHASEE]/self.refreshRate,
             pDirChange[CHASER]/self.refreshRate,
             pDirChange[DISTRACTOR]/self.refreshRate]
    def setAspeed(self,aSpeed):  self.aSpeed=aSpeed/self.refreshRate
  
    def initDisplay(self,sz=None):
        if sz==None: sz=(1280,1280)
        elif type(sz)==int: sz=(sz,sz)
        wind=visual.Window(monitor=self.monitor,fullscr=self.fullscr,
            size=sz,units='deg',color=self.bckgCLR,pos=self.winPos,
            winType='pyglet')
        return wind
    def norm2pix(self,xy):
        return (np.array(xy)) * np.array(self.monitor.getSizePix())/2.0
    def norm2deg(self,xy):
        xy=self.norm2pix(xy)
        return pix2deg(xy,self.monitor)
    def pix2deg(self,pix):
        return pix2deg(pix,self.monitor)
    def deg2pix(self,deg):
        return deg2pix(deg,self.monitor)
    def save(self,filepath):
        f=open(filepath,'wb')
        try: pickle.dump(self,f);f.close()
        except: f.close(); raise
    @staticmethod
    def load(filepath):
        f=open(filepath,'rb')
        try: out=pickle.load(f);f.close()
        except: f.close(); raise
        return out

# monitors
dell=monitors.Monitor('dell', width=37.8, distance=50); dell.setSizePix((1280,1024))

laptop={'monitor' :     dell,
        'refreshRate':  60,                 # [hz]
        'fullscr':      True,
        'winPos':       (0,0)              # in pixels; X,Y axis; center at 0,0
        }
mychase={'phiRange':     [120,0*2],         # in degrees [0-360]
        'agentSize':    1,                  # in degrees of visial angle
        'initDistCC':   [12.0 ,18.0],       # in degrees of visial angle
        'pDirChange':   [4.8,5.4,4.8],          # avg number of direction changes per second
        'bckgCLR':      [-0,-0,-0],
        'agentCLR':     1,                  # [1 -1]
        'mouseoverCLR': 0.5,                # [1 -1]
        'selectedCLR':  -0.5,               # [1 -1]
        'trialDur':     30,                 # in seconds
        'aSpeed':       14.5,               # in degrees of visual angle per second
        'rejDist':      3.0,                 # in degress of visual angle
         'maze':        EmptyMaze((1,1),dispSize=(26,26),lw2cwRatio=0.0),
         'nragents':    14,
         'mask':        RING
        }

gao09={'phiRange':      [120,None],         
        'agentSize':    1,                  
        'initDistCC':   [12.0 ,18.0],       
        'pDirChange':   [5.9,5.9,5.9],         
        'bckgCLR':      [-1,-1,-1],
        'agentCLR':     1,                  
        'mouseoverCLR': 0.5,                
        'selectedCLR':  -0.5,               
        'trialDur':     10,                 
        'aSpeed':       14.5,             
        'rejDist':      5.0,                 
         'maze':        EmptyMaze((1,1),dispSize=(32,24),lw2cwRatio=0.0),
         'nragents':    5,
         'mask':        RING
       }

gao10e4={'phiRange':     [120,None],         
        'agentSize':    1,                  
        'initDistCC':   [12.0 ,18.0],       
        'pDirChange':   [5.4,5.4,5.4],# double-check these settings         
        'bckgCLR':      [-0,-0,-0],
        'agentCLR':     1,                  
        'mouseoverCLR': 0.5,                
        'selectedCLR':  -0.5,               
        'trialDur':     8,                 
        'aSpeed':       5.1,             
        'rejDist':      0.0,                 
         'maze':        EmptyMaze((1,1),dispSize=(18,18),lw2cwRatio=0),
         'nragents':    5
       }


gao10e3={'phiRange':    [120,None],         
        'agentSize':    1,                  
        'initDistCC':   [12.0 ,18.0],       
        'pDirChange':   [5.4,5.4,5.4],   
        'bckgCLR':      [-0,-0,-0],
        'agentCLR':     1,                  
        'mouseoverCLR': 0.5,                
        'selectedCLR':  -0.5,               
        'trialDur':     8,                 
        'aSpeed':       5.1,             
        'rejDist':      0.0,                 
         'maze':        None,
         'nragents':    5
       }
meyerhoff13={
        'phiRange':      [120,0],         
        'agentSize':    0.5,                  
        'initDistCC':   [5.0 ,18.0],       
        'pDirChange':   [6,6,6],         
        'bckgCLR':      [-1,-1,-1],
        'agentCLR':     1,                  
        'mouseoverCLR': 0.5,                
        'selectedCLR':  -0.5,               
        'trialDur':     5,                 
        'aSpeed':       7.25,             
        'rejDist':      5.0,                 
         'maze':        EmptyMaze((1,1),dispSize=(23.2,13.45),lw2cwRatio=0.0),
         'nragents':    3,
        'mask':         CIRCLE
       }
def initQ(expsettings,pcsettings=laptop):
    pcsettings.update(expsettings)
    return Settings(**pcsettings)
