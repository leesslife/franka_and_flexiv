import pyrealsense2 as rs
import numpy as np
import cv2
import sys
sys.path.append("..")
#import matplotlib.pyplot as plt
#from robot import Panda
from cam import CameraL

def rodrigues_trans2trmat(tcp_r,tcp_t):
    H=np.zeros((4,4))
    H[:3,:3]=tcp_r
    H[:3,3]=tcp_t
    H[3,3]=1
    return H

class calibration(object):
    def __init__(self,mtx,pattern_size=(8,6),square_size=0.015,hand_eye="EIH"):
        self.mtx=mtx                                 #内部参数
        self.pattern_size=pattern_size
        self.square_size=square_size
        self.hand_eye=hand_eye
        self.pose_list=[]
        self.init_calib()   
        self.externMat=[]
    
    def init_calib(self):
        self.objp=np.zeros((self.pattern_size[0]*self.pattern_size[1],3),np.float32)
        self.objp[:,:2]=self.square_size*np.mgrid[0:self.pattern_size[0],0:self.pattern_size[1]].T.reshape(-1,2)
        # 这里又换了回来！
        for i in range(self.pattern_size[0]*self.pattern_size[1]):
            x,y=self.objp[i,0],self.objp[i,1]
            self.objp[i,0],self.objp[i,1]=y,x 
        # Arrays to store object points and image points from all the images
        self.objpoints=[]
        self.imgpoints=[]
        self.images=[]
        self.criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)

    def detectFeature(self,color,show=True):
        img=color
        self.gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        # 找出焦点
        ret,corners=cv2.findChessboardCorners(img,self.pattern_size,None,
                                              cv2.CALIB_CB_ADAPTIVE_THRESH)
        # 获取亚像素点
        if ret==True:
            self.objpoints.append(self.objp)
            corners2=corners
            if(cv2.__version__).split('.')=='2':
                cv2.cornerSubPix(self.gray,corners,(5,5),(-1,-1),self.criteria)
                corners2=corners
            else:
                corners2=cv2.cornerSubPix(self.gray,corners,(5,5),(-1,-1),self.criteria)
        # 得到角点的列表
        self.imgpoints.append(corners2)
        if show:
            # 画出角点来进行观察
            cv2.drawChessboardCorners(img,self.pattern_size,corners2,ret)
            cv2.imshow('findCorners',img)
            if cv2.waitKey(0)==ord('s'):
                cv2.destroyAllWindows()

    
    def detectAllFeature(self):
        for i in range(len(self.images)):
            #cv2.imshow("display",self.images[i])
            self.detectFeature(self.images[i])

    def rodrigues_trans2tr(self,rvec, tvec):
        r, _ = cv2.Rodrigues(rvec)
        tvec.shape = (3,)
        T = np.identity(4)
        T[0:3, 3] = tvec
        T[0:3, 0:3] = r
        return T

    def cal(self,optimize=False):
        # 找出外部矩阵与内部矩阵
        ret,mtx,dist,rvecs,tvecs=cv2.calibrateCamera(self.objpoints,self.imgpoints,self.gray.shape[::-1],self.mtx,None)
        Hg2c=[]
        pose_list=np.array(self.pose_list)  # 这张列表里面为 w*h* shape(H)的矩阵
        for i in range(len(rvecs)):
            #tt=self.rodrigues_trans2tr(rvecs[i],tvecs[i]/1000.)
            tt=self.rodrigues_trans2tr(rvecs[i],tvecs[i])   # 将旋转举证与位移举证变成H举证
            Hg2c.append(tt)                                 
            self.externMat.append(tt)

        Hg2c=np.array(Hg2c)
        
        rot,pos=cv2.calibrateHandEye(pose_list[:,:3,:3],pose_list[:,:3,3],Hg2c[:,:3,:3],Hg2c[:,:3,3],method=cv2.CALIB_HAND_EYE_PARK)
        camT=np.identity(4)
        camT[:3,:3]=rot
        camT[:3,3]=pos[:,0]
        Hc2m=camT
        return Hc2m
    
                
if __name__=="__main__":
    cam=CameraL()
    calib=calibration(cam.mtx)

    flexivimagnum=10
    
    for i in range(10):
        temp=np.load('./franka/H_{}.npy'.format(i))
        imgtemp=cv2.imread('./franka/{}.jpg'.format(i))
        calib.images.append(imgtemp)
        calib.pose_list.append(temp)
    
    calib.detectAllFeature()
    print("======================Franka_hand_eye==============================")
    H11=calib.cal()
    print(H11)
    H12=calib.pose_list[0]@H11@calib.externMat[0]
    print("======================Franka2G==============================")
    print(H12)

    calib2=calibration(cam.mtx,pattern_size=(6,8))
    for i in range(10):
        temp_r=np.load('./flexiv/r_{}.npy'.format(i))
        temp_t=np.load('./flexiv/t_{}.npy'.format(i))
        temp=rodrigues_trans2trmat(temp_r,temp_t)
        #print(temp)
        imgtemp=cv2.imread('./flexiv/{}.jpg'.format(i))
        calib2.images.append(imgtemp)
        calib2.pose_list.append(temp)
    
    calib2.detectAllFeature()
    print("======================Flexiv_hand_eye==============================")
    H21=calib2.cal()
    print(H21)
    H22=calib2.pose_list[0]@H21@calib2.externMat[0]
    print("======================Flexiv2Goal==============================")
    print(H22)

    print("======================frankabase2Flexivbase==============================")
    print(np.linalg.inv(H22))

    #rd=np.array([[0    ,1.0 ,0.0  ,0.0 ],
    #             [-1.0 ,0.0 ,0.0  ,0.12],
    #             [0.0  ,0.0 ,1.0  ,0.0 ],
    #             [0.0  ,0,0 ,0.0  ,1.0 ]],dtype=float)
    rd=np.array([0,1.0,0.0,0.0,
                -1.0,0.0,0.0,0.12,
                0.0,0.0,1.0,0.0,
                0.0,0.00,0.0,1.0]).reshape(4,4)
    #print(rd)

    print(np.dot(np.dot(H12,rd),np.linalg.inv(H22)))
