#!/usr/bin/python3.6

import pyrealsense2 as rs
import numpy as np
import cv2
import sys
sys.path.append("..")
#import matplotlib.pyplot as plt
from control_api import *
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
                cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1),self.criteria)
                corners2=corners
            else:
                corners2=cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1),self.criteria)
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
    
    def averageRotation(self,R):
        n=len(R)
        tmp=np.zeros((3,3),dtype=R[0].dtype)
        for i in range(n):
            tmp=tmp+R[i]
        Rbarre=tmp/n
        RTR=np.dot(Rbarre.T,Rbarre)   # 把所有图片的旋转矩阵的batch_size 通道加起来，再求他们的ATA
        [D,V]=np.linalg.eig(RTR)  # 计算出特征值和特征分解 D，特征值，V特征向量
        D=np.diag(np.flipud(D))   # 将特征矩阵上下反转，如果一维向量，还是左右
        V=np.fliplr(V)
        sqrtD=np.sqrt(np.linalg.inv(D))  # 对特征举证的各项求倒数，然后求平方根
        if np.linalg.det(Rbarre[0:3,0:3])<0:
            sqrtD[2,2]=-1*sqrtD[2,2]
        temp=np.dot(V,sqrtD)           # 乘以新生成的V*sqartD 
        temp=np.dot(temp,V.T)          # V*sqartD&V.T
        Rvag=np.dot(Rbarre,temp)       # Rbarre*V*sqartD&V.T
        return Rvag                    # 他这一顿操作之后 不是R@RT@R=R？
    
    def averageTransformation(self,H):
        '''
        :param H: [h] list, h is 4*4 pose.(np.array())
        :return: average pose h
        '''
        n=len(H)
        Ravg=[]
        for i in range(n):
            Ravg.append(H[i][0:3,0:3])
        R=self.averageRotation(Ravg)
        Tavg = 0
        for i in range(n):
            Tavg+=H[i][0:3,3]
        Havg=np.zeros((4,4))
        Havg[0:3,0:3]=R
        Havg[0:3,3]=Tavg/n
        Havg[3,3]=1
        return Havg

def get_pos_rot_from_xyzq(xyzq):
    pos = np.array([xyzq[0], xyzq[1], xyzq[2]])     #x,y,z
    rot = batch_quat_to_mat(np.array([[xyzq[3],xyzq[4], xyzq[5], xyzq[6]]]))[0]   # w,x,y,z
    return pos, rot 

if __name__=="__main__":
    cam=CameraL()
    flexiv = FlexivRobot("192.168.2.100","192.168.2.200")
    #print(flexiv.get_joint_pos())
    jointList = [[ 0.57415557,-0.63808942,-0.36224535,1.06023717,-0.07712983,0.05254694,1.64508402],
                 [ 0.36230326,-0.5789426 ,-0.38528445,1.06835866,0.20858671 ,0.09320185,1.64508605],
                 [ 0.08745624,-0.59872097,-0.35263091,1.08790219,0.39574349 ,0.08156291,1.24172497],
                 [ 0.44913852,-0.05271466,0.07169211 ,1.7189635 ,-0.26682037,0.24607733,1.98834431],
                 [ 0.05322517,-0.20821288,-0.21772449,1.42384195,0.20912233 ,0.13523588,1.4372431 ],
                 [ 0.26435652,-0.63727915,-0.15906759,1.32645059,-0.00560286,0.43366978,1.63151181],
                 [ 0.05058723,-0.60423452,-0.17162758,1.30044985,0.2946589  ,0.38322628,1.35885429],
                 [ 0.25154066,-0.47618136,-0.10336179,1.98862803,-0.00307886,1.27898288,1.65889716],
                 [ 0.18193814,-0.75773472,-0.09209981,1.4511745 ,0.09069394 ,0.64562631,1.60264587],]
   
    calib=calibration(cam.mtx,pattern_size=(8,6))

    for i,joint in enumerate(jointList):
        array=np.array(joint)
        flexiv.move_jnt_pos(array,
                            max_jnt_vel=[12, 12, 14, 14, 28, 28, 28],
                            max_jnt_acc=[7.2, 7.2, 8.4, 8.4, 16.8, 16.8, 16.8])

        color,depth=cam.get_data()
        cv2.imshow("vis",color)
        action=cv2.waitKey(-1)
        if action & 0xFF ==ord('q'):
            break
        if action & 0xFF ==ord('s'):
            print("saving the {}-th data".format(i))
            pos, rot = get_pos_rot_from_xyzq(flexiv.get_tcp_pose())
            np.save('./save/flexiv/t_{}.npy'.format(i), pos)
            np.save('./save/flexiv/r_{}.npy'.format(i), rot)
            cv2.imwrite('./save/flexiv/{}.jpg'.format(i), color)
            continue
    array_s=np.array([ 0.55921108,-0.46804309,0.13940856 ,1.26770318,-0.00258606,0.62576026,1.60263598])
    flexiv.move_jnt_pos(array_s,
                        max_jnt_vel=[12, 12, 14, 14, 28, 28, 28],
                        max_jnt_acc=[7.2, 7.2, 8.4, 8.4, 16.8, 16.8, 16.8])

    imgnum=len(jointList)

    for i in range(imgnum):
        temp_r=np.load('./save/flexiv/r_{}.npy'.format(i))
        print(temp_r)
        temp_t=np.load('./save/flexiv/t_{}.npy'.format(i))
        temp=rodrigues_trans2trmat(temp_r,temp_t)
        imgtemp=cv2.imread('./save/flexiv/{}.jpg'.format(i))
        calib.images.append(imgtemp)
        calib.pose_list.append(temp)
  
    calib.detectAllFeature()
    print("======================Flexiv_hand_eye==============================")
    HFlH2C=calib.cal()
    np.save('./save/flexiv2franka/flexivH2E.npy',HFlH2C)
    print(HFlH2C)

    print("======================Flexiv_to_Goal==============================")
    HF2GL=[]
    for i in range(len(calib.pose_list)):
        HF2GL.append(calib.pose_list[i]@HFlH2C@calib.externMat[i])
    flexiv2GoalH=calib.averageTransformation(HF2GL)
    
    np.save('./save/flexiv2franka/flexiv2GoalH.npy',flexiv2GoalH)
    print(flexiv2GoalH)