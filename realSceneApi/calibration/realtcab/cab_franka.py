#!/usr/bin/python3.8

import pyrealsense2 as rs
import numpy as np
import cv2
import sys
sys.path.append("..")
#import matplotlib.pyplot as plt
from robot import Panda
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
        return Rvag
    
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



'''
if __name__=="__main__":
    cam=CameraL()
    num_record=0
    panda=Panda()
    while(True):
        color,depth = cam.get_data()
        cv2.imshow('vis', color)
        action=cv2.waitKey(1)
        if action & 0xFF == ord('q'):
            cv2.destroyAllWindows() 
            break
        if action & 0xFF == ord('s'):
            print("saving the {}-th data".format(num_record))
            current_pose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
            # time.sleep(1)
            np.save('./franka/H_{}.npy'.format(num_record), current_pose)
            cv2.imwrite('./franka/{}.jpg'.format(num_record), color)
            num_record+=1
'''
'''
if __name__ == "__main__":
    cam = CameraD400()
    panda = Panda()
    jointList = [[-0.16390678621279564, -0.6791972863269473, 0.16378642220873582, -2.494650543919316, 0.10657260429859161, 1.863999169005288, 0.7705911846210559],
                 [-0.747860636764451, -0.8281878829466498, 1.0318797409540117, -2.3930792641855043, 0.2860740681839982, 1.8532774123880598, 0.7908645075159377],
                 [-0.23978973600974976, -0.26543558388649235, -0.282111360968205, -2.3520484751316535, 0.27992621284723274, 1.8615258617603918, 0.22772078000066565],
                 [-0.3200761370750531, -1.266139867682206, 0.1446611231458994, -3.0304218578046225, 0.2911064845522245, 2.036694043362426, 0.457444145068656],
                 [-0.5073305669280521, -0.4878137231584181, 0.4762185306214449, -2.7689264097338833, 0.2874546642712036, 2.35799840742723, 0.51454026906651],]

    # jointList = [[-0.2552, -1.02, 0.1290, -2.06, 0.348, 1.942, 0.530],  # joint1, up
    #              [0.956, -1.022, -1.396, -1.57, 0.040, 1.878, -0.3762],  # joint2 right
    #              [-0.314, -1.01202, 0.180, -2.315, 0.401, 2.122, 0.589],  # joint3 front
    #              [-1.732, -1.747, 0.3881, -0.871, 1.2427, 1.302, 1.429],  # joint4 left
    #              ]
    calib = calibration(cam.mtx)
    pose_list = []
    for i, joint in enumerate(jointList):
        panda.moveJoint(joint)
        current_pose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
        print(current_pose)
        # time.sleep(1)
        color, depth = cam.get_data() # output rgb and depth image
        # while True:
        #     cv2.imshow('color', color)
        #     cv2.waitKey(1)
        #     color, depth = cam.get_data()  # output rgb and depth image

        calib.detectFeature(color)
        calib.pose_list.append(current_pose)

        # os.makedirs(calib_image_root, exist_ok=True)
        # os.makedirs(osp.join(calib_image_root, 'color'), exist_ok=True)
        # os.makedirs(osp.join(calib_image_root, 'depth'), exist_ok=True)
        # cv2.imwrite(osp.join(calib_image_root, 'depth', 'franka_%0.3d.png' % (i + 1)), depth)
        # cv2.imwrite(osp.join(calib_image_root, 'color', 'franka_%0.3d.png' % (i + 1)), color)
        # pose_list.append(current_pose)

    camT = calib.cal(optimize=True)
    # np.save(osp.join(calib_image_root,'pose_list.npy'), np.array(pose_list))
    np.save('config/franka_campose.npy',camT)

    pdb.set_trace()
'''
'''
  print("saving the {}-th data".format(num_record))
            current_pose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
            # time.sleep(1)
            np.save('./franka/H_{}.npy'.format(num_record), current_pose)
            cv2.imwrite('./franka/{}.jpg'.format(num_record), color)
            num_record+=1
'''
'''
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
'''

if __name__=="__main__":
    cam=CameraL()
    panda=Panda()
    #jointList = [[-0.16390678621279564, -0.6791972863269473, 0.16378642220873582, -2.494650543919316, 0.10657260429859161, 1.863999169005288, 0.7705911846210559],
    #             [-0.747860636764451, -0.8281878829466498, 1.0318797409540117, -2.3930792641855043, 0.2860740681839982, 1.8532774123880598, 0.7908645075159377],
    #             [-0.23978973600974976, -0.26543558388649235, -0.282111360968205, -2.3520484751316535, 0.27992621284723274, 1.8615258617603918, 0.22772078000066565],
    #             [-0.3200761370750531, -1.266139867682206, 0.1446611231458994, -3.0304218578046225, 0.2911064845522245, 2.036694043362426, 0.457444145068656],
    #             [-0.5073305669280521, -0.4878137231584181, 0.4762185306214449, -2.7689264097338833, 0.2874546642712036, 2.35799840742723, 0.51454026906651],]
    jointList = [[-0.29184415,0.31398767,0.07634848,-1.60790508,0.3141821,1.6415344,0.53429869],
                 [5.39674457e-02,2.49776054e-01,7.81049964e-02,-1.61017372e+00,-1.64782431e-05,1.73955496e+00,8.93200217e-01],
                 [0.4016288,0.44281331,0.07855819,-1.65690384,-0.45722737,1.82258037,1.46298679],
                 [0.41441133,0.41413299,-0.25937122,-2.05262694,0.09563554,2.40022035,0.96552527],
                 [0.42553192,-0.21836805,-0.29693422,-2.77981344,0.03619938,2.92524835,0.84064069],
                 [0.43966681,-0.28789059,-0.24778903,-1.93785587,-0.02333507,1.6070656,0.99800457],
                 [0.51711036,-0.87635069,-0.92735053,-2.53334855,-0.19870598,2.06143964,0.61170183],
                 [-0.66165546,-0.36291108,1.09610637,-2.3240342,0.01958286,2.15838297,1.27832223],
                 [0.00366497,-0.80523715,0.34523749,-3.02699471,0.20178118,2.58517056,0.93169011],]
    calib=calibration(cam.mtx)
    for i,joint in enumerate(jointList):
    #while True:
        panda.moveJoint(joint)
        color,depth=cam.get_data()
        cv2.imshow("vis",color)
        action=cv2.waitKey(-1)
        if action & 0xFF ==ord('q'):
            break
        if action & 0xFF ==ord('s'):
            print("saving the {}-th data".format(i))
            current_pose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
            #current_Joint = np.array(panda.robot.read_once().q)
            #print(current_Joint)
            np.save('./save/franka/H_{}.npy'.format(i), current_pose)
            cv2.imwrite('./save/franka/{}.jpg'.format(i), color)
        pass

    imgnum=len(jointList)

    for i in range(imgnum):
        temp=np.load('./save/franka/H_{}.npy'.format(i))
        imgtemp=cv2.imread('./save/franka/{}.jpg'.format(i))
        calib.images.append(imgtemp)
        calib.pose_list.append(temp)
        
    calib.detectAllFeature()
    print("======================Franka_hand_eye==============================")
    H_Fr2C=calib.cal()
    print(H_Fr2C)
    
    HF2GL=[]
    for i in range(len(calib.pose_list)):
        HF2GL.append(calib.pose_list[i]@H_Fr2C@calib.externMat[0])
    
    print(calib.averageTransformation(HF2GL))