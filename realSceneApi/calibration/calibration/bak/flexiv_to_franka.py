import cv2
import numpy as np
import glob


def rodrigues_trans2tr(rvec,tvec):
    r,_ =cv2.Rodrigues(rvec)
    tvec.shape=(3,)
    T=np.identity(4)
    T[0:3,3]=tvec
    T[0:3,0:3]=r 
    return T

def rodrigues_trans2trmat(tcp_r,tcp_t):
    H=np.zeros((4,4))
    H[:3,:3]=tcp_r
    H[:3,3]=tcp_t
    H[3,3]=1
    return H

class cabliration:
    def __init__(self,cornersize,objsize,criteria,robot_flag):
        self.cornersize=cornersize
        self.objsize=objsize
        self.objp=np.zeros((cornersize[0]*cornersize[1],3),np.float32)
        self.objp[:,:2]=np.mgrid[0:cornersize[0],0:cornersize[1]].T.reshape(-1,2)*objsize
        self.worldp=[]
        self.imgpoints=[]
        self.criteria=criteria
        self.tcp_r=[]
        self.tcp_t=[]
        self.robot_flag=robot_flag
    
    def init(self,imagepath):
        self.images=glob.glob(imagepath)
    
    def findcornerSubP(self):
        for fname in self.images:  #对所有图像进行处理
            img=cv2.imread(fname)
            cv2.imshow('findCorners',img)
            cv2.waitKey(1)
            self.gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            
            ret,corners=cv2.findChessboardCorners(img,self.cornersize,None,cv2.CALIB_CB_ADAPTIVE_THRESH)

            if ret==True:
                cv2.cornerSubPix(self.gray,corners,(5,5),(-1,-1),self.criteria)
                self.worldp.append(self.objp)
                self.imgpoints.append(corners)
                cv2.drawChessboardCorners(img,self.cornersize,corners,ret)
                cv2.imshow('findCorners',img)
                if cv2.waitKey(0)==ord('q'):
                    cv2.destroyAllWindows()
                    continue
    
    def cal_END2CAM(self):
        self.ret,self.mtx,self.dist,self.rvecs,self.tvecs=cv2.calibrateCamera(self.worldp,
                                                                              self.imgpoints,
                                                                              self.gray.shape[::-1],
                                                                              None,None)
    
    def cal_project_error(self):
        total_error=0
        for i in range(len(self.worldp)):
            imgpoints2,_=cv2.projectPoints(self.worldp[i],self.rvecs[i],self.tvecs[i],self.mtx,self.dist)
            error=cv2.norm(self.imgpoints[i],imgpoints2,cv2.NORM_L2)/len(imgpoints2)
            total_error+=error
        return total_error/len(self.worldp)

    def get_tcp_fromNPY(self,robot_flag):
        for i in range(len(self.rvecs)):
            if(robot_flag=="flexiv"):
                self.tcp_r.append(np.load('./flexiv/r_{}.npy'.format(i)))
                self.tcp_t.append(np.load('./flexiv/t_{}.npy'.format(i)))
            if(robot_flag=="franka"):
                temp=np.load('./franka/H_{}.npy'.format(i))
                r=temp[:3,:3]
                t=temp[:3,3]
                #print(r)
                #print(t)
                self.tcp_r.append(temp[:3,:3])
                self.tcp_t.append(temp[:3,3])

    
    def cal_eye_handM(self,robot_flag):
        HCAM2G=[]
        for i in range(len(self.rvecs)):
            tt=rodrigues_trans2tr(self.rvecs[i],self.tvecs[i])
            HCAM2G.append(tt)
        
        HCAM2G=np.array(HCAM2G)
        result_r,result_t=cv2.calibrateHandEye(self.tcp_r,self.tcp_t,HCAM2G[:,:3,:3],HCAM2G[:,:3,3])
        hand_to_eye_mat=np.zeros((4,4))
        hand_to_eye_mat[:3,:3]=result_r

        hand_to_eye_mat[:3,3]=result_t[:,0]
        hand_to_eye_mat[3,3]=1
        if(robot_flag=="flexiv"):
            np.save('./flexiv_hand_to_eye_mat.npy',hand_to_eye_mat)
        if(robot_flag=="franka"):
            np.save('./franka_hand_to_eye_mat.npy',hand_to_eye_mat)
        return hand_to_eye_mat


def cal_robot_cablibration(criteria,robot_flag):
    cab=cabliration((8,6),0.015,criteria,robot_flag)
    cab.init("./"+robot_flag+"/*.jpg")
    cab.findcornerSubP()
    cab.cal_END2CAM()
    error=cab.cal_project_error()
    print("error is:::",error)
    cab.get_tcp_fromNPY(robot_flag)
    franka_hand_to_eye_mat=cab.cal_eye_handM(robot_flag)
    #print(franka_hand_to_eye_mat)
    return franka_hand_to_eye_mat,cab.rvecs,cab.tvecs,cab.tcp_r,cab.tcp_t

def cal_C2GH_vector(rvecs,tvecs):
    H_vector=[]
    for i in range(len(rvecs)):
        H=rodrigues_trans2tr(rvecs[i],tvecs[i])
        H_vector.append(H)
    return H_vector

def cal_W2EH_vector(tcp_r,tcp_t):
    H_vector=[]
    for i in range(len(tcp_r)):
        H=rodrigues_trans2trmat(tcp_r[i],tcp_t[i])
        H_vector.append(H)
    return H_vector

def base2base(flexiv_hand_to_eye_mat,
              flexiv_C2GH,
              flexiv_W2EH,
              franka_hand_to_eye_mat,
              franka_C2GH,
              franka_W2EH):
    flexiv_eye_to_hand_mat=np.linalg.inv(flexiv_hand_to_eye_mat)
    #print("=======================flexiv_eye_to_hand_mat======================")
    #print(flexiv_eye_to_hand_mat)
    flexiv_E2WH=np.linalg.inv(flexiv_W2EH)
    
    flexiv_G2CH=np.linalg.inv(flexiv_C2GH)
    #print("============================flexiv_G2CH============================")
    #print(flexiv_G2CH)

    freye2fleyeH=np.dot(franka_C2GH,flexiv_G2CH)
    #print("===========================freye2fleyeH============================")
    #print(freye2fleyeH)
    #franka_eye_to_hand_mat=np.linalg.inv(flexiv_hand_to_eye_mat)
    #H1=np.dot(franka_W2EH,franka_hand_to_eye_mat)  # franka world to franka eye
    H1=np.dot(flexiv_W2EH,flexiv_hand_to_eye_mat)  # franka world to franka eye
    print("===========================H1============================")
    print(H1)
    #franka_G2CH=np.linalg.inv(franka_C2GH)
    H2=np.dot(H1,flexiv_C2GH)
    print("===========================H2============================")
    print(H2)
    #H2=np.dot(H1,freye2fleyeH)                     # franka eye to flexiv eye
    #H3=np.dot(H2,flexiv_eye_to_hand_mat)           # flexiv eye to flexive end
    #H4=np.dot(H3,flexiv_E2WH)                      # flexiv end to flexiv world  
    #print("===========================H============================")
    #print(H4)
    return H1                                      # return franka_world to flexiv world

    
def cal_franka_to_flexiv(flexiv_hand_to_eye_mat,
                         flexiv_C2GH_vector,
                         flexiv_W2EH_vector,
                         franka_hand_to_eye_mat,
                         franka_C2GH_vector,
                         franka_W2EH_vector):
    pass

if __name__=="__main__":
    criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
    flexiv_hand_to_eye_mat,flexiv_rvecs,flexiv_tvecs,flexiv_tcpr,flexiv_tcpt=cal_robot_cablibration(criteria,"flexiv")  #flexiv hand_to_eye H Matrix
    print("====================flexiv_hand_toeye_mat=====================")
    print(flexiv_hand_to_eye_mat)
    flexiv_C2GH_vector=cal_C2GH_vector(flexiv_rvecs,flexiv_tvecs)                 #flexiv_H      cam_to_goal
    print("======================flexiv_C2GH_vector=========================")
    print(flexiv_C2GH_vector[0])
    flexiv_W2EH_vector=cal_W2EH_vector(flexiv_tcpr,flexiv_tcpt)                # world to end for each image 
    print("========================flexiv_W2EH===========================")
    print(flexiv_W2EH_vector[0])

    franka_hand_to_eye_mat,franka_rvecs,franka_tvecs,franka_tcpr,franka_tcpt=cal_robot_cablibration(criteria,"franka")  #franka hand_to_eye H Matrix
    print("====================franka_hand_toeye_mat=====================")
    print(franka_hand_to_eye_mat)
    franka_C2GH_vector=cal_C2GH_vector(franka_rvecs,franka_tvecs)                 #franka_H      cam_to_goal
    print("======================franka_C2GH_vector=========================")
    print(franka_C2GH_vector[0])
    franka_W2EH_vector=cal_W2EH_vector(franka_tcpr,franka_tcpt)                # world to end for each image 
    print("========================franka_W2EH===========================")
    print(franka_W2EH_vector[0])
    
    f2fl=base2base(flexiv_hand_to_eye_mat,
                   flexiv_C2GH_vector[0],
                   flexiv_W2EH_vector[0],
                   franka_hand_to_eye_mat,
                   franka_C2GH_vector[0],
                   franka_W2EH_vector[0])
    print(f2fl)
    '''
    f2fl2=base2base(flexiv_hand_to_eye_mat,
                   flexiv_C2GH_vector[1],
                   flexiv_W2EH_vector[1],
                   franka_hand_to_eye_mat,
                   franka_C2GH_vector[1],
                   franka_W2EH_vector[1])
    #print(f2fl2)

    f2fl2=base2base(flexiv_hand_to_eye_mat,
                   flexiv_C2GH_vector[2],
                   flexiv_W2EH_vector[2],
                   franka_hand_to_eye_mat,
                   franka_C2GH_vector[2],
                   franka_W2EH_vector[2])
    #print(f2fl2)

    f2fl2=base2base(flexiv_hand_to_eye_mat,
                   flexiv_C2GH_vector[3],
                   flexiv_W2EH_vector[3],
                   franka_hand_to_eye_mat,
                   franka_C2GH_vector[3],
                   franka_W2EH_vector[3])
    #print(f2fl2)
    f2fl2=base2base(flexiv_hand_to_eye_mat,
                   flexiv_C2GH_vector[4],
                   flexiv_W2EH_vector[4],
                   franka_hand_to_eye_mat,
                   franka_C2GH_vector[4],
                   franka_W2EH_vector[4])
    '''