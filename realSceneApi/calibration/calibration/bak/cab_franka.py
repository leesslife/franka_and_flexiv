import cv2
import numpy as np
import glob

def rodrigues_trans2tr(rvec, tvec):
        r, _ = cv2.Rodrigues(rvec)
        tvec.shape = (3,)
        T = np.identity(4)
        T[0:3, 3] = tvec
        T[0:3, 0:3] = r
        return T

# 找棋盘格角点
# 阈值

criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
print(criteria)

#棋盘格模板规格
w=6
h=8
# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵

objp=np.zeros((w*h,3),np.float32)
#print(np.mgrid[0:w,0:h].T.reshape(-1,2))
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)*0.015
# 储存棋盘格角点的世界坐标和图像坐标对

worldobjpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维像素点？

images = glob.glob('franka/*.jpg') #读取图像

for fname in images:  #对所有图像进行处理
    img = cv2.imread(fname)
    cv2.imshow('findCorners',img)
    cv2.waitKey(1)
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    ret,corners=cv2.findChessboardCorners(img,(w,h),None,cv2.CALIB_CB_ADAPTIVE_THRESH)
    #找到棋盘角点
    #ret,corners=cv2.findChessboardCorners(img,(w,h),None)
  
    print(ret)
   
    #如果找到足够的点对，将其存储起来
    if ret==True:
        cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
        worldobjpoints.append(objp)
        imgpoints.append(corners)
        print(len(objp))
        print(len((corners)))
        cv2.drawChessboardCorners(img,(w,h),corners,ret)
        cv2.imshow('findCorners',img)
        if cv2.waitKey(0)==ord('s'):
            cv2.destroyAllWindows()
            continue


for i in range(len(imgpoints)):
    print("world obj point================")
    print(worldobjpoints[i])
    print("img obj point================")
    print(imgpoints[i])


cv2.destroyAllWindows()
print(gray.shape)
ret,mtx,dist,rvecs,tvecs=cv2.calibrateCamera(worldobjpoints,
                                             imgpoints,
                                             gray.shape[::-1],
                                             None,None)
#print(("ret:"),ret)
#print (("mtx:\n"),mtx)        # 内参数矩阵
#print (("dist:\n"),dist)      # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
#print (("rvecs:\n"),rvecs)    # 旋转向量  # 外参数
#print (("tvecs:\n"),tvecs)    # 平移向量  # 外参数

# 去畸变
img2=cv2.imread('franka/1.jpg')
h,w=img2.shape[:2]
newcameramtx,roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h)) #自由比例
dst=cv2.undistort(img2,mtx,dist,None,newcameramtx)
cv2.imwrite('calibresult.jpg',dst)


# 反投影误差
total_error=0
for i in range(len(worldobjpoints)):
    imgpoints2,_=cv2.projectPoints(worldobjpoints[i],rvecs[i],tvecs[i],mtx,dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error+=error

print(("total error: "),total_error/len(worldobjpoints))



# reading the tcp poses
tcp_r = []
tcp_t = []
for i in range(len(rvecs)):
    temp=np.load('./franka/H_{}.npy'.format(i))
    tcp_r.append(temp[:3,:3])
    tcp_t.append(temp[:3,3])
    #tcp_t.append(np.load('./flexiv/t_{}.npy'.format(i)))


#print(tcp_r[0].shape,":::",type(tcp_r))
#print(tcp_t[0].shape,":::",type(tcp_t))
result_r = np.zeros((3, 3))
result_t = np.zeros((3, 1))

#a
#r_compute=[]
#for i in range(len(rvecs)):
#    r_compute.append(cv2.Rodrigues(rvecs[i]))
    
#print(tcp_r[0])
#print(rvecs[0].shape)
#print(tcp_t[0].shape)
Hg2c=[]
for i in range(len(rvecs)):
    tt=rodrigues_trans2tr(rvecs[i],tvecs[i])
    Hg2c.append(tt)

Hg2c=np.array(Hg2c)
#for i in range(len(tcp_t)):
#    r,j=cv2.Rodrigues(tcp_r[i])
#    trvecs.append(r)
#    ttvecs.append(np.array(tcp_t[i]))
 
result_r,result_t=cv2.calibrateHandEye(tcp_r,tcp_t,Hg2c[:,:3,:3],Hg2c[:,:3,3])
#print(result_r)
#print(result_t)

eye_to_hand_mat = np.zeros((4, 4))
eye_to_hand_mat[:3, :3] = result_r
eye_to_hand_mat[:3, 3] = result_t[:, 0]
eye_to_hand_mat[3, 3] = 1.
print("============eye_to_hand_mat===========")
print(eye_to_hand_mat)
np.save('./eye_to_hand_mat.npy', eye_to_hand_mat)

#手眼矩阵 end_to_cam
#计算手眼转换矩阵误差
norm=0.0

base_to_end_mat1=np.zeros((4,4))
base_to_end_mat1[:3,:3]=tcp_r[0]
base_to_end_mat1[:3,3]=tcp_t[0].T
print(tcp_t[0].T)
base_to_end_mat1[3,3]=1

base_to_end_mat2=np.zeros((4,4))
base_to_end_mat2[:3,:3]=tcp_r[1]
base_to_end_mat2[:3,3]=tcp_t[1].T
print(tcp_t[1].T)
base_to_end_mat2[3,3]=1

print("=====================")
cam_to_calTarget1=np.zeros((4,4))
R,J=cv2.Rodrigues(rvecs[0])
cam_to_calTarget1[:3,:3]=np.array(R)
cam_to_calTarget1[:3,3]=tvecs[0].T
print(tvecs[0])
cam_to_calTarget1[3,3]=1
cam_to_calTarget1_r=cam_to_calTarget1.T
print(cam_to_calTarget1)
print("=====================")


print("=====================")
cam_to_calTarget2=np.zeros((4,4))
R2,J2=cv2.Rodrigues(rvecs[1])
cam_to_calTarget2[:3,:3]=np.array(R2)
cam_to_calTarget2[:3,3]=tvecs[1].T
print(tvecs[1])
cam_to_calTarget2[3,3]=1
cam_to_calTarget2_r=cam_to_calTarget2.T
print(cam_to_calTarget2)
print("=====================")


A=np.dot(base_to_end_mat2.T,base_to_end_mat1)
B=np.dot(cam_to_calTarget2,cam_to_calTarget1.T)
print("=================A====================")
print(A)
print("=================B====================")
print(B)

L=A@eye_to_hand_mat-eye_to_hand_mat@B

print(np.linalg.norm(L))

for i in range(len(tcp_t)):

    pass

