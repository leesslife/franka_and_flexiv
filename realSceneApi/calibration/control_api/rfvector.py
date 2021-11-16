import numpy as np
import math


__all__ =[
    "compute_norm",
    "compute_quat_u2v",
    "normalize",
]

def compute_norm(degree_diff):
    #result=0.0
    #for x in np.nditer(degree_diff):
    #    result+=x*x
    #return math.sqrt(result)
    return np.linalg.norm(degree_diff)



    # u source_matrix base  u[0]=a1 u[1]=a2 u[2]=a3
    # v target_matrix_base  v[0]=b1 v[1]=b2 v[2]=b3
    # axb=>  轴
    # c1=a2b3-a3b2    
    # c2=a3b1-a1b3
    # c3=a1b2-a2b1
    # cos.theta=P.Q/|P||Q| 角度
    # q=[cos(theta/2),nx.sin(theta/2),ny.sin(theta/2),nz.sin(theta/2)]^T
    # q_d=[w,x,y,z]
def compute_quat_u2v(u,v):
    c1=u[1]*v[2]-u[2]*v[1]
    c2=u[2]*v[0]-u[0]*v[2]
    c3=u[0]*v[1]-u[1]*v[0]
    nx=c1/math.sqrt(c1*c1+c2*c2+c3*c3)
    ny=c2/math.sqrt(c1*c1+c2*c2+c3*c3)
    nz=c3/math.sqrt(c1*c1+c2*c2+c3*c3)
    theta=math.acos(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)))
    half_theta=theta/2
    w=math.cos(half_theta)
    x=math.sin(half_theta)*nx
    y=math.sin(half_theta)*ny
    z=math.sin(half_theta)*nz 
    return np.array([w,x,y,z])


#Normlize(q)=q/Norm(q)=q / sqrt(w2 + x2 + y2 + z2)
def normalize(q):
    return q/np.linalg.norm(q)

    

#a=np.arange(3)

#print(compute_norm(a))

#def generate_random_pose(flag="vec"):
    """Generate random pose.

    Returns:
        pose: 4x4 or 7D numpy array.
    """
#    pos = np.random.rand(3) - 0.5   #np.random.rand(d0,d1,d2……dn) 返回一个均匀分布的矩阵
#    u = np.random.rand(3) - 0.5     #(-0.5,0.5)
#    v = np.random.rand(3) - 0.5
#    quat = compute_quat_u2v(u, v)
#    return quat

#x=generate_random_pose()
#print(x)
#x[3]=100
#print(x)
#print(compute_norm(generate_random_pose()))
#y=normalize(x)
#print(y)
#print(compute_norm(y))