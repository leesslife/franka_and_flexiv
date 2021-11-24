#!/usr/bin/python3.8

import numpy as np

if __name__=="__main__":
    flexiv=np.load('./save/flexiv2franka/flexiv2GoalH.npy')
    franka=np.load('./save/flexiv2franka/franka2GoalH.npy')
    
    #print(franka)
    #print(flexiv)
    #print(np.linalg.inv(flexiv))
    print("===========================franka2Flexiv===========================")
    H=franka@np.linalg.inv(flexiv)
    print(franka@np.linalg.inv(flexiv))
    np.save('./save/flexiv2franka/franka2flexiv.npy', H)
    #p1=np.array([-0.19492407,0.36433888,0.91063812,-0.05186166])
    #p2=np.array([-0.03899382,-0.65657185,0.11517781,1])
    #print(np.dot(p1,p2))
'''
    [[-0.98053607 -0.07122325 -0.18296521  0.67016549]
     [-0.00169941 -0.92877035  0.37065178  0.14848133]
     [-0.19633169  0.36374838  0.91057179 -0.05082125]
     [ 0.          0.          0.          1.        ]]
    #rd=np.array([1.0, 0.0, 0.0,  0.0,
    #             0.0, 1.0, 0.0,  0.12,
    #             0.0, 0.0, -1.0, 0.0,
    #            0.0, 0.00,0.0,1.0]).reshape(4,4)
    #print(np.dot(np.dot(franka,rd),np.linalg.inv(flexiv)))
    '''