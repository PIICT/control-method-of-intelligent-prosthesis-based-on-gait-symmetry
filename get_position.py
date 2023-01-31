import numpy as np
import math
from math import sqrt, pow, acos

from scipy.spatial.transform import Rotation as R

def get_bvh_data(root,index):
    context=open('%s%d.bvh'%(root,index)).read().split('\n')
    offsets=[]
    Euler_angle=[]
    tot=0
    while True:
        if tot==7:
            break
        for item in context:
            if tot==7:
                break
            if ('OFFSET' in item) and ('.' in item):
                tot+=1
                item=item.split(' ')
                cur=[]
                for i in item:
                    if '.' in i:
                        cur.append(float(i))
                offsets.append(cur)
    offsets=offsets[1:]

    for i in range(351,len(context)):
        if len(context[i])>10:
            d=context[i].split(' ')
            cur=[]
            for j in range(24):
                cur.append(float(d[j]))
            Euler_angle.append(cur)

    print(offsets)
    print(len(Euler_angle))
    for i in range(10):
        print(Euler_angle[i])

    return offsets,Euler_angle

def get_pos(offsets,Euler_angle):
    # 计算每个点的笛卡尔坐标
    joint_Cartesian_coordinates={'Hips':[],'RightUpLeg':[],'RightLeg':[],'RightFoot':[],'LeftUpLeg':[],'LeftLeg':[],'LeftFoot':[]}

    for i in range(len(Euler_angle)):
        #坐标的顺序是XYZ
        joint_Cartesian_coordinates['Hips'].append(np.array(Euler_angle[i][:3]))
        #欧拉角顺序是YXZ
        #泰勒布莱恩角旋转顺序，每次旋转的轴不同
        euler_Hip=Euler_angle[i][3:6]

        euler_RightUpLeg=Euler_angle[i][6:9]
        euler_RightLeg=Euler_angle[i][9:12]
        euler_RightFoot=Euler_angle[i][12:15]

        euler_LeftUpLeg=Euler_angle[i][15:18]
        euler_LeftLeg=Euler_angle[i][18:21]
        euler_LeftFoot=Euler_angle[i][21:]

        #计算旋转矩阵
        #注意看是内部旋转还是外在旋转，现在用的是内在旋转
        #子节点的旋转矩阵=父节点的旋转矩阵x子节点相对父节点的旋转矩阵
        r_hip=R.from_euler('yxz',euler_Hip,degrees=True).as_matrix()

        r_RightUpLeg=np.dot(r_hip,R.from_euler('yxz',euler_RightUpLeg,degrees=True).as_matrix())
        r_RightLeg=np.dot(r_RightUpLeg,R.from_euler('yxz',euler_RightLeg,degrees=True).as_matrix())
        r_RightFoot=np.dot(r_RightLeg,R.from_euler('yxz',euler_RightFoot,degrees=True).as_matrix())

        r_LeftUpLeg=np.dot(r_hip,R.from_euler('yxz',euler_LeftUpLeg,degrees=True).as_matrix())
        r_LeftLeg=np.dot(r_LeftUpLeg,R.from_euler('yxz',euler_LeftLeg,degrees=True).as_matrix())
        r_LeftFoot=np.dot(r_LeftLeg,R.from_euler('yxz',euler_LeftFoot,degrees=True).as_matrix())


        #子节点的坐标=父节点的坐标+子节点的偏置×子节点的旋转矩阵
        joint_Cartesian_coordinates['RightUpLeg'].append(joint_Cartesian_coordinates['Hips'][-1]+np.dot(np.array(offsets[0]),r_RightUpLeg))
        joint_Cartesian_coordinates['RightLeg'].append(joint_Cartesian_coordinates['RightUpLeg'][-1]+np.dot(np.array(offsets[1]),r_RightLeg))
        joint_Cartesian_coordinates['RightFoot'].append(joint_Cartesian_coordinates['RightLeg'][-1]+np.dot(np.array(offsets[2]),r_RightFoot))

        joint_Cartesian_coordinates['LeftUpLeg'].append(joint_Cartesian_coordinates['Hips'][-1]+np.dot(np.array(offsets[0]),r_LeftUpLeg))
        joint_Cartesian_coordinates['LeftLeg'].append(joint_Cartesian_coordinates['LeftUpLeg'][-1]+np.dot(np.array(offsets[1]),r_LeftLeg))
        joint_Cartesian_coordinates['LeftFoot'].append(joint_Cartesian_coordinates['LeftLeg'][-1]+np.dot(np.array(offsets[2]),r_LeftFoot))


    return joint_Cartesian_coordinates

def angle_of_vector(v1, v2):
    pi = 3.1415
    vector_prod = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    length_prod = sqrt(pow(v1[0], 2) + pow(v1[1], 2) + pow(v1[2],2)) * sqrt(pow(v2[0], 2) + pow(v2[1], 2) + pow(v2[2],2))
    cos = vector_prod * 1.0 / (length_prod * 1.0 + 1e-6)
    return (acos(cos) / pi) * 180


def get_angle(joint_Cartesian_coordinates):
    angle=[]
    for i in range(len(joint_Cartesian_coordinates['Hips'])):
        cur=[]
        v1=joint_Cartesian_coordinates['RightLeg'][i]- joint_Cartesian_coordinates['RightUpLeg'][i]
        v2=joint_Cartesian_coordinates['RightFoot'][i]- joint_Cartesian_coordinates['RightLeg'][i]

        v3=joint_Cartesian_coordinates['LeftLeg'][i]- joint_Cartesian_coordinates['LeftUpLeg'][i]
        v4=joint_Cartesian_coordinates['LeftFoot'][i]- joint_Cartesian_coordinates['LeftLeg'][i]

        cur.append(angle_of_vector(v1,v2))
        cur.append(angle_of_vector(v3,v4))

        angle.append(cur)
    return angle

if __name__=='__main__':
    for i in range(103,111):
        print(i)
        offsets,Euler_angle=get_bvh_data('../data/BVH/',i)
        print(1)
        joint_Cartesian_coordinates=get_pos(offsets,Euler_angle)
        print(2)
        angle=get_angle(joint_Cartesian_coordinates)
        print(3)
        with open('../result/0621/BVH/%d.txt'%(i),'w') as f:
            for item in angle:
                f.write("%f\t%f\n"%(item[0],item[1]))
