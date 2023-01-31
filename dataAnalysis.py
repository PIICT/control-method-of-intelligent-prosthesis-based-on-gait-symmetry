from bvh import *
from math import sqrt, pow, acos
import csv

def angle_of_vector(v1, v2):
    pi = 3.1415926
    vector_prod = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    length_prod = sqrt(pow(v1[0], 2) + pow(v1[1], 2) + pow(v1[2],2)) * sqrt(pow(v2[0], 2) + pow(v2[1], 2) + pow(v2[2],2))
    cos = vector_prod * 1.0 / (length_prod * 1.0 + 1e-6)
    return (acos(cos) / pi) * 180

def get_bvh_data(root,index):
    file_path="%s%d.bvh"%(root,index)
    print(index,'.bvh')
    anim=Bvh()
    anim.parse_file(file_path)
    res=[]
    for i in range(anim.frames):
        positions, rotations = anim.frame_pose(i)
        res.append(positions)
    return res

def get_angle(positions):
    angle=[]
    for i in range(len(positions)):
        cur=[]
        #右腿
        v1=positions[i][1]- positions[i][2]
        v2=positions[i][2]- positions[i][3]

        #左腿
        v3=positions[i][5]- positions[i][6]
        v4=positions[i][6]- positions[i][7]

        cur.append(angle_of_vector(v1,v2))
        cur.append(angle_of_vector(v3,v4))
        angle.append(cur)
    return angle

def get_height(positions):
    # 看ankle、hip、spine的高度变化
    # [Hip, RightUpLeg,RightLeg, RightFoot, LeftUpLeg, LeftLeg, LeftFoot, Spine, Spine1, Spine2]
    height=[]
    for i in range(len(positions)):
        cur=[]
        for j in range(4):
            cur.append(positions[i][j][1])
        for j in range(3):
            cur.append(positions[i][5+j][1])
        for j in range(3):
            cur.append(positions[i][9+j][1])
        height.append(cur)
    return height

def get_pos(positions):
    pos=[]
    # [Hip, RightUpLeg,RightLeg, RightFoot, LeftUpLeg, LeftLeg, LeftFoot, Spine, Spine1, Spine2]
    for i in range(len(positions)):
        cur=[]
        for j in range(4):
            for k in range(3):
                cur.append(positions[i][j][k])
        for j in range(3):
            for k in range(3):
                cur.append(positions[i][5+j][k])
        for j in range(3):
            for k in range(3):
                cur.append(positions[i][9+j][k])
        pos.append(cur)
    return pos

def get_hip_angle(positions):
    angle=[]
    for i in range(len(positions)):
        cur=[]
        #右腿
        v1=positions[i][0]- positions[i][1]
        v2=positions[i][1]- positions[i][2]

        #左腿
        v3=positions[i][0]- positions[i][5]
        v4=positions[i][5]- positions[i][6]

        cur.append(angle_of_vector(v1,v2))
        cur.append(angle_of_vector(v3,v4))
        angle.append(cur)
    return angle

def get_spine_thigh(positions):
    angle=[]
    for i in range(len(positions)):
        cur=[]
        #右腿
        v1=positions[i][9]- positions[i][0]
        v2=positions[i][1]- positions[i][2]

        #左腿
        v3=positions[i][9]- positions[i][0]
        v4=positions[i][5]- positions[i][6]

        cur.append(angle_of_vector(v1,v2))
        cur.append(angle_of_vector(v3,v4))
        angle.append(cur)
    return angle

def get_spine_angle(positions):
    angle=[]
    for i in range(len(positions)):
        cur=[]
        v1=positions[i][11]- positions[i][10]
        v2=positions[i][10]- positions[i][9]

        cur.append(angle_of_vector(v1,v2))
        angle.append(cur)
    return angle

def store_angle(root,index,angle):
    with open('%s%d.txt'%(root,index),'w') as f:
        for item in angle:
            f.write("%f\t%f\n"%(item[0],item[1]))

def store_sigle_angle(root,index,angle):
    with open('%s%d.txt'%(root,index),'w') as f:
        for item in angle:
            f.write("%f\n"%(item[0]))

def parse_bvh_to_angle(file_root,index):
    result_root='../result/BVH/'
    positions=get_bvh_data(file_root,index)
    angle=get_angle(positions)
    store_angle(result_root,index,angle)

def parse_bvh_to_hip_angle(file_root,index):
    result_root='../result/BVH/hip/'
    positions=get_bvh_data(file_root,index)
    angle=get_hip_angle(positions)
    store_angle(result_root,index,angle)

def parse_bvh_to_spine_thigh(file_root,index):
    # angle of (spine,hip), (upleg,leg)
    # 看大腿与身体的夹角
    result_root='../result/BVH/spine_thigh/'
    positions=get_bvh_data(file_root,index)
    angle=get_spine_thigh(positions)
    store_angle(result_root,index,angle)

def parse_bvh_to_spine_angle(file_root,index):
    # angle of (spine2,spine1) (spine1,spine)
    # 看脊柱的弯曲程度
    result_root='../result/BVH/spine/'
    positions=get_bvh_data(file_root,index)
    angle=get_spine_angle(positions)
    store_sigle_angle(result_root,index,angle)


def parse_bvh_to_ankle_hip_spine_height(file_root,index):
    # 看ankle、hip、spine的高度变化
    result_root='../result/BVH/height/'
    positions=get_bvh_data(file_root,index)
    height=get_height(positions)
    with open('%sheight%d.csv'%(result_root,index),'w') as f:
        writer=csv.writer(f)
        for i in height:
            writer.writerow(i)

def parse_bvh_to_ankle_hip_spine_pos(file_root,index):
    # 看ankle、hip、spine的位置变化
    result_root='../result/BVH/pos/'
    positions=get_bvh_data(file_root,index)
    height=get_pos(positions)
    with open('%sheight%d.csv'%(result_root,index),'w') as f:
        writer=csv.writer(f)
        for i in height:
            writer.writerow(i)

if __name__ == '__main__':
    root='../data/BVH/'
    for i in range(178,190):
        print(i)
        # parse_bvh_to_spine_angle(root,i)
        # parse_bvh_to_angle(root,i)
        parse_bvh_to_ankle_hip_spine_pos(root,i)


