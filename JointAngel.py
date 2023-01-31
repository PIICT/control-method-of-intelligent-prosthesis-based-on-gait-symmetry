import numpy as np
from scipy import interpolate
import scipy.signal as signal
import datetime
import math
import matplotlib.pyplot as plt

from config import AngelConfing

def c(tt):
	return math.cos(tt)

def s(tt):
	return math.sin(tt)

def sqrt(tt):
	return math.sqrt(tt)

def pow(tt):
	return math.pow(tt,2)


def get_axis(input_data,params,output):
	theta_1=params[0][0]
	theta_2=params[1][0]
	phi_1=params[2][0]
	phi_2=params[3][0]

	m=input_data.shape[0]
	for i in range(m):
		output[i][0]=sqrt(\
			pow(input_data[i][1]*s(phi_1)- input_data[i][2]*c(phi_1)*s(theta_1) )+\
			pow(input_data[i][2]*c(phi_1)*c(theta_1)- input_data[i][0]*s(phi_1) )+\
			pow(input_data[i][0]*c(phi_1)*s(theta_1)- input_data[i][1]*c(phi_1)*c(theta_1) ))-\
		sqrt(\
			pow(input_data[i][4]*s(phi_2)- input_data[i][5]*c(phi_2)*s(theta_2) )+\
			pow(input_data[i][5]*c(phi_2)*c(theta_2)- input_data[i][3]*s(phi_2) )+\
			pow(input_data[i][3]*c(phi_2)*s(theta_2)- input_data[i][4]*c(phi_2)*c(theta_2) ))

	return output

def combine(data,num):
    res=[]
    begin=0
    while begin+num<len(data):
        cur=np.zeros((len(data[0])))
        for i in range(num):
            cur+=np.array(data[begin+i])
        cur/=num
        res.append(cur.tolist())
        begin+=num

    return res

class joint_angel():
	def __init__(self,diff,origin_data):
		self.DATASET_NUM=len(origin_data[0])
		self.DELTA_T=diff
		self.ITER_STEP=AngelConfing.ITER_STEP
		self.ITER_CNT=AngelConfing.ITER_CNT

		self.vj1=np.zeros((3,1))
		self.vj2=np.zeros((3,1))

		self.origin_data_1=np.array(origin_data[0])
		self.origin_data_2=np.array(origin_data[1])

		self.SUM=0.0
		if self.DATASET_NUM>1000:
			self.LENS=1000
		else:
			self.LENS=self.DATASET_NUM
		# self.LENS=self.DATASET_NUM
		print(diff)

	def get_jacobian(self,func,input_data,params,output):
		m=input_data.shape[0]
		n=params.shape[0]

		out0=np.zeros((m,1))
		out1=np.zeros((m,1))
		param0=np.zeros((n,1))
		param1=np.zeros((n,1))

		for j in range(n):
			param0=params.copy()
			param1=params.copy()
			param0[j][0]-=self.ITER_STEP
			param1[j][0]+=self.ITER_STEP
			
			out1 = func(input_data,param1,out1)
			out0 = func(input_data,param0,out0)

			output[0:m,j:j+1]=(out1-out0)/(2*self.ITER_STEP)

		return output

	def gauss_newton(self,func,input_data,output,params):
		m=input_data.shape[0]
		n=params.shape[0]

		#jacobian
		jmat=np.zeros((m,n))
		r=np.zeros((m,1))
		tmp=np.zeros((m,1))

		pre_mse=0.0
		mse=0.0

		result=[]

		for i in range(self.ITER_CNT):			
			mse=0.0
			tmp=func(input_data,params,tmp)
			r=output-tmp
			jmat=self.get_jacobian(func,input_data,params,jmat)

			mse=np.dot(r.transpose(),r)
			mse/=m
			if abs(mse- pre_mse )<1e-7:
				print("均方根误差更新过小，已收敛")
				break

			pre_mse=mse
			a=np.dot(jmat.transpose(),jmat)

			b=np.linalg.pinv(a)
			c=np.dot(b,jmat.transpose())
			d=np.dot(c,r)
			result.append((mse,params.copy()))

			print("i=%d, mes:%lf"%(i,mse))

			params+=d

		pre=params
		# minn=10000.0
		# for item in result:
			# if item[0]<minn:
				# pre=item[1]
				# minn=item[0]

		return pre

	def joint_axis(self):
		#计算关节轴向数据输入接口
		print(self.LENS)
		input_data=np.zeros((self.LENS,6))
		output=np.zeros((self.LENS,1))

		for i in range(self.LENS):
			k=0
			for j in range(3):
				input_data[i][k]=self.origin_data_1[i][j]
				k+=1
			for j in range(3):
				input_data[i][k]=self.origin_data_2[i][j]
				k+=1
			output[i][0]=0.0

		params_axis=np.zeros((4,1))
		# for i in range(4):
			# params_axis[i][0]=0

		# theta_1=params[0][0]
		# theta_2=params[1][0]
		# phi_1=params[2][0]
		# phi_2=params[3][0]

		params_axis[0][0]=math.pi/2
		params_axis[1][0]=math.pi/2
		params_axis[2][0]=0.0
		params_axis[3][0]=0.0


		
		r=np.zeros((self.LENS,1))
		r=get_axis(input_data,params_axis,r)
		res=np.dot(r.transpose(),r)
		print('res:',res)
		params_axis=self.gauss_newton(get_axis,input_data,output,params_axis)

		r=np.zeros((self.LENS,1))
		r=get_axis(input_data,params_axis,r)
		res=np.dot(r.transpose(),r)
		print('res:',res)
		self.vj1=np.array([[c(params_axis[2][0])*c(params_axis[0][0])],\
			[c(params_axis[2][0]*s(params_axis[0][0]))],\
			[s(params_axis[2][0])]])
		self.vj2=np.array([[c(params_axis[3][0])*c(params_axis[1][0])],\
			[c(params_axis[3][0])*s(params_axis[1][0])],\
			[s(params_axis[3][0])]])

		print('j1:')
		print(self.vj1)
		print('j2:')
		print(self.vj2)

	def test_angel(self,current_data):
		g1=np.array([[current_data[0][0],current_data[0][1],current_data[0][2]]])
		g2=np.array([[current_data[1][0],current_data[1][1],current_data[1][2]]])
		aaa=np.dot(g2,self.vj2)-np.dot(g1,self.vj1)
		self.SUM=self.SUM+aaa[0][0]

		return math.degrees(self.SUM*self.DELTA_T)

def combine(data,num):
    res=[]
    begin=0
    while begin+num<len(data):
        cur=np.zeros((len(data[0])))
        for i in range(num):
            cur+=np.array(data[begin+i])
        cur/=num
        res.append(cur.tolist())
        begin+=num

    return res


def get_data_j1j2(path):
	d1=[]
	d2=[]
	ground_truth=[]
	with open(path) as f:
		context=f.read()
		context=context.split('\n')
		for item in context:
			if len(item)==0:
				continue
			cur1=[]
			cur2=[]
			item=item.split('\t')
			flag=False
			for i in range(3):
				cur1.append(float(item[i]))
				cur2.append(float(item[i+3]))
				if abs(float(item[i]))>15 or abs(float(item[i+3]))>15:
					flag=True
			if flag:
				continue
			d1.append(cur1)
			d2.append(cur2)
			ground_truth.append(float(item[-1]))
	return d1,d2,ground_truth


def get_data(path):
	whole=[]
	with open(path) as f:
		context=f.read()
		context=context.split('\n')
		for item in context:
			if len(item)==0:
				continue
			cur=[]
			item=item.split('\t')
			for i in item:
				cur.append(float(i))
			flag=False
			for i in range(len(cur)-1):
				if abs(cur[i+1])>15:
					flag=True
			if flag:
				continue

			cur=cur[4:]
			whole.append(cur)
	return whole

def get_j1_j2(path,store_path):
    '''
    实时显示计算的角度数值
    calibration:校准时间，用于计算j1,j2,o1,o2,以及offset角度
    calibration=10

    '''
    root=''
    d1,d2,ground_truth=get_data_j1j2(path)
    result=[]
    # d2=get_data(root+'2.txt')

    print(len(d1),len(d2))
    print(len(d1[0]))
    
    lens1=min(len(d1),len(d2))
    d1=d1[:lens1]
    d2=d2[:lens1]


    ##将采样频率降到50hz，即每2个点合并
    num=4
    d1=combine(d1,num)
    d2=combine(d2,num)
    diff=0.01*num
    print(len(d1),len(d2))

    # 保留原始数据，计算关节夹角用到
    origin_train_data=[d1,d2]

    a=joint_angel(diff,origin_train_data)
    a.joint_axis()
    # a.vj1=np.array([[0.99604164],[0.999998  ],[0.08581699]])
    # a.vj2=np.array([[ 0.99582127],[-0.06204801],[ 0.06700785]])

    for i in range(len(d1)):
    	# result.append(abs(a.test_angel((d1[i],d2[i]))+ground_truth[0]))
    	result.append(a.test_angel((d1[i],d2[i])))
    minn=result[0]
    pos=0
    tot=0
    for i in result:
    	if i<minn:
    		pos=tot
    		minn=i
    	tot+=1
    print(minn)
    lens=len(result)

    with open(store_path,'w') as f:
    	for i in range(len(d1)):
    		f.write("%f\t%f\n"%(result[i],ground_truth[i]))
    return pos,minn

def get_imu_angle(index,store_path):
    '''
    实时显示计算的角度数值
    calibration:校准时间，用于计算j1,j2,o1,o2,以及offset角度
    calibration=10

    '''
    root='../data/imu/'
    p1='%s%d_1.txt'%(root,index)
    p2='%s%d_2.txt'%(root,index)
    d1=get_data(p1)
    d2=get_data(p2)
    result=[]

    print(len(d1),len(d2))
    print(len(d1[0]))
    print(d1[0])
    # exit(0)
    
    lens1=min(len(d1),len(d2))
    d1=d1[:lens1]
    d2=d2[:lens1]

    num=2
    d1=combine(d1,num)
    d2=combine(d2,num)
    diff=0.01*num
    print(len(d1),len(d2))

    # 保留原始数据，计算关节夹角用到
    origin_train_data=[d1,d2]

    a=joint_angel(0.01*num,origin_train_data)
    a.joint_axis()
    print("j1,j2计算完毕")

    for i in range(len(d1)):
    	result.append(a.test_angel((d1[i],d2[i])))

    with open(store_path,'w') as f:
    	for i in range(len(d1)):
    		f.write("%f\n"%(result[i]))
