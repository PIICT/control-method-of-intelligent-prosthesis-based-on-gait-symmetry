class AngelConfing():
	"""parameter for AngelConfing"""
	diff = 0.01 #插值所取得时间差
	ITER_CNT=100 #高斯牛顿迭代次数
	ITER_STEP=1e-7 #迭代步长
	train=1
	test=1
	window_size=16
	root='../data/slice/'
	mid_size=3 #中值滤波窗口大小