%本函数创建一个航迹结构体
%
function out = initTrackStruct2D()
	out.measure_x = 0; %目标x方向坐标
	out.measure_y = 0; %目标y方向坐标
	out.mspeed_x = 0; %目标x方向的测量速度
	out.mspeed_y = 0; %目标y方向的测量速度

	out.speculate_x = 0; %目标x方向的预测值
	out.speculate_y = 0; %目标y方向的预测值
	out.speculate_vx = 0; %目标x方向的速度预测值
	out.speculate_vy = 0; %目标y方向的速度预测值

	out.filter_x = 0; %目标x方向的滤波值
	out.filter_y = 0; %目标y方向的滤波值
	out.filter_vx = 0; %目标x方向的速度滤波值
	out.filter_vy = 0; %目标y方向的速度滤波值

	out.win_size = 0; %雷达时间窗大小
	out.win_now_time = 0; %当前窗口时间
	out.win_point_num = 0; %当前时间窗内录取的点迹数量

	out.time = 0; %当前航迹最新点迹的采样时间
    
    out.if_have_point = 0;%当前航迹是否录取到点

	out.point_buff = [];%当前航迹的历史点迹

	out.score = 0; %航迹分数

	%卡尔曼参数部分
	out.q = 1; %量测序列噪声的方差,x方向和y方向q(距离不是速度),量测噪声为零均值的高斯白噪声
	out.r = 3; %观测噪声误差的方差,x方向和y方向r(距离不是速度),观测噪声为零均值的高斯白噪声
	out.T = 0.2; %系统采样时间
	out.F = []; %状态转移矩阵
	out.Q = []; %量测误差协方差矩阵
	out.H = []; %观测矩阵
	out.R = []; %观测误差协方差矩阵
	out.X_K = []; %系统状态向量为[ x;vx;y;vy ]
	out.P_K = []; %k时刻估计的协方差矩阵
	out.P_K1_K = []; %协方差的预测值
	out.P_K_buff = []; %协方差矩阵的历史值

end
