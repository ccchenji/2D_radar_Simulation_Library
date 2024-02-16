%2维数据滤波器文件

function out = filters2D ()
	out.kfFilter = @kfFilter;
	out.initKf = @initKalman;
	out.NNSF = @NNSF;
	out.PDA = @PDA;
	out.JPDA = @JPDA;
end

%联合概率数据互联算法
function out = JPDA(track_table,targets,time)
	gama = 16; %互联门限值
	B = 0.0001; %杂波密度常数设置为1
	target_use_v = []; %用于记录录取的点迹位置
	[m,n] = size(track_table);%得到track_table的大小
	JPDA_parameter = []; %JPDA相关的参数 X_K1_K, Z_K1_K, P_K1_K, S_K1, inv_S_K1, sqrt(det(S_K1))
	%先计算JPDA相关参数
	for i = 1 : n
		now_track = track_table(i); %得到当前航迹
		%参数计算
		X_K1_K = now_track.F*now_track.X_K;%状态的一步预测
		Z_K1_K = now_track.H * X_K1_K;%量测的一步预测
		P_K1_K = now_track.F * now_track.P_K * (now_track.F)' + now_track.Q;%协方差的一步预测
		S_K1 = now_track.H * P_K1_K * (now_track.H)' + now_track.R;%新息协方差
		inv_S_K1 = inv(S_K1);
		%将参数放入JPDA_parameter以便下面算法使用
		JPDA_parameter{i,1} = X_K1_K;
		JPDA_parameter{i,2} = Z_K1_K;
		JPDA_parameter{i,3} = P_K1_K;
		JPDA_parameter{i,4} = S_K1;
		JPDA_parameter{i,5} = inv_S_K1;
		JPDA_parameter{i,6} = sqrt((det(S_K1)));
	end
	% 计算Gjt矩阵
	[tm,tn] = size(targets);
	Gjt = zeros([tn+1,n]); %初始化Gjt矩阵,最后一行代表航迹没有点迹互联
	targets_index = cell([1,n]); %航迹录取的点迹索引
	for i = 1 : tn
		target = targets(i);
		X_K1 = [target.x;target.vx;target.y;target.vy];
		if_have_point = 0;
		for j = 1 : n
			now_track = track_table(j);
			Z_K1 = now_track.H * X_K1;
			Z_K1_K = JPDA_parameter{j,2};
			inv_S_K1 = JPDA_parameter{j,5};
			%计算目标点距状态预测的马氏距离
			d = (Z_K1 - Z_K1_K)' * inv_S_K1 * (Z_K1 - Z_K1_K);
			if d <= gama %如果d小于门限值就计算Gij,否则Gij为0
				Gjt(i,j) = 1/(2 * pi * JPDA_parameter{j,6}) * exp(-1/2*d);
				targets_index{j} = [targets_index{j},i];
				if_have_point = 1;
			end
		end
		if if_have_point == 1
			target_use_v = [target_use_v,i]; %记录被录取点迹的位置
		end
	end
	%计算Gjt的最后一行
	for i = 1 : n
		Gjt(end,i) = 1/(2 * pi * JPDA_parameter{i,6});
	end
	%计算Sj和St
	Sj = sum(Gjt,2); %所有行的和，得到一个列向量
	b = zeros(1,n); %防止当Gjt只有两行时，[1,end-1]为向量不是矩阵,使用sum得到是一个数,不是行向量,后面运算会出错
	St = sum([Gjt(1:end-1,:);b]); %所有列的和，得到一个行向量
	%计算估计值
	for i = 1 : n
		now_track = track_table(i); %得到当前航迹
		now_track.time = time; %更新航迹时间
		now_track.P_K1_K = JPDA_parameter{i,3}; %更新目标的P_K1_K
		now_index = targets_index{i}; %得到录取点迹的索引
		XK = zeros(size(now_track.X_K)); %用于状态X_K的更新值
		XKK = zeros(size(now_track.P_K)); %该参数用于计算P_K
		if isempty(now_index) %判断当前航迹是否录取到点迹
			now_track.if_have_point = 0; %航迹没有录取到点
			now_track.X_K = JPDA_parameter{i,1}; %没有录取到点,使用预测值更新航迹状态
		else %航迹录取到点
			now_track.if_have_point = 1;%当前航迹录取到点迹
			[im,in] = size(now_index); %得到索引矩阵大小
			K = JPDA_parameter{i,3} * (now_track.H') * JPDA_parameter{i,5}; %计算卡尔曼增益
			for j = 1 : in
				target = targets(now_index(j)); %得到当前点迹
				X_K1 = [target.x;target.vx;target.y;target.vy]; %得到量测点迹状态
				Z_K1 = now_track.H * X_K1; %得到观测值
				Z_K1_K = JPDA_parameter{i,2}; %得到量测预测值
				V_K1 = Z_K1 - Z_K1_K; %得到新息
				XKj = K*V_K1; %得到当前点迹的状态更新值
                
				gjt = Gjt(now_index(j),i); 
				beta = gjt/( St(i)+Sj(now_index(j)) - gjt + B);

				XK = XK + beta * XKj;
				XKK = XKK + beta*XKj*XKj';
			end
			g0t = Gjt(end,i);
			beta0 = g0t/( St(i)+Sj(end) - g0t + B);
			X_K1_K =  JPDA_parameter{i,1};
			P_K1_K = JPDA_parameter{i,3};
			now_track.X_K = X_K1_K + K*XK;
			now_track.P_K = beta0*P_K1_K+ (1-beta0)*(P_K1_K - K*now_track.H*P_K1_K) + K*(XKK - XK*XK')*K';
			%now_track.P_K = JPDA_parameter{i,3} - (1-beta0)*K*JPDA_parameter{i,4}*K' + XKK - XK*XK';
			%now_track.P_K = JPDA_parameter{i,3} - (0.1)*K*JPDA_parameter{i,4}*K' + XKK - XK*XK';
			now_track.P_K_buff = [now_track.P_K_buff;now_track.P_K];
			now_track.point_buff = [now_track.point_buff;now_track.X_K(1),now_track.X_K(3),now_track.X_K(2),now_track.X_K(4),now_track.time];
		end
		track_table(i) = now_track;
	end
	targets(target_use_v) = []; %将已录取的点迹删除
	out = {track_table,targets}; %将航迹和点迹返回
end

%该函数用于计算与β有关的中间参数
function out = PDA_beta(b,Vk,inv_S) 
	%计算e
	e = [];
	[m,n] = size(Vk);
	for i = 1:n
		vi = Vk(:,i);
		ei = (vi')*inv_S*vi/(-2);
		e = [e,ei];
	end
	e = exp(e);
	sume = sum(e); %求和

	%计算beta0,以及betai
	beta0 = b/(b+sume);
	beta = [];
	for i = 1:n
		beta=[beta,e(i)/(b+sume)];
	end

	%计算V_K1,以及betaVV
	V_K1 = zeros([m,1]);
	betaVV = zeros([m,m]);
	for i = 1:n
		V_K1 = beta(i).*Vk(:,i) + V_K1;
		betaVV = (beta(i).*Vk(:,i))*(Vk(:,i)') + betaVV;
	end

	out = {beta0,V_K1,betaVV};

end

%概率数据互联算法(PDA)
%track_table为跟踪航迹表
%targets 为目标点
%out = {track_table,targets}
function out = PDA(track_table,targets,time)
	%门限值为gama,PG为门概率，PD为目标检测概率
	gama = 16; PG = 0.9997; PD = 1;
	%得到航迹表大小
	[m,n] = size(track_table);
	for i = 1:n
		%用于记录录取点迹的位置
		target_use_v = [];
		%用于记录录取点迹的数量
		target_num = 0;
		%存储波门内点迹的新息
		v_k1_arr = [];
		%得到当前航迹
		now_track = track_table(i);
		%将录取点迹标志位置0
		now_track.if_have_point = 0;
		%更新当前航迹时间
		now_track.time = time;
		%参数计算
		X_K1_K = now_track.F*now_track.X_K;%状态的一步预测
		Z_K1_K = now_track.H * X_K1_K;%量测的一步预测
		P_K1_K = now_track.F * now_track.P_K * (now_track.F)' + now_track.Q;%协方差的一步预测
		now_track.P_K1_K = P_K1_K; %更新目标的P_K1_K
		S_K1 = now_track.H * P_K1_K * (now_track.H)' + now_track.R;%新息协方差
		inv_S_K1 = inv(S_K1);
		%得到目标点数
		[tm,tn] = size(targets);
		%循环录取点迹
		for j = 1 : tn
			%得到当前点迹值
			target = targets(j);
			%解析雷达测量数据
			X_K1 = [target.x;target.vx;target.y;target.vy];
			Z_K1 = now_track.H * X_K1;
			%计算目标点距状态预测的马氏距离
			d = (Z_K1 - Z_K1_K)' * inv_S_K1 * (Z_K1 - Z_K1_K);
			%通过马式距离来判断目标是否在波门内
			if d <= gama %当门限值为16则在二维情况下 PG=0.9997
				target_use_v = [target_use_v,j]; %存储已录取点迹的坐标
				now_track.if_have_point = 1; %当前航迹录取到点迹
				v_k1_arr = [v_k1_arr,Z_K1 - Z_K1_K]; %存储波门内点迹的新息
				target_num = target_num + 1; %当前录取的点迹数量加1
			end
		end
		%计算卡尔曼增益
		K = P_K1_K * (now_track.H') * inv_S_K1;
		if now_track.if_have_point == 1 %如果录取到点
			%计算 b,非参数模型,Cnz = 2 ,Mk = target_num
			b = sqrt(2*pi)/gama*target_num/2*(1-PD*PG)/PD;
			%计算beta相关参数
			beta_temp = PDA_beta(b,v_k1_arr,inv_S_K1);
			beta0 = beta_temp{1}; %得到beta0
			V_K1 = beta_temp{2}; %得到新息
			%状态更新
			now_track.X_K = X_K1_K + K * V_K1;
			%更新协方差矩阵
			PC = P_K1_K - K*now_track.H*P_K1_K;
			PV = K*(beta_temp{3} - V_K1*V_K1')*(K');
			now_track.P_K = P_K1_K.*beta0 + PC.*(1-beta0) + PV ;
		else
			now_track.X_K = X_K1_K;
			%now_track.P_K = P_K1_K - K * now_track.H * P_K1_K;
		end
		now_track.P_K_buff = [now_track.P_K_buff;now_track.P_K];
		%将点迹放入历史点迹
		now_track.point_buff = [now_track.point_buff;now_track.X_K(1),now_track.X_K(3),now_track.X_K(2),now_track.X_K(4),now_track.time];
		targets(target_use_v) = []; %将已录取的点迹删除
		%将航迹更新
		track_table(i) = now_track;
	end
	out = {track_table,targets}; %将航迹和点迹返回

end



%最近邻域滤波器
%track_table为跟踪航迹表
%targets 为目标点
%out = {track_table,targets}
function out = NNSF(track_table,targets,time)

	%得到航迹表大小
	[m,n] = size(track_table);
	for i = 1:n
		%用于记录录取点迹的位置
		target_use_v = [];
		%当前马式距离最小值，以及最小值下标
		dmin = 16; location_min = 0;
		%得到当前航迹
		now_track = track_table(i);
		%将录取点迹标志位置0
		now_track.if_have_point = 0;
		%更新当前航迹时间
		now_track.time = time;
		%开始进行最近邻域参数计算
		X_K1_K = now_track.F*now_track.X_K;%状态的一步预测
		Z_K1_K = now_track.H * X_K1_K;%量测的一步预测
		P_K1_K = now_track.F * now_track.P_K * (now_track.F)' + now_track.Q;%协方差的一步预测
		now_track.P_K1_K = P_K1_K; %更新目标的P_K1_K
		S_K1 = now_track.H * P_K1_K * (now_track.H)' + now_track.R;%新息协方差
		inv_S_K1 = inv(S_K1);
		%得到目标点数
		[tm,tn] = size(targets);
		%循环录取点迹
		for j = 1 : tn
			%得到当前点迹值
			target = targets(j);
			%解析雷达测量数据
			X_K1 = [target.x;target.vx;target.y;target.vy];
			Z_K1 = now_track.H * X_K1;
			%计算目标点距状态预测的马氏距离
			d = (Z_K1 - Z_K1_K)' * inv_S_K1 * (Z_K1 - Z_K1_K);
			%如果在波门内
			if d <= 16
				target_use_v = [target_use_v,j]; %点迹设置为被录取
				now_track.if_have_point = 1; %当前航迹录取到点迹
				if d < dmin
					dmin = d;
					location_min=  j;
				end
			end
		end
		if now_track.if_have_point == 1 %如果录取到点
			target = targets(location_min);
			X_K1 = [target.x;target.vx;target.y;target.vy]; %目标原始量测
			Z_K1 = now_track.H * X_K1; %观测值
		else
			Z_K1 = Z_K1_K;
		end
		%得到新息
		V_K1 = Z_K1 - Z_K1_K;
		%计算卡尔曼增益
		K = P_K1_K * (now_track.H') * inv_S_K1;
		%状态更新
		now_track.X_K = X_K1_K + K * V_K1;
		%协方差更新
		now_track.P_K = P_K1_K - K * now_track.H * P_K1_K;
		now_track.P_K_buff = [now_track.P_K_buff;now_track.P_K];
		now_track.point_buff = [now_track.point_buff;now_track.X_K(1),now_track.X_K(3),now_track.X_K(2),now_track.X_K(4),now_track.time]; %将点迹放入历史点迹
		targets(target_use_v) = []; %将已录取的点迹删除
		%将航迹更新
		track_table(i) = now_track;
	end
	out = {track_table,targets}; %将航迹和点迹返回
end

%4阶卡尔曼滤波器,[x,vx,y,vy]
function target = kfFilter(target)
	%状态的一步预测
	X_K1_K = target.F*target.X_K;
	%量测的一步预测
	Z_K1_K = target.H * X_K1_K;

	%解析雷达测量数据
	X_K1 = [target.measure_x;target.mspeed_x;target.measure_y;target.mspeed_y];
	Z_K1 = target.H * X_K1;

	%得到新息
	V_K1 = Z_K1 - Z_K1_K;

	%协方差的一步预测
	P_K1_K = target.F * target.P_K * (target.F)' + target.Q;

	target.P_K1_K = P_K1_K; %更新目标的P_K1_K

	%新息协方差
	S_K1 = target.H * P_K1_K * (target.H)' + target.R;

	%计算卡尔曼增益
	K = P_K1_K * (target.H') * inv(S_K1);

	%状态更新
	target.X_K = X_K1_K + K * V_K1;
	%协方差更新
	target.P_K = P_K1_K - K * target.H * P_K1_K;

	%滤波值更新
	target.filter_x = target.X_K(1);
	target.filter_vx = target.X_K(2);
	target.filter_y = target.X_K(3);
	target.filter_vy = target.X_K(4);

	target.point_buff = [target.point_buff;target.filter_x,target.filter_y,target.filter_vx,target.filter_vy,target.time];

end


%初始化卡尔曼参数
function target = initKalman(target,q,r,T)
	target.T = T;
	target.q = q;
	target.r = r;

	target.F = [1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1]; %系统状态转移矩阵
	target.Q = [q,q/T,0,0;q/T,2*q/(T^2),0,0;0,0,q,q/T;0,0,q/T,2*q/(T^2)]; %系统过程噪声的协方差(q为距离维协方差)
	target.H = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1]; %系统的观测矩阵
	target.R = [r,r/T,0,0;r/T,2*r/(T^2),0,0;0,0,r,r/T;0,0,r/T,2*r/(T^2)]; %系统观测噪声的协方差(r为距离维协方差)
	target.X_K = [target.measure_x;target.mspeed_x;target.measure_y;target.mspeed_y]; %系统状态向量初值
	target.P_K = [r,r/T,0,0;r/T,2*r/(T^2),0,0;0,0,r,r/T;0,0,r/T,2*r/(T^2)]; %系统初始协方差
	target.P_K1_K = target.F * target.P_K * (target.F)' + target.Q;
end
