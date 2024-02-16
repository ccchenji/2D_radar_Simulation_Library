% 航迹起始算法
function out = startTrack2D()

	out.intuitive = @intuitiveMethod; %直观法
	out.logical = @logicalMethod; %逻辑法

end

%逻辑法航迹起始
%输入为起始航迹表和点迹数据
%输出为航迹数据
function out = logicalMethod(start_track_table,targets,time)
	%逻辑法一些参数信息
	Vmax = 30; Vmin = -30;%速度最大最小值,单位是m/s
	Accmax = 4.5; %加速度的最大值
	gamma = 5; %门限值

	[m,n] = size(start_track_table); %得到表的大小,表按照1行n列的形式存储
	track_have_point =[]; %用于记录未录取航迹的下标

	%先对已有的start航迹进行直观法互联
	for i = 1:n
		now_track = start_track_table(i); %得到一条航迹
		acc_min  = Accmax + 1; %用于存储最小的acc的值
		[tm,tn] = size(targets); %得到点迹数据的大小,targets为1行n列的结构体向量
		dot_use_v = []; %点迹录取矩阵
		if_have_point = 0;%用于判断当前航迹是否录取到点迹
		min_location = 0; %用于记录最小点迹的位置
		for j = 1:tn
			target = targets(j);
			T = target.time - now_track.time; %得到采样时间
			dx = max(0, target.x - now_track.measure_x - Vmax*T) + max(0,-target.x + now_track.measure_x + Vmin*T);
			dy = max(0, target.y - now_track.measure_y - Vmax*T) + max(0,-target.y + now_track.measure_y + Vmin*T);
			Ax = abs((now_track.mspeed_x - target.vx) / (now_track.time - target.time));
			Ay = abs((now_track.mspeed_y - target.vy) / (now_track.time - target.time));
			Acc = sqrt(Ax^2+Ay^2);
			if Acc < Accmax && dx^2 + dy^2 < gamma
				dot_use_v = [dot_use_v,j]; %录取点迹的位置
				if_have_point = 1; %当前航迹录取到点
				if Acc < acc_min
					acc_min = Acc;
					min_location = j;
				end
			end
		end
		if if_have_point == 0
			%如果时间窗内只有一个点则将该航迹消亡
			if now_track.win_point_num <= 1
				track_have_point = [track_have_point,i]; %未录取点迹的航迹
			else
				%用预测值代表测量值
				measure_x_last = now_track.measure_x;
				measure_y_last = now_track.measure_y;
				now_track.measure_x = now_track.speculate_x;
				now_track.measure_y = now_track.speculate_y;
				now_track.speculate_x =2*now_track.measure_x - measure_x_last;
				now_track.speculate_y =2*now_track.measure_y - measure_y_last;
			end
		else 
			target = targets(min_location);
			now_track.speculate_x = now_track.measure_x;
			now_track.speculate_y = now_track.measure_y;
			now_track.measure_x = target.x;
			now_track.measure_y = target.y;
			now_track.mspeed_x = target.vx;
			now_track.mspeed_y = target.vy;
			now_track.speculate_x = 2*now_track.measure_x - now_track.speculate_x;
			now_track.speculate_y = 2*now_track.measure_y - now_track.speculate_y;
			now_track.win_point_num = now_track.win_point_num + 1;
		end
		targets(dot_use_v) = []; %已录取的点迹删除
		now_track.time = time;
		now_track.point_buff = [now_track.point_buff;[now_track.measure_x,now_track.measure_y,now_track.mspeed_x,now_track.mspeed_y,now_track.time]];
		now_track.win_now_time = now_track.win_now_time + 1; %时间窗内时间加1
		start_track_table(i) = now_track; %将航迹更新到点起始航迹表
	end

	%未录取点迹的航迹进行删除处理
	start_track_table(track_have_point) = [];
	%所有剩余的targets点迹作为新的起始航迹进行起始
	[tm,tn] = size(targets);
	for i = 1 : tn
		now_track = initTrackStruct2D(); %得到一个航迹结构体
		now_track.measure_x = targets(i).x;
		now_track.measure_y = targets(i).y;
		now_track.mspeed_x = targets(i).vx;
		now_track.mspeed_y = targets(i).vy;
		now_track.time = targets(i).time;
		now_track.point_buff = [now_track.point_buff;[now_track.measure_x,now_track.measure_y,now_track.mspeed_x,now_track.mspeed_y,now_track.time]];
		now_track.win_size = 4; %时间窗口设置为4
		now_track.win_now_time = 1;%当前时间窗的当前时间
		now_track.win_point_num = 1;%当前时间窗内录取的点数

		%将新形成的起始航迹加入起始航迹表
		start_track_table = [start_track_table,now_track];
	end

	out = start_track_table; %将新的起始航迹表返回
end




%直观法起始
%输入为起始航迹表和点迹数据
%输出为航迹数据
function out = intuitiveMethod(start_track_table,targets,time)

	%直观法一些参数信息
	Vmax = 40; Vmin = 0;%速度最大最小值,单位是m/s
	Accmax = 8; %加速度的最大值

	[m,n] = size(start_track_table); %得到表的大小,表按照1行n列的形式存储
	track_have_point =[]; %用于记录未录取航迹的下标

	%先对已有的start航迹进行直观法互联
	for i = 1:n
		now_track = start_track_table(i); %得到一条航迹
		acc_min  = Accmax + 1; %用于存储最小的acc的值
		[tm,tn] = size(targets); %得到点迹数据的大小,按照1行n列排列
		dot_use_v = []; %点迹录取矩阵
		if_have_point = 0;%用于判断当前航迹是否录取到点迹
		for j = 1:tn
			target = targets(j);
			Vx = abs((now_track.measure_x - target.x) / (now_track.time - target.time));
			Vy = abs((now_track.measure_y - target.y) / (now_track.time - target.time));
			Ax = abs((now_track.mspeed_x - target.vx) / (now_track.time - target.time));
			Ay = abs((now_track.mspeed_y - target.vy) / (now_track.time - target.time));
			Acc = sqrt(Ax^2+Ay^2);
			if Vx >= Vmin && Vx <= Vmax && Vy >= Vmin && Vy <= Vmax && Acc < Accmax
				dot_use_v = [dot_use_v,j]; %录取点迹的位置
				if_have_point = 1; %当前航迹录取到点
				if Acc < acc_min
					acc_min = Acc;
					now_track.measure_x = target.x;
					now_track.measure_y = target.y;
					now_track.mspeed_x = target.vx;
					now_track.mspeed_y = target.vy;
					now_track.time = target.time;
				end
			end
		end
		targets(dot_use_v) = []; %已录取的点迹删除
		if if_have_point == 0
			track_have_point = [track_have_point,i]; %未录取点迹的航迹
		else 
			now_track.win_point_num = now_track.win_point_num + 1;
			now_track.point_buff = [now_track.point_buff;[now_track.measure_x,now_track.measure_y,now_track.mspeed_x,now_track.mspeed_y,now_track.time]];
			now_track.time = time;
		end
		start_track_table(i) = now_track; %将航迹更新到点起始航迹表
	end

	%未录取点迹的航迹进行删除处理
	start_track_table(track_have_point) = [];
	%所有剩余的targets点迹作为新的起始航迹进行起始
	[tm,tn] = size(targets);
	for i = 1 : tn
		now_track = initTrackStruct2D(); %得到一个航迹结构体
		now_track.measure_x = targets(i).x;
		now_track.measure_y = targets(i).y;
		now_track.mspeed_x = targets(i).vx;
		now_track.mspeed_y = targets(i).vy;
		now_track.time = targets(i).time;
		now_track.point_buff = [now_track.point_buff;[now_track.measure_x,now_track.measure_y,now_track.mspeed_x,now_track.mspeed_y,now_track.time]];

		%将新形成的起始航迹加入起始航迹表
		start_track_table = [start_track_table,now_track];
	end

	out = start_track_table; %将新的起始航迹表返回

end
