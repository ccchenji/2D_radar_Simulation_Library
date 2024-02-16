% 该文件用于二维目标的一些操作

function out = targets2D()
	out.creatTargets2D = @creatTargets2D; %创建二维平面的运动目标
	out.linearMotion = @linearMotion; %目标线性运动
	out.uniformClutter = @clutterGeneratorUniform; %产生均匀分布的杂波
	out.targets2mat = @targets2mat; %将target数据转换为mat数据
	out.targetsMove = @targetsMove; %目标运动函数
	out.targetsFusion = @targetsFusion; %目标点迹融合
end

%创建二维平面的运动目标
%ID,为目标的ID
%x,y为目标在笛卡尔坐标系下的坐标
%vx,vy为目标在x和y方向的分速度
%run_fun为目标的运动方程
function out = creatTargets2D(ID,x,y,vx,vy,run_fun)
	out.ID = ID; %目标的id
	out.x = x; %目标在二维平面的x坐标
	out.y = y; %目标在二维平面的y坐标
	out.vx = vx; %目标x方向的速度
	out.vy = vy; %目标y方向的速度
	out.run_fun = run_fun; %目标的运动方程
	out.q = 0; %目标的运动噪声,目标x方向和y方向的速度均为均值为0,方差为q的高斯白噪声
	out.time = 0; %当前的仿真时间,默认为0
	out.type = 0; %当前target为杂波还是目标,0代表目标,1代表杂波. 默认为0
end

%目标点迹融合
%输入的是两个需要融合的目标点迹
function out = targetsFusion(targets1,targets2)
	[m1,n1] = size(targets1); %得到矩阵1大小
	gama = 2;
	for i = 1 : n1
		[m2,n2] = size(targets2); %得到矩阵2大小
		fusion_location = [];
		%通过距离和时间对目标进行数据融合，因为雷达的观测误差r取值较大，因此速度维波动较大
		for j = 1 : n2
			err_x = abs( targets1(i).x - targets2(j).x );
			err_y = abs( targets1(i).y - targets2(j).y );
			if err_x < gama && err_y < gama %达到融合条件，进行航迹融合
				targets1(i).x = 0.5*targets1(i).x + 0.5*targets2(j).x;
				targets1(i).y = 0.5*targets1(i).y + 0.5*targets2(j).y;
				targets1(i).vx = 0.5*targets1(i).vx + 0.5*targets2(j).vx;
				targets1(i).vy = 0.5*targets1(i).vy + 0.5*targets2(j).vy;
				fusion_location = [fusion_location,j];
			end
		end
		targets2(fusion_location) = [];
	end
	out = [targets1,targets2];
end

%目标做直线运动,targetsMove函数做回调时使用
%入口参数为目标target
%now_time为当前仿真时间
function out = linearMotion(target,now_time)
	err_time = now_time - target.time;
	vq = target.q*err_time; %速度的方差
	dq = (err_time^3)*vq/2; %距离的方差
	dat = randn(1,4); %产生1行4列的服从标准正态分布的随机数
	if err_time > 0
		target.x = target.x + target.vx * err_time;
		target.y = target.y + target.vy * err_time;
		target.time = now_time;
		target.x = target.x + sqrt(dq)*dat(1);
		target.y = target.y + sqrt(dq)*dat(2);
		target.vx = target.vx + sqrt(vq)*dat(3);
		target.vy = target.vy + sqrt(vq)*dat(4);
	end
	out = target;
end

%空运动函数,主要用于杂波,杂波做运动
function out = emptyMotion(target,now_time)
	target.time = now_time;
	out = target;
end


%产生杂波均匀分布的杂波
%count为杂波数量
%range为杂波范围 [dismin,dismax,vmin,vmax]
%time为当前仿真时间
function out = clutterGeneratorUniform(count,range,time)

	%用泊松分布确认要得到的杂波数量
	out=[];
	count = poissrnd(count);
	for i = 1:count
		rand_dat = rand(1,4);
		x = ( range(1,2) - range(1,1) ) * rand_dat(1) + range(1,1);
		y = ( range(2,2) - range(2,1) ) * rand_dat(2) + range(2,1);
		vx = ( range(3,2) - range(3,1) ) * rand_dat(3) + range(3,1);
		vy = ( range(4,2) - range(4,1) ) * rand_dat(4) + range(4,1);
		cullter_taget = creatTargets2D(0,x,y,vx,vy,@emptyMotion); %创建一个二维目标
		cullter_taget.type = 1;%目标为杂波,type设置为1
		cullter_taget.time = time;
		out = [out,cullter_taget];
	end

end

%该函数将targets结构体数据转换为mat数据 [x,y,vx,vy,time]
function out = targets2mat(targets) 

	out = [];
	[m,n] = size(targets); %得到targets的大小
	for i = 1:n
		target = targets(i);
		out = [out;target.x,target.y,target.vx,target.vy,target.time];
	end

end

%目标运动函数
function out = targetsMove(targets,time) 
	%得到目标的数量
	[m,n]  = size(targets);
	for i = 1 : n
		target = targets(i); %得到第i个目标
		run_fun = target.run_fun; %得到目标的运动函数
		targets(i)=run_fun(target,time); %目标运动
	end
	out = targets;
end
