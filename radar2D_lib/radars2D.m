%该文件用于二维雷达的一些操作

function out = radars2D()
	out.creatRadars2D = @creatRadars2D; %创建一个2D雷达
	out.radarDetect = @radarDetect; %雷达检测函数
end

%创建一个2D雷达,雷达的辐射范围为扇形
function out = creatRadars2D(ID,towards,x,y,range_distance,range_angle)

	out.ID = ID; %雷达id
	out.towards = towards;%雷达朝向
	out.location_x = x; %雷达x坐标
	out.location_y = y; %雷达y坐标
	out.range_distance = range_distance; %雷达辐射范围->距离
	out.range_angle = range_angle; %雷达辐射范围->角度,以雷达朝向中心线,[towards-range_angle,towards+range_angle]
	out.r = 0; %雷达观测噪声,噪声为零均值的高斯白噪声,默认噪声方差为r=0,即没有观测噪声
	out.time = 0; %当前仿真时间,默认为0
	out.targets = []; %雷达当前检测到的目标

end

%雷达检测函数
function out = radarDetect(radar,targets,time) 
	%清空雷达检测
	radar.targets = [];
	%得到误差时间
	err_time = time - radar.time;
	%更新雷达时间
	radar.time = time;
	%得到目标数据量
	[m,n] = size(targets);
	for i = 1:n
		target = targets(i);
		%雷达和目标的距离
		r = sqrt( (target.x - radar.location_x)^2 + (target.y - radar.location_y)^2 );

		%如果距离满足则判断角度
		if r < radar.range_distance
			errx = target.x - radar.location_x ;
			erry = target.y - radar.location_y ;
			angle = acos(abs(errx) / r) * 180 /3.14 ;
			if errx >=0 && erry >=0
				angle = angle;
			elseif errx <= 0 && erry >= 0
				angle = 180 - angle;
			elseif errx <= 0 && erry <= 0
				angle = 180 + angle;
			elseif errx >=0 && erry <= 0
				angle = 360 - angle;
			end
			flag = 0;
			%判断角度是否在雷达视场范围
			if angle >= radar.towards - radar.range_angle && angle <= radar.towards + radar.range_angle
				flag = 1;
			elseif angle - 360 >= radar.towards - radar.range_angle && angle - 360 <= radar.towards + radar.range_angle
				flag = 1;
			end

			%目标在雷达视场范围内
			if flag == 1
				dat = randn(1,4);
				vr = radar.r * err_time ; %观测速度的方差
				dr = (err_time^3)*vr / 2; %观测距离的方差
				target.x = target.x + sqrt(dr)*dat(1);
				target.y = target.y + sqrt(dr)*dat(2);
				target.vx = target.vx + sqrt(vr)*dat(3);
				target.vy = target.vy + sqrt(vr)*dat(4);
				radar.targets = [radar.targets,target];
			end

		end

	end
	out = radar;
end