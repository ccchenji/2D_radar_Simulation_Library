%导入库的路径
addpath(genpath('./radar2D_lib'));
%卡尔曼滤波仿真,二维平面做匀速运动
clc;clear;

%导入二维目标产生器
targets2D = targets2D();

%导入2D雷达生成器
radars2D = radars2D();

%导入滤波器
filters2D = filters2D();



%仿真的帧数
frames = 50;
%仿真的时间的步进值
emulation_step_time = 1.5; 
%仿真开始时间
emulation_time = 1;

%初始化一个2D仿真目标
target = targets2D.creatTargets2D(1,80,100,11,13,targets2D.linearMotion);
target.time = emulation_time;
target.q = 1;

%初始化一条航迹
track = initTrackStruct2D();

%初始化一台雷达
radar = radars2D.creatRadars2D(1,45,0,0,4000,45);
radar.r = 100;
radar.time = emulation_time;

target_mat=[]; %目标的历史点迹 [x,y,vx,vy,time]
filter_target_mat=[]; %滤波值的历史点迹 [x,vx,y,vy,time,qx,qvx,qy,qvy]
obs_target_mat=[]; %观测值历史点迹 [x,y,vx,vy,time]
%仿真开始
for i = 1 : frames
	emulation_time = emulation_time + emulation_step_time;
	target = targets2D.targetsMove(target,emulation_time);
	target_mat = [target_mat;targets2D.targets2mat(target)];

	%雷达观测
	radar = radars2D.radarDetect(radar,target,emulation_time);
	target_obs = radar.targets;
	obs_target_mat = [obs_target_mat;targets2D.targets2mat(target_obs)];
	[m,n] = size(target_obs);
	if n > 0
		track.measure_x = target_obs.x;
		track.measure_y = target_obs.y;
		track.mspeed_x = target_obs.vx;
		track.mspeed_y = target_obs.vy;
		track.time = target_obs.time;
	end

	%kf滤波
	if i == 1
		track = filters2D.initKf(track,0.000005,1,emulation_step_time);
	else
		track = filters2D.kfFilter(track);
	end
	filter_target_mat = [filter_target_mat;(track.X_K)',track.time,track.P_K(1,1),track.P_K(2,2),track.P_K1_K(1,1),track.P_K1_K(2,2)]; %得到滤波数据
end


font_size = 20;
%真实、量测和滤波轨迹3d图
f1 = figure('Name','真实、量测和滤波轨迹3D','NumberTitle','off');
figure(f1);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('坐标x/m','FontSize',font_size);
ylabel('坐标y/m','FontSize',font_size);
zlabel('时间t/s','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot3(target_mat(:,1),target_mat(:,2),target_mat(:,5),'.','linewidth',2,'color','#c52a20');
plot3(obs_target_mat(:,1),obs_target_mat(:,2),obs_target_mat(:,5),'*','linewidth',1,'color','#4DBEEE')
plot3(filter_target_mat(:,1),filter_target_mat(:,3),filter_target_mat(:,5),'-','linewidth',2,'color','#EDB120');
view(3);
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('真实','量测','滤波','FontSize',font_size);

%真实、量测和滤波轨迹二维图
f2 = figure('Name','真实、量测和滤波轨迹2D','NumberTitle','off');
figure(f2);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('坐标x/m','FontSize',font_size);
ylabel('坐标y/m','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot(target_mat(:,1),target_mat(:,2),'.','linewidth',2,'color','#c52a20');
plot(obs_target_mat(:,1),obs_target_mat(:,2),'*','linewidth',1,'color','#4DBEEE')
plot(filter_target_mat(:,1),filter_target_mat(:,3),'-','linewidth',2,'color','#EDB120');
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('真实','量测','滤波','FontSize',font_size);

%x轴位置速度图
f3 = figure('Name','x轴位置速度图','NumberTitle','off');
figure(f3);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('位置/m','FontSize',font_size);
ylabel('速度/(m/s)','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot(target_mat(:,1),target_mat(:,3),'.','linewidth',2,'color','#c52a20');
plot(obs_target_mat(:,1),obs_target_mat(:,3),'-.','linewidth',1,'color','#4DBEEE')
plot(filter_target_mat(:,1),filter_target_mat(:,2),'-','linewidth',2,'color','#EDB120');
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('真实','量测','滤波','FontSize',font_size);

%y轴位置速度图
f4 = figure('Name','y轴位置速度图','NumberTitle','off');
figure(f4);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('位置/m','FontSize',font_size);
ylabel('速度/(m/s)','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot(target_mat(:,2),target_mat(:,4),'.','linewidth',2,'color','#c52a20');
plot(obs_target_mat(:,2),obs_target_mat(:,4),'-.','linewidth',1,'color','#4DBEEE')
plot(filter_target_mat(:,3),filter_target_mat(:,4),'-','linewidth',2,'color','#EDB120');
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('真实','量测','滤波','FontSize',font_size);

x_step = 1:frames-1;
%更新位置协方差图
f5 = figure('Name','x轴预测和更新位置误差协方差','NumberTitle','off');
figure(f5);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('跟踪步数','FontSize',font_size);
ylabel('位置误差协方差/(m^2)','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot(x_step,filter_target_mat(2:end,8),'--','linewidth',2,'color','#4DBEEE');
plot(x_step,filter_target_mat(2:end,6),'-','linewidth',2,'color','#EDB120');
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('预测位置','更新位置','FontSize',font_size);

%更新位置协方差图
f6 = figure('Name','x轴预测和更新速度误差协方差','NumberTitle','off');
figure(f6);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('跟踪步数','FontSize',font_size);
ylabel('速度误差协方差/(m^2/s^2)','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot(x_step,filter_target_mat(2:end,9),'--','linewidth',2,'color','#4DBEEE');
plot(x_step,filter_target_mat(2:end,7),'-','linewidth',2,'color','#EDB120');
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('预测位置','更新位置','FontSize',font_size);

%删除库
rmpath(genpath('./radar2D_lib'));
