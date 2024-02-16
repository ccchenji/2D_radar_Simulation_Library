%导入库的路径
addpath(genpath('./radar2D_lib'));
%目标起始算法的仿真
clc;clear;

%导入二维目标产生器
targets2D = targets2D();

%导入航迹起始算法
start_track = startTrack2D();


%仿真的帧数
frames = 4;
%仿真的时间的步进值
emulation_step_time = 0.2;
%仿真开始时间
emulation_time = 1;

%初始化5个目标,目标的过程噪声均为0
target = targets2D.creatTargets2D(1,80,100,20,0,targets2D.linearMotion);
target.time = emulation_time;
target.q = 0;
targets= [target];

target = targets2D.creatTargets2D(2,85,90,22,0,targets2D.linearMotion);
target.time = emulation_time;
targets= [targets,target];

target = targets2D.creatTargets2D(3,90,80,23,0,targets2D.linearMotion);
target.time = emulation_time;
targets= [targets,target];

target = targets2D.creatTargets2D(4,85,70,18,0,targets2D.linearMotion);
target.time = emulation_time;
targets= [targets,target];

target = targets2D.creatTargets2D(5,80,60,19,0,targets2D.linearMotion);
target.time = emulation_time;
targets= [targets,target];

%杂波相关参数
clutter_range = [1,200;1,200;20,30;20,30]; %杂波范围
clutter_num = 100; %杂波数量

%航迹起始表
intu_start_track_table = []; %直接法起始的航迹
logc_start_track_table = []; %逻辑法起始的航迹


font_size = 20;

f1 = figure('Name','杂波点与目标数据图','NumberTitle','off');
figure(f1);
xlabel('x轴/m','FontSize',font_size);
ylabel('y轴/m','FontSize',font_size);
hold on;
box on;

targets_mat = [];
for i = 1:frames

	emulation_time = emulation_time + emulation_step_time;

	%产生杂波
	cullters = targets2D.uniformClutter(clutter_num,clutter_range,emulation_time);
	cullters_mat = targets2D.targets2mat(cullters); %得到mat格式的杂波数据
	if i == 1
		plot(cullters_mat(:,1),cullters_mat(:,2),'*');%绘制杂波图
	elseif i == 2
		plot(cullters_mat(:,1),cullters_mat(:,2),'+');%绘制杂波图
	elseif i == 3
		plot(cullters_mat(:,1),cullters_mat(:,2),'.');%绘制杂波图
	elseif i == 4
		plot(cullters_mat(:,1),cullters_mat(:,2),'square');%绘制杂波图
	end

	%目标运动
	targets = targets2D.targetsMove(targets,emulation_time);
	targets_mat = [targets_mat;targets2D.targets2mat(targets)];

	%航迹起始
	intu_start_track_table = start_track.intuitive(intu_start_track_table,[cullters,targets],emulation_time);
	logc_start_track_table = start_track.logical(logc_start_track_table,[cullters,targets],emulation_time);
end
plot(targets_mat(:,1),targets_mat(:,2),'ro');%绘制目标图
set(gca,'FontSize',16);	%设置坐标刻度字体
legend('第一帧杂波','第二帧杂波','第三帧杂波','第四帧杂波','目标','FontSize',20);

f2 = figure('Name','直观法航迹起始','NumberTitle','off');
figure(f2);
xlabel('x轴/m','FontSize',font_size);
ylabel('y轴/m','FontSize',font_size);
axis([0,200,0,200]);
hold on;
box on;
[m,n] = size(intu_start_track_table);
start_targets = [];
for i = 1:n
	if intu_start_track_table(i).win_point_num / intu_start_track_table(i).win_size >= 3/4
		start_targets = [start_targets;intu_start_track_table(i).point_buff];
	end
end
plot(targets_mat(:,1),targets_mat(:,2),'ro');
plot(start_targets(:,1),start_targets(:,2),'b+');
set(gca,'FontSize',16);	%设置坐标刻度字体
legend('真实航迹','起始航迹','FontSize',20);


f3 = figure('Name','3/4逻辑法航迹起始','NumberTitle','off');
figure(f3);
xlabel('x轴/m','FontSize',font_size);
ylabel('y轴/m','FontSize',font_size);
axis([0,200,0,200]);
hold on;
box on;
[m,n] = size(logc_start_track_table);
start_targets = [];
for i = 1:n
	if logc_start_track_table(i).win_point_num / logc_start_track_table(i).win_size >= 3/4
		start_targets = [start_targets;logc_start_track_table(i).point_buff];
	end
end
plot(targets_mat(:,1),targets_mat(:,2),'ro');
plot(start_targets(:,1),start_targets(:,2),'b+');
set(gca,'FontSize',16);	%设置坐标刻度字体
legend('真实航迹','起始航迹','FontSize',20);

%删除库
rmpath(genpath('./radar2D_lib'));