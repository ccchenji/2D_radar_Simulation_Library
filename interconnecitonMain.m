%导入库的路径
addpath(genpath('./radar2D_lib'));
%该文件用于仿真互联算法
clc;clear;

%导入二维目标产生器
targets2D = targets2D();

%导入2D雷达生成器
radars2D = radars2D();

%导入滤波器
filters2D = filters2D();

%导入航迹起始算法生成器
startTrack2D = startTrack2D();

%导入航迹质量管理算法生成器
trackQuality2D = trackQuality2D();

%蒙特卡洛实验次数
N = 50;
%仿真的帧数
frames = 100;
%仿真的时间的步进值
emulation_step_time = 0.25; 
%NNSF每一步的位置均方根误差和速度均方根误差
err_NNSF_L = zeros([frames,1]);err_NNSF_S = err_NNSF_L;
%PDA每一步的位置均方根误差和速度均方根误差
err_PDA_L = err_NNSF_L;err_PDA_S = err_NNSF_L;
%最后一次蒙特卡洛实验得到的NNSF和PDA的可靠航迹
reliable_mat_NNSF_last = [];reliable_mat_PDA_last = [];
%最后一次蒙特卡洛实验得到的目标值和观测值
target_mat_last = []; obs_target_mat_last = [];

%开始进行N此蒙特卡洛实验
for j = 1:N
	%仿真开始时间
	emulation_time = 1;

	%初始化一个2D仿真目标
	target = targets2D.creatTargets2D(1,80,100,11,13,targets2D.linearMotion);
	target.time = emulation_time;
	target.q = 0;


	%初始化一台雷达
	radar = radars2D.creatRadars2D(1,45,0,0,4000,45);
	radar.r = 2;
	radar.time = emulation_time;

	%杂波相关参数
	clutter_range = [1,200;1,200;20,30;20,30]; %杂波范围
	clutter_num = 10; %杂波数量


	target_mat=[]; %目标的历史点迹 [x,y,vx,vy,time]
	cullters_mat=[]; %杂波的历史点迹
	obs_target_mat=[]; %观测值历史点迹 [x,y,vx,vy,time]

	%NNSF算法
	start_track_table_NNSF = []; %起始航迹表
	reliable_track_table_NNSF = []; %可靠航迹表
	delete_track_table_NNSF = []; %消亡航迹表

	%PDA算法
	start_track_table_PDA = []; %起始航迹表
	reliable_track_table_PDA = []; %可靠航迹表
	delete_track_table_PDA = []; %消亡航迹表
	%仿真开始
	for i = 1 : frames
		emulation_time = emulation_time + emulation_step_time;
		target = targets2D.targetsMove(target,emulation_time);
		target_mat = [target_mat;targets2D.targets2mat(target)];
	
	    %产生杂波
	    clutter_range=[target.x-7,target.x+7;target.y-7,target.y+7;target.vx-3,target.vx+3;target.vy-3,target.vy+3];
		cullters = targets2D.uniformClutter(clutter_num,clutter_range,emulation_time);
		cullters_mat = targets2D.targets2mat(cullters); %得到mat格式的杂波数据

		%雷达观测
		radar = radars2D.radarDetect(radar,[target,cullters],emulation_time);
		target_obs = radar.targets;
		obs_target_mat = [obs_target_mat;targets2D.targets2mat(target_obs)];

		%航迹互联
		target_obs_PDA = target_obs;
		target_obs_NNSF = target_obs;

		out_PDA=filters2D.PDA(reliable_track_table_PDA,target_obs_PDA,emulation_time);
		reliable_track_table_PDA = out_PDA{1};
		target_obs_PDA = out_PDA{2};

		out_NNSF=filters2D.NNSF(reliable_track_table_NNSF,target_obs_NNSF,emulation_time);
		reliable_track_table_NNSF = out_NNSF{1};
		target_obs_NNSF = out_NNSF{2};
		%航迹起始
		start_track_table_PDA = startTrack2D.logical(start_track_table_PDA,target_obs_PDA,emulation_time);
		start_track_table_NNSF = startTrack2D.logical(start_track_table_NNSF,target_obs_NNSF,emulation_time);

		%航迹质量管理
		%可靠航迹质量管理
		out_PDA = trackQuality2D.scoreFunReliable(reliable_track_table_PDA,delete_track_table_PDA);
		reliable_track_table_PDA = out_PDA{1};
		delete_track_table_PDA = out_PDA{2};

		out_NNSF=trackQuality2D.scoreFunReliable(reliable_track_table_NNSF,delete_track_table_NNSF);
		reliable_track_table_NNSF = out_NNSF{1};
		delete_track_table_NNSF = out_NNSF{2};

		%起始航迹质量管理
		out_PDA = trackQuality2D.scoreFunStart(start_track_table_PDA,reliable_track_table_PDA,0.01,2,emulation_step_time);
		start_track_table_PDA = out_PDA{1};
		reliable_track_table_PDA = out_PDA{2};

		out_NNSF=trackQuality2D.scoreFunStart(start_track_table_NNSF,reliable_track_table_NNSF,0.5,2,emulation_step_time);
		start_track_table_NNSF = out_NNSF{1};
		reliable_track_table_NNSF = out_NNSF{2};
	end

	reliable_mat = reliable_track_table_NNSF(1).point_buff;
	target_mat_temp = target_mat;
 	[m,n] = size(reliable_mat);
	%将target_mat和reliable_mat进行时间匹配
	err_time = (reliable_mat(1,5) - target_mat(1,5))/emulation_step_time;
	target_mat(1:err_time,:) = [];
 	for i = m+1:frames
 		reliable_mat(i,:) = reliable_mat(i-1,:);
		target_mat(i,:) = target_mat(i-1,:);
 	end
	%得到平方和
	err_NNSF_L = err_NNSF_L + (target_mat(:,1) - reliable_mat(:,1)).^2 + (target_mat(:,2) - reliable_mat(:,2)).^2;
	err_NNSF_S = err_NNSF_S + (target_mat(:,3) - reliable_mat(:,3)).^2 + (target_mat(:,4) - reliable_mat(:,4)).^2;

	target_mat = target_mat_temp;
	reliable_mat = reliable_track_table_PDA(1).point_buff;
 	[m,n] = size(reliable_mat);
	%将target_mat和reliable_mat进行时间匹配
	err_time = (reliable_mat(1,5) - target_mat(1,5))/emulation_step_time;
	target_mat(1:err_time,:) = [];
 	for i = m+1:frames
 		reliable_mat(i,:) = reliable_mat(i-1,:);
		target_mat(i,:) = target_mat(i-1,:);
 	end
	err_PDA_L = err_PDA_L + (target_mat(:,1) - reliable_mat(:,1)).^2 + (target_mat(:,2) - reliable_mat(:,2)).^2;
	err_PDA_S = err_PDA_S + (target_mat(:,3) - reliable_mat(:,3)).^2 + (target_mat(:,4) - reliable_mat(:,4)).^2;

	if j == N
		target_mat_last = target_mat_temp;
		obs_target_mat_last = obs_target_mat;
		reliable_mat_NNSF_last = reliable_track_table_NNSF(1).point_buff;
		reliable_mat_PDA_last = reliable_track_table_PDA(1).point_buff;
	end

	%帧数显示
	j


end

err_NNSF_L = sqrt(err_NNSF_L./N); err_NNSF_S = sqrt(err_NNSF_S./N);
err_PDA_L = sqrt(err_PDA_L./N); err_PDA_S = sqrt(err_PDA_S./N);

font_size = 20;

%真实、量测和滤波轨迹二维图
f1 = figure('Name','真实、量测和滤波轨迹','NumberTitle','off');
figure(f1);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('坐标x/m','FontSize',font_size);
ylabel('坐标y/m','FontSize',font_size);
%axis([0,1000,0,1000]);
hold on;
grid on;
box on;
plot(target_mat_last(:,1),target_mat_last(:,2),'-+','linewidth',2,'color','#C52A20'); %真实轨迹
plot(obs_target_mat_last(:,1),obs_target_mat_last(:,2),'.','linewidth',1,'color','#FB9A99'); %量测轨迹
plot(reliable_mat_NNSF_last(:,1),reliable_mat_NNSF_last(:,2),'-','linewidth',2,'color','#EDB120'); %NNSF
plot(reliable_mat_PDA_last(:,1),reliable_mat_PDA_last(:,2),'--','linewidth',2,'color','#A5DAD1'); %PDA
%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
legend('真实轨迹','量测轨迹','NNSF','PDA','FontSize',font_size);

x_step = 1:frames;
%位置均方根误差
f2 = figure('Name','位置均方根误差','NumberTitle','off');
figure(f2);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('跟踪步数','FontSize',font_size);
ylabel('位置均方根误差/(m)','FontSize',font_size);
hold on;
grid on;
box on;
plot(x_step,err_NNSF_L,'-','linewidth',2,'color','#EDB120');
plot(x_step,err_PDA_L,'-','linewidth',2,'color','#4DBEEE');
legend('NNSF','PDA','FontSize',font_size);

%速度均方根误差
f3 = figure('Name','速度均方根误差','NumberTitle','off');
figure(f3);
set(gca,'FontSize',16);	%设置坐标刻度字体
xlabel('跟踪步数','FontSize',font_size);
ylabel('速度均方根误差/(m/s)','FontSize',font_size);
hold on;
grid on;
box on;
plot(x_step,err_NNSF_S,'-','linewidth',2,'color','#EDB120');
plot(x_step,err_PDA_S,'-','linewidth',2,'color','#4DBEEE');
legend('NNSF','PDA','FontSize',font_size);



%速度均方根误差


%%真实、量测和滤波轨迹二维图
%f1 = figure('Name','真实、量测和滤波轨迹','NumberTitle','off');
%figure(f1);
%title('真实、量测和滤波轨迹2D','FontSize',15);
%xlabel('坐标x/m','FontSize',15);
%ylabel('坐标y/m','FontSize',15);
%%axis([0,1000,0,1000]);
%hold on;
%grid on;
%plot(target_mat(:,1),target_mat(:,2),'-+','linewidth',2,'color','#C52A20'); %真实轨迹
%plot(obs_target_mat(:,1),obs_target_mat(:,2),'.','linewidth',1,'color','#FB9A99'); %量测轨迹
%reliable_mat = reliable_track_table_NNSF(1).point_buff;
%plot(reliable_mat(:,1),reliable_mat(:,2),'-','linewidth',2,'color','#EDB120'); %NNSF
%reliable_mat = reliable_track_table_PDA(1).point_buff;
%plot(reliable_mat(:,1),reliable_mat(:,2),'--','linewidth',2,'color','#A5DAD1'); %PDA
%%plot(filter_target_mat(:,1),filter_target_mat(:,2),'y-','linewidth',2);
%legend('真实轨迹','量测轨迹','NNSF','PDA','FontSize',9);

%删除库
rmpath(genpath('./radar2D_lib'));