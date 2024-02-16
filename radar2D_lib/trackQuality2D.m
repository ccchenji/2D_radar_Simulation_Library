%航迹质量管理
function out = trackQuality2D ()
	out.scoreFunStart = @scoreFunStart;
	out.scoreFunReliable = @scoreFunReliable;
end

%打分法可靠航迹质量管理
%reliable_track_table 可靠航迹表
%out 为更新的可靠航迹和已经消亡的航迹
function out = scoreFunReliable (reliable_track_table,delete_track_table)
	[m,n] = size(reliable_track_table);
	%需要删除的航迹
	delete_track = [];
	for i = 1:n
		now_track = reliable_track_table(i);
		if now_track.if_have_point == 1 %如果当前录取有点迹
			now_track.score = now_track.score + 1;
			if now_track.score >= 20
				now_track.score = 20;
			end
		else
			now_track.score = now_track.score - 2;
		end

		if now_track.score <= 0
			delete_track = [delete_track,i];
		end
		reliable_track_table(i) =now_track;
	end
	delete_track_table = [delete_track_table,reliable_track_table(delete_track)]; %得到消亡的航迹
	reliable_track_table(delete_track) = []; %删除消亡的航迹
	out = {reliable_track_table,delete_track_table};
end

%打分法起始航迹质量管理
%start_track_table 起始航迹表
%reliable_track 可靠航迹表
%out ,更新后的起始航迹和可靠航迹表
function out = scoreFunStart(start_track_table,reliable_track_table,q,r,step_time)
	filters = filters2D();
	[m,n] = size(start_track_table);
	%需要从起始航迹表中删除的航迹
	delete_track = [];
	for i = 1:n
		now_track = start_track_table(i);
		if now_track.win_now_time >= now_track.win_size %时间窗到了
			if now_track.win_point_num / now_track.win_size >= 3/4
				now_track = filters.initKf(now_track,q,r,step_time); %初始化航迹卡尔曼滤波参数
				now_track.score = 7; %航迹初始分数为7
				reliable_track_table = [reliable_track_table,now_track]; %将起始航迹提升为可靠航迹
			end
			delete_track = [delete_track,i];
		end
		start_track_table(i) = now_track;
	end
	start_track_table(delete_track) = [];
	out = {start_track_table,reliable_track_table};
end
