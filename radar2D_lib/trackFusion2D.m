%改文件为航迹融合相关函数
function out = trackFusion2D()
	out.KNN = @KNN;
	out.CI = @CI;
	out.Convex = @Convex;
end

%k近邻域关联
%判断航迹radar1_tracks和航迹radar2_tracks 在关联区域的关联成功率
function out = KNN(radar1_tracks,radar2_tracks,err)
	[m1,n1] = size(radar1_tracks); %雷达1航迹数量
	[m2,n2] = size(radar2_tracks); %雷达2航迹数量
	track_use_v = []; %用于记录融合航迹的位置
	dmin = sum(err) + 3;
	for i = 1 : n1
		K = 0; %K的初始值
		for j = 1 : n2
			%取四帧的数据作关联
			err_table = abs(radar1_tracks(i).point_buff(end-3:end,1:4) - radar2_tracks(j).point_buff(end-3:end,1:4));
			for k = 1 : 4
				if err_table(k,1) < err(1) && err_table(k,2) < err(2) && err_table(k,3) < err(3) && err_table(k,4) < err(4)
					K = K + 1;
				end
			end
			if K >= 3 %如果K大于三则说明可以融合
				track_use_v = [track_use_v,j]; %记录下标
			end
		end
	end
end

%快速CI法航迹融合
%输入为要融合的航迹，输出为融合后的航迹
function out = CI(fusion_track,radar1_track,radar2_track,time)
	P1 = radar1_track.P_K;
	P2 = radar2_track.P_K;
	X1 = radar1_track.X_K;
	X2 = radar2_track.X_K;
	det_P12 = det(P1 + P2);
	w1 = ( det_P12 - det(P2) + det(P1) ) / ( 2*det_P12 );
	w2 = 1 - w1;
	P_K = inv(w1 * inv(P1) + w2 * inv(P2));
	X_K = P_K * ( w1 * inv(P1)* X1 + w2 * inv(P2) * X2 );
	fusion_track.X_K = X_K;
	fusion_track.P_K = P_K;
	fusion_track.point_buff = [fusion_track.point_buff;X_K(1),X_K(3),X_K(2),X_K(4),time];
	out = fusion_track;
end

% 凸组合算法航迹融合
function out = Convex(fusion_track,radar1_track,radar2_track,time)
	P1 = radar1_track.P_K;
	P2 = radar2_track.P_K;
	X1 = radar1_track.X_K;
	X2 = radar2_track.X_K;
	P_K = inv( inv(P1) + inv(P2) );
	X_K = P_K * ( inv(P1) * X1 + inv(P2) * X2 );
	fusion_track.X_K = X_K;
	fusion_track.P_K = P_K;
	fusion_track.point_buff = [fusion_track.point_buff;X_K(1),X_K(3),X_K(2),X_K(4),time];
	out = fusion_track;
end




























