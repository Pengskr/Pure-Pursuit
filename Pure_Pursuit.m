% 纯跟踪算法的本质：
% 参考人类驾驶员的行为，以车的后轴为基点，通过控制前轮的偏角delta，
% 使车辆沿一条经过预瞄点的 圆弧 行驶
clc
clear
close all
% load path_S.mat
load path_Circle.mat
% load path_Circle_clockwise.mat

%% 相关参数定义
RefPos = path;
targetSpeed = 10;   % m/s
Kv = 0.1;           % 前视距离系数
Kp = 0.8;           % 速度P控制器系数
Ld0 = 2;            % Ld0 预瞄距离的下限值
dt = 0.1;           % 时间间隔，单位：s
L = 2.9;            % 车辆轴距，单位：m

% 纯跟踪本质是一个P控制器
Ki = 0.5;           % 积分调节系数
Err_integ = 0;

% 绘制参考轨迹
figure
plot(RefPos(:,1), RefPos(:,2), 'b', 'LineWidth', 2);
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
grid on;
grid minor
axis equal
hold on 

% 计算参考航向角
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
refHeading = atan2(diff_y , diff_x);    % 参考航向角

%% 车辆初始状态
pos = RefPos(1, :)+1;
v = 0;
heading = refHeading(1) + 0.02;

% 将初始状态存入实际状态数组中
pos_actual = pos;
heading_actual = heading;
delta_actual = 0;
v_actual = v;

%% 主程序

% 循环遍历轨迹点
idx = 1;
latError_PP = [];
sizeOfRefPos = size(RefPos, 1);
lookaheadPoint = [0, 0;];
while idx < sizeOfRefPos-1  % 由于diff_x，diff_y计算方式的特殊性，不考虑最后一个参考点
    % 寻找预瞄点
    [lookaheadPoint(end+1,:),idx] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0);

    % 计算控制量
    [delta, latError]  = pure_pursuit_control(lookaheadPoint(end,:), idx, pos, heading, v, RefPos,refHeading, Kv, Ld0,L);

    % 前轮转角 PI控制
    Err_integ = Err_integ + latError * dt;
    delta = delta + Ki * Err_integ;
    
    % 如果误差过大，退出循迹
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end

    % 计算加速度
    a = Kp* (targetSpeed-v)/dt;

    % 更新状态量
    [pos, heading, v] = updateState(a, pos, heading, v, delta, L, dt);
    
    % 保存每一步的实际量
    pos_actual(end+1, :) = pos;
    heading_actual(end+1, :) = heading;
    delta_actual(end+1, :) = delta;
    v_actual(end+1, :) = v;
    latError_PP(end+1, :) = [idx, latError];
end

%% 画图
% 跟踪轨迹
for i = 1:size(pos_actual,1)-1
    scatter(pos_actual(i,1), pos_actual(i,2), 200, '.r');    % 实际位置(x,y)
    % 绘制预瞄点
%     U = lookaheadPoint(i+1,:)-pos_actual(i,:);
%     f = quiver(pos_actual(i,1), pos_actual(i,2), U(1), U(2), 'g', 'LineWidth', 1);    
    f1 = plot([pos_actual(i,1), lookaheadPoint(i+1,1)], [pos_actual(i,2), lookaheadPoint(i+1,2)], 'g--', 'LineWidth', 0.1);
    f2 = scatter(lookaheadPoint(i+1,1), lookaheadPoint(i+1,2), 100, '.g');
    % 实际航向
    quiver(pos_actual(i,1), pos_actual(i,2), cos(heading_actual(i,:)), sin(heading_actual(i,:)),0.5, 'm', 'LineWidth', 1);     % 实际航向
    % 前轮转角
    quiver(pos_actual(i,1), pos_actual(i,2), cos(heading_actual(i,:)+delta_actual(i,:)), sin(heading_actual(i,:)+delta_actual(i,:)),0.2, 'k', 'LineWidth', 1);

    pause(0.01);
    delete(f1), delete(f2)
end
% 最后一个轨迹点没有预瞄点
i = size(pos_actual,1);
scatter(pos_actual(i,1), pos_actual(i,2), 200, '.r');    % 实际位置(x,y)
quiver(pos_actual(i,1), pos_actual(i,2), cos(heading_actual(i,:)), sin(heading_actual(i,:)), 0.5, 'm', 'LineWidth', 1);     % 实际航向
quiver(pos_actual(i,1), pos_actual(i,2), cos(heading_actual(i,:)+delta_actual(i,:)), sin(heading_actual(i,:)+delta_actual(i,:)),0.2, 'k', 'LineWidth', 1);
pause(0.01);
legend('参考车辆轨迹', '实际行驶轨迹', '实际航向')
hold off

% 横向误差
figure
subplot(1, 2, 1)
plot(latError_PP(:, 2));
grid on;
grid minor
title("横向误差")
ylabel('横向误差 / m');

% 前轮转角
subplot(1, 2, 2)
plot(delta_actual(:,1));
grid on; grid minor; title('前轮转角');

% 航向角
figure
subplot(1, 2, 1)
plot(refHeading);
grid on; grid minor; title('参考航向角');
subplot(1, 2, 2)
plot(heading_actual);
grid on; grid minor; title('实际航向角');

% 保存
save latError_PP.mat latError_PP


%% 子函数

function [lookaheadPoint,idx_target]=findLookaheadPoint(pos, v, RefPos, Kv, Ld0)
    % 找到距离当前位置最近的一个参考轨迹点的序号
    sizeOfRefPos = size(RefPos,1);
    for i = 1:sizeOfRefPos-1
        dist(i,1) = norm(RefPos(i,:) - pos);   
    end
    [~,idx] = min(dist); 

    % 从最近的参考轨迹点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
    L_steps = 0;    % 参考轨迹上几个相邻点的累计距离
    Ld = Kv*v + Ld0;% 预瞄距离
    while L_steps < Ld && idx < sizeOfRefPos
%         % 原定义
%         L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
        L_steps = norm(RefPos(idx + 1,:) - pos);
        idx = idx+1;
        % 我认为L_steps的定义不合理，应该定义L_steps为路径上的点到当前位置的距离norm(RefPos(idx + 1,:) - pos)
    end
    idx_target = idx;
    lookaheadPoint = RefPos(idx,:);

end

function [delta,latError] = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v,RefPos,refHeading, Kv, Ld0, L)
    sizeOfRefPos = size(RefPos,1);
    Point_temp = lookaheadPoint;

    alpha = atan2(Point_temp(1,2) - pos(2), Point_temp(1,1) - pos(1))  - heading;
    Ld = Kv*v + Ld0;

    % 求位置、航向角的误差
    x_error  = Point_temp(1,1) - pos(1);
    y_error = Point_temp(1,2) - pos(2);
    heading_r = refHeading(idx);
    % 根据百度Apolo，计算横向误差
%     latError = y_error*cos(heading_r) - x_error*sin(heading_r);
    latError = y_error*cos(heading) - x_error*sin(heading);     % 应当使用实际航向角计算横向误差
    
    % 前轮转角 P控制
%     delta = atan2(2*L*sin(alpha), Ld);  % 横向误差可用Ld,sina表示
    delta = atan2(2*L*latError, Ld^2);
end

function [pos_new, heading_new, v_new] = updateState(a, pos_old, heading_old, v_old,delta,wheelbase, dt)
    pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
    pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
    heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
    % 物理上航向角的绝对值小于等于180，需要对数学上算出的航向角作修正使其具有物理意义
    if abs(heading_new)>pi
        heading_new = (heading_new>0)*(heading_new-2*pi) + (heading_new<0)*(heading_new+2*pi);
    end 
    v_new =  v_old + a*dt;
end