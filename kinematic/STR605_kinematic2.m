% 本脚本利用RTB工具箱实现 STR6-05机器人的运动学仿真
% 主要流程如下：
%  1、读取轨迹
%  2、速度规划 （ctrai jtraj等，以及自行编写的梯形速度规划和S型速度规划等）
%  3、运动学逆解（传递齐次矩阵序列，得关节解序列）
%  4、利用关节序列绘图生成动画

%----------建立模型-------------------------%
close all;
mdl_str6_05();

%-------------------------------------------%

%-------------读取轨迹-------------------------%
StateWind=figure(2); %用于显示机器人运动状态的窗口
hold on;
StateWind.OuterPosition=[0,100,600,900];
% subplot(311);
% xaxis(0,1);
% subplot(312);
% xaxis(0,1);
% yaxis(0,1.5);
% subplot(313);
% xaxis(0,1);

VisonWind=figure(1); %用于显示机器人的运动的窗口
hold on;
VisonWind.OuterPosition=[800,250,600,800];

line_ideal=animatedline('color', [0 0 1]);
axis([-0.5,0.8,-0.5,1,-0.6,1]);
view(177,20);
a=tic;
for i = 0:0.01:1
    n=uint32(100*i+1);
    [position(n,1),position(n,2),position(n,3)] = line_function1(i);
    addpoints(line_ideal,position(n,1),position(n,2),position(n,3) );
    b=toc(a);
    if b>(1/3000)
       drawnow
       a=tic;
    end
     
end
str6_05.plot(qz);


%---------------轨迹速度规划----------------------%
% 参考文献：万传恒_《六自由度工业机器人轨迹规划算法研究》
% 在参数归一化的轨迹中，应用梯形速度规划，可能仅适用于直线轨迹
%  2017/8/28 by K.roc
%[u,t]=SCurveBlend(@line_function1, 0, 0,0.4,0.6,6,100); %S型速度规划函数 success 情况a
%[u,t]=SCurveBlend(@line_function1, 0, 0,0.8,0.6,1,100); %S型速度规划函数 success 情况b
%[u,t]=SCurveBlend(@line_function1, 0, 0.3,0.4,0.6,2,100); %S型速度规划函数 success 情况c
%[u,t]=SCurveBlend(@line_function1, 0, 0.35,0.8,0.6,1,100); %S型速度规划函数 success 情况d
%[u,t]=SCurveBlend(@line_function1, 0.3, 0,0.4,0.6,2,100); %S型速度规划函数 success 情况e
[u,t]=SCurveBlend(@line_function1, 0.35, 0,0.8,0.6,1,100); %S型速度规划函数 success 情况f
%[u,t]=SCurveBlend(@line_function1, 0.3, 0.3,0.4,0.6,2,100); %S型速度规划函数 success 情况e
%[u,t]=SCurveBlend(@line_function1, 0, 0,0.4,0.6,0.8,100); %S型速度规划函数 success 情况h
%[u,t]=SCurveBlend(@line_function1, 0, 0.1,0.4,0.6,0.8,100); %S型速度规划函数 success 情况h 非对称
% [u,t]=ParabolicBlend(@line_function1, 0.8, 2.5, 100); %梯形速度规划函数
%u=0:0.01:1; 均匀速度规划
clear position;
[position(:,1),position(:,2),position(:,3)] = line_function1(u);

pose= [ 0  0   1
        0  -1  0 
        1  0   0];
n=size(position,1);
T= zeros(4,4,n);
T(1:3,4,:)=position(:,1:3).';
for i=1:n
T(1:3,1:3,i)= pose;
end

%------------------运动学逆解---------------%
Q= str6_05.ikine(T);
 
T_real=str6_05.fkine(Q);  % 运行学正解用以验证

vx=gradient(squeeze(T_real(1,4,:) ) )./gradient(t); %通过数值梯度方法，近似模拟微分求速度
vy=gradient(squeeze(T_real(2,4,:) ) )./gradient(t);
vz=gradient(squeeze(T_real(3,4,:) ) )./gradient(t);
v=sqrt(vx.^2+vy.^2+vz.^2);


line_real=animatedline( 'color',[ 1 0.753 0], 'LineWidth',6 );

figure(2);
% subplot(311);  %分解速度的显示
% vx_real=animatedline('color',[1 0 0]);
% subplot(312);
% vy_real=animatedline('color',[0 1 0]);
% subplot(313);
% vz_real=animatedline('color',[0 0 1]);
v_real=animatedline('color',[ 1 0 0]);

for i=1:n
    figure(1);
    str6_05.plot(Q(i,:));
    addpoints(line_real, T_real(1,4,i), T_real(2,4,i), T_real(3,4,i));
    drawnow 
%      name=['picture', num2str(i)];   %将帧动画保存
%      saveas(gcf,name,'png');

    figure(2); 
%     subplot(311); %更新速度分解图
%     addpoints(vx_real, t(i,1), vx(i,1) );
%     drawnow 
%     
%      subplot(312);
%     addpoints(vy_real, t(i,1), vy(i,1) );
%     drawnow  
%     
%     subplot(313);
%     addpoints(vz_real, t(i,1), vz(i,1) );
%     drawnow 
addpoints(v_real,t(i,1),v(i,1));
end




