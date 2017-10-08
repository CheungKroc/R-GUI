% 本脚本利用RTB工具箱实现 STR6-05机器人的动力学仿真
% 主要流程如下：
%  1、读取轨迹
%  2、速度规划 （使用自行编写的梯形速度规划和S型速度规划）
%  3、运动学逆解（传递齐次矩阵序列，得关节解序列）
%  4、获得关节解序列每一时刻对应的关节速度、关节加速度，以求得电机力矩（可以是数值梯度方法求得，也可以是雅可比矩阵求得，还可以是B样条曲线拟合得到）
%  5、利用关节序列绘图生成动画 （与动力学的运算结果关系不大，但能够作为力矩运算正确的判断依据之一）

%----------建立模型-------------------------%
close all;
mdl_str6_05();

%---------------------------------------end----%

%-------------读取轨迹-------------------------%
StateWind=figure(2); %用于显示机器人运动状态的窗口
hold on;
StateWind.OuterPosition=[0,0,1360,1280];
% subplot(311);  %分解速度显示
% xaxis(0,1);
% subplot(312);
% xaxis(0,1);
% yaxis(0,1.5);
% subplot(313);
% xaxis(0,1);

subplot(3,5,6); %工作空间速度子图
title('v  of line function4(end-effector)');

subplot(3,5,2); %关节空间力矩子图 
title('Torque1');
Torque1=animatedline('color',[0  0 1]);
subplot(3,5,3);
title('Torque2');
Torque2=animatedline('color',[0  1 0]);
subplot(3,5,7);
title('Torque3');
Torque3=animatedline('color',[0  1 1]);
subplot(3,5,8);
title('Torque4');
Torque4=animatedline('color',[1  0 0]);
subplot(3,5,12);
title('Torque5');
Torque5=animatedline('color',[1  0 1]);
subplot(3,5,13);
title('Torque6');
Torque6=animatedline('color',[1  1 0]);

subplot(3,5,4); %关节值子图 
title('q1');
q1=animatedline('color',[0  0 1]);
subplot(3,5,5);
title('q2');
q2=animatedline('color',[0  1 0]);
subplot(3,5,9);
title('q3');
q3=animatedline('color',[0  1 1]);
subplot(3,5,10);
title('q4');
q4=animatedline('color',[1  0 0]);
subplot(3,5,14);
title('q5');
q5=animatedline('color',[1  0 1]);
subplot(3,5,15);
title('q6');
q6=animatedline('color',[1  1 0]);



VisonWind=figure(1); %用于显示机器人的运动的窗口
hold on;
VisonWind.OuterPosition=[1360,250,600,800];

line_ideal=animatedline('color', [0 0 1]);
axis([-0.5,0.8,-0.5,1,-0.6,1]);
view(177,20);
a=tic;
for i = 0:0.01:1
    n=uint32(100*i+1);
    [position(n,1),position(n,2),position(n,3)] = line_function4(i);
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
[u,t]=SCurveBlend(@line_function4, 0, 0,0.4,0.6,6,100); %S型速度规划函数 success 情况a
%[u,t]=SCurveBlend(@line_function4, 0, 0,0.8,0.6,1,100); %S型速度规划函数 success 情况b
%[u,t]=SCurveBlend(@line_function4, 0, 0.3,0.4,0.6,2,100); %S型速度规划函数 success 情况c 非对称
%[u,t]=SCurveBlend(@line_function4, 0, 0.35,0.8,0.6,1,100); %S型速度规划函数 success 情况d 非对称
%[u,t]=SCurveBlend(@line_function4, 0.3, 0,0.4,0.6,2,100); %S型速度规划函数 success 情况e  非对称
%[u,t]=SCurveBlend(@line_function4, 0.35, 0,0.8,0.6,1,100); %S型速度规划函数 success 情况f  非对称
%[u,t]=SCurveBlend(@line_function4, 0.3, 0.3,0.4,0.6,2,100); %S型速度规划函数 success 情况e
%[u,t]=SCurveBlend(@line_function4, 0, 0,0.4,0.6,0.8,100); %S型速度规划函数 success 情况h
%[u,t]=SCurveBlend(@line_function4, 0, 0.1,0.4,0.6,0.8,100); %S型速度规划函数 success 情况h 非对称
% [u,t]=ParabolicBlend(@line_function4, 0.8, 2.5, 100); %梯形速度规划函数
%u=0:0.01:1; 均匀速度规划
clear position;
[position(:,1),position(:,2),position(:,3)] = line_function4(u);

pose= [ 0  0   1
        0  -1  0 
        1  0   0];
n=size(position,1);
T= zeros(4,4,n);
T(1:3,4,:)=position(:,1:3).';
for i=1:n
T(1:3,1:3,i)= pose;
end
%----------------------------------------------end%


%------------------运动学逆解---------------%
Q= str6_05.ikine(T);
 
T_real=str6_05.fkine(Q);  % 运行学正解用以验证

vx=gradient(squeeze(T_real(1,4,:) ) )./gradient(t); %通过数值梯度方法，近似模拟微分求工作空间速度
vy=gradient(squeeze(T_real(2,4,:) ) )./gradient(t);
vz=gradient(squeeze(T_real(3,4,:) ) )./gradient(t);
v=sqrt(vx.^2+vy.^2+vz.^2);


line_real=animatedline( 'color',[ 1 0.753 0], 'LineWidth',6 );%工作空间位置曲线

figure(2);
% subplot(311);  %分解速度的显示
% vx_real=animatedline('color',[1 0 0]);
% subplot(312);
% vy_real=animatedline('color',[0 1 0]);
% subplot(313);
% vz_real=animatedline('color',[0 0 1]);
subplot(356);
v_real=animatedline('color',[ 1 0 0]); %工作空间速度曲线
%-------------------------------------------------end-%

%------------------逆动力学---------------------%
QD=zeros(n,6);
QDD=zeros(n,6);
for i=1:6
QD(:,i)= gradient(Q(:,i))./gradient(t);     %通过数值梯度方法获得关节速度和关节加速度
QDD(:,i)= gradient(QDD(:,i))./gradient(t);
end
Grav=[0 0 -9.78];
Torque= str6_05.rne(Q,QD,QDD);

%----------------------------------------------end%

%--------------------动画生成-------------------%
for i=1:n
    figure(1);
    str6_05.plot(Q(i,:));  %机器人简图动画
    addpoints(line_real, T_real(1,4,i), T_real(2,4,i), T_real(3,4,i));  %工作空间位置曲线动画
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
    subplot(3,5,6);
    addpoints(v_real,t(i,1),v(i,1));   %工作空间速度曲线动画
    drawnow
    
    subplot(3,5,2);
    addpoints(Torque1,t(i,1),Torque(i,1));   %关节速度曲线动画
    drawnow
    
    subplot(3,5,3);
    addpoints(Torque2,t(i,1),Torque(i,2));   %关节速度曲线动画
    drawnow
    
    subplot(3,5,7);
    addpoints(Torque3,t(i,1),Torque(i,3));   %关节速度曲线动画
    drawnow
    
    subplot(3,5,8);
    addpoints(Torque4,t(i,1),Torque(i,4));   %关节速度曲线动画
    drawnow
    
    subplot(3,5,12);
    addpoints(Torque5,t(i,1),Torque(i,5));   %关节速度曲线动画
    drawnow
    
    subplot(3,5,13);
    addpoints(Torque6,t(i,1),Torque(i,6));   %关节速度曲线动画
    drawnow
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(3,5,4);
    addpoints(q1,t(i,1),Q(i,1));   %关节曲线动画
    drawnow
    
    subplot(3,5,5);
    addpoints(q2,t(i,1),Q(i,2));   %关节曲线动画
    drawnow
    
    subplot(3,5,9);
    addpoints(q3,t(i,1),Q(i,3));   %关节曲线动画
    drawnow
    
    subplot(3,5,10);
    addpoints(q4,t(i,1),Q(i,4));   %关节曲线动画
    drawnow
    
    subplot(3,5,14);
    addpoints(q5,t(i,1),Q(i,5));   %关节曲线动画
    drawnow
    
    subplot(3,5,15);
    addpoints(q6,t(i,1),Q(i,6));   %关节曲线动画
    drawnow

end
% figure(2);
% name='result_pic';   %将帧动画保存
%  saveas(gcf,name,'png');
%----------------------------------------------end%