% 本脚本利用RTB工具箱实现 STR6-05机器人的运动学仿真
% 主要流程如下：
%  1、读取轨迹
%  2、速度规划 （ctrai jtraj等，以及自行编写的梯形速度规划和S型速度规划等）
%  3、运动学逆解（传递齐次矩阵序列，得关节解序列）
%  4、利用关节序列绘图生成动画

%----------建立模型-------------------------%
mdl_str6_05;

%-------------------------------------------%

%-------------读取轨迹-------------------------%
figure(1); %用于显示机器人的运动的窗口
line_ideal=animatedline('color', [0 0 1]);
axis([-0.5,0.8,-0.5,1,-0.6,1]);   %与axes区别
view(89,20);
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
hold on;
str6_05.plot(qz);


%---------------轨迹速度规划----------------------%
% 参考文献：万传恒_《六自由度工业机器人轨迹规划算法研究》
% 在参数归一化的轨迹中，应用梯形速度规划，可能仅适用于直线轨迹
%  2017/8/28 by K.roc
[u,t]=ParabolicBlend(@line_function4, 4, 2.5, 100); %梯形速度规划函数
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

%------------------运动学逆解---------------%
Q= str6_05.ikine(T);
 
T_real=str6_05.fkine(Q);  % 运行学正解用以验证

line_real=animatedline( 'color',[ 1 0.753 0], 'LineWidth',6 );

for i=1:n
     str6_05.plot(Q(i,:));
    addpoints(line_real, T_real(1,4,i), T_real(2,4,i), T_real(3,4,i));
    drawnow 
%      name=['picture', num2str(i)];   %将帧动画保存
%      saveas(gcf,name,'png');

end





