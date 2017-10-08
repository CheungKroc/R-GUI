% ���ű�����RTB������ʵ�� STR6-05�����˵Ķ���ѧ����
% ��Ҫ�������£�
%  1����ȡ�켣
%  2���ٶȹ滮 ��ʹ�����б�д�������ٶȹ滮��S���ٶȹ滮��
%  3���˶�ѧ��⣨������ξ������У��ùؽڽ����У�
%  4����ùؽڽ�����ÿһʱ�̶�Ӧ�Ĺؽ��ٶȡ��ؽڼ��ٶȣ�����õ�����أ���������ֵ�ݶȷ�����ã�Ҳ�������ſɱȾ�����ã���������B����������ϵõ���
%  5�����ùؽ����л�ͼ���ɶ��� ���붯��ѧ����������ϵ���󣬵��ܹ���Ϊ����������ȷ���ж�����֮һ��

%----------����ģ��-------------------------%
close all;
mdl_str6_05();

%---------------------------------------end----%

%-------------��ȡ�켣-------------------------%
StateWind=figure(2); %������ʾ�������˶�״̬�Ĵ���
hold on;
StateWind.OuterPosition=[0,0,1360,1280];
% subplot(311);  %�ֽ��ٶ���ʾ
% xaxis(0,1);
% subplot(312);
% xaxis(0,1);
% yaxis(0,1.5);
% subplot(313);
% xaxis(0,1);

subplot(3,5,6); %�����ռ��ٶ���ͼ
title('v  of line function4(end-effector)');

subplot(3,5,2); %�ؽڿռ�������ͼ 
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

subplot(3,5,4); %�ؽ�ֵ��ͼ 
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



VisonWind=figure(1); %������ʾ�����˵��˶��Ĵ���
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


%---------------�켣�ٶȹ滮----------------------%
% �ο����ף��򴫺�_�������ɶȹ�ҵ�����˹켣�滮�㷨�о���
% �ڲ�����һ���Ĺ켣�У�Ӧ�������ٶȹ滮�����ܽ�������ֱ�߹켣
%  2017/8/28 by K.roc
[u,t]=SCurveBlend(@line_function4, 0, 0,0.4,0.6,6,100); %S���ٶȹ滮���� success ���a
%[u,t]=SCurveBlend(@line_function4, 0, 0,0.8,0.6,1,100); %S���ٶȹ滮���� success ���b
%[u,t]=SCurveBlend(@line_function4, 0, 0.3,0.4,0.6,2,100); %S���ٶȹ滮���� success ���c �ǶԳ�
%[u,t]=SCurveBlend(@line_function4, 0, 0.35,0.8,0.6,1,100); %S���ٶȹ滮���� success ���d �ǶԳ�
%[u,t]=SCurveBlend(@line_function4, 0.3, 0,0.4,0.6,2,100); %S���ٶȹ滮���� success ���e  �ǶԳ�
%[u,t]=SCurveBlend(@line_function4, 0.35, 0,0.8,0.6,1,100); %S���ٶȹ滮���� success ���f  �ǶԳ�
%[u,t]=SCurveBlend(@line_function4, 0.3, 0.3,0.4,0.6,2,100); %S���ٶȹ滮���� success ���e
%[u,t]=SCurveBlend(@line_function4, 0, 0,0.4,0.6,0.8,100); %S���ٶȹ滮���� success ���h
%[u,t]=SCurveBlend(@line_function4, 0, 0.1,0.4,0.6,0.8,100); %S���ٶȹ滮���� success ���h �ǶԳ�
% [u,t]=ParabolicBlend(@line_function4, 0.8, 2.5, 100); %�����ٶȹ滮����
%u=0:0.01:1; �����ٶȹ滮
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


%------------------�˶�ѧ���---------------%
Q= str6_05.ikine(T);
 
T_real=str6_05.fkine(Q);  % ����ѧ����������֤

vx=gradient(squeeze(T_real(1,4,:) ) )./gradient(t); %ͨ����ֵ�ݶȷ���������ģ��΢�������ռ��ٶ�
vy=gradient(squeeze(T_real(2,4,:) ) )./gradient(t);
vz=gradient(squeeze(T_real(3,4,:) ) )./gradient(t);
v=sqrt(vx.^2+vy.^2+vz.^2);


line_real=animatedline( 'color',[ 1 0.753 0], 'LineWidth',6 );%�����ռ�λ������

figure(2);
% subplot(311);  %�ֽ��ٶȵ���ʾ
% vx_real=animatedline('color',[1 0 0]);
% subplot(312);
% vy_real=animatedline('color',[0 1 0]);
% subplot(313);
% vz_real=animatedline('color',[0 0 1]);
subplot(356);
v_real=animatedline('color',[ 1 0 0]); %�����ռ��ٶ�����
%-------------------------------------------------end-%

%------------------�涯��ѧ---------------------%
QD=zeros(n,6);
QDD=zeros(n,6);
for i=1:6
QD(:,i)= gradient(Q(:,i))./gradient(t);     %ͨ����ֵ�ݶȷ�����ùؽ��ٶȺ͹ؽڼ��ٶ�
QDD(:,i)= gradient(QDD(:,i))./gradient(t);
end
Grav=[0 0 -9.78];
Torque= str6_05.rne(Q,QD,QDD);

%----------------------------------------------end%

%--------------------��������-------------------%
for i=1:n
    figure(1);
    str6_05.plot(Q(i,:));  %�����˼�ͼ����
    addpoints(line_real, T_real(1,4,i), T_real(2,4,i), T_real(3,4,i));  %�����ռ�λ�����߶���
    drawnow 
%      name=['picture', num2str(i)];   %��֡��������
%      saveas(gcf,name,'png');

    figure(2); 
%     subplot(311); %�����ٶȷֽ�ͼ
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
    addpoints(v_real,t(i,1),v(i,1));   %�����ռ��ٶ����߶���
    drawnow
    
    subplot(3,5,2);
    addpoints(Torque1,t(i,1),Torque(i,1));   %�ؽ��ٶ����߶���
    drawnow
    
    subplot(3,5,3);
    addpoints(Torque2,t(i,1),Torque(i,2));   %�ؽ��ٶ����߶���
    drawnow
    
    subplot(3,5,7);
    addpoints(Torque3,t(i,1),Torque(i,3));   %�ؽ��ٶ����߶���
    drawnow
    
    subplot(3,5,8);
    addpoints(Torque4,t(i,1),Torque(i,4));   %�ؽ��ٶ����߶���
    drawnow
    
    subplot(3,5,12);
    addpoints(Torque5,t(i,1),Torque(i,5));   %�ؽ��ٶ����߶���
    drawnow
    
    subplot(3,5,13);
    addpoints(Torque6,t(i,1),Torque(i,6));   %�ؽ��ٶ����߶���
    drawnow
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(3,5,4);
    addpoints(q1,t(i,1),Q(i,1));   %�ؽ����߶���
    drawnow
    
    subplot(3,5,5);
    addpoints(q2,t(i,1),Q(i,2));   %�ؽ����߶���
    drawnow
    
    subplot(3,5,9);
    addpoints(q3,t(i,1),Q(i,3));   %�ؽ����߶���
    drawnow
    
    subplot(3,5,10);
    addpoints(q4,t(i,1),Q(i,4));   %�ؽ����߶���
    drawnow
    
    subplot(3,5,14);
    addpoints(q5,t(i,1),Q(i,5));   %�ؽ����߶���
    drawnow
    
    subplot(3,5,15);
    addpoints(q6,t(i,1),Q(i,6));   %�ؽ����߶���
    drawnow

end
% figure(2);
% name='result_pic';   %��֡��������
%  saveas(gcf,name,'png');
%----------------------------------------------end%