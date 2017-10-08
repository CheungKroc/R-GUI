% ���ű�����RTB������ʵ�� STR6-05�����˵��˶�ѧ����
% ��Ҫ�������£�
%  1����ȡ�켣
%  2���ٶȹ滮 ��ctrai jtraj�ȣ��Լ����б�д�������ٶȹ滮��S���ٶȹ滮�ȣ�
%  3���˶�ѧ��⣨������ξ������У��ùؽڽ����У�
%  4�����ùؽ����л�ͼ���ɶ���

%----------����ģ��-------------------------%
close all;
mdl_str6_05();

%-------------------------------------------%

%-------------��ȡ�켣-------------------------%
StateWind=figure(2); %������ʾ�������˶�״̬�Ĵ���
hold on;
StateWind.OuterPosition=[0,100,600,900];
% subplot(311);
% xaxis(0,1);
% subplot(312);
% xaxis(0,1);
% yaxis(0,1.5);
% subplot(313);
% xaxis(0,1);

VisonWind=figure(1); %������ʾ�����˵��˶��Ĵ���
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


%---------------�켣�ٶȹ滮----------------------%
% �ο����ף��򴫺�_�������ɶȹ�ҵ�����˹켣�滮�㷨�о���
% �ڲ�����һ���Ĺ켣�У�Ӧ�������ٶȹ滮�����ܽ�������ֱ�߹켣
%  2017/8/28 by K.roc
%[u,t]=SCurveBlend(@line_function1, 0, 0,0.4,0.6,6,100); %S���ٶȹ滮���� success ���a
%[u,t]=SCurveBlend(@line_function1, 0, 0,0.8,0.6,1,100); %S���ٶȹ滮���� success ���b
%[u,t]=SCurveBlend(@line_function1, 0, 0.3,0.4,0.6,2,100); %S���ٶȹ滮���� success ���c
%[u,t]=SCurveBlend(@line_function1, 0, 0.35,0.8,0.6,1,100); %S���ٶȹ滮���� success ���d
%[u,t]=SCurveBlend(@line_function1, 0.3, 0,0.4,0.6,2,100); %S���ٶȹ滮���� success ���e
[u,t]=SCurveBlend(@line_function1, 0.35, 0,0.8,0.6,1,100); %S���ٶȹ滮���� success ���f
%[u,t]=SCurveBlend(@line_function1, 0.3, 0.3,0.4,0.6,2,100); %S���ٶȹ滮���� success ���e
%[u,t]=SCurveBlend(@line_function1, 0, 0,0.4,0.6,0.8,100); %S���ٶȹ滮���� success ���h
%[u,t]=SCurveBlend(@line_function1, 0, 0.1,0.4,0.6,0.8,100); %S���ٶȹ滮���� success ���h �ǶԳ�
% [u,t]=ParabolicBlend(@line_function1, 0.8, 2.5, 100); %�����ٶȹ滮����
%u=0:0.01:1; �����ٶȹ滮
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

%------------------�˶�ѧ���---------------%
Q= str6_05.ikine(T);
 
T_real=str6_05.fkine(Q);  % ����ѧ����������֤

vx=gradient(squeeze(T_real(1,4,:) ) )./gradient(t); %ͨ����ֵ�ݶȷ���������ģ��΢�����ٶ�
vy=gradient(squeeze(T_real(2,4,:) ) )./gradient(t);
vz=gradient(squeeze(T_real(3,4,:) ) )./gradient(t);
v=sqrt(vx.^2+vy.^2+vz.^2);


line_real=animatedline( 'color',[ 1 0.753 0], 'LineWidth',6 );

figure(2);
% subplot(311);  %�ֽ��ٶȵ���ʾ
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
addpoints(v_real,t(i,1),v(i,1));
end




