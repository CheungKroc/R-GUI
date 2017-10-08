% ���ű�����RTB������ʵ�� STR6-05�����˵��˶�ѧ����
% ��Ҫ�������£�
%  1����ȡ�켣
%  2���ٶȹ滮 ��ctrai jtraj�ȣ��Լ����б�д�������ٶȹ滮��S���ٶȹ滮�ȣ�
%  3���˶�ѧ��⣨������ξ������У��ùؽڽ����У�
%  4�����ùؽ����л�ͼ���ɶ���

%----------����ģ��-------------------------%
mdl_str6_05;

%-------------------------------------------%

%-------------��ȡ�켣-------------------------%
figure(1); %������ʾ�����˵��˶��Ĵ���
line_ideal=animatedline('color', [0 0 1]);
axis([-0.5,0.8,-0.5,1,-0.6,1]);   %��axes����
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


%---------------�켣�ٶȹ滮----------------------%
% �ο����ף��򴫺�_�������ɶȹ�ҵ�����˹켣�滮�㷨�о���
% �ڲ�����һ���Ĺ켣�У�Ӧ�������ٶȹ滮�����ܽ�������ֱ�߹켣
%  2017/8/28 by K.roc
[u,t]=ParabolicBlend(@line_function4, 4, 2.5, 100); %�����ٶȹ滮����
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

%------------------�˶�ѧ���---------------%
Q= str6_05.ikine(T);
 
T_real=str6_05.fkine(Q);  % ����ѧ����������֤

line_real=animatedline( 'color',[ 1 0.753 0], 'LineWidth',6 );

for i=1:n
     str6_05.plot(Q(i,:));
    addpoints(line_real, T_real(1,4,i), T_real(2,4,i), T_real(3,4,i));
    drawnow 
%      name=['picture', num2str(i)];   %��֡��������
%      saveas(gcf,name,'png');

end





