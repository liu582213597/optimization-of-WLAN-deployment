%% 2021.3.22
clc;
clear;
close all;

%% ��������
data.C01=1;  %���򸲸���
data.a0=-92+zeros(3200,1);     %��֤�ն�����ͨ����������͹���
data.a_max=20;  %����AP������书�ʣ�dBm��
data.gama=2.5;  %·������ָ��
data.alpha=50;  %��׼����d0�Ĺ���
data.num_AP=15; %AP����
data.d0=1;     %��׼����
data.mapx=80;%��ͼX�᳤��
data.mapy=20;%��ͼY�᳤��
data.mapz=10;%��ͼZ�᳤��
%% �����������
data.node_number_x=data.mapx;
data.node_number_y=data.mapy;
data.node_gap=1; %Ŀ�������
data.node0_position=[5,5];
data.node_size=[data.node_number_x,data.node_number_y,2];  %80*20*2 ��3200����
data.size_map=[0,data.mapx,0,data.mapy,0 data.mapz]; %��ͼ��Χ
% ����ÿһ��Ŀ��������
data.node_position_x=zeros(3200,1);
data.node_position_y=zeros(3200,1);
data.node_position_z=zeros(3200,1);
for k=1:2
    for i=1:data.node_number_x
        for j=1:data.node_number_y
            data.node_position_x((k-1)*data.node_size(1)*data.node_size(2)+(i-1)*data.node_size(2)+j)=i*1-0.5;
            data.node_position_y((k-1)*data.node_size(1)*data.node_size(2)+(i-1)*data.node_size(2)+j)=j*1-0.5;
            if k==1
                data.node_position_z((k-1)*data.node_size(1)*data.node_size(2)+(i-1)*data.node_size(2)+j)=1;
            else 
                data.node_position_z((k-1)*data.node_size(1)*data.node_size(2)+(i-1)*data.node_size(2)+j)=k*10-14;
            end
        end 
    end
end
%% ���컯���ǵĲ�ͬ���չ���Ҫ��
for i=1:1600
    if data.node_position_x(i) < 31
        data.a0(i)=-92;
    else
        data.a0(i)=-73;
    end
end
for i=1601:3200
    if data.node_position_y(i) > 11.2
        data.a0(i)=-92;
    else
        data.a0(i)=-73;
    end
end

%% �ϰ����ʼ��
data.obstacle_x=[ %ǽ��λ�� x  y1 y2 z1 z2���� 
    %��¥
    0 0 12 0 10; 
    7.6 0 8.2 0 5;
    19.3 0 8.2 0 5;
    31 0 8.2 0 5;
    35.3 0 8.2 0 5;
    44.5 0 8.2 0 5;
    52.3 0 8.2 0 5;
    60.1 0 8.2 0 5;
    64 0 8.2 0 5;
    67.9 0 8.2 0 5;
    71.8 0 8.2 0 5;
    7.6 11.2 20 0 5;
    19.3 11.2 20 0 5;
    31 11.2 20 0 5;
    35.3 11.2 20 0 5;
    44.5 11.2 20 0 5;
    52.3 11.2 20 0 5;
    56.2 11.2 20 0 5;
    60.1 11.2 20 0 5;
    64 11.2 20 0 5;
    72.9 11.2 20 0 5;
    %��¥
    3.7 0 8.2 5 10;
    7.6 0 8.2 5 10;
    11.5 0 8.2 5 10;
    15.4 0 8.2 5 10;
    19.3 0 8.2 5 10;
    23.2 0 8.2 5 10;
    27.1 0 8.2 5 10;
    31 0 8.2 5 10;
    36.6 0 8.2 5 10; 
    41.5 0 8.2 5 10;
    45.6 0 8.2 5 10;
    49.5 0 8.2 5 10;
    53.4 0 8.2 5 10;
    57.3 0 8.2 5 10;
    61.2 0 8.2 5 10;
    65.1 0 8.2 5 10;
    69 0 8.2 5 10;
    72.9 0 8.2 5 10;
    7.6 11.2 20 5 10;
    19.3 11.2 20 5 10;
    31 11.2 20 5 10;
    49.5 11.2 20 5 10;
    57.3 11.2 20 5 10;
    64.9 11.2 20 5 10;
    77.9 11.2 20 5 10;
    ];
%% ��ͼ
% data.obstacle_size=[{'-sk'},{'-sy'},{'-sr'}];
% data.jingdu=0.1; %��ͼ����
% scatter3(data.node_position_x,data.node_position_y,data.node_position_z,'.');


%% �㷨����
option.peiod=10  ;  %Maximum Iterations ����������
option.genMax=option.peiod;
option.M=150;         %Population Size ��Ⱥ��ģ
option.popSize=option.M;
option.show_t=1;    %�Ƿ���ʾ��������
option.show_pc=1;   %�Ƿ���ʾ��������ͼ
option.creat_x=@creat_x_1;  
option.aimFcn=@aimFcn_1;
option.by=@bianyi_1;
option.c1 = 1;      %ѧϰ����c1
option.c2 = 1;      %ѧϰ����c2
% option.w=0.8;                  % PSO�������ԵĹ���Ȩ��
option.wmax=0.99;% PSO�������ԵĹ���Ȩ��w���ֵ
option.wmin=0.1; % PSO�������ԵĹ���Ȩ��w��Сֵ

%% ��ʼ��Ⱥ���� 
x=cell(option.popSize,1);
t=option.creat_x(option,data);
fit=zeros(option.popSize,1);
parfor j=1:option.popSize
    x{j}=option.creat_x(option,data);
    fit(j)=option.aimFcn(x{j},option,data);
    v{j}=rand(size(x{j}));
end

data.x=x;
data.fit=fit;
data.v=v;

%% 
tic
[fit_record1(i,:),best_result1]=PSO-Levy-DFOA(option,data);   %����Ѱ���㷨
toc


