%% 2021.3.22
clc;
clear;
close all;

%% 载入数据
data.C01=1;  %区域覆盖率
data.a0=-92+zeros(3200,1);     %保证终端正常通信质量的最低功率
data.a_max=20;  %无线AP的最大发射功率（dBm）
data.gama=2.5;  %路径耗损指数
data.alpha=50;  %基准距离d0的功率
data.num_AP=15; %AP个数
data.d0=1;     %基准距离
data.mapx=80;%地图X轴长度
data.mapy=20;%地图Y轴长度
data.mapz=10;%地图Z轴长度
%% 生成相关数据
data.node_number_x=data.mapx;
data.node_number_y=data.mapy;
data.node_gap=1; %目标对象间距
data.node0_position=[5,5];
data.node_size=[data.node_number_x,data.node_number_y,2];  %80*20*2 共3200个点
data.size_map=[0,data.mapx,0,data.mapy,0 data.mapz]; %地图范围
% 生成每一个目标点的坐标
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
%% 差异化覆盖的不同接收功率要求
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

%% 障碍物初始化
data.obstacle_x=[ %墙的位置 x  y1 y2 z1 z2类型 
    %三楼
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
    %四楼
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
%% 画图
% data.obstacle_size=[{'-sk'},{'-sy'},{'-sr'}];
% data.jingdu=0.1; %画图精度
% scatter3(data.node_position_x,data.node_position_y,data.node_position_z,'.');


%% 算法参数
option.peiod=10  ;  %Maximum Iterations 最大迭代次数
option.genMax=option.peiod;
option.M=150;         %Population Size 种群规模
option.popSize=option.M;
option.show_t=1;    %是否显示迭代次数
option.show_pc=1;   %是否显示迭代收敛图
option.creat_x=@creat_x_1;  
option.aimFcn=@aimFcn_1;
option.by=@bianyi_1;
option.c1 = 1;      %学习因子c1
option.c2 = 1;      %学习因子c2
% option.w=0.8;                  % PSO搜索策略的惯性权重
option.wmax=0.99;% PSO搜索策略的惯性权重w最大值
option.wmin=0.1; % PSO搜索策略的惯性权重w最小值

%% 初始种群生成 
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
[fit_record1(i,:),best_result1]=PSO-Levy-DFOA(option,data);   %迭代寻优算法
toc


