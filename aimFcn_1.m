function [fit,result]=aimFcn_1(x,~,data)

    temp=x;
    %z轴高度控制，控制AP高度位于楼层顶部
    temp(temp(:,3)>5,3)=10;
    temp(temp(:,3)<5,3)=5;
    %功率dBm转换为mW
    A0=power(10,x(:,4)*0.1);
    %初始化每个目标节点接收功率小于最低接收功率，有效覆盖节点记为0
    node_num=0;
    RecPower=-94+zeros(data.node_size(1)*data.node_size(2)*data.node_size(3),1);
    %% 计算每一个目标节点是否被AP覆盖
    for i=1:data.node_size(1)*data.node_size(2)*data.node_size(3)
        for j=1:data.num_AP
            %计算由障碍物引起的衰减 
            beta=0;
            if(temp(j,2)-11.2)*(11.2-data.node_position_y(i))>0
                beta=beta+12;
            end
            if(temp(j,2)-8.2)*(8.2-data.node_position_y(i))>0
                beta=beta+12;
            end
            if(temp(j,3)-5.1)*(5.1-data.node_position_z(i))>0
                beta=beta+20;
            end
            for k=1:46
                if(temp(j,1)-data.obstacle_x(k,1))*(data.obstacle_x(k,1)-data.node_position_x(i))>0
                    x0=data.obstacle_x(k,1);
                    t1=temp(j,1)-x0;
                    t2=data.node_position_x(i)-x0;
                    y0=(temp(j,2)*t2 - data.node_position_y(i)*t1)/(t2-t1);
                    z0=(temp(j,3)*t2 - data.node_position_z(i)*t1)/(t2-t1);
                    if data.obstacle_x(k,2)<y0 && y0<data.obstacle_x(k,3) && data.obstacle_x(k,4)<z0 && z0<data.obstacle_x(k,5) 
                        beta=beta+12;
                    end
                end
            end
            %计算目标节点与AP之间的距离
            temp_d=sqrt((temp(j,1)-data.node_position_x(i))^2+(temp(j,2)-data.node_position_y(i))^2+(temp(j,3)-data.node_position_z(i))^2);
            temp_beta=data.alpha+10*data.gama*log10(temp_d/data.d0)+beta;%信号衰减公式
            if temp(j,4)-temp_beta> RecPower(i)%如果目标节点接收到当前AP的功率大于前一AP的功率，则更新目标节点的接收功率
                RecPower(i)=temp(j,4)-temp_beta;
            end
            if temp(j,4)-temp_beta>data.a0(i)%如果目标节点接收到当前AP的功率满足最低功率覆盖要求，则记为当前节点已经被覆盖，结束循环
                node_num=node_num+1;
                break;
            end
        end
    end
    %计算目标区域覆盖率
    C1=node_num/(data.node_size(1)*data.node_size(2)*data.node_size(3));
    %拉格朗日松弛法，引入惩罚系数将覆盖率为100%这个硬性约束条件转化为软约束
    punishiment=0;
    if C1<1
        punishiment=punishiment+(1-C1)^2*100000000;
    end

    %% 计算目标函数
    fit=sum(A0)+punishiment;
    if nargout>1
        result.punishiment=punishiment; %惩罚项
        result.total_A=sum(A0); %总功率
        result.C1=C1; %覆盖率
        result.AP_position=x;    %各AP实际位置
        result.AP_position2=temp;%各AP虚拟位置
        result.RecPower=RecPower;
    end
end