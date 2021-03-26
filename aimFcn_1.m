function [fit,result]=aimFcn_1(x,~,data)

    temp=x;
    %z��߶ȿ��ƣ�����AP�߶�λ��¥�㶥��
    temp(temp(:,3)>5,3)=10;
    temp(temp(:,3)<5,3)=5;
    %����dBmת��ΪmW
    A0=power(10,x(:,4)*0.1);
    %��ʼ��ÿ��Ŀ��ڵ���չ���С����ͽ��չ��ʣ���Ч���ǽڵ��Ϊ0
    node_num=0;
    RecPower=-94+zeros(data.node_size(1)*data.node_size(2)*data.node_size(3),1);
    %% ����ÿһ��Ŀ��ڵ��Ƿ�AP����
    for i=1:data.node_size(1)*data.node_size(2)*data.node_size(3)
        for j=1:data.num_AP
            %�������ϰ��������˥�� 
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
            %����Ŀ��ڵ���AP֮��ľ���
            temp_d=sqrt((temp(j,1)-data.node_position_x(i))^2+(temp(j,2)-data.node_position_y(i))^2+(temp(j,3)-data.node_position_z(i))^2);
            temp_beta=data.alpha+10*data.gama*log10(temp_d/data.d0)+beta;%�ź�˥����ʽ
            if temp(j,4)-temp_beta> RecPower(i)%���Ŀ��ڵ���յ���ǰAP�Ĺ��ʴ���ǰһAP�Ĺ��ʣ������Ŀ��ڵ�Ľ��չ���
                RecPower(i)=temp(j,4)-temp_beta;
            end
            if temp(j,4)-temp_beta>data.a0(i)%���Ŀ��ڵ���յ���ǰAP�Ĺ���������͹��ʸ���Ҫ�����Ϊ��ǰ�ڵ��Ѿ������ǣ�����ѭ��
                node_num=node_num+1;
                break;
            end
        end
    end
    %����Ŀ�����򸲸���
    C1=node_num/(data.node_size(1)*data.node_size(2)*data.node_size(3));
    %���������ɳڷ�������ͷ�ϵ����������Ϊ100%���Ӳ��Լ������ת��Ϊ��Լ��
    punishiment=0;
    if C1<1
        punishiment=punishiment+(1-C1)^2*100000000;
    end

    %% ����Ŀ�꺯��
    fit=sum(A0)+punishiment;
    if nargout>1
        result.punishiment=punishiment; %�ͷ���
        result.total_A=sum(A0); %�ܹ���
        result.C1=C1; %������
        result.AP_position=x;    %��APʵ��λ��
        result.AP_position2=temp;%��AP����λ��
        result.RecPower=RecPower;
    end
end