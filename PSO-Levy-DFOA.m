function [fit_record,result]=PSO-Levy-DFOA(option,data)
creat_x=option.creat_x;
fitness=option.aimFcn;
x=data.x;
fit=data.fit;
v=data.v;
isFight=zeros(option.M,1);
%% ��ʼ����
[fit,position]=sort(fit);
% ��¼��Ⱥλ��
x=x(position);
fit_p=fit;
x_p=x;
%��ʼ�����Ÿ�����������
best_fit_p=fit_p(1);
worst_fit_p=fit_p(1);
best_x_p=x_p(1);
%����Ѱ�ſ�ʼ
for t=1:option.peiod
    w = option.wmax-(t-1)*(option.wmax-option.wmin)/(option.peiod-1); %Ȩ�����Եݼ�
    temp_v=v;
    temp_x=x;
    temp_fit=fit;
    %��ӬȺ��λ�ø���
    parfor i=1:option.M
        %�ϲ���Ⱥ����PSO�������Խ���λ�ø���
        if isFight(i)==0
            temp_v{i}= w*v{i} + rand(size(x{i})).*option.c1.*(x_p{i}-x{i}) + rand(size(x{i})).*option.c2.*(best_x_p{1} - x{i});%PSO����
            temp_x{i}=x{i} + temp_v{i};%λ�ø���
            temp_x{i}=test_x(temp_x{i},option,data);
            temp_fit(i)=fitness(temp_x{i},option,data);
        else %������Ⱥ����Levy Fight����λ�ø���
            temp_x{i}=x{i} - LevyFight(data.num_AP,4);%Levy����
            temp_x{i}=test_x(temp_x{i},option,data);
            temp_fit(i)=fitness(temp_x{i},option,data);
        end
    end
    [best_fit_p_1,position]=min(temp_fit);
    % �������ŵĸ�����Ϣ
    if best_fit_p_1<best_fit_p
        best_fit_p=best_fit_p_1;
        best_x_p=temp_x(position);
    end
    %������������Ϣ
    if best_fit_p_1>worst_fit_p
        worst_fit_p=best_fit_p_1;
    end
    isFight=zeros(option.M,1);
    %������Ⱥ������������Ϣ
    for i=1:option.M
        if fit_p(i)>temp_fit(i)
            fit_p(i)=temp_fit(i);
            x_p(i)=temp_x(i);
            v{i}=temp_v{i};
        end
        %���ݵ�ǰ���������Ÿ����������֮��Ĳ�ֵ���ж��´η��в���PSO�������Ի���Levy Fight
        if worst_fit_p-fit_p(i)>fit_p(i)-best_fit_p
            isFight(i)=1;
        end
    end
    x=temp_x;
    v=temp_v;
    fit=temp_fit;
    fit_record(t)=best_fit_p;
    fit_record_mean(t)=mean(fit_p);
end
%%
result.best_x=best_x_p{1};
[result.best_fit,result.detail]=fitness(result.best_x,option,data); 