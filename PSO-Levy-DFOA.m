function [fit_record,result]=PSO-Levy-DFOA(option,data)
creat_x=option.creat_x;
fitness=option.aimFcn;
x=data.x;
fit=data.fit;
v=data.v;
isFight=zeros(option.M,1);
%% 开始计算
[fit,position]=sort(fit);
% 记录种群位置
x=x(position);
fit_p=fit;
x_p=x;
%初始化最优个体与最差个体
best_fit_p=fit_p(1);
worst_fit_p=fit_p(1);
best_x_p=x_p(1);
%迭代寻优开始
for t=1:option.peiod
    w = option.wmax-(t-1)*(option.wmax-option.wmin)/(option.peiod-1); %权重线性递减
    temp_v=v;
    temp_x=x;
    temp_fit=fit;
    %果蝇群体位置更新
    parfor i=1:option.M
        %较差子群采用PSO搜索策略进行位置更新
        if isFight(i)==0
            temp_v{i}= w*v{i} + rand(size(x{i})).*option.c1.*(x_p{i}-x{i}) + rand(size(x{i})).*option.c2.*(best_x_p{1} - x{i});%PSO搜索
            temp_x{i}=x{i} + temp_v{i};%位置更新
            temp_x{i}=test_x(temp_x{i},option,data);
            temp_fit(i)=fitness(temp_x{i},option,data);
        else %较优子群采用Levy Fight进行位置更新
            temp_x{i}=x{i} - LevyFight(data.num_AP,4);%Levy飞行
            temp_x{i}=test_x(temp_x{i},option,data);
            temp_fit(i)=fitness(temp_x{i},option,data);
        end
    end
    [best_fit_p_1,position]=min(temp_fit);
    % 更新最优的个体信息
    if best_fit_p_1<best_fit_p
        best_fit_p=best_fit_p_1;
        best_x_p=temp_x(position);
    end
    %更新最差个体信息
    if best_fit_p_1>worst_fit_p
        worst_fit_p=best_fit_p_1;
    end
    isFight=zeros(option.M,1);
    %更新种群内其他个体信息
    for i=1:option.M
        if fit_p(i)>temp_fit(i)
            fit_p(i)=temp_fit(i);
            x_p(i)=temp_x(i);
            v{i}=temp_v{i};
        end
        %根据当前个体与最优个体和最差个体之间的差值来判定下次飞行采用PSO搜索策略还是Levy Fight
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