 function x=LevyFight(rows,columns)
% % % % Mantegna方法模拟Levy飞行
    x =zeros(rows,columns);
    beta = 1.5;
    sigma_u = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);

    sigma_v = 1;
    for i=1:rows
        for j=1:columns
            u = normrnd(0,sigma_u);
            v = normrnd(0,sigma_v);
            s = u/(abs(v))^(1/beta);
            x(i,j) = x(i,j)+1*s;
        end
    end
end
