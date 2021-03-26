function x=test_x(x,~,data)
%¾ÀÕıÔ½½çAP
    position= x<=0;
    x(position)=0.1*rand();
    
    position= x(:,1)>=data.mapx;
    x(position,1)=data.mapx-0.1*rand();
    
    position= x(:,2)>=data.mapy;
    x(position,2)=data.mapy-0.1*rand();
    
    position= x(:,3)>=data.mapz;
    x(position,3)=data.mapz-0.1*rand();
    
    position= x(:,4)>=data.a_max;
    x(position,4)=data.a_max-0.1*rand();
end