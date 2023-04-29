function x = ZeroOrderHold(tGrid,xGrid,t)
% Variable time step Zero Order Hold of a signal

if t <= tGrid(1)
    x = xGrid(:,1);
elseif t >= tGrid(end)
    x = xGrid(:,end);
else 
    Idx = find(t<tGrid,1); 
    x = xGrid(:,Idx-1);
end 

end 

function fooTest
    
    t = 0:5;
    x = rand(1,5);
    figure, grid on, hold on, 
    plot(t(2:end),x,'rx');
    tF = linspace(0,5);
    for ii = 1:100
        xF(ii) = ZeroOrderHold(t,x,tF(ii));
    end
    plot(tF,xF)

end 