function AddControlHistoryVsDistanceToPlot(x, eta, params,hFig)

set(0,'CurrentFigure',hFig);
grid on, hold on, title('Acceleration')
time = linspace(0,params.simTimeHours,params.numSteps); % ylim([0,params.aMax*1e3])
[~, timeToPlot, etaPlot]  = ConstructPiecewiseConstantFunction(time,eta);
xToPlot = interpn(time,x',timeToPlot);
r = -vecnorm(xToPlot(1:3,:));
deltaV = calcDeltaV(eta, params);

plot(r*1e3,etaPlot*1000,'DisplayName',['\DeltaV=',num2str(deltaV),'m/s']), 

xlabel('Distance to Target (m)'), 
ylabel('Acceleration m/s^2')
legend('show','location','northeast');
end

function deltaV = calcDeltaV(eta, params)
    dt = params.simTimeHours*60*60/numel(eta);
    deltaV = sum(eta.*dt)*1e3; 
end 