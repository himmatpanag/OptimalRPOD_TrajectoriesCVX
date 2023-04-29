function AddControlHistoryToPlot(ax, eta, params,fSec)
if nargin < 4, fSec = false;end
axes(ax);
    grid on, hold on, title('Acceleration')
    time = linspace(0,params.simTimeHours,params.numSteps); % ylim([0,params.aMax*1e3])
    [~, timeToPlot, etaPlot]  = ConstructPiecewiseConstantFunction(time,eta);

    deltaV = calcDeltaV(eta, params);
    if fSec
        plot(timeToPlot.*3600,etaPlot*1000,'DisplayName',['\DeltaV=',num2str(deltaV),'m/s']), 
        xlabel('Time (sec)'), 
    else 
        plot(timeToPlot,etaPlot*1000,'DisplayName',['\DeltaV=',num2str(deltaV),'m/s']), 
        xlabel('Time (hrs)'), 
    end 
    ylabel('Acceleration m/s^2')
    legend('show','location','northeast');
end

function deltaV = calcDeltaV(eta, params)
    dt = params.simTimeHours*60*60/numel(eta);
    deltaV = sum(eta.*dt)*1e3; 
end 