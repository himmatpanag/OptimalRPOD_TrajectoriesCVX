function AddControlHistoryToPlot(ax, eta, params)
    axes(ax);
    grid on, hold on, title('Acceleration magnitude')
    time = linspace(0,params.simTimeHours,params.numSteps); ylim([0,params.aMax*1e3])
    deltaV = calcDeltaV(eta, params);
    plot(time,eta*1000,'DisplayName',['\DeltaV=',num2str(deltaV),'m/s']), 
    xlabel('Time (hrs)'), ylabel('Acceleration m/s^2')
    legend('show','location','northeast');
end

function deltaV = calcDeltaV(eta, params)
    dt = params.simTimeHours*60*60/numel(eta);
    deltaV = sum(eta.*dt)*1e3; 
end 