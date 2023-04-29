function hFig = PlotTrajectorySummary(eta,x,u,params,hFig)
    
    if nargin < 5
        hFig = figure('Name',params.caseDesc);
    end 
    set(0,'CurrentFigure',hFig);
    ax = subplot(2,2,1:2); grid on, hold on
    plot(x(1,:),x(2,:)), title('Approach trajectory in LVLH coordinates')
    xlabel('R-bar (km)'), ylabel('V-bar (km)'), axis equal
    
    set(hFig,'CurrentAxes',ax);
    PlotThrustDirection(x',u',eta/params.aMax,true)
    
    AddControlHistoryToPlot(subplot(2,2,3), eta, params)    
%     plot(time,u(1,:)*1000,'DisplayName','a R-bar')
%     plot(time,u(2,:)*1000,'DisplayName','a V-bar')

    time = linspace(0,params.simTimeHours,params.numSteps); 
    subplot(2,2,4), grid on, hold on, title('Velocity components')
    yyaxis left, ylabel('Velocity m/s')
    plot(time,x(4,:)*1e3,'DisplayName','V Bar velocity')
    yyaxis right, ylabel('Velocty m/s')
    plot(time,x(5,:)*1e3,'DisplayName','R Bar velocity')
    legend('show','Location','best')
end 