function hFig = PlotTrajectorySummaryThree(eta,x,u,params,contamProfile,maxContam,str,hFig)
    
    if nargin < 5
        hFig = figure('Name',params.caseDesc);
    end 
    set(0,'CurrentFigure',hFig);
    ax = subplot(2,2,1); grid on, hold on
    plot(x(1,:),x(2,:)), title('Approach trajectory in LVLH coordinates')
    xlabel('R-bar (km)'), ylabel('V-bar (km)'), axis equal
    
    set(hFig,'CurrentAxes',ax);
    PlotThrustDirection(x',u',eta/params.aMax,true)

    subplot(2,2,3);
    AddControlHistoryVsDistanceToPlot(x, eta, params,hFig)

    time = linspace(0,params.simTimeHours.*3600,params.numSteps); % ylim([0,params.aMax*1e3])
    PlotContaminatonVsDistance(time,x,contamProfile,str,hFig,maxContam,subplot(2,2,2));
 
    PlotThrustAngleVsDist(time, x,u,params.aMax, str,hFig);
end 

function PlotThrustAngleVsDist(time, x,u,aMax, str,hFig)

set(0,'CurrentFigure',hFig);
throttleTol = 1e-3; % 1% throttle 

accelTimes = time(1:end-1) + diff(time)./2;

allTimes = [time(1),accelTimes,time(end)];
allData = [u(:,1),u,u(:,end)];

uFunc = @(t) interpn(allTimes,allData',t,'nearest');

N = size(x,2);
timesToPlot = linspace(time(1),time(end),3*N);
xToPlot = interpn(time,x',timesToPlot);
r = -vecnorm(xToPlot(1:3,:));

thrustDirAngle = zeros(size(timesToPlot));
for ii = 1:numel(timesToPlot) 
    tt = timesToPlot(ii);
    xtt = interpn(time,x',tt);
    utt = uFunc(tt);
    etatt = vecnorm(utt);
    uttUnit = utt./etatt;
    rtt = vecnorm(xtt(1:3));
    if etatt > throttleTol * aMax % this is in km/s
        thrustDirAngle(ii) = acos(dot(uttUnit',xtt(1:3)')./rtt)*180/pi;
    end
end

subplot 224, grid on, hold on 
title('Plume Angle From Target')
plot(r*1e3,thrustDirAngle,'DisplayName', str); xlabel('Distance from Target (m)')
ylabel('Angle (deg)')
legend('show','Location','best')

end 
