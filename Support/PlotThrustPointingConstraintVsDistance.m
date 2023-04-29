function PlotThrustPointingConstraintVsDistance(time, x,u,angle,aMax, str,hFig)

set(0,'CurrentFigure',hFig);
throttleTol = 1e-3; % 1% throttle 

accelTimes = time(1:end-1) + diff(time)./2;

allTimes = [time(1),accelTimes,time(end)];
allData = [u(:,1),u,u(:,end)];

uFunc = @(t) interpn(allTimes,allData',t,'nearest');

N = size(x,2);
timesToPlot = linspace(time(1),time(end),3*N);
constraint = zeros(size(timesToPlot));
c = cos(angle);

thrustDirAngle = zeros(size(timesToPlot));
for ii = 1:numel(timesToPlot) 
    tt = timesToPlot(ii);
    xtt = interpn(time,x',tt);
    utt = uFunc(tt);
    etatt = vecnorm(utt);
    uttUnit = utt./etatt;
    rtt = vecnorm(xtt(1:3));
    constraint(ii) = xtt(1:3)'*utt - rtt * etatt * c;
    if etatt > throttleTol * aMax % this is in km/s
        thrustDirAngle(ii) = acos(dot(uttUnit',xtt(1:3)')./rtt)*180/pi;
    end
end

xToPlot = interpn(time,x',timesToPlot);
r = -vecnorm(xToPlot(1:3,:));

subplot 211, grid on; hold on; 
title('Contamination Constraint Function')
plot(r*1e3,constraint,'DisplayName', str);
xlabel('Distance from Target (m)')

subplot 212, grid on, hold on 
title('Plume Angle From Target')
plot(r*1e3,thrustDirAngle,'DisplayName', str); xlabel('Distance from Target (m)')
ylabel('Angle (deg)')
legend('show','Location','best')

end 