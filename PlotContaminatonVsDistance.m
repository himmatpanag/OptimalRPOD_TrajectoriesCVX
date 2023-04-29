function [tSorted,dataSorted] = PlotContaminatonVsDistance(t,x,contamination,strName, hFig,maxContam, ax)

if nargin < 3
    hFig = figure; 
end  

if nargin < 2
    strName = '';
end 

if nargin < 4 
    maxContam = 1;
    yStr = 'Contamination (#Particles)';
else 
    yStr = 'Contamination Normalized';
end

set(0,'CurrentFigure',hFig);

if nargin < 5
    grid on, hold on;
else 
    hFig.CurrentAxes = ax;
    grid on; hold on;
end 

idxsToPlot = find(contamination.contaminationPerStep > 0);
timesToPlot = contamination.stepTimes(idxsToPlot);
dataToPlot = contamination.contaminationPerStep(idxsToPlot);

[tSorted, I] = sort(timesToPlot);

xToPlot = interpn(t,x',tSorted);
r = vecnorm(xToPlot(:,1:3)');

dataSorted = dataToPlot(I);
plot(-r*1e3,dataSorted./maxContam,'DisplayName',strName);
legend('show','Location','best');
xlabel('Dist from Target (m)')
ylabel(yStr)
title('Contamination Over Trajectory')

end