function [times, contamRate] = PlotContaminationRate(contamination,strName,hFig)

if nargin < 3
    hFig = figure; 
end  

if nargin < 2
    strName = '';
end 

set(0,'CurrentFigure',hFig);

grid on, hold on
times = [];% zeros(numel(contamination.contaminationPerStep),1);
contamRate = [];
h = min(abs(diff(contamination.stepTimes(:))));

for ii = 1:size(contamination.contaminationPerStep,1)
    T = contamination.stepTimes(ii,:);
    if mean(contamination.contaminationPerStep(ii,:)) > 0
        times = [times, T];
        rate = diff(contamination.contaminationPerStep(ii,:))./diff(T);
        contamRate = [contamRate, rate, rate(end)];
    else 
        times = [times, T(1)+h/10,T(end)-h/10];
        contamRate = [contamRate, 0, 0];
    end
end 

% idxsToPlot = find(contamination.contaminationPerStep >= 0);
% timesToPlot = contamination.stepTimes(idxsToPlot);
% dataToPlot = contamination.contaminationPerStep(idxsToPlot);

% [tSorted, I] = sort(timesToPlot);
% dataSorted = dataToPlot(I);

% contamRate = diff(dataSorted);%./diff(tSorted);
% tSorted(end) = [];

plot(times./3600,contamRate,'DisplayName',strName);
legend('show','Location','best');
xlabel('Time (sec)'), ylabel('ContamRate (Particles/Sec)')

end 