function [funcHandle, tData, dataUpsampled]  = ConstructPiecewiseConstantFunction(time,data)

% Assume that times contains 1 data entry more than data, i.e. times gives
% intervals on which data is defined

if iscolumn(time)
    time = time';
end
if iscolumn(data)
    data = data';
end 

h = min(diff(time));
tData = sort([time,time(1:end-1)+h/10,time(2:end)-h/10]);

accelTimes = time(1:end-1) + diff(time)./2;

allTimes = [time(1),accelTimes,time(end)];
allData = [data(1),data,data(end)];

funcHandle = @(t) interpn(allTimes,allData,t,'nearest');
dataUpsampled = funcHandle(tData);

end 