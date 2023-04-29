function [PlateGridPoints, densityAtGridPoints] = PlumePlateIntersection(thrusterParams,plumeOrigin,plumeDir,plateCentre,plateVec1,plateVec2)
% Calculates density of plume on the surface of a plate
% plateVec1 = vector pointing in direction of one side of plate, with magnitude = half dimension1 of plate
% plateVec2 = vector pointing in direction of other side of plate, with magnitude = half dimension2 of plate

N = 100; 
% Break up plate into ~N squares = (N+1)^2 grid points

plateDims = 2*[norm(plateVec1),norm(plateVec2)]; % Multiply by 2 because each plateDir = 1/2 dimension of plate
a = sqrt(prod(plateDims)/N); 
gridSize = round(plateDims./a); % Number of grid points in each direction
plateCornerA = plateCentre - plateVec1 - plateVec2; % bottom left corner of plate
PlateGridPoints = zeros(3,gridSize(1)+1,gridSize(2)+1);
densityAtGridPoints = zeros(gridSize(1)+1,gridSize(2)+1);

%figure; grid on; hold on; 
for ii = 1:gridSize(1)+1
    for jj = 1:gridSize(2)+1
        PlateGridPoints(:,ii,jj) = plateCornerA + 2*plateVec1/gridSize(1)*(ii-1)+...
            2*plateVec2/gridSize(2)*(jj-1);
        
        plateGridPointRelThrusterOrigin = PlateGridPoints(:,ii,jj)-plumeOrigin;
        distThrusterOriginGridPoint = vecnorm(plateGridPointRelThrusterOrigin);
        angle = acos(plumeDir,plateGridPointRelThrusterOrigin./distThrusterOriginGridPoint);
        densityAtGridPoints(ii,jj) = CalculatePlumeDensity(distThrusterOriginGridPoint, angle, thrusterParams);

        % Plots
        % plot3(PlateGridPoints(1,ii,jj),PlateGridPoints(2,ii,jj),PlateGridPoints(3,ii,jj),'rx');
    end
end 

end 