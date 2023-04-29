function PlotsForConferencePaper
saveDir = pwd;
currentDir = pwd; 
hFigPolar = PlumePlot;

allFigs = {hFigPolar};
SuccessiveContaminationReductionUsingCVX(113171900);
cd(saveDir);

for ii = 1:numel(allFigs)
    hf = allFigs{ii};
    saveas(hf,hf.Name,'png');
end

cd(currentDir);
    

end 

function hFigPolar = PlumePlot
hFigPolar = figure('Name','PlumeDensity','Units','normalized','Position',[.2 .2 .7 .7]);
plumeIdx = 1;

thrusterParams = ThrusterPlumeParameters(plumeIdx); 

N = 100; 
Rmax = 20;
radiusGridPoints = linspace(0.1,Rmax,N);
thetaGridPoints = linspace(0,pi,N);
[r,th] = meshgrid(radiusGridPoints,thetaGridPoints);

plumeDensity = zeros(N,N);
x = zeros(N,N); 
y = zeros(N,N);
for ii = 1:numel(radiusGridPoints)
    for jj = 1:numel(thetaGridPoints)
        x(ii,jj) = radiusGridPoints(ii)*cos(thetaGridPoints(jj));
        y(ii,jj) = radiusGridPoints(ii)*sin(thetaGridPoints(jj));
        plumeDensity(ii,jj) = CalculatePlumeDensity(radiusGridPoints(ii),...
            thetaGridPoints(jj),thrusterParams);
    end
end 

figure(hFigPolar);
ax = gca; ax.ZScale = 'log'; 
contourLevels = 10.^(18:25);
contourf(x,y,plumeDensity,'LevelListMode','manual','LevelList',contourLevels,...
    'LevelStepMode','manual')
ax.CLim = [1e18,1e24];
set(gca,'ColorScale','log')
xlim([-1,Rmax])
colorbar;
title('Bipropellant Plume Density Profile (num particles/m^3)')
xlabel('x Distance from Thruster Exit Plane (m)')
ylabel('z Distance from Thruster Exit Plane (m)')

end