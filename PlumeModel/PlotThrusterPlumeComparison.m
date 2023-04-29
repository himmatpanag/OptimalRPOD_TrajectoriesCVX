function PlotThrusterPlumeComparison()

ambientParticleDensity = 1e-8; 

hFigPolar = figure;
for plumeIdx = 1:3

    params = ThrusterPlumeParameters(plumeIdx); 
    params.thrust = 1; 
    
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
                thetaGridPoints(jj),params);
        end
    end 
        
%     figure(hFig1);
%     subplot(2,2,plumeIdx);
%     title(string(params.thrusterType))
%     ax = gca; ax.ZScale = 'log'; 
%     contourLevels = 10.^(18:25);
%     contourf(r,th,plumeDensity,'LevelListMode','manual','LevelList',contourLevels,...
%         'LevelStepMode','manual')
%     ax.CLim = [1e18,1e24];
%     set(gca,'ColorScale','log')
%     c = colorbar;

    figure(hFigPolar);
    subplot(2,2,plumeIdx);
    ax = gca; ax.ZScale = 'log'; 
    contourLevels = 10.^(18:25);
    contourf(x,y,plumeDensity,'LevelListMode','manual','LevelList',contourLevels,...
        'LevelStepMode','manual')
    ax.CLim = [1e18,1e24];
    set(gca,'ColorScale','log')
    xlim([-1,Rmax])
    colorbar;
    title(string(params.thrusterType))

end 

% figure; 
% polarcont(radiusGridPoints,thetaGridPoints,plumeDensity,5);


% % DEBUG
% r = 1; 
% for jj = 1:N
%     plumeVsTheta(jj) = CalculatePlumeDensity(r,thetaGridPoints(jj),params);
% end
%     
% figure, semilogy(thetaGridPoints, plumeVsTheta)
% th = 0; 
% for jj = 1:N
%     plumeVsRad(jj) = CalculatePlumeDensity(radiusGridPoints(jj),th,params);
% end
% figure, semilogy(radiusGridPoints, plumeVsRad)



end 