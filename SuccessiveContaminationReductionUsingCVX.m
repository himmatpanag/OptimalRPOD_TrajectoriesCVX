function SuccessiveContaminationReductionUsingCVX(CW_RPO_caseNum)

%% Set up CW transfer
if nargin < 1, CW_RPO_caseNum = 113171900; end
if nargin < 3, fPlotContamSuccApp = false; end
% CW_RPO_caseNum = 113181900;
% CW_RPO_caseNum = 113171900;

SOCP_Params = CW_RPO_TestCondition(CW_RPO_caseNum);
SOCP_Params.solver = Solver.MOSEK;
SOCP_Params.limitThrust = true; 

%% Set up spaceraft parameters 
% Step 0 Define spacecraft mass ffs man!
SOCP_Params.numSteps = 100; 
SOCP_Params.simTimeHours = .1;

params = SpacecraftParameters;
Box = GetSpacecraftBoundingBox(1);
time = linspace(0,SOCP_Params.simTimeHours*3600,SOCP_Params.numSteps);
timeTarget = [0,time(end)];
posTarget = zeros(3,2);
orientationDCM_Target = repmat(eye(3),1,1,2);    
SOCP_Params.aMax = 1e-3 * params.thrusterParameters.thrust/params.massParameters.totalMass; % km/s

%% Step 1 Generate unconstrained trajectory that points thruster at target somewhere during flight
% Use traj from AE508?
[eta, x, u] = OptimalApproachTrajCW(SOCP_Params);
etaUnconst = eta; xUnconst = x; uUnconst = u;

if false 
    % Generate trajectory using primer vector theory 
    w = SOCP_Params.Omega; 
    a_max = SOCP_Params.aMax;
    x0 = [SOCP_Params.rInit; SOCP_Params.vInit];
    xf = [SOCP_Params.rFinal; SOCP_Params.vFinal];
    tf = SOCP_Params.simTimeHours*3600; 
    lam0_guess = [-0.078680309500705  -1.004392627562854  -0.000000000000000   0.000102632531872   0.000054579832117   0.000000000000000]';
    rho = 1e-8; 
    [t,x,thrustDir,delta,lam0] = ...
        LowThrust_CSI_CW_RPO(w,x0,xf,tf,a_max,lam0_guess,rho);
    
    hf = PlotAE508OptimizationResults(t,x,thrustDir,delta,CW_RPO_caseNum);
    PlotThrustAngles(t,x,delta,thrustDir,'Unconstrained',gcf);
end 

% Plot initial contamination and traj
% initialContaminationProfile = CalculateContaminationOverTrajectory('ZeroOrderHold',...
%     params.thrusterParameters,time,x(1:3,:)*1e3,u*1e3,params.massParameters.totalMass,...
%     timeTarget,posTarget,orientationDCM_Target,Box);

%% Step 2 Define cone angle alpha and pass previous trajectory into solver
SOCP_Params.thrustPointingConstraint.active = true; 
SOCP_Params.thrustPointingConstraint.useLinearApprox = true; 
SOCP_Params.thrustPointingConstraint.activeRadius = 0.1; % within 100m of target, constraints are active. 
SOCP_Params.thrustPointingConstraint.angle = 15*pi/180; 
SOCP_Params.thrustPointingConstraint.useSphericalTarget = false; 
SOCP_Params.thrustPointingConstraint.numConstraintsPerStep = 1;
SOCP_Params.thrustPointingConstraint.useLinearApprox = true;         
SOCP_Params.trustRegion.useTrustRegion = false;
SOCP_Params.trustRegion.pos = 1e-4; 
SOCP_Params.trustRegion.eta = 1e-1; % 1e-1; % 1e-1; 
SOCP_Params.trustRegion.u = 1e-1; 

%% Plots
figVis = 'off';
hFig01 = figure('Name','ContaminationVsAlpha','Visible',figVis);
hFig02 = figure('Name','ConstraintVsAlpha','Visible',figVis);
hFig03 = figure('Name','TrajSummaryVsAlpha','Visible',figVis);
hFig04 = figure('Name','EtaDistanceVsAlpha','Visible',figVis);
hFig05 = figure('Name','TrajSummary3VsAlpha','Visible',figVis);
hFig06 = figure('Name','ConstraintDistanceVsAlpha','Visible',figVis);

maxContam = 1.25112e22;
AddControlHistoryVsDistanceToPlot(x, eta, SOCP_Params,hFig04);
% PlotContaminaton(initialContaminationProfile,'Unconstrained',hFig01,maxContam);
% PlotTrajectorySummaryTwo(etaUnconst,xUnconst,uUnconst,SOCP_Params,initialContaminationProfile,maxContam,'Unconstrained',hFig03);
% PlotTrajectorySummaryThree(etaUnconst,xUnconst,uUnconst,SOCP_Params,initialContaminationProfile,maxContam,'Unconstrained',hFig05);
PlotThrustPointingConstraint(time, xUnconst,uUnconst,SOCP_Params.thrustPointingConstraint.angle, SOCP_Params.aMax, 'Unconstrained', hFig02);
PlotThrustPointingConstraintVsDistance(time, xUnconst,uUnconst,SOCP_Params.thrustPointingConstraint.angle, SOCP_Params.aMax, 'Unconstrained', hFig06);

hf = figure('Name','UnconstTraj'); grid on, hold on
plot(x(1,:),x(2,:)), title('Unconstrained approach trajectory in LVLH coordinates')
xlabel('R-bar (km)'), ylabel('V-bar (km)'), axis equal
PlotThrustDirection(x',u',eta/SOCP_Params.aMax,true)
legend('Trajectory','Control')
PlotPoint.Target([0;0;0])
plot(0,0.2,'ro','MarkerSize',10,'DisplayName','ChaserInitial')
plot(0,0.02,'bo','MarkerSize',10,'DisplayName','ChaserFinal')
hf.Units = 'normalized';
hf.Position = [0.1 0.1 .25 .3];
% saveas(hf,hf.Name,'png'); %saveas(hf,hf.Name,'fig');

%% Step 3 Solve new problem with pointing constraint, this is a linear constraint 
N = 10; iterTol = 1e-8; % .1cm
angles = 18:2:26;
for jj = 1:numel(angles)
    SOCP_Params.thrustPointingConstraint.angle = angles(jj)*pi/180; 
    alphaStr = ['alpha = ',num2str(angles(jj)),' deg'];

    [hFig1,hFig2,hFig3,hFig4,hFig5,hFig6] = CreateFigures(figVis,alphaStr);
    PlotTrajectorySummary(etaUnconst,xUnconst,uUnconst,SOCP_Params,hFig1);    
    PlotTrajXY(time,xUnconst,hFig4,'Unconstrained');    
    PlotThrustPointingConstraint(time, xUnconst,uUnconst,SOCP_Params.thrustPointingConstraint.angle, SOCP_Params.aMax, 'Unconstrained', hFig6);
    
    if fPlotContamSuccApp, PlotContaminaton(initialContaminationProfile,'Unconstrained',hFig2,maxContam);
        PlotContaminationRate(initialContaminationProfile,'Unconstrained',hFig3); end 

    for ii = 1:N
        fprintf('Starting iteration %d\n',ii)
        SOCP_Params.thrustPointingConstraint.prevTraj = x; 
        SOCP_Params.thrustPointingConstraint.prevControl = u;
        SOCP_Params.thrustPointingConstraint.prevEta = eta;
        [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);
        
        strName = ['Iter',num2str(ii)];
        PlotTrajectorySummary(eta,x,u,SOCP_Params,hFig1);
        PlotTrajXY(time,x,hFig4,strName);
        PlotThrustPointingConstraint(time, x,u,SOCP_Params.thrustPointingConstraint.angle,SOCP_Params.aMax, strName, hFig6);
        PlotChangeInSolution(time,x,eta, SOCP_Params,strName,hFig5);
    
        if fPlotContamSuccApp
            contaminationProfile = CalculateContaminationOverTrajectory('ZeroOrderHold',params.thrusterParameters,time,x(1:3,:)*1e3,u*1e3,params.massParameters.totalMass,timeTarget,posTarget,orientationDCM_Target,Box);
            PlotContaminaton(contaminationProfile,strName,hFig2,maxContam);
            PlotContaminationRate(contaminationProfile,strName,hFig3);
        end 

        if max(vecnorm(SOCP_Params.thrustPointingConstraint.prevTraj(1:3,:)-x(1:3,:))) < iterTol
            break            
        end 
    
    end
    fprintf('Converged after %d iterations\n',ii)
    
%     contaminationProfile = CalculateContaminationOverTrajectory('ZeroOrderHold',...
%     params.thrusterParameters,time,x(1:3,:)*1e3,u*1e3,params.massParameters.totalMass,...
%     timeTarget,posTarget,orientationDCM_Target,Box);
% 
%     PlotContaminaton(contaminationProfile,alphaStr,hFig01,maxContam);
%     PlotTrajectorySummaryTwo(eta,x,u,SOCP_Params,contaminationProfile,maxContam,alphaStr,hFig03);
%     PlotTrajectorySummaryThree(eta,x,u,SOCP_Params,contaminationProfile,maxContam,alphaStr,hFig05);
    PlotThrustPointingConstraint(time, x,u,SOCP_Params.thrustPointingConstraint.angle, SOCP_Params.aMax, alphaStr, hFig02);
    AddControlHistoryVsDistanceToPlot(x, eta, SOCP_Params,hFig04);
    PlotThrustPointingConstraintVsDistance(time, x,u,SOCP_Params.thrustPointingConstraint.angle, SOCP_Params.aMax, alphaStr, hFig06);

    figure(hFig1); ax = subplot(2,2,1:2); PlotCircleConstraint(ax,SOCP_Params.thrustPointingConstraint.activeRadius,[0;0]);
    allFigs = {hFig1;hFig2;hFig3;hFig4;hFig5;hFig6;hFig01;hFig02;hFig03;hFig04;hFig05;hFig06};
    for ii = 1:numel(allFigs)
        set(allFigs{ii}, 'Visible', 'on');
    end 
    if ~fPlotContamSuccApp, close(hFig2); close(hFig3); end
end 

PlotGradientHessianOfPointingConstraint(time,x,u,eta,SOCP_Params.thrustPointingConstraint.angle)
legend('show','Location','best')

%% Step 4 Increase angle and repeat or stop 

end

function [hFig1,hFig2,hFig3,hFig4,hFig5,hFig6] = CreateFigures(figVis,nameAppend)

hFig1 = figure('Name',['TrajSummary',nameAppend],'Visible',figVis); 
hFig2 = figure('Name',['Contamination',nameAppend],'Visible',figVis); grid on; hold on; 
hFig3 = figure('Name',['ContaminationRate',nameAppend],'Visible',figVis); grid on; hold on; 
hFig4 = figure('Name',['TrajPosComparison',nameAppend],'Visible',figVis); 
hFig5 = figure('Name',['TrajChange',nameAppend],'Visible',figVis); 
hFig6 = figure('Name',['Constraint',nameAppend],'Visible',figVis);

end 

function PlotTrajXY(t,x,hFig,strName)
set(0,'CurrentFigure',hFig);
subplot 211
grid on; hold on; 
plot(t,x(1,:),'DisplayName',strName);
legend('show','Location','best');

subplot 212
grid on; hold on; 
plot(t,x(2,:),'DisplayName',strName);
legend('show','Location','best');
end 

function allFigs = terationPlots(eta,x,u,SOCP_Params,fPlotContamination,plotInBackground)

if plotInBackground
    figVis = 'off';
else 
    figVis = 'on';
end 

if fPlotContamination
end 

end 