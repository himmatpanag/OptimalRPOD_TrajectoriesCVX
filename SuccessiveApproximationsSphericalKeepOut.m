function [eta, x, u] = SuccessiveApproximationsSphericalKeepOut()
% Trying to replicate result in Ortolano paper
% This function implements a spherical keep out zone for proximity
% operations near a non rotating spacecraft using successive approximations
% to push violating points outside of the keep out zone. Each problem
% solved is convex, however it is ?unclear (to me for now)? whether this method 
% converges to the optimal solution of the original problem. 

%% Setup problem: 
SOCP_Params = GetSOCP_Inputs();
SOCP_Params.numSteps = 41;
rng(SOCP_Params.randomSeed);
% Initial and final states of chaser
SOCP_Params.rInit = [0;0;0.1]; % x is r-bar, y is v-bar, z is cross track
SOCP_Params.vInit = randn(3,1).*1e-5; %std dev = 10cm/s

SOCP_Params.rFinal = [0.1;0;0]; % x is r-bar, y is v-bar, z is cross track
SOCP_Params.vFinal = [0;0;0];
SOCP_Params.simTimeHours = 0.5; 

time = linspace(0,SOCP_Params.simTimeHours,SOCP_Params.numSteps).*60;
sphereRadius = 0.08;
numSuccessiveApproximations = 6;
currentIterationNum = 0; 

hFigWaypoints = figure('Name','Waypoint Following'); 
ax1 = subplot(2,2,2); 
ax2 = subplot(2,2,3:4); grid on; hold on; 
ax3 = subplot(2,2,1); 

plot(time([1,SOCP_Params.numSteps]),[sphereRadius,sphereRadius].*1e3,'r--','DisplayName','KeepOutZone')
xlabel('Time (min)'), ylabel('Distance from centre m');
legend('show','location','northwest');

%% Step 1: Solve unconstrained problem
[eta, x, u] = OptimalApproachTrajCW(SOCP_Params);

AddCW_TrajToPlot(x,ax1,'UnconstrainedTrajectory');
s = PlotSphereConstraint(ax1,sphereRadius); set(s,'FaceColor',[0.1185 0.6541 0.8834]);
ShowViolatingPoints(ax1,x,sphereRadius);
axes(ax2); plot(time,vecnorm(x(1:3,:)).*1e3,'DisplayName',['itNum: ',num2str(currentIterationNum)]);
AddControlHistoryToPlot(ax3, eta, SOCP_Params);

%% Step 2: Find violating points and add constraint which pushes point slightly 
% further out of the sphere. The question here is in which direction do you
% push the trajectory? Pushing it to the closest point near the ellipse may
% not be the lowest cost. Is there some convex constraint you can add that 
% pushes it in the direction of lowest cost? 
% Can you reformulate the problem in spherical coordinates to 
% eliminate the nonconvexity? (NO)

err = 1; 
tolerance = 1e-6; % ~1cm

while currentIterationNum < numSuccessiveApproximations && err > tolerance
    currentIterationNum = currentIterationNum+1; 
    
    xPrev = x; 
    if currentIterationNum == 1
        previouslyViolatingPoints = [];
        constraintNumToPointIdx = [];
    else 
        previouslyViolatingPoints = violatingPointIdxs;
        constraintNumToPointIdx = zeros(1,numel(SOCP_Params.planarConstraint));
        for jj = 1:numel(SOCP_Params.planarConstraint)
            constraintNumToPointIdx(jj) = SOCP_Params.planarConstraint(jj).elementNum;
        end 
    end 
    violatingPointIdxs = find(vecnorm(x(1:3,:)) < sphereRadius);
    if ~isempty(violatingPointIdxs), SOCP_Params.usePlanarConstraints = true; end
    for ii = 1:numel(violatingPointIdxs)
        pointIdx = violatingPointIdxs(ii);        
        currentDistFromCentre = vecnorm(x(1:3,pointIdx));
        pushDistance = (sphereRadius-currentDistFromCentre)/...
            (numSuccessiveApproximations-currentIterationNum-2);
        % Formulate Constraints
        if currentIterationNum == 0
            constraintIdx = ii;
        elseif ~isempty(find(previouslyViolatingPoints==pointIdx,1))
            % Update the constraint rather than adding another constraint
            constraintIdx = find(constraintNumToPointIdx==pointIdx);
        else % A new point is now violating the spherical constraint
            constraintIdx = numel(constraintNumToPointIdx)+1; 
        end 
        SOCP_Params.planarConstraint(constraintIdx).elementNum = pointIdx;
        SOCP_Params.planarConstraint(constraintIdx).dirVector = x(1:3,pointIdx)./currentDistFromCentre;
        SOCP_Params.planarConstraint(constraintIdx).minRadius = currentDistFromCentre+pushDistance;

    end 

    %% Step 3: Solve constrained problem
    [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);    
    
    AddCW_TrajToPlot(x,ax1,['itNum: ',num2str(currentIterationNum)]);
    axes(ax2); plot(time,vecnorm(x(1:3,:)).*1e3,'DisplayName',['itNum: ',num2str(currentIterationNum)]);
    AddControlHistoryToPlot(ax3, eta, SOCP_Params);

    %% Step 4: Calculate error
    err = min(min(x-xPrev));
end 

end 

function ShowViolatingPoints(ax,x,rad)
    axes(ax); 
    x2 = x(1:3,:);
    violatingPoints = vecnorm(x2) < rad;
%     violatingPoints = x2(:,vecnorm(x2) < rad);
    plot3(x(3,violatingPoints),x(2,violatingPoints),x(1,violatingPoints),...
        'rx','DisplayName','ViolatingPoints')

end 

function s = EmptyPlanarConstraint()
    s.elementNum = [];
    s.dirVector = [];
    s.minRadius = [];
end 

function SOCP_Params = GetSOCP_Inputs()

    SOCP_Params.numSteps = 101; % Number of discretization steps in control input 
    SOCP_Params.simTimeHours = 3;    
    SOCP_Params.coneConstraintActive = false;  
    SOCP_Params.usePlanarConstraints = false; 
    SOCP_Params.planarConstraints = EmptyPlanarConstraint;

    SOCP_Params.Omega = 0.001; 
    SOCP_Params.aMax = 0.02*1e-3; % Max acceleration of thrusters km/s/s;        
 
    SOCP_Params.randomSeed = 1;    
end 