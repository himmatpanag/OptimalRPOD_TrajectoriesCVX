function SOCP_OrtolanoResults()
    % Trying to replicate the results in the Ortolano paper
    % Non rotating target. Optimal trajectories for the Hill-Clohessy-Wiltshire 
    % model in cartesian coordinates using SOCPs.     

    %% Initial approach from Ch13. 
    % Use the code below to generate rInit. Note vInit = 0 since the CW
    % frame is rotating.
    % a = 7.07499742e3; % Page 237 of Ortolano Phd Thesis. 
    % GetInitialPos(a,5), GetInitialPos(a,10)
    hFig = figure; 
    for caseNum = [113122900, 113132900, 114122900, 114132900]
        SOCP_Params = CW_RPO_TestCondition(caseNum);
        [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);
        PlotTrajectorySummary(eta,x,u,SOCP_Params,hFig);
    end 
    
    CheckCW_DynamicsSatisfied(SOCP_Params,x,u)
    
    % Plot trajectories using TwoBodyODE and see if they match x from the
    % CVX solver.
    
    %% Initial approaches from Ch7
    SOCP_Params = CW_RPO_TestCondition(125002990);    
    hFigAllCases = figure('Name','ApproachTraj5Cases'); 
    for caseNum = 1:5
        switch caseNum
            case 1    
                SOCP_Params.rInit = [0;10;0]; 
            case 2 
                SOCP_Params.rInit = [1;10;0];
            case 3 
                SOCP_Params.rInit = [-1;10;0];
            case 4 
                SOCP_Params.rInit = [0;10;1];
            case 5 
                SOCP_Params.rInit = [0;10;-1];
        end 
        SOCP_Params.caseDesc = ['Unconstrained Approach Case',num2str(caseNum)];

        [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);
        AddControlHistoryToPlot(gca, eta, SOCP_Params)
    end 
    
    %% Waypoint following Ch 7
    SOCP_Params.caseDesc = 'WaypointFollowingUnconstrained';
    SOCP_Params.vFinal = [0;0;0];
    SOCP_Params.simTimeHours = 0.5;
    hFigWaypoints = figure('Name','Waypoint Following'); ax = gca;
    waypoints = [0,200,0;0,0,100;100,0,0;0,-100,0;0,100,0].*1e-3;
    waypointLabels = 'ABCDE';
    for transferNum = 1:size(waypoints,1)-1
        SOCP_Params.vInit = randn(3,1).*1e-5;
        SOCP_Params.rInit = waypoints(transferNum,:)';% x is r-bar, y is v-bar, z is cross track        
        SOCP_Params.rFinal = waypoints(transferNum+1,:)';
        [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);
        AddCW_TrajToPlot(x,ax,['Transfer ',num2str(transferNum)]);        
    end
    for waypointNum = 1:size(waypoints,1)
        text(waypoints(waypointNum,3),waypoints(waypointNum,2),...
            waypoints(waypointNum,1), waypointLabels(waypointNum));
    end
    title('Waypoint following with 30 minute transfers');
         
    %% Final approach with cone constraint % Ch 7    
    initialPositions = [0,0.1,0;.01,.1,0;-.01,.1,0;0,.1,.01;0,.1,-.01]';        
    caseNums = 0:10:40;
    for caseNum = caseNums
        hFig = figure; 
        for ii = 1:size(initialPositions,2)            
            SOCP_Params = CW_RPO_TestCondition(caseNum);
            SOCP_Params.rInit = initialPositions(:,ii);
            SOCP_Params.vInit = randn(3,1).*1e-5;
            [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);

            AddCW_TrajToPlot(x,subplot(2,1,1),['Case ',num2str(ii)]);
            title('Final Approach with cone constraint')
            AddControlHistoryToPlot(subplot(2,1,2), eta, SOCP_Params); 
        end
        PlotConeConstraint(subplot(2,1,1),SOCP_Params.coneAngle,0.1,SOCP_Params.coneAxisVector);
    end 
    
    %% Final approach but results are from CH 13 of phd. 
    for caseNum = [11111100, 12111100, 11111101, 12111101]
        SOCP_Params = CW_RPO_TestCondition(caseNum);
        [eta, x, u] = OptimalApproachTrajCW(SOCP_Params);
        PlotTrajectorySummary(eta,x,u,SOCP_Params,hFig)
    end 
    
    %% TODO: 
    % Implement thrust vectoring constraint 
    % Implement rotating body optimization 

end

function CheckCW_DynamicsSatisfied(params,x,u)

    N = params.numSteps; % Number of discretization steps in control input 
    Omega = params.Omega; 
    OmegaSq = params.Omega^2;
    
    simTime = params.simTimeHours*60*60; 
    dt = simTime/N;
    A = zeros(6,6);
    A(4,1) = 3*OmegaSq; A(6,3) = -OmegaSq;
    A(4,5) = 2*Omega; A(5,4)=-2*Omega;
    A(1,4) = 1; A(2,5) = 1; A(3,6) = 1; 
    B = [zeros(3,3);eye(3)];
    
    Phi=expm(A*dt);
        
    A2 = [-A,B;zeros(3,9)];
    A2tExp = expm(A2*dt);
    IntegralExpAdtB = A2tExp(1:6,7:9);
    Bd = Phi*IntegralExpAdtB;
    
    for ii = 2:params.numSteps
        k(:, ii) = (-x(:,ii) + Phi*x(:,ii-1) + Bd*u(:,ii-1));    
    end 

    fprintf('Num violated constraints:  \t\t  %d\n',nnz(k~=0))
    fprintf('Num constraints where error > eps: \t  %d\n',nnz(k>eps))
    fprintf('Num constraints where error > 1e-8: \t  %d\n',nnz(k>1e-8))
    fprintf('Largest constraint error: %d \t %d\n',max(max(k)))

end 

function rInit = GetInitialPos(a,alongTrackDist)
    df = alongTrackDist/a;
    rInit = [a*(cos(df)-1),a*sin(df),0];
    
    % Data given in paper but not required. Page 237
    a = 7.07499742e3; e = 0; i = 0.488692190; Omega = 0.349065850; 
    omega = 1.46607657; % arg periapse
    nu = 0.78539816; % true anomaly   
end