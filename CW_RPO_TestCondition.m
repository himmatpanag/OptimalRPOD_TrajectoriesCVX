function SOCP_Params = CW_RPO_TestCondition(caseNum)
    % This function generates the input conditions to solve the optimal control 
    % problem of Rendezvous and Proximity Operations using the (Cartesian) 
    % CW equations. The problem is formulated as a SOCP and inputs are fed 
    % to cvx, which then solves the SOCP using an interior point method. 
    
    % Each digit of caseNum varies some condition of the RPO problem
    % Useful to have all possible conditions defined in one file 
    
    if nargin < 1 
        caseNum = 00090;
    end 
        
    switch floor(rem(caseNum/1000,10)) % 4th digit
        case 0 % Final distance from target
            SOCP_Params.rFinal = [0;0.005;0];% [0;0.005;0]; % km 
        case 1 
            SOCP_Params.rFinal = [0;0.02;0]; % = 20m % V bar
        case 2
            SOCP_Params.rFinal = [0;0.2;0]; % = 200m % V bar
    end
    
    switch floor(rem(caseNum/10000,10)) % 5th digit
        case 0 % Multiple initial positions, as in Ch 7
            initialPositions = [0,0.1,0;.01,.1,0;-.01,.1,0;0,.1,.01;0,.1,-.01]';    
            SOCP_Params.rInit = initialPositions(:,1);
        case 1 % As in ch 13
            SOCP_Params.rInit = [0;0.1;0];
        case 2 
            SOCP_Params.rInit = [0.001766785022959;4.999999583796066;0];
        case 3 
            SOCP_Params.rInit = [0.007067139209739;9.999996670368782;0];
    end
        
    switch floor(rem(caseNum/1e5,10)) % 6th digit
        case 0
            SOCP_Params.Omega = 0.001; % From email?
        case 1 
            % From ch 13, pg238
            SOCP_Params.Omega = 0.001060922896439; 
            % radTarget = 7074.94742; parameters = GetDefaultParameters; sqrt(parameters.mu./radTarget^3);
    end
    
    switch floor(rem(caseNum/1e6,10)) % 7th digit, final time
        case 0
            SOCP_Params.simTimeHours = 10/60;
        case 1 % 30% of one revolution
            SOCP_Params.simTimeHours = 0.3 * 2*pi/SOCP_Params.Omega/60/60;
        case 2 
            SOCP_Params.simTimeHours = 0.6 * 2*pi/SOCP_Params.Omega/60/60;
        case 3
            SOCP_Params.simTimeHours = 1.2 * 2*pi/SOCP_Params.Omega/60/60;
        case 4
            SOCP_Params.simTimeHours = 2.4 * 2*pi/SOCP_Params.Omega/60/60;
        case 5
            SOCP_Params.simTimeHours = 3;
        case 6
            SOCP_Params.simTimeHours = 6;
    end
    
    switch floor(rem(caseNum/1e7,10)) % 8th digit, aMax
        case 0
            SOCP_Params.aMax = 0.0035*1e-3; % Max acceleration of thrusters km/s/s; 
        case 1 
            SOCP_Params.aMax = .5*1e-6; % 0.5mm/s/s
        case 2
            SOCP_Params.aMax = 5*1e-6; % 0.5mm/s/s
    end
    
    switch floor(rem(caseNum/1e8,10)) % 9th digit, numSteps
        case 0 
            SOCP_Params.numSteps = 60;
        case 1 
            SOCP_Params.numSteps = 108;
    end 
    
    switch floor(rem(caseNum,10)) % Approach direction
        case 0 % V Bar
            SOCP_Params.coneAxisVector = [0;1;0];            
        case 1 % R Bar
            SOCP_Params.coneAxisVector = [1;0;0];
            SOCP_Params.rFinal = SOCP_Params.rFinal([2,1,3]);
    end 
    
    switch floor(rem(caseNum/10,10))
        case 0
            SOCP_Params.vInit = [0;0;0];
        case 10
            SOCP_Params.vInit = [0.2;0;0]*1e-3;
        case 2
            SOCP_Params.vInit = [-0.2;0;0]*1e-3;
        case 3
            SOCP_Params.vInit = [0;0.2;0]*1e-3;
        case 4
            SOCP_Params.vInit = [0;-0.2;0]*1e-3;        
        case 9 
            SOCP_Params.randomSeed = 1;
            rng(SOCP_Params.randomSeed);
            SOCP_Params.vInit = randn(3,1).*1e-5; %std dev = 10cm/s            
    end     

    switch floor(rem(caseNum/100,10)) % 3rd digit
        case 0
            SOCP_Params.coneAngle = 10*pi/180;
            SOCP_Params.coneConstraintActive = true; 
        case 1 
            SOCP_Params.coneAngle = 20*pi/180;
            SOCP_Params.coneConstraintActive = true; 
        case 9 % No cone
            SOCP_Params.coneAngle = 0;
            SOCP_Params.coneConstraintActive = false; 
    end 

    SOCP_Params.caseDesc = 'Final Approach with Cone Constraint';        
    SOCP_Params.usePlanarConstraints = false; 
    SOCP_Params.planarConstraints = struct;
    SOCP_Params.vFinal = [0;0;0];

end 