function [eta, x, u] = OptimalApproachTrajCW(params)
    % This function uses CVX to generate the optimal trajectory of a convex
    % RPO problem. This function is limited to a non rotating target spacecraft
    % and uses the CW equations in cartesian coordinates. 

    N = params.numSteps; % Number of discretization steps in control input 

    Omega = params.Omega; 
    OmegaSq = params.Omega^2;
    f = ones(N,1)./OmegaSq;
    
    simTime = params.simTimeHours*60*60; 
    dt = simTime/N;
    A = zeros(6,6);
    A(4,1) = 3*OmegaSq; A(6,3) = -OmegaSq;
    A(4,5) = 2*Omega; A(5,4)=-2*Omega;
    A(1,4) = 1; A(2,5) = 1; A(3,6) = 1; 
    B = [zeros(3,3);eye(3)];

    Phi=expm(A*dt);
    
    %% Need to calculate Bd = Phi * Integral(expm(-Au)*B du from u = 0 to dt)
    % Equation 8 in 2021 Ortolano paper 
    % Method 1, Source: https://math.stackexchange.com/questions/658276/integral-of-matrix-exponential#:~:text=is%20given%20by%20x(t,eAtx0.&text=for%20some%20T%3E0.
    % Construct matrix A2 = [-A,B;0;0]; 
    A2 = [-A,B;zeros(3,9)];
    A2tExp = expm(A2*dt);
    IntegralExpAdtB = A2tExp(1:6,7:9);
    Bd = Phi*IntegralExpAdtB;
    
    % Method 2 Jordan form - use this to check above approach 
    % requires symbolic toolbox
    %     [V,J]=jordan(-A);
    %     M = J(1:2,1:2); % upper triangular [0,1;0,0]
    %     C = J(3:end,3:end);
    %     K = C\(expm(dt.*C)-eye(4));
    %     IntegralExpMt = [dt,(dt^2)/2;0,dt]; 
    %     IntegralExpAdt = V * [IntegralExpMt,zeros(2,4);zeros(4,2),K] * (V\eye(6));
    %     % The imaginary parts of this matrix are ~0. 
    %     IntegralExpAdtB = real(IntegralExpAdt)*B;
    %     Bd = Phi*IntegralExpAdtB;    
    % Confirmed both methods produce same answer within 1e-10
    
    %% Optimization
    cvx_begin
        variable eta(N)
        variable x(6,N)
        variable u(3,N)

        minimize(f'*eta)
        subject to 
        x(:,1) == [params.rInit;params.vInit];
        x(:,N) == [params.rFinal;params.vFinal];
        
        for ii = 1:N
            norm(u(:,ii)) <= eta(ii);
            0 <= eta(ii);
            eta(ii) <= params.aMax;
            
            if params.coneConstraintActive 
                coneAxisUnitVec = params.coneAxisVector;
                norm(x(1:3,ii))*cos(params.coneAngle) <= ...
                    coneAxisUnitVec'*x(1:3,ii);
            end 
            
            if params.usePlanarConstraints
                for kk = 1:numel(params.planarConstraint)
                    % Assert that dot(x_kk,i) > Rmin;
                    dot(x(1:3,params.planarConstraint(kk).elementNum),...
                        params.planarConstraint(kk).dirVector) ...
                        >= params.planarConstraint(kk).minRadius;
                end 
            end 
        end 
        for ii = 2:N
            x(:,ii) == Phi*x(:,ii-1) + Bd*u(:,ii-1);
        end         
    cvx_end 
end 