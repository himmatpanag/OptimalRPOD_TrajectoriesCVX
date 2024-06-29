function [eta, x, u] = OptimalApproachTrajCW(params)
    % This function uses CVX to generate the optimal trajectory of a convex
    % RPO problem. This function is limited to a non rotating target spacecraft
    % and uses the CW equations in cartesian coordinates. 

    N = params.numSteps; % Number of discretization steps in control input 

    Omega = params.Omega; 
    OmegaSq = params.Omega^2;
    if params.limitThrust
        scale = params.aMax;
    else 
        scale = 1; 
    end 
    xScale = params.aMax/vecnorm(params.rInit);
%     xScale = 1; 
    f = ones(N-1,1)./N; % Order 0<f<aMax; scale;%.*1e-2;%./OmegaSq;
    
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
    switch params.solver 
        case Solver.MOSEK
            cvx_solver MOSEK
        case Solver.SDPT3
            cvx_solver SDPT3
    end 
        
    % Define Variables
    variable eta(N-1)
    variable x(6,N)
    variable u(3,N-1)
    
    % Set up cost function and boundary conditions
    minimize(f'*eta)
    subject to 
    x(:,1) == [params.rInit;params.vInit]; 
    x(:,N) == [params.rFinal;params.vFinal];
    
    for ii = 1:N-1 % Relaxation constraint
        norm(u(:,ii)) <= eta(ii);
        0 <= eta(ii);
        if params.limitThrust
            eta(ii) <= params.aMax;
        end 
        
        % Dynamics
        xScale.*(x(:,ii+1) - Phi*x(:,ii) - Bd*u(:,ii)) == 0;
    end
    
    %% Other Constraints
    if params.coneConstraintActive % This is a second order cone constrant
        for ii = 2:N-1
            coneAxisUnitVec = params.coneAxisVector;
            norm(x(1:3,ii))*cos(params.coneAngle) <= ...
                coneAxisUnitVec'*x(1:3,ii);
        end 
    end 
        
    if params.usePlanarConstraints
        for ii = 2:N-1
            for kk = 1:numel(params.planarConstraint)
                % Assert that dot(x_kk,i) > Rmin;
                dot(x(1:3,params.planarConstraint(kk).elementNum),...
                    params.planarConstraint(kk).dirVector) ...
                    >= params.planarConstraint(kk).minRadius;
            end 
        end             
    end 

    if params.trustRegion.useTrustRegion 
        if params.trustRegion.pos > 0 
            for ii = 1:N
                xPrev = params.thrustPointingConstraint.prevTraj;% (:,xIdx);
                norm(x(1:3,ii)-xPrev(1:3,ii)) <= params.trustRegion.pos
            end 
        end 
        if params.trustRegion.eta > 0 
            for ii = 1:N-1
                etaPrev = params.thrustPointingConstraint.prevEta;% (:,xIdx);
                abs(eta(ii)-etaPrev(ii)) <= params.trustRegion.eta .* params.aMax;
            end 
        end 
    end 

    if params.thrustPointingConstraint.active
        for ii = 1:N-1
            if params.thrustPointingConstraint.useLinearApprox                     
                for jj = 1:2 % Take endpoints of each interval
                    xIdx = ii + jj - 1;
                    xPrev = params.thrustPointingConstraint.prevTraj(:,xIdx);
                    uPrev = params.thrustPointingConstraint.prevControl(:,ii);
                    etaPrev = params.thrustPointingConstraint.prevEta(ii);
                    rPrev = vecnorm(xPrev(1:3));
                    if rPrev < params.thrustPointingConstraint.activeRadius % KM!
                        if params.thrustPointingConstraint.useSphericalTarget
                            % Need to linearise cos(alpha) term as well
                            % Param should be in km!
                            alpha = asin(SOCP_Params.thrustPointingConstraint.targetRadius/rPrev); 
                        else 
                            c = cos(params.thrustPointingConstraint.angle);
                            hFuncPrev = dot(xPrev(1:3),uPrev)-rPrev*etaPrev*c;
                            gradhFunc_eta = -rPrev*c;
                            gradhFunc_x = uPrev - (xPrev(1:3)/rPrev) .* etaPrev * c;
                            gradhFunc_u = xPrev(1:3);
                            
                            % Linearised constraint: 
                            (hFuncPrev + gradhFunc_eta' * (eta(ii)-etaPrev) + ...
                                gradhFunc_u' * (u(:,ii)-uPrev) + ...
                                gradhFunc_x' * (x(1:3,xIdx)-xPrev(1:3))) <= 0 

                            % Alternative constraint, appears to be more stable
%                             -hFuncPrev + gradhFunc_u'*u(:,ii) + ...
%                                 gradhFunc_x' * (x(1:3,xIdx)) + ...
%                                 gradhFunc_eta * eta(ii) <= 0
                        end
                    end 
                    
                end

            elseif params.thrustPointingConstraint.useSecondOrderConeApproximation
                % TODO!
            else % Simplified linearisation of constraint
                % Takes the middle of the previous traj to generate
                % direction vector
%                 numConst = params.thrustPointingConstraint.numConstraintsPerStep;

%                 for jj = 1:numConst                    
%                     xPrev = interpn([0,numConst],params.thrustPointingConstraint.prevTraj(:,ii:ii+1)',jj)';
%                     rPrev = vecnorm(xPrev(1:3));
%                     if rPrev < 0.100
%                         alpha = 0;
%                         if params.thrustPointingConstraint.useSphericalTarget
%                              % Param should be in km!
%                             alpha = asin(SOCP_Params.thrustPointingConstraint.targetRadius/rPrev);
%                         end 
%                         xPrev(1:3)*u(:,ii) <= rPrev*eta(ii)*cos(alpha + params.thrustPointingConstraint.angle);
%                     end 
%                 end 
            end 
                    
        end
    end 
    cvx_end 
end 