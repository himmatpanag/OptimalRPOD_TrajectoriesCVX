function solStruct = WriteCVX_SolStruct(x,eta,u,CVX_Params)

throttle = eta./CVX_Params.aMax;
theta = zeros(size(eta)); phi = zeros(size(eta));
constraint = zeros(size(eta));
c = cos(CVX_Params.thrustPointingConstraint.angle);
thrustDirAngle = zeros(size(eta));

for ii = 1:numel(eta) 
    xtt = (x(:,ii)+x(:,ii+1))/2;
    utt = u(:,ii);
    etatt = vecnorm(utt);
    uttUnit = utt./etatt;
    rtt = vecnorm(xtt(1:3));
    constraint(ii) = xtt(1:3)'*utt - rtt * etatt * c;
    if etatt > 1e-3 * CVX_Params.aMax % 1% throttle 
        thrustDirAngle(ii) = acos(dot(uttUnit',xtt(1:3)')./rtt)*180/pi;
    end
end

for ii = 1:numel(theta)
    utt = u(:,ii);
    etatt = vecnorm(utt);
    if etatt > 1e-3 * CVX_Params.aMax % 1% throttle 
        xtt = (x(:,ii)+x(:,ii+1))/2;
        pos = xtt(1:3);
        r1 = pos(1); r2 = pos(2); r3 = pos(3);
        rNorm = norm(pos(1:3));
        r12 = sqrt(r1^2 + r2^2);
    
        gamma = acos(r3/rNorm); % dot(z,r)/norm(r); z = [0;0;1]
        n = -[r2;-r1;0]/r12; % cross(X(1:3),z)/norm(cross(X(1:3),z));
        q0 = cos(gamma/2);
        quatVec = n.*sin(gamma/2);
        skewSymmetric = @(v) [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
        RotMatrix = ((q0^2 - dot(quatVec,quatVec)) * eye(3) + 2.*quatVec*quatVec' + 2*q0*skewSymmetric(quatVec));
    
        [~,t,p] = CartesianToSpherical(-RotMatrix*u(:,ii)); % plume is negagtive control in rotaated frame
        theta(ii) = 180-t; phi(ii) = p;
    end
end 
time = linspace(0,CVX_Params.simTimeHours*3600,CVX_Params.numSteps);
solStruct.constraint.type =  POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
solStruct.constraintViolation = constraint;
solStruct.thrustDirAngle = thrustDirAngle;
solStruct.u = u;
solStruct.theta = theta; 
solStruct.phi = phi; 
solStruct.t = time(1:end-1);
solStruct.throttle = throttle; 
solStruct.x = x; 
solStruct.CVX_Params = CVX_Params;

end 