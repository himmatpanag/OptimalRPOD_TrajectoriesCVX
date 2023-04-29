function params = MassParameters(type)

switch type 
    case 1 
        params.totalMass = 450; 
        params.fuelMass = 50; 
        params.payloadMass = 80;
        params.inertia = CalculateInertiaOfPrism([2,3,1], params.totalMass);
end 

end

function inertia = CalculateInertiaOfPrism(dimensions, mass)
% dimensions are body (x, y, z)

inertia.origin = [0;0;0];
inertia.DCM_BodyToInertialFrame = eye(3);

inertia.Ixx = 1/12 * mass * (dimensions(2)^2 + dimensions(3)^2);
inertia.Iyy = 1/12 * mass * (dimensions(1)^2 + dimensions(3)^2);
inertia.Izz = 1/12 * mass * (dimensions(2)^2 + dimensions(1)^2);
inertia.Ixy = 0;
inertia.Iyx = 0;
inertia.Ixz = 0;
inertia.Izx = 0;
inertia.Iyz = 0;
inertia.Izy = 0;

end 