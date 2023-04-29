function totalFlux = CalculatePlumeFluxThroughSphere(plumeProfile)

% See notes on 8th June
% https://en.wikipedia.org/wiki/Spherical_coordinate_system#Integration_and_differentiation_in_spherical_coordinates
% http://citadel.sjfc.edu/faculty/kgreen/vector/Block3/flux/node6.html
% Ex6 https://tutorial.math.lamar.edu/Classes/CalcIII/ChangeOfVariables.aspx 

rM = plumeProfile.measurementRadius;
fun = @(phi,theta) interpn(plumeProfile.angleBPs,...
    plumeProfile.flux,acos( cos(phi)*sin(theta) ),'linear',0) *sin(theta);
thetas = linspace(0,pi);
phis = linspace(-plumeProfile.angleBPs(end),plumeProfile.angleBPs(end));

F = zeros([100,100]);
for ii = 1:numel(phis)
    for jj = 1:numel(thetas)
        F(ii,jj) = fun(phis(ii),thetas(jj));
    end
end 
totalFlux = rM^2 * trapz(thetas,trapz(phis,F,2));

% totalFlux = rM^2 * integral2(fun,0,pi,-plumeProfile.angleBPs(end),...
%     plumeProfile.angleBPs(end));

end
