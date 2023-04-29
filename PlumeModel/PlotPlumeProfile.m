function PlotPlumeProfile(plumeProfile)

if nargin < 1 
    thrusterParams = ThrusterPlumeParameters(3);
    plumeProfile = thrusterParams.plumeProfile;
end 
func = @(phi,theta) interpn(plumeProfile.angleBPs,plumeProfile.flux,...
    acos( cos(phi)*sin(theta) )); 
figure; grid on; hold on; 
phis = linspace(-plumeProfile.angleBPs(end),plumeProfile.angleBPs(end));
thetas = linspace(0,pi);
Z = zeros([100,100]);
for ii = 1:numel(phis)
    for jj = 1:numel(thetas)
        Z(ii,jj) = func(phis(ii),thetas(jj));
    end
end 
figure, grid on, hold on, 
surf(phis*180/pi,thetas*180/pi,Z)
xlabel('Phi (Azimuth) deg'),ylabel('Theta (Polar Angle) (deg)')
zlabel('Particle Flux N/m^2*s')
title("Plume profile flux at radius = " + plumeProfile.measurementRadius + "m");


end 