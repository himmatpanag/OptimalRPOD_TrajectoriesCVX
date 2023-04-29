function rho = CalculatePlumeDensity(radius, angle, thrusterParams)
% rho = particle density

g0 = 9.81; 
Na = 6.0221408e+23; % Avogadro's number
massFlowRate = thrusterParams.thrust/(g0*thrusterParams.Isp); % kg/s

R0 = thrusterParams.plumeProfile.measurementRadius;
densityAtR0 = interpn(thrusterParams.plumeProfile.angleBPs,thrusterParams.plumeProfile.flux,...
    angle);
rhoExperimentalProfile = densityAtR0 * R0^2/radius^2; 

if contains(thrusterParams.plumeProfile.fluxUnit,'Particles')
    % Scale rho from experimental flux in data to actual particle flux
    MolesPerSecond = massFlowRate*1000/thrusterParams.plumeComposition.weightedAverageMolecularWeight;
    totalParticlesFlux = MolesPerSecond*Na;
    rho = rhoExperimentalProfile * totalParticlesFlux/thrusterParams.plumeProfile.totalFlux;
elseif contains(thrusterParams.plumeProfile.fluxUnit,'Grams')
    % scale based on mass flow rate
    massDensity = rhoExperimentalProfile * massFlowRate*1e3/thrusterParams.plumeProfile.totalFlux;
    rho = CalculateNumParticleFromMass(massDensity,thrusterParams.plumeComposition);
else
    error('Flux units not supported');
end 

end 