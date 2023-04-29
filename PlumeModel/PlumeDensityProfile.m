function profile = PlumeDensityProfile(profileNum)

switch profileNum    
    case 1 % Biprop 
        % Source: Cassini mission
        profile = CassiniMissionData(1);

    case 2 % Monoprop
        % Source: Cassini mission 
        profile = CassiniMissionData(2);
    case 3 % Cold Gas
        % Source: Basics of Plume Impingement Analysis for Small Chemical and Cold Gas Thrusters, G. Dettleff, M. Grade, 2011
        profile.measurementRadius = 0.1; % m 
        profile.angleBPs = (0:10:150)*pi/180; % rad
        profile.flux = [3e23,2.2e23,8e22,1.7e22,6e21,2.2e21,1.3e21,...
            5e20,3e20,1.8e20,6e19,2.5e19,1e19,3.5e18,3.1e18,2.5e18];
        profile.background = 1e18; 
        profile.totalFlux = CalculatePlumeFluxThroughSphere(profile);
        profile.fluxUnit = 'ParticlesPerSecond';
end 
end 

function profile = CassiniMissionData(type)
    % Source: Cassini Plumes
    switch type 
        case 2 % monoprop
            profile.fluxUnit = 'GramsPerSecond';
            profile.totalFlux = 0.4; % g/s
            profile.measurementRadius = 1; % m, flux per steradian = flux through 1m^2 of 1m radius sphere
            profile.angleBPs = [0:10:120,180]*pi/180;
            massFluxPerSolidAngle = [0.75301584
                0.54754525
                0.20271493
                0.04153756
                0.01192873
                0.00523656
                0.00289448
                0.00171844
                0.00102253
                0.00060604
                0.00036097
                0.00021492
                0.00012823
                6.1275E-06]';
            profile.flux = massFluxPerSolidAngle;
        case 1 % biprop (main engine)
                profile.measurementRadius = 1; % m, flux per steradian = flux through 1m^2 of 1m radius sphere
                lbToGram = 453.592;
                profile.fluxUnit = 'GramsPerSecond';
                profile.totalFlux = 162; % g/s
                profile.angleBPs = pi/180.*[0
                    4.88
                    9.57
                    13.74
                    20.51
                    24.94
                    30.89
                    35.36
                    39.80
                    45.73
                    50.15
                    54.56
                    60.46
                    64.86
                    70.72
                    75.10
                    79.50
                    85.33
                    90.28
                    94.10
                    99.91
                    105.69
                    110.00
                    115.70
                    120.00
                    124.89
                    130.25
                    134.86
                    142.56
                    180]';
                massFluxPerSolidAngle = [1.30
                    1.25
                    1.08
                    0.77
                    0.29
                    0.15
                    0.62e-1
                    0.33e-1
                    0.19e-1
                    0.10e-1
                    0.67810e-2
                    0.46e-2
                    0.32e-2
                    0.24e-2
                    0.17e-2
                    0.13e-2
                    0.99e-3
                    0.67e-3
                    0.50e-3
                    0.38e-3
                    0.26e-3
                    0.18e-3
                    0.13e-4
                    0.81e-4
                    0.52e-4
                    0.32e-4
                    0.14e-5
                    0.59e-6
                    0.91e-6
                    0.91e-6]'*lbToGram;
                profile.flux=massFluxPerSolidAngle;
                    
    end 
 
end 