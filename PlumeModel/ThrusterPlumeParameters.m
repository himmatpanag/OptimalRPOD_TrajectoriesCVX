function params = ThrusterPlumeParameters(type)

switch type 
    case 1 % Typical bipropellant thruster for a 300kg spacecraft. 
        % Space crew dragon 12.5tonnes, draco engines 400N each, Isp 300
        % sec
        params.thrusterType = ThrusterType.Bipropellant;
        params.thrust = 10; % N
        params.Isp = 300; % seconds
        params.plumeProfile = PlumeDensityProfile(1);
        params.plumeComposition = PlumeComposition(1);
    case 2 % monoprop
        params.thrusterType = ThrusterType.MonoPropellant;
        params.thrust = 6; % N
        params.Isp = 270; % seconds
        params.plumeProfile = PlumeDensityProfile(2);
        params.plumeComposition = PlumeComposition(2);
    case 3 % Cold Gas
        params.thrusterType = ThrusterType.ColdGas;
        params.thrust = 1; % N
        params.Isp = 90; % seconds
        params.plumeProfile = PlumeDensityProfile(3);
        params.plumeComposition = PlumeComposition(3);
        
end 
end 