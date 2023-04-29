function composition = PlumeComposition(type)
    switch type 
        case 1  %  Bipropellant
            % Source Cassini Plumes
            composition.fractionType = 'Mass'; 
            composition.exhaust = {Propellant.Nitrogen,0.4219;
                Propellant.Water,0.2914;
                Propellant.CarbonMonoxide, 0.1765;
                Propellant.CarbonDioxide,0.0881;
                Propellant.HydrogenDiatomic,0.0161};
            composition.weightedAverageMolecularWeight = 0; % TODO!
                
        case 2  % MonoProp          
            composition.fractionType = 'Mole'; 
            composition.exhaust = {Propellant.HydrogenDiatomic,.424;
                Propellant.Nitrogen,.288;
                Propellant.Ammonia,.280};
            composition.weightedAverageMolecularWeight = 0; % TODO!
             
        case 3 % Cold Gas
            composition.fractionType = 'Mass';
            composition.exhaust = {Propellant.Nitrogen,1};
            composition.weightedAverageMolecularWeight = Propellant.Nitrogen.molecularMass;

        case 32 % Cold gas xenon
            composition.fractionType = 'Mass';
            composition.exhaust = {Propellant.Xenon,1};
            composition.weightedAverageMolecularWeight = Propellant.Xenon.molecularMass;

    end 
end 