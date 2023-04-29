function N = CalculateNumParticleFromMass(mass,plumeComposition)
% Mass in grams 
Na = 6.0221408e+23; % Avogadro's number

switch plumeComposition.fractionType 
    case 'Mass'
        N = 0; 
        for ii = 1:size(plumeComposition.exhaust,1)
            N = N + mass*plumeComposition.exhaust{ii,2}/...
                plumeComposition.exhaust{ii,1}.molecularMass*Na;
        end 
    case 'Mole'
        x = 0; 
        for ii = 1:size(plumeComposition.exhaust,1)
            x = x + plumeComposition.exhaust{ii,1}.molecularMass * ...
                plumeComposition.exhaust{ii,2};
        end 
        N = mass * Na/x;
end 
    
end 