classdef Propellant 
    properties
        molecularMass
    end 
    methods 
        function m = Propellant(mass)
            m.molecularMass = mass;
        end 
    end 
   enumeration
      Hydazine (32.05)
      Nitrogen (14)
      CarbonDioxide (44.01)
      Xenon (131.3)
      MMH (46.07) % 4CH3NHNH2 + 5N2O4 → 9N2 + 4CO2 + 12H2O  
      DinitrogenTetroxide (92.01)
      Water (18)
      CarbonMonoxide (28)
      HydrogenDiatomic (2)
      Ammonia (17.031)

   end
end