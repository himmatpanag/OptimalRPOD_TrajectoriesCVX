function plates = ComputeSpacecraftPlatePositions(plumeOrigin,BoundingBox)

if false
    BoundingBox = GetSpacecraftBoundingBox(1);
    plumeOrigin = [-5;0;0];
end 

plateStruct = GenerateEmptyPlateStruct;
plates = repmat(plateStruct,3,1);
% For now assume that the SC is fixed with respect to the LVLH frame, i.e.
% it is rotating at the same rate as the orbit period (tidally locked)

I = eye(3);
for ii = 1:3
    e = I(:,ii); % unit vector corresponding to plate
    plate1PosBodyFrame = BoundingBox.origin+e*BoundingBox.dimensions(ii)/2;
    plate2PosBodyFrame = BoundingBox.origin-e*BoundingBox.dimensions(ii)/2;

    plate1Pos = BoundingBox.orientation.DCM_LVLH_To_Body'* plate1PosBodyFrame;
    plate2Pos = BoundingBox.orientation.DCM_LVLH_To_Body'* plate2PosBodyFrame;
    
    if norm(plumeOrigin-plate1Pos) < norm(plumeOrigin-plate2Pos) % Get the plate that is closest to the plume source
        plates(ii).plateCentre = plate1Pos;        
    else 
        plates(ii).plateCentre = plate2Pos;
    end 
    
    idx1 = mod(ii,3)+1;
    idx2 = mod(ii+1,3)+1;
    plateVec1BodyFrame = I(:,idx1) * BoundingBox.dimensions(idx1)/2;
    plateVec2BodyFrame = I(:,idx2) * BoundingBox.dimensions(idx2)/2;
    plates(ii).plateVec1 = BoundingBox.orientation.DCM_LVLH_To_Body' * plateVec1BodyFrame;
    plates(ii).plateVec2 = BoundingBox.orientation.DCM_LVLH_To_Body' * plateVec2BodyFrame;

end 

end 

function plateStruct = GenerateEmptyPlateStruct
    plateStruct.plateCentre = [0;0;0];
    plateStruct.plateVec1 = [0;0;0];
    plateStruct.plateVec2 = [0;0;0];
end 