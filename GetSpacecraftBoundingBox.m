function Box = GetSpacecraftBoundingBox(SpacecraftCondition)

switch SpacecraftCondition
    case 1 
        Box.origin = [0;0;0];
        Box.dimensions = [2;3;1]; % in m
        Box.orientation.quaternionLVLH_To_Body = [1;0;0;0]; % first 3 cmponents must be unit vec
        Box.orientation.DCM_LVLH_To_Body = eye(3);

    case 2 % half the size of case 1  
        Box.origin = [0;0;0];
        Box.dimensions = 0.5.*[2;3;1]; % in m
        Box.orientation.quaternionLVLH_To_Body = [1;0;0;0]; % first 3 cmponents must be unit vec
        Box.orientation.DCM_LVLH_To_Body = eye(3);
end 

end