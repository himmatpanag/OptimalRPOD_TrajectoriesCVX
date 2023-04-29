function contamination = CalculateContaminationOverTrajectory(accelType,thrusterParams,timeChaser,posChaser,accelChaser,massChaser,timeTarget,posTarget,orientationDCM_LVLH_ToBody_Target,Box)
 % TODO: TargetS/C Box is rotating with time (i.e. with respect to the LVLH
 % frame)

switch accelType
    case 'Interpolate'
    
        throttleTol = 1e-3;
        contaminationPerSecond = zeros(size(timeChaser));
        
        orientationTargetVec = reshape(shiftdim(orientationDCM_LVLH_ToBody_Target,2),[numel(timeTarget),9]);    
        for ii = 1:numel(timeChaser)
            accelChaserii = interpn(timeChaser,accelChaser',timeChaser(ii));
            if massChaser*vecnorm(accelChaserii)/thrusterParams.thrust > throttleTol            
                                
                Box.orientation.DCM_LVLH_To_Body = reshape(interpn(timeTarget, orientationTargetVec, timeChaser(ii)),[3,3]);
                plates = ComputeSpacecraftPlatePositions(posChaser(:,ii),Box);
                plumeOrigin = posChaser(:,ii);
                plumeDir = -accelChaserii./vecnorm(accelChaserii); 
                for jj = 1:3                
                    contaminationPerSecond(ii) = contaminationPerSecond(ii) + ...
                        CalculatePlumeFluxThroughPlate(thrusterParams, plumeOrigin, plumeDir,...
                        plates(jj).plateCentre, plates(jj).plateVec1, plates(jj).plateVec2);
                end 
            end
        end 
        contamination = cumtrapz(timeChaser,contaminationPerSecond);
    
    case 'ZeroOrderHold' % The SOCP returns a piecewise constant control history 
        throttleTol = 1e-3;
        plateLengthTol = 0.05; % ignore plates of length < 5cm
        orientationTargetVec = reshape(shiftdim(orientationDCM_LVLH_ToBody_Target,2),[numel(timeTarget),9]);    
        stepSize = diff(timeChaser)./2;
        accelTimes = timeChaser(1:end-1) + stepSize;
        N = 30;
        contamination.contaminationPerStep = zeros(numel(accelTimes),N+1);
        contamination.stepTimes = zeros(numel(accelTimes),N+1);
        TotalContaminationEndPrevStep = 0;
        for ii = 1:numel(accelTimes)
            accelChaserii = accelChaser(:,ii);
            throttle = massChaser*vecnorm(accelChaserii)/thrusterParams.thrust;
            timesInStep = linspace(timeChaser(ii),timeChaser(ii+1),N+1);
            if throttle > throttleTol       
                contaminationInTimeStep = zeros(N+1,1);                
                plumeDir = -accelChaserii./vecnorm(accelChaserii); 
                for jj = 1:N+1 % 1:2                    
   
                    Box.orientation.DCM_LVLH_To_Body = reshape(interpn(timeTarget, orientationTargetVec, timesInStep(jj)),[3,3]);                    
                    plumeOrigin = interpn(timeChaser,posChaser',timesInStep(jj)); 
                    plates = ComputeSpacecraftPlatePositions(plumeOrigin,Box);                                        

                    for kk = 1:3 
                        if vecnorm(plates(kk).plateVec1) > plateLengthTol && vecnorm(plates(kk).plateVec2) > plateLengthTol
                        contaminationInTimeStep(jj) = contaminationInTimeStep(jj) + ...
                            throttle .* CalculatePlumeFluxThroughPlate(thrusterParams, plumeOrigin, plumeDir,...
                            plates(kk).plateCentre, plates(kk).plateVec1, plates(kk).plateVec2);
                        end 
                    end 
                end
                contaminationPerStep =  TotalContaminationEndPrevStep + ...
                    cumtrapz(timesInStep,contaminationInTimeStep);
                contamination.contaminationPerStep(ii,:) = contaminationPerStep;% (2:end-1)                
                TotalContaminationEndPrevStep = contamination.contaminationPerStep(ii,end);
            end  
            contamination.stepTimes(ii,:) = timesInStep;% (2:end); % Should skips first timePoint?
        end 
end 

end 
%     func = @GetContaminationPerSecond;
%     contamination2 = integral(@(t) func(t,thrusterParams,timeChaser,posChaser,accelTimes,accelChaser,massChaser,timeTarget,posTarget,orientationDCM_Target,Box),...
%         timeChaser(1),timeChaser(end));
 


% function c = GetContaminationPerSecond(times,thrusterParams,timeChaser,posChaser,accelTimes,accelChaser,massChaser,timeTarget,posTarget,orientationDCM_Target,Box)
%     c = zeros(size(times));
%     orientationTargetVec = reshape(shiftdim(orientationDCM_Target,2),[numel(timeTarget),9]);
% 
%     for ii = 1:numel(times)
%         time = times(ii);
%         accelChaserii = interpn(accelTimes,accelChaser',time,'nearest');
%         massChaserii = interpn(timeChaser,massChaser,time);
%         throttleTol = 1e-3;
% 
%         if massChaserii*vecnorm(accelChaserii)/thrusterParams.thrust > throttleTol            
%             % Box.origin = posTarget(:,ii); This is the origin for the  CW
%             % frame. Modify this if the target is rotating with respect to
%             % the LVLH frame
%             posChaserii = interpn(timeChaser, posChaser',time);
%             Box.orientation.DCM_LVLH_To_Body = reshape(interpn(timeTarget, orientationTargetVec, time),[3,3]);
%             plates = ComputeSpacecraftPlatePositions(posChaserii,Box);
%             plumeOrigin = posChaserii;
%             plumeDir = -accelChaserii./vecnorm(accelChaserii); 
%             for jj = 1:3                
%                 c(ii) = c(ii) + ...
%                     CalculatePlumeFluxThroughPlate(thrusterParams, plumeOrigin, plumeDir,...
%                     plates(jj).plateCentre, plates(jj).plateVec1, plates(jj).plateVec2);
%             end         
%         end 
%     end 
% 
% end 