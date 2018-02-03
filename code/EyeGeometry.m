classdef EyeGeometry
    % Model of eye motion geometry. 
    % 
    % Use animateCamera() to show a sketch of eye and link motion (use
    % arrow keys to move camera). 
    % 
    % Use randomize() to perturb the parameters (e.g. link length) randomly
    % consistent with uncertainty in these parameters. Fitting measurements
    % to many random perturbations will help to calibrate these values. 
    % 
    % Use getExtension() to find the actuator extension that corresponds to
    % given yaw and pitch angles. This can be compared to the measured 
    % actuator position to find the zero point of the actuator. This 
    % depends on the robot's position when it is turned on, so it has to be
    % calculated each time. 
    
    % TODO: some uncertainty as well in actual angle of encoder disk
    
    properties
        linkLength;
        eyeCentre;
        eyePitchAxisOffset;
        actuatorOriginLeft; 
        actuatorOriginRight; 
        actuatorAngle;  
        
        ballLeft; % position of left ball join in eye coordinates 
        ballRight;
        
        eyeBoundsX;
        eyeBoundsY;
        eyeBoundsZ;
    end
    
    methods 
        function eg = EyeGeometry(varargin)
            
            if nargin > 0
                rightEye = varargin{1};
            else 
                rightEye = 1;
            end
                        
            eg.linkLength = .0604; 
            eg.eyePitchAxisOffset = -0.002; % pitch centre of rotation in z direction (from camera origin)
            eg.actuatorAngle = degtorad(17.3);
            
            if rightEye
                eg.eyeCentre = [.03; .021; .08];
                eg.actuatorOriginLeft = [0.015; .048; .019];
                eg.actuatorOriginRight = [0.015+.029; .048; .019];
            else
                eg.eyeCentre = [-.03; .021; .08];
                eg.actuatorOriginLeft = [-0.015-.029; .048; .019];
                eg.actuatorOriginRight = [-0.015; .048; .019];
            end
            
            % ball joint position in camera coordinates
            eg.ballLeft = [-.01; .015; .015]; %-.014 .02 .02
            eg.ballRight = [.01; .015; .015];
            
            % for plotting camera box
            eg.eyeBoundsX = [-.015 .015];
            eg.eyeBoundsY = [-.015 .015];
            eg.eyeBoundsZ = [-.03 .02];
        end
        
        function randomize(eg)
            % Adds random perturbations to uncertain parameters. The
            % parameters can be estimated by fitting data against many
            % eye geometries with randomized parameters. The distributions
            % reflect an estimate of the uncertainty in the measurements. 
            eg.linkLength = eg.linkLength + .003*randn;
            eg.eyePitchAxisOffset = eg.eyePitchAxisOffset + .002*randn;
            eg.actuatorAngle = eg.actuatorAngle + degtorad(3)*randn;
            eg.eyeCentre = eg.eyeCentre + .002*randn(3,1);
            
            % we assume symmetry in actuator origins
            actuatorOriginOffsetYZ = .002*randn(2,1);
            eg.actuatorOriginLeft = eg.actuatorOriginLeft + [.001*randn; actuatorOriginOffsetYZ];
            eg.actuatorOriginRight = eg.actuatorOriginRight + [.001*randn; actuatorOriginOffsetYZ];
        end
        
        function result = getHeadCoords(eg, eyeCoords, yaw, pitch)
            % Calculates positions of points in eye coordinates subject to
            % yaw and pitch angles. 
            % 
            % eyeCoords: (3 x n) points in eye coordinates
            % yaw: yaw angle of eye (rad)
            % pitch: pitch angle of eye (rad)
            
            
            translateForward = augmentMatrix(eye(3), [0; 0; -eg.eyePitchAxisOffset]);
            translateBackward = augmentMatrix(eye(3), [0; 0; eg.eyePitchAxisOffset]);
            rotateYaw = augmentMatrix([cos(yaw) 0 -sin(yaw); 0 1 0; sin(yaw) 0 cos(yaw)], zeros(3,1));
            rotatePitch = augmentMatrix([1 0 0; 0 cos(pitch) sin(pitch); 0 -sin(pitch) cos(pitch)], zeros(3,1));
            translateEye = augmentMatrix(eye(3), eg.eyeCentre);

            A = translateEye * rotateYaw * translateBackward * rotatePitch * translateForward;
            
            result = A * augmentVectors(eyeCoords);
            result = result(1:3,:);  
        end
        
        function result = getCameraCorners(eg)
            % result: (3x8) positions of camera corners in camera coords
            x = eg.eyeBoundsX;
            y = eg.eyeBoundsY;
            z = eg.eyeBoundsZ;
            result = [
                x(1) y(1) z(1); %left-bottom-back
                x(2) y(1) z(1); %right-bottom-back
                x(2) y(2) z(1); %right-top-back
                x(1) y(2) z(1); %left-top-back
                x(1) y(1) z(2); %left-bottom-front
                x(2) y(1) z(2); %right-bottom-front
                x(2) y(2) z(2); %right-top-front
                x(1) y(2) z(2); %left-top-front
                ]'; 
        end
        
        function h = animateCamera(eg)
            h = figure('keypressfcn', @moveCamera);
            h.UserData = struct('eg', eg, 'yaw', 0, 'pitch', 0);
            showMoveCamera(eg, 0, 0);
        end
        
        function drawCamera(eg, yaw, pitch) 
            eyeCoords = eg.getCameraCorners();
            corners = eg.getHeadCoords(eyeCoords, yaw, pitch);
            
            plot3([corners(1,1:4) corners(1,1)], [corners(2,1:4) corners(2,1)], [corners(3,1:4) corners(3,1)], 'k')
            plot3([corners(1,5:8) corners(1,5)], [corners(2,5:8) corners(2,5)], [corners(3,5:8) corners(3,5)], 'k')
            for i = 1:4
                ind = [i 4+i];
                plot3(corners(1,ind), corners(2,ind), corners(3,ind), 'k')
            end
            axis equal
        end
        
        function [actuator, extension] = getExtension(eg, yaw, pitch)
            % Find how far each linear actuator is extended from its fully
            % withdrawn position. 
            % 
            % yaw: yaw angle of camera
            % pitch: pitch angle of camera
            % 
            % actuator: (3 x 2) positions of actuator endpoints [left
            %   right]
            % extension: (1 x 2) extension of each actuator [left right]
            
            actuator = zeros(3,2);
            extension = zeros(1,2);

            for i = 1:2
                if i == 1
                    ball = eg.getHeadCoords(eg.ballLeft, yaw, pitch); 
                    actuatorOrigin = eg.actuatorOriginLeft;
                else
                    ball = eg.getHeadCoords(eg.ballRight, yaw, pitch); 
                    actuatorOrigin = eg.actuatorOriginRight;
                end
                fun = @(extension) (eg.linkLength - getLinkLength(actuatorOrigin, eg.actuatorAngle, extension, ball))^2;
                [extension(i), fval] = fminbnd(fun,0,.08);
                actuator(:,i) = getActuator(actuatorOrigin, eg.actuatorAngle, extension(i));
                
                if fval > 1e-6
                    warning('inconsistent geometry')
                end
            end
            
        end
    end
end

function moveCamera(hObject, eventdata)
    limit = .7; % range of motion limit (rad) in each direction
    if strcmp(eventdata.Key, 'downarrow')
        hObject.UserData.pitch = max(-limit, hObject.UserData.pitch - .1);
    elseif strcmp(eventdata.Key, 'uparrow')
        hObject.UserData.pitch = min(limit, hObject.UserData.pitch + .1);
    elseif strcmp(eventdata.Key, 'leftarrow')
        hObject.UserData.yaw = min(limit, hObject.UserData.yaw + .1);
    elseif strcmp(eventdata.Key, 'rightarrow')
        hObject.UserData.yaw = max(-limit, hObject.UserData.yaw - .1);
    elseif strcmp(eventdata.Key, '0')
        hObject.UserData.yaw = 0;
        hObject.UserData.pitch = 0;
    end

    showMoveCamera(hObject.UserData.eg, hObject.UserData.yaw, hObject.UserData.pitch);
end

function showMoveCamera(eg, yaw, pitch)
    clf()
    configPlot()
    hold on
    plot3(0,0,0,'rx')
    plot3(eg.eyeCentre(1), eg.eyeCentre(2), eg.eyeCentre(3), 'kx');

    eg.drawCamera(yaw, pitch);  

    ballLeft = eg.getHeadCoords(eg.ballLeft, yaw, pitch);
    ballRight = eg.getHeadCoords(eg.ballRight, yaw, pitch);
    [actuator, ~] = eg.getExtension(yaw, pitch);

    linkageLeft = [eg.actuatorOriginLeft actuator(:,1) ballLeft];
    linkageRight = [eg.actuatorOriginRight actuator(:,2) ballRight];

    plot3(linkageLeft(1,:), linkageLeft(2,:), linkageLeft(3,:), 'bo-')
    plot3(linkageRight(1,:), linkageRight(2,:), linkageRight(3,:), 'bo-')
        
    if eg.eyeCentre(1) > 0
        xlim([-.01 .07])
    else 
        xlim([-.07 .01])
    end
    ylim([-.01 .1])
    zlim([-.01 .11])

end

function result = augmentMatrix(rotation, translation)
    result = zeros(4,4);
    result(1:3,1:3) = rotation;
    result(1:3,4) = translation;
    result(4,4) = 1;
end

function result = augmentVectors(vectors)
    result = [vectors; ones(1, size(vectors,2))];
end

function configPlot()
    h = gca();
    h.CameraPosition = [1 1 1];
    h.CameraUpVector = [0 1 0];
    h.CameraUpVectorMode = 'manual';
    h.Projection = 'perspective';
    h.XDir = 'reverse';
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

function result = getLinkLength(actuatorOrigin, actuatorAngle, extension, ball)
    actuator = getActuator(actuatorOrigin, actuatorAngle, extension);
    result = norm(ball - actuator, 2);
end

function result = getActuator(actuatorOrigin, actuatorAngle, extension)
    result = actuatorOrigin + extension * [0; sin(actuatorAngle); cos(actuatorAngle)];
end

