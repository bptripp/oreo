function [] = eye_controller()

    close all;

    %% Globals

    global left;
    global right;
    global libname;
    global eye_cal;

    left = 1;
    right = 2;
    libname = 'macaque';

    %% Load the Library 
    if not(libisloaded(libname))
        loadlibrary(libname, 'macaque.h');
        calllib(libname,'start');
    end

    %% Calibrate
    recal = input('do you wish to recalibrate?','s');
    eye_cal = struct();
    
    filename = 'calib.mat';
    if exist(filename, 'file') ~= 2 || recal == 'y'
        calibration();
        %% Load the Library, the cal routine unloads it
        if not(libisloaded(libname))
            loadlibrary(libname, 'macaque.h');
            calllib(libname,'start');
        end
    end

    eye_cal = load(filename, 'eye');
    eye_cal = eye_cal.eye;

    %Set-up clean up function
    cleanup_result = onCleanup(@()cleanup_all());

    %% Configure Eye Drives
    
    %set the position control parameters, enable the control loops
    arrayfun(@(axis_id) config_eye_drive(axis_id), 1:4);
    
    %set the position control parameters, enable the control loops
    arrayfun(@(axis_id) config_neck_drive(axis_id), 1:3);

    %% Control Loop

    while true
        action = input('enter command:  ', 's');
        
        if strcmp(action,'exit') == 1
            break;
        elseif strcmp(action,'help') == 1
            disp('Possible commands are:');
            disp('neck y:[yaw angle degrees] p:[pitch angle degrees] r:[roll angle degrees]');
            disp('      -> this moves the eye gimbals to specified angles'); 
            disp('point x:[right] y:[up] z:[forward] (in metres)');
            disp('      -> this orients the eye gimbals toward a 3D point');             
            disp('exit  -> this exits you from the program stopping the eye and neck gimbals');

        elseif strncmpi(action, 'neck', length('neck')) == 1
            [des_angle_deg, num] = sscanf(action,'neck y:%f p:%f r:%f');
            if(num ~= 3)
                disp('Bad input');
                continue;
            end
            motion([], des_angle_deg, 2, 500);
            
        elseif strncmpi(action, 'point', length('point')) == 1
            [des_point, num] = sscanf(action,'point x:%f y:%f z:%f');
            if(num ~= 3)
                disp('Bad input');
                continue;
            end
            saccade(des_point, [0;0;0], 0, 2);
%             motion(des_point, [], 2, 500);
            
        else
            disp('Bad Input');
        end

    end

end

%% Misc Helpers
function [] = config_eye_drive(axis_id)
    global libname;
    
    %set accel and speed low to gain more accuracy
    %note CPOS CSPD and CACC can be changed on the fly during a motion
    calllib(libname,'setEyeAccel', axis_id, 10.0); % 1.0 is modest
    calllib(libname,'setEyeSpeed', axis_id, 10.0);
        
    %CPR relative positioning
    %calllib(libname,'sendMsgTypeAEye',axis_id, 22793, 3758030848,2,0);
    
    %CPA absolute positioning
    calllib(libname,'sendMsgTypeAEye',axis_id, 22793, 4294909952,2,0);

    %PP3 note speed loop is not closed in config therefore we cannot use
    %this
    %calllib(libname,'sendMsgTypeAEye',axis_id, 22793, 3217131265,2,0);
    
    %PP1 note only closing the position loop and not the speed loop gives
    %us more bandwidth
    calllib(libname,'sendMsgTypeAEye',axis_id, 22793, 3183576321,2,0);

    %TUM1, update trajectory from current trajectory pos and speed, must be
    %called before UPD but after setting mode (pp1 pp3 etc.)
    calllib(libname,'sendMsgTypeAEye',axis_id, 22793, 4294918144,2,0);
    
    %TUM0, update trajectory from current motor speed and pos
    %calllib(libname,'sendMsgTypeAEye',axis_id, 22793, 3221159936,2,0);
    
    eye_axis_on(axis_id);
end

function [] = config_neck_drive(axis_id)

    global libname;

    %STA command to avoid large motions when enabling the axis
    calllib(libname,'sendMsgTypeANeck',axis_id, 11442, 552,1,0);
    
    %set accel and speed low to gain more accuracy
    %not CPOS CSPD and CACC can be changed on the fly during a motion
    calllib(libname,'setNeckAccel', axis_id, 10);%radians per second squared
    calllib(libname,'setNeckSpeed', axis_id, 5);%radians per second
        
    %CPR
    %calllib(libname,'sendMsgTypeANeck',axis_id, 22793, 3758030848,2,0);
    
    %CPA
    calllib(libname,'sendMsgTypeANeck',axis_id, 22793, 4294909952,2,0);

    %PP3 note speed loop is not closed in config therefore we cannot use
    %this
    %calllib(libname,'sendMsgTypeANeck',axis_id, 22793, 3217131265,2,0);
    
    %PP1 note only closing the position loop and not the speed loop gives
    %us more bandwidth
    calllib(libname,'sendMsgTypeANeck',axis_id, 22793, 3183576321,2,0);

    %TUM1, update trajectory from current trajectory pos and speed, must be
    %called before UPD but after setting mode (pp1 pp3 etc.)
    calllib(libname,'sendMsgTypeANeck',axis_id, 22793, 4294918144,2,0);
    
    %TUM0, update trajectory from current motor speed and pos
    %calllib(libname,'sendMsgTypeANeck',axis_id, 22793, 3221159936,2,0);
end

function cleanup_all()
    global libname;
    
    calllib(libname,'disEyePollData');
    calllib(libname,'disNeckPollData');
    
    %wait to flush out current poll messages
    pause(0.5);
    
    for ii=1:4
        eye_axis_off(ii);
    end
    
    for ii=1:3
        neck_axis_stop(ii);
    end
    
    pause(0.5);
    
    clear;
    libname = 'macaque';
    calllib(libname,'cleanup');
    unloadlibrary(libname);
    disp('Clean up complete');
end

function [] = eye_axis_on(axis_id) 
    global libname;
    %reset the target position (tpos) to the current position(apos) using
    %STA command to avoid large motions when enabling the axis
    calllib(libname,'sendMsgTypeAEye',axis_id, 11442, 552,1,0);
    
    %send the axis on command
    calllib(libname,'sendMsgTypeAEye',axis_id, 258, 0, 0, 0);
end

function [] = eye_axis_off(axis_id) 
    global libname;
    calllib(libname,'sendMsgTypeAEye',axis_id, 2, 0, 0, 0);     
end

function [] = neck_axis_stop(axis_id)
    global libname;
    
    %STA set target to actual position
    calllib(libname,'sendMsgTypeANeck',axis_id, 11442, 552,1,0);
    
    %Set the Speed to zero so that CPOS doesn't cause TPOS to be updated
    %from APOS
    calllib(libname,'setNeckSpeed', axis_id, 0.0);%radians per second
    
    %Send the update message
    calllib(libname,'sendMsgTypeANeck',axis_id, 264, 0, 0, 0);
end

%% High Level Control Interface
function [] = motion(des_eye, des_neck, log_period_ms, total_time_ms)
    
    close all;

    global left;
    global right;
    global libname;
    global eye_cal;
    
    loop_time     = log_period_ms/1000;
    size          = total_time_ms/log_period_ms;
    
    %space the motion commands evenly over the total 
    if(~isempty(des_eye))
        eye_motion_period = ceil(size / (length( des_eye(1,:) ) ) );
        eye_motion_index  = 1;
    else
        eye_motion_period = 0;
    end
    
    %space the motion commands evenly over the total time
    if(~isempty(des_neck))
        neck_motion_period = ceil(size / (length( des_neck(1,:) ) ) );
        neck_motion_index  = 1;
    else
        neck_motion_period = 0;   
    end
    
    eye_angle  = zeros(size,4);
    neck_angle = zeros(size,3);
    eye_time   = zeros(size,1);
    neck_time  = zeros(size,1);
    
    calllib(libname,'enEyePollData');    
    calllib(libname,'enNeckPollData');

    eye_data  = calllib(libname,'getEyeData');
    neck_data = calllib(libname,'getNeckData');
    
    %give things a little bit to get fired up
    pause(0.1);

    correct_period = 10; %round(100 / log_period_ms)
    int_error = [0;0];
    for t=1:size
        
        if(eye_motion_period~= 0 && mod(t-1,eye_motion_period) == 0)
%             disp('*')
            if length(des_eye(:,1)) == 2 % des_eye is [yaw, pitch] in rad 
                move_eyes(des_eye(:, eye_motion_index));
            elseif length(des_eye(:,1)) == 3 % des_eye is [right, up, forward] in metres
                disp('pointing')
                point_eyes(des_eye(:, eye_motion_index))
            else
                error('Bad eye command')
            end
            eye_motion_index = eye_motion_index + 1;
        end
        
        if(neck_motion_period ~=0 && mod(t-1,neck_motion_period) == 0)
            move_neck(des_neck(:, neck_motion_index));
            neck_motion_index = neck_motion_index + 1;
        end
        
        eye_time(t)    = eye_data.Value.time;
        eye_angle(t,1) = eye_data.Value.yaw(left) - eye_cal(left).yaw_offset;
        eye_angle(t,2) = eye_data.Value.pitch(left) - eye_cal(left).pitch_offset;
        eye_angle(t,3) = eye_data.Value.yaw(right) - eye_cal(right).yaw_offset;
        eye_angle(t,4) = eye_data.Value.pitch(right) - eye_cal(right).pitch_offset;
        
        neck_time(t)   = neck_data.Value.time;
        neck_angle(t,1)= neck_data.Value.yaw;
        neck_angle(t,2)= neck_data.Value.pitch;
        neck_angle(t,3)= neck_data.Value.roll;
        
        pause(loop_time);
        
%         if ~isempty(des_eye) && mod(t, correct_period) == 0
%             target = [des_eye(:,eye_motion_index-1)];
%             yaw_error = mean(eye_angle(t,[1 3])) - des_eye(1,eye_motion_index-1);
%             pitch_error = mean(eye_angle(t,[2 4])) - des_eye(2,eye_motion_index-1);
% %             move_eyes(-.5*[yaw_error; pitch_error])
%             error = [yaw_error; pitch_error]
%             int_error = int_error + error
%             Kp = 0; Ki = 0;
%             move_eyes(target - Kp*error - Ki*int_error)
%         end
    end
    
    calllib(libname,'disEyePollData');    
    calllib(libname,'disNeckPollData');
    
    clear eye_data;
    clear neck_data;
    
    figure; hold on;
    title('neck angle vs time');
    plot(neck_time,neck_angle);
    legend('yaw','pitch','roll');
    
    figure; hold on;
    title('eye angle vs time');
    plot(eye_time,eye_angle);
    legend('yaw left','pitch left',' yaw right', 'pitch right');
    
    save('motion-data.mat', 'neck_time', 'neck_angle', 'eye_time', 'eye_angle')
end


function [] = move_neck(des_angle_deg)
    
    global libname;
    
    for ii=1:3       
        des = deg2rad(des_angle_deg(ii));
        %TODO limit here
        calllib(libname, 'setNeckPos', ii, des);
    end
    
    %Send the update message
    calllib(libname,'sendMsgTypeANeck',1, 264, 0, 0, 1);
end

function [] = point_eyes(des_point)
    % des_point: 3D point that both eyes should point to, in head coordinates 
    %   [right, up, forward] from robot's perspective, with origin centred
    %   between eye centres of rotation when eyes are pointed forward
    % 
    % Note the pitch centre of rotation is slightly aft of the yaw centre, which
    % is accounted for in get_angles(...)
    
    disp('pointing eyes')
    global left;
    global right;
    global libname;
    global eye_cal;
    
    if(isempty(fieldnames(eye_cal)))
        return;
    end
    
    % clip nearby points to prevent eyes from banging together 
    des_point(3) = max(des_point(3), .5);
    
    % For consistency with other code, left and right eyes are opposite of
    % anatomical left and right eyes. 
    % Positive pitch is up. Positive yaw is to the robot's left. 
    eye_separation = .058;
    des_point_left = des_point - [eye_separation/2; 0; 0]; 
    des_point_right = des_point + [eye_separation/2; 0; 0];
    
    [des_yaw_left, des_pitch_left] = get_angles(des_point_left);
    [des_yaw_right, des_pitch_right] = get_angles(des_point_right);
    fprintf('left: yaw %4.2f pitch %4.2f\n', des_yaw_left, des_pitch_left)
    fprintf('right: yaw %4.2f pitch %4.2f\n', des_yaw_right, des_pitch_right)
    
    for ii=1:2
        [r_act_ff,drdyaw,drdpitch]=inverse_kin_jac(des_yaw_right, des_pitch_right, eye_cal(ii).act(right));
        [l_act_ff,dldyaw,dldpitch]=inverse_kin_jac(des_yaw_left, des_pitch_left, eye_cal(ii).act(left));
        
        calllib(libname,'setEyePos', eye_cal(ii).act(right).axis_id, r_act_ff);
        calllib(libname,'setEyePos', eye_cal(ii).act(left).axis_id, l_act_ff);    
    end
    
    %Send the update message
    calllib(libname,'sendMsgTypeAEye',1, 264, 0, 0, 1);    
end

function do_move_eyes(yaw_left, pitch_left, yaw_right, pitch_right)
    % Converts yaw and pitch angle for each eye to actuator positions and
    % issues movement commands. 
    
    global left;
    global right;
    global libname;
    global eye_cal;

    fprintf('do_move_eyes: [%4.2f %4.2f %4.2f %4.2f]\n', yaw_left, pitch_left, yaw_right, pitch_right)
    
    if(isempty(fieldnames(eye_cal)))
        return;
    end
    
    for ii=1:2
        [r_act_ff,drdyaw,drdpitch]=inverse_kin_jac(yaw_right, pitch_right, eye_cal(ii).act(right));
        [l_act_ff,dldyaw,dldpitch]=inverse_kin_jac(yaw_left, pitch_left, eye_cal(ii).act(left));
        
        calllib(libname,'setEyePos', eye_cal(ii).act(right).axis_id, r_act_ff);
        calllib(libname,'setEyePos', eye_cal(ii).act(left).axis_id, l_act_ff);    
    end
    
    %Send the update message
    calllib(libname,'sendMsgTypeAEye',1, 264, 0, 0, 1);    
end

function [yaw_left, pitch_left, yaw_right, pitch_right] = get_eye_angles(point)
    % Get yaw and pitch of both eyes for orientation to a 3D point. 
    % point: [right, up, forward] in head coords, relative to point between
    %   eye rotation centres
    
    % clip nearby points to prevent eyes from banging together 
    point(3) = max(point(3), .5);
    
    % For consistency with other code, left and right eyes are opposite of
    % anatomical left and right eyes. 
    % Positive pitch is up. Positive yaw is to the robot's left. 
    eye_separation = .058;
    point_left = point - [eye_separation/2; 0; 0]; 
    point_right = point + [eye_separation/2; 0; 0];
    
    [yaw_left, pitch_left] = get_angles(point_left);
    [yaw_right, pitch_right] = get_angles(point_right);
%     fprintf('left: yaw %4.2f pitch %4.2f\n', yaw_left, pitch_left)
%     fprintf('right: yaw %4.2f pitch %4.2f\n', yaw_right, pitch_right)
end

function [yaw, pitch] = get_angles(point)
    % Get yaw and pitch of one eye oriented at point relative to eye's
    % centre of rotation. 
    
    pitch_centre_offset = -.002; %TODO: this is a guess
    yaw = -atan(point(1) / point(3));
    horizontal_distance = (point(1)^2 + point(3)^2)^.5 - pitch_centre_offset;
    pitch = atan(point(2) / horizontal_distance);
    
    max_angle = deg2rad(30);
    yaw = max(-max_angle, min(max_angle, yaw));
    pitch = max(-max_angle, min(max_angle, pitch));
end

function [] = saccade(des_eye, des_neck, saccade_onsets_ms, log_period_ms)
    % Perform a saccade sequence.
    % des_eye: list of 3D targets to which eyes should point (3 x #saccades;
    %   target coords are [right up forward] from point between eye centres 
    %   of rotation in head-fixed coords)
    % des_neck: list of [yaw pitch roll] (3 x #saccades)
    % saccade_onsets_ms: times when saccades start (1 x #saccades) 
    % log_period_ms: approx period of data storage [TODO: control period?]
    
    close all;

    global left;
    global right;
    global libname;
    global eye_cal;
    
    loop_time     = log_period_ms/1000;

    assert(size(des_eye, 2) == length(saccade_onsets_ms));
    assert(size(des_neck, 2) == length(saccade_onsets_ms));
    
    T = saccade_onsets_ms(end) + 1000; % enough time to finish last saccade
    steps = T / log_period_ms;
    
    eye_angle  = zeros(steps,4);
    neck_angle = zeros(steps,3);
    time   = zeros(steps,1); % in seconds?
    
    calllib(libname,'enEyePollData');    
    calllib(libname,'enNeckPollData');

    eye_data  = calllib(libname,'getEyeData');
    neck_data = calllib(libname,'getNeckData');
    
    %give things a little bit to get fired up
    pause(0.1);

    last_saccade_ind = -1;
    for k=1:steps
        % we need some of these for feedback control, and we'll save them
        time(k) = eye_data.Value.time;
        eye_angle(k,1) = eye_data.Value.yaw(left) - eye_cal(left).yaw_offset;
        eye_angle(k,2) = eye_data.Value.pitch(left) - eye_cal(left).pitch_offset;
        eye_angle(k,3) = eye_data.Value.yaw(right) - eye_cal(right).yaw_offset;
        eye_angle(k,4) = eye_data.Value.pitch(right) - eye_cal(right).pitch_offset;
%         eye_angle(k,1) = eye_data.Value.yaw(left);
%         eye_angle(k,2) = eye_data.Value.pitch(left);
%         eye_angle(k,3) = eye_data.Value.yaw(right);
%         eye_angle(k,4) = eye_data.Value.pitch(right);
        
        neck_angle(k,1)= neck_data.Value.yaw;
        neck_angle(k,2)= neck_data.Value.pitch;
        neck_angle(k,3)= neck_data.Value.roll;
        
        current_time_ms = (time(k) - time(1)) * 1000;
        saccade_ind = find(saccade_onsets_ms <= current_time_ms, 1, 'last');
 
        % for each new saccade we issue a single command to move neck
        if saccade_ind > last_saccade_ind
            neck_target = des_neck(:,saccade_ind);
            move_neck(neck_target);
        end
        
        % for the eyes we need continuous feedback control because the
        % board-level controllers are for actuator positions rather than
        % angles
        eye_target = des_eye(:,saccade_ind);
        [yaw_left, pitch_left, yaw_right, pitch_right] = get_eye_angles(eye_target);        
        
        Kp = .0; Ki = 0; % PI control gains        
        target = [yaw_left; pitch_left; yaw_right; pitch_right];
        
        if saccade_ind > last_saccade_ind %reset error integration with each saccade 
            integrated_error = zeros(4,1);
        end
        error = eye_angle(k,:)' - target;
        integrated_error = integrated_error + error * loop_time;
        
        command = target - Kp*error - Ki*integrated_error;        
        do_move_eyes(command(1), command(2), command(3), command(4));
        
        last_saccade_ind = saccade_ind;
                        
        pause(loop_time);        
    end
    
    calllib(libname,'disEyePollData');    
    calllib(libname,'disNeckPollData');
    
    clear eye_data;
    clear neck_data;
    
    figure; hold on;
    title('neck angle vs time');
    plot(time,neck_angle);
    legend('yaw','pitch','roll');
    
    figure; hold on;
    title('eye angle vs time');
    plot(time,eye_angle);
    legend('yaw left','pitch left',' yaw right', 'pitch right');
    
    save('motion-data.mat', 'time', 'neck_angle', 'eye_angle')
end

