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
            disp('circle');
            disp('      -> this moves the eye gimbals in a circle');
            disp('eye y:[yaw angle degrees] p:[pitch angle degrees]');
            disp('      -> this moves the eye gimbals to specified angles');           
            disp('neck y:[yaw angle degrees] p:[pitch angle degrees] r:[roll angle degrees]');
            disp('      -> this moves the eye gimbals to specified angles'); 
            disp('exit  -> this exits you from the program stopping the eye and neck gimbals');
        elseif strcmp(action,'circle') == 1
            t = 1:100;
            des_yaw = 20*cos(2*pi*t/50);
            des_pitch = 20*sin(2*pi*t/50);
            des_neck = zeros(3,length(t));
            des_neck(1,:) = 20*sin(2*pi*t/100);
            des_neck(2,:) = 20*sin(2*pi*t/50);
            des_neck(3,:) = 10*sin(2*pi*t/50);
            des_eye = [des_yaw; des_pitch];      
            motion(des_eye, des_neck, 5, 2500);

        elseif strncmpi(action, 'eye', length('eye')) == 1
            [des_angle_deg, num] = sscanf(action,'eye y:%f p:%f');
            if(num ~= 2)
                disp('Bad input');
                continue;
            end               
            motion(des_angle_deg, [], 2, 500);

        elseif strncmpi(action, 'neck', length('neck')) == 1
            [des_angle_deg, num] = sscanf(action,'neck y:%f p:%f r:%f');
            if(num ~= 3)
                disp('Bad input');
                continue;
            end
            motion([], des_angle_deg, 2, 500);

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
    calllib(libname,'setNeckAccel', axis_id, 1.5);%radians per second squared
    calllib(libname,'setNeckSpeed', axis_id, 1.5);%radians per second
        
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
    pause(0.6);

    for t=1:size
        
        if(eye_motion_period~= 0 && mod(t-1,eye_motion_period) == 0)
            move_eyes(des_eye(:, eye_motion_index));
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
end

function [] = move_eyes(des_angle_deg)
    global left;
    global right;
    global libname;
    global eye_cal;
    
    if(isempty(fieldnames(eye_cal)))
        return;
    end
    
    des_yaw = deg2rad(des_angle_deg(1));
    des_pitch = deg2rad(des_angle_deg(2));
    
    for ii=1:2
        [r_act_ff,drdyaw,drdpitch]=inverse_kin_jac(des_yaw, des_pitch, eye_cal(ii).act(right));
        [l_act_ff,dldyaw,dldpitch]=inverse_kin_jac(des_yaw, des_pitch, eye_cal(ii).act(left));
        
        %send new linear actuator positions
        calllib(libname,'setEyePos', eye_cal(ii).act(right).axis_id, r_act_ff);
        calllib(libname,'setEyePos', eye_cal(ii).act(left).axis_id, l_act_ff);    
    end
    
    %Send the update message
    calllib(libname,'sendMsgTypeAEye',1, 264, 0, 0, 1);
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