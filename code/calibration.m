function [] = calibration()

global libname;

libname = 'macaque';

if not(libisloaded(libname))
    loadlibrary(libname, 'macaque.h');
    calllib(libname,'start');
end

close all;

global left;
global right;
global yaw_idx;
global pitch_idx;
left = 1;
right = 2;
pitch_idx = 1;
yaw_idx = 2;

%% Defaults From Physical Geometry

% sphereical joint centers on camera in neutral pos
cam = [0.01425, 0.020, .020]';

%neutral position of end of actuators (i.e. when commanded to 0)
neut = [0.0145; 0.03518; -.03839];

% angle actuator makes with z axis
psi = degtorad(17.3);
dir = [ 0; sin(psi); cos(psi)];

%link length joining the actuator end to the camera spherical joint
link = 0.0604;

%% Structure Init

act = struct('axis_id',{1,2,3,4},'cam',cam,'neut',neut,'dir',dir,'link',link);

eye = struct('act',{[],[]},'yaw',0,'pitch',0,'yaw_offset',0,'pitch_offset',0);
eye(left).act = [act(left), act(right)];
eye(right).act = [act(left+2), act(right+2)];
eye(left).act(left).cam(1) = -eye(left).act(left).cam(1);
eye(left).act(left).neut(1) = -eye(left).act(left).neut(1);
eye(right).act(left).cam(1) = -eye(right).act(left).cam(1);
eye(right).act(left).neut(1) = -eye(right).act(left).neut(1);

%% Configure Drives
   
%set the position control parameters, enable the control loops
arrayfun(@(axis_id) config_drive(axis_id), [act.axis_id]);

%% Calibrate
eye = calibrate_task(eye, 10000);
filename = 'calib.mat';
save(filename, 'eye');

cleanup();

end

function [] = config_drive(axis_id)

    global libname;
    
    axis_on(axis_id);
    
end

function [] = cleanup()

    global libname;
    
    calllib(libname,'disEyePollData');
    
    %wait to flush out current poll messages
    pause(0.5);
    
    %stop all does nothing unless we configure the drives with a speed
    %control loop
    %stop_all();
    
    for i=1:4
        axis_off(i);     
    end
    
    clear;
    libname = 'macaque';
    calllib(libname,'cleanup');
    unloadlibrary(libname);
end

function [] = axis_on(axis_id) 
    global libname;
    %reset the target position (tpos) to the current position(apos) using
    %STA command to avoid large motions when enabling the axis
    calllib(libname,'sendMsgTypeAEye',axis_id, 11442, 552,1,0);
    
    %send the axis on command
    calllib(libname,'sendMsgTypeAEye',axis_id, 258, 0, 0, 0);
end

function [] = axis_off(axis_id) 
    global libname;
    calllib(libname,'sendMsgTypeAEye',axis_id, 2, 0, 0, 0);     
end

function [eye] = calibrate_task(eye, max_samples)
    
    global libname;  
    global left;
    global right;
    global pitch_idx;
    global yaw_idx;
    
    %pointers to the most recent data
    cal_angles   = calllib(libname,'getEyeData');
    cal_pos      = calllib(libname,'getEyeCalData');
        
    cal = struct(   'angle',{zeros(max_samples,2),zeros(max_samples,2)}, ...
                    'pos',zeros(max_samples,2));
    sample_counts = [1,1];
    
    global state;
    state=0;
    [state] = update_state(state, cal_pos, cal);
  
    %auto calibrate routine to get the calibration data
    while sample_counts(left) < max_samples && sample_counts(right) < max_samples 
        
        %sample axes
        for i=0:1  
            eye_idx = i+1;
            r_axis  = i*2 + right;
            l_axis  = i*2 + left;
            
            %only grab the data if we are still in active calibrate mode
            if cal_pos.Value.complete(i*2+1) == 1 && cal_pos.Value.complete(i*2+2) == 1
                cal(eye_idx).angle(sample_counts(eye_idx),pitch_idx) = cal_angles.Value.pitch(eye_idx);
                cal(eye_idx).angle(sample_counts(eye_idx),yaw_idx) = cal_angles.Value.yaw(eye_idx);
                cal(eye_idx).pos(sample_counts(eye_idx), left)   = cal_pos.Value.pos(l_axis);
                cal(eye_idx).pos(sample_counts(eye_idx), right) = cal_pos.Value.pos(r_axis); 
                sample_counts(eye_idx) = sample_counts(eye_idx)+1;
            end
        end
        
        %update state
        [state] = update_state(state, cal_pos, cal);
        
        if(state == 6)
            break;
        end
        
        pause(0.1);        
    end
        
    disp('Data Collection Complete');
    %cleanup the library
    for i=1:4
        axis_off(i);     
    end
        
    %data used by optimize function
    global sample_count;
    global cal_data;
    global act_meas;
    
    % optimization
    for i=1:2

        cal_data = cal(i);
        sample_count = sample_counts(i);
                
        %adjust for the index
        cal_data.angle(:,pitch_idx) = cal_data.angle(:,pitch_idx) - cal_angles.Value.pitch_offset(i);
        cal_data.angle(:,yaw_idx)   = cal_data.angle(:,yaw_idx) - cal_angles.Value.yaw_offset(i);
        
        figure; hold on;
        plot(cal(i).angle(1:sample_counts(i),pitch_idx),'r');
        plot(cal(i).angle(1:sample_counts(i),yaw_idx),'b');
        title('Angle');
        legend('pitch', 'yaw');
        
        %calibrate the gimbal geometry
        figure;hold on;
        plot(cal(i).pos(1:sample_counts(i),right),'b');
        plot(cal(i).pos(1:sample_counts(i),left),'r');
        title('Position');
        legend('pos_r', 'pos_l');
        
        act_meas = cal(i).pos(:,right);
        guess = [eye(i).act(right).cam;eye(i).act(right).neut;eye(i).act(right).dir;eye(i).act(right).link];
        [results] = fminsearch(@min_fxn_geometry, guess);
        [eye(i).act(right)] = parse_calib_result(results, eye(i).act(right));

        act_meas = cal(i).pos(:,left);
        guess = [eye(i).act(left).cam;eye(i).act(left).neut;eye(i).act(left).dir;eye(i).act(left).link];
        [results] = fminsearch(@min_fxn_geometry, guess);
        [eye(i).act(left)] = parse_calib_result(results, eye(i).act(left));
        
        eye(i).yaw_offset = cal_angles.Value.yaw_offset(i);
        eye(i).pitch_offset = cal_angles.Value.pitch_offset(i);
    end
    
end

function [state] = update_state(state, cal_state, cal_data)
       
    global libname; 

    if (cal_state.Value.complete(1) == 0 && cal_state.Value.complete(2) == 0 ...
            && cal_state.Value.complete(3) == 0 && cal_state.Value.complete(4) == 0)
        pause(1);
        
        if(state==0)
            %start moving up
            for i=1:4
                calllib(libname,'startEyeCal',i,-0.06);
            end
            state=1;
            
        elseif(state==1)
            %at the top, go down
            for i=1:4
                calllib(libname,'startEyeCal',i,0.06);
            end
            state=2;
            
        elseif(state==2)
            %both reached the bottom, go halfway
            for n=0:1
                for i=1:2
                    mid = ( max(cal_data(n+1).pos(:,i)) - min(cal_data(n+1).pos(:,i)) ) / 2;
                    calllib(libname,'startEyeCal',n*2+i,-mid); 
                end
            end
            calllib(libname,'sendMsgTypeAEye',1, 264, 0, 0, 1);
            state=3;
            
        elseif(state==3)
            %reached the middle
            for i=1:4
                calllib(libname,'startEyeCal',i,(2*mod(i,2)-1)*0.06);
            end
            state=4;
            
        elseif(state==4)
            %reached the left
            for i=1:4
                calllib(libname,'startEyeCal',i,(1-2*mod(i,2))*0.06);
                if(i==2)
                    %give the left eye an extra second to prevent it from hanging on the right eye yoke
                    pause(1.5);
                end
            end
            state=5;
        else
            %done
            state=6;
        end
        
        disp(sprintf('state change to state:%d\n',state));
        
    end
end

function [act] = parse_calib_result(result, act)
    global sample_count;  
    global cal_data;
    global yaw_idx;
    global pitch_idx;
    global act_meas;
    
    act_cal = zeros(sample_count,1);
    act = struct('axis_id',act.axis_id,'cam',result(1:3),'neut',result(4:6), ...
                 'dir',result(7:9),'link',result(10));

    for i=1:sample_count
        act_cal(i) = inverse_kin_jac(cal_data.angle(i,yaw_idx), ...
                                     cal_data.angle(i,pitch_idx), ...
                                     act);
    end
    
    figure;hold on;
    plot(act_cal(1:sample_count),'r');
    plot(act_meas(1:sample_count),'b');
    legend('calibrated pos','measured pos');
            
end

function f = min_fxn_geometry(x)
    global yaw_idx;
    global pitch_idx;
    global cal_data;
    global act_meas;
    global sample_count;
    
    act.cam = x(1:3);
    act.neut = x(4:6);
    act.dir = x(7:9);
    act.link = x(10);
    
    err=0;
    for i=1:sample_count
        [act_est] = inverse_kin_jac(cal_data.angle(i,yaw_idx), ...
                                    cal_data.angle(i,pitch_idx), ...
                                    act);
        err = err + (act_est - act_meas(i))^2;
    end
    
    f=err;
end 