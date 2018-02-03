function [] = calibration()

%close all sockets that may have been left open
pnet('closeall');

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
psi=degtorad(17.3);
dir = [ 0; sin(psi); cos(psi)];

%link length joining the actuator end to the camera spherical joint
link = 0.0604;

%% Structure Init

act = struct('axis_id',{1,2,3,4},'cam',cam,'neut',neut,'dir',dir,'link',link);

eye = struct('act',{[],[]},'yaw',0,'pitch',0);
eye(1).act = [act(left), act(right)];
eye(2).act = [act(left+2), act(right+2)];
eye(left).act(left).cam(1) = -eye(left).act(left).cam(1);
eye(left).act(left).neut(1) = -eye(left).act(left).neut(1);
eye(right).act(left).cam(1) = -eye(right).act(left).cam(1);
eye(right).act(left).neut(1) = -eye(right).act(left).neut(1);

%% Connect to Drives

eyes_ip = '192.168.2.14';
eyes_axis_id = 120;

neck_ip = '192.168.2.15';
neck_axis_id = 121;

%connect to system, quit if fails
eye_axis_sock = connect(eyes_ip);

if eye_axis_sock == -1
    return;
end

%% Configure Drives
   
%reset the target position (tpos) to the current position(apos)
sta_msg = build_msg_typea_group(1,11442,552);
send_msg_typea(eye_axis_sock, eyes_ip, sta_msg);

%set the position control parameters, enable the control loops
arrayfun(@(axis_id) config_drive(eye_axis_sock, eyes_ip, axis_id), [act.axis_id]);

%% Calibrate
eye = calibrate_task(eye_axis_sock, eyes_ip, eyes_axis_id, eye, 200);
filename = 'calib.mat';
save(filename, 'eye');

cleanup(eye_axis_sock, eyes_ip);

end

function [] = config_drive(sock, ip, axis_id)

    apos_set_axis_pos = build_msg_typea(axis_id,9374,[0,0]);
    %print_msg(apos_set_axis_pos);

    %apos_set_axis_cpr = build_msg_typea(axis_id,22793,[57343,0]);
    %print_msg(apos_set_axis_1_cpr);
    
    apos_set_axis_cpa = build_msg_typea(axis_id,22793,[65535,8192]);
    %print_msg(apos_set_axis_1_cpr);

    apos_set_axis_modepp3 = build_msg_typea(axis_id,22793,[49089,34561]);
    %print_msg(apos_set_axis_1_modepp)

    apos_set_axis_tum1 = build_msg_typea(axis_id,22793,[65535,16384]);
    %print_msg(apos_set_axis_1_tum1)
    
    %apos_set_axis_tum0 = build_msg_typea(axis_id,22793,[49151,0]);
    %print_msg(apos_set_axis_tum0)

    axis_on(sock, ip, axis_id);

    send_msg_typea(sock, ip, apos_set_axis_pos);
    send_msg_typea(sock, ip, apos_set_axis_cpa);
    send_msg_typea(sock, ip, apos_set_axis_modepp3);
    send_msg_typea(sock, ip, apos_set_axis_tum1);

end

function [] = cleanup(sock, ip)
    stop_all(sock, ip);
    axis_off(sock, ip, 1);
    axis_off(sock, ip, 2);
    axis_off(sock, ip, 3);
    axis_off(sock, ip, 4);
    disconnect(sock);
end

function [] = udp_write(sock,ip,port,data)
    pnet(sock, 'write', uint8(data));
    pnet(sock, 'writepacket', ip, port);
end

function [result] = udp_read_block(sock)
    size=pnet(sock,'readpacket');
    result=pnet(sock,'read', size,'uint8');
end

function [result] = connect(ip)
    %input ip address
    %output, socket bound to local host response port
    conn_port = 30689;
    conn = pnet('udpsocket',51243);
    pnet(conn,'setwritetimeout',1);
    pnet(conn,'setreadtimeout',1);

    udp_write(conn, ip, conn_port, 1);
    result = udp_read_block(conn);
    
    if result == 2
        disp('********Connected**********');
    else
        disp('********Connection Failed**********');
        pnet(conn,'close');
        result = -1;
        return;
    end
    
    udp_write(conn, ip, conn_port, [3,0,194,1,0]);
    result = udp_read_block(conn);
    
    if result == 4
        disp('********Baud Rate Set to 1Mbps**********');
    else
        disp('********Setting Baud Rate Failed**********');
        pnet(conn,'close');
        result = -1;
        return;
    end
        
    result = conn;
    
    if sync(result,ip) == -1
        result = -1;
    end
    
end

function [result] = disconnect(sock, ip)
    udp_write(sock,ip,30689,5);
    result = udp_read_block(sock);
    if result == 6
        disp('********Disconnected**********');
    else
        disp('********Disconnection Failed**********');
    end
    
    pnet(sock,'close');
end

function [result] = sync(sock, ip)
    sync = 255*ones(15,1);
    udp_write(sock, ip, 1700, sync);
    result = udp_read_block(sock);
    %disp(result);
    if(length(result)~=15)
        disp('********Sync Failed**********');
        result=-1;
        return   
    end
    
    for i=1:15
        if result(i)~=13
            disp('********Sync Failed**********');
            result=-1;
            return
        end
    end
    disp('********Sync Success**********');
    result = 0;
end

function [] = axis_on(sock,ip,dest_addr) 
    data = int16.empty;    
    msg = build_msg_typea(dest_addr,258,data);
    udp_write(sock,ip,1700,msg);
end

function [] = axis_off(sock,ip,dest_addr) 
    data = int16.empty;      
    msg = build_msg_typea(dest_addr,2,data);
    udp_write(sock,ip,1700,msg);
end

function [] = set_position(sock, ip, axis, pos)
    %pos is position in m
    pos = convert_m_to_iu(pos);
    pos = typecast(int32(pos),'uint16');
    msg = build_msg_typea(axis,9374,pos);
    send_msg_typea(sock, ip, msg);
end

function [] = stop3(sock, ip, dest_addr)
    data = int16.empty;    
    msg = build_msg_typea(dest_addr,452,data);
    udp_write(sock, ip, 1700, msg);
end

function [] = stop_all(sock,ip)
    data = int16.empty;    
    msg = build_msg_typea_group(1,452,data);
    udp_write(sock,ip,1700,msg);
end

function [msg] = build_msg_typea(dest_addr, optcode, data)  
%dest_addr is the destiantion axis id
%optcode is the operaton code to send
%data is an 1D arry of type uint16 numbers

    dest_addr_formatted = build_addr(dest_addr,0,0);
       
    if(isempty(data))
        msg=zeros(6,1);
    else
        data = get_2byte_stream(data);
        msg=zeros(6+length(data),1);
        msg(6:6+length(data)-1) = data;
    end
        
    msg(1)= length(msg)-2;    
    msg(2:3) = get_2byte_stream(dest_addr_formatted);
    msg(4:5) = get_2byte_stream(optcode);
    msg(length(msg)) = gen_checksum(msg);
end

function [msg] = build_msg_typea_group(dest_addr, optcode, data)  
%dest_addr is the destiantion axis id
%optcode is the operaton code to send
%data is an 1D arry of type uint16 numbers

    dest_addr_formatted = build_addr(dest_addr,0,1);
       
    if(isempty(data))
        msg=zeros(6,1);
    else
        data = get_2byte_stream(data);
        msg=zeros(6+length(data),1);
        msg(6:6+length(data)-1) = data;
    end
        
    msg(1)= length(msg)-2;    
    msg(2:3) = get_2byte_stream(dest_addr_formatted);
    msg(4:5) = get_2byte_stream(optcode);
    msg(length(msg)) = gen_checksum(msg);
end

function [msg] = build_msg_typeb(dest_addr, send_addr, var_addr, words)
    if words == 1
        optcode=45060;
    elseif words == 2
        optcode=45061;
    else
        return;
    end
    
    dest_addr_formatted = build_addr(dest_addr,0,0);
    send_addr_formatted = build_addr(send_addr,1,0);
    
    msg=zeros(10,1);
    msg(1)= length(msg)-2;    
    msg(2:3) = get_2byte_stream(dest_addr_formatted);
    msg(4:5) = get_2byte_stream(optcode);
    msg(6:7) = get_2byte_stream(send_addr_formatted);
    msg(8:9) = get_2byte_stream(var_addr);
    msg(10)  = gen_checksum(msg);
end

function [result] = send_msg_typea(sock, ip, msg)
    udp_write(sock, ip, 1700, msg);
    result = get_ack(sock, ip);
end

function [result] = send_msg_typeb(sock, ip, msg, words)
    udp_write(sock, ip, 1700, msg);
   
    response = udp_read_block(sock);
    while(response == 79)
        response = udp_read_block(sock);
    end
    result = parse_response_msg(response);
end

function [msg_cksum] = gen_checksum(msg)
    msg_cksum = bitand(sum(msg(1:length(msg)-1)),255);
end

function [addr] = build_addr(axis_id, is_host, is_group)
    addr = bitshift(bitand(axis_id,255), 4);
    addr = bitor(addr,bitshift(bitand(is_group,1),12));
    addr = bitor(addr,bitand(is_host,1));
end

function [addr] = get_addr(b)
    addr = swapbytes(typecast(uint8(b),'uint16'));
    addr = bitand(bitshift(addr,-4),255);
end

function [bytes] = get_2byte_stream(val)
    bytes = typecast(swapbytes(uint16(val)),'uint8');
end

function [result] = get_16bit_uint(b)
    result = swapbytes(typecast(uint8(b), 'uint16'));
end

function [result] = get_16bit_int(b)
    result = swapbytes(typecast(uint8(b), 'int16'));
end

function [result] = get_32bit_int(b)
    result = typecast(uint8([b(2),b(1),b(4),b(3)]),'int32');
end

function [result] = get_ack(sock, ip)
    result = udp_read_block(sock);
    if(result==79)
       result = 0;
       return;
    end
    
    disp('Failed to recieve Ack on MSG send,trying resync');
    sync(sock, ip);
end

function [result,axis] = parse_response_msg(msg)
    result = -1;
    axis = -1;
  
    if(length(msg) < 12)
        disp('Malformed Packet')
        return;
    end
    
    size = msg(1);
    dst_addr = get_addr(msg(2:3));
    optcode = get_16bit_uint(msg(4:5));
    axis = get_addr(msg(6:7));
    value_addr = get_16bit_uint(msg(8:9));
    
    if( msg(length(msg)) ~= gen_checksum(msg))
        disp('CKSUM failed')
        result = -1;
        return
    end
    
    if(optcode == 46084)
        result = get_16bit_int(msg(10:11));
    elseif(optcode == 46085)
        result = get_32bit_int(msg(10:13));
    end
    
    %disp(sprintf('msg size:%d\tdest axis:%d\tsrc axis:%d\topt code:%x\tvalue addr:%x\tvalue:%d',size,dst_addr,axis,optcode,value_addr,result)); 

end

function [pos] = convert_iu_to_m(iu)
    cpr = 2048;
    pole_pitch = 0.018;%in meters
    pos = pole_pitch*double(iu)/cpr;
end

function [rad] = convert_iu_to_rad(iu)
    cpr = 5000;
    rad = 2*pi*double(iu)/(4*cpr);
end

function [iu] = convert_m_to_iu(m)
    cpr = 2048;
    pole_pitch = 0.018;%in meters
    iu = int32(cpr*m/pole_pitch);
end

function [eye] = calibrate_task(sock, ip, host_id, eye, max_samples)

    global left;
    global right;
    global pitch_idx;
    global yaw_idx;
    
    global sample_count;%number of samples obtained
    global cal_data;
    global act_l;
    global act_r;
    
    open_loop_inc = 0.002;%m
    state=0;
    
    act_state = struct ('max',{0,0,0,0}, 'min',0, 'pos_open_loop',0,'dir',-1);
    
    sample_msg = struct ('get_angle', {[],[],[],[]}, 'get_pos', [], 'get_err', []);
    
    cal = struct(   'angle',{zeros(max_samples,2),zeros(max_samples,2)}, ...
                    'pos',zeros(max_samples,2), ...
                    'err',zeros(max_samples,2));
                
    %build sampling messages
    for i=0:1
        for n=1:2
            sample_msg(i*2+n).get_angle = build_msg_typeb(eye(i+1).act(1+mod(n+i,2)).axis_id, host_id, 2076, 2);
            sample_msg(i*2+n).get_pos   = build_msg_typeb(eye(i+1).act(n).axis_id, host_id, 552, 2);
            sample_msg(i*2+n).get_err   = build_msg_typeb(eye(i+1).act(n).axis_id, host_id, 554, 1);
        end
    end
    
    %set accel low to gain more accuracy,but not so low that we are moving
    %at the next sample
    accel = typecast(uint32(1000),'uint16');
    accel_msg = build_msg_typea_group(1,9378,accel);
    send_msg_typea(sock, ip, accel_msg);
    
    update_msg = build_msg_typea_group(1,264,int16.empty);
    
    %auto calibrate routine
    for sample_count=1:max_samples
        
        %sample axes and check limits and update states
        for i=0:1
            
            for n=1:2
                axis_idx = i*2 + n;
                [ cal(i+1).angle(sample_count, n), ...
                  cal(i+1).pos(sample_count, n), ...
                  cal(i+1).err(sample_count, n)] = ... 
                  sample_data(sock, ip, sample_msg(axis_idx));
                [ act_state(axis_idx)] = check_limit(act_state(axis_idx), ... 
                  cal(i+1).pos(sample_count, n), ... 
                  cal(i+1).err(sample_count, n));
            end
            
            r_axis = i*2 + right;
            l_axis = i*2 + left;
                         
            %send new linear actuator positions
            act_state(r_axis).pos_open_loop = act_state(r_axis).pos_open_loop + ...
                                               open_loop_inc * act_state(r_axis).dir;
            act_state(l_axis).pos_open_loop = act_state(l_axis).pos_open_loop + ...
                                               open_loop_inc*act_state(l_axis).dir;

            set_position(sock, ip, l_axis,  act_state(l_axis).pos_open_loop);
            set_position(sock, ip, r_axis, act_state(r_axis).pos_open_loop);    
        end
        
        %update state
        [act_state, state] = update_state(act_state,state);
        
        %send update message
        send_msg_typea(sock, ip, update_msg);
        
        if(state == 5)
            break;
        end
        
        pause(0.2);
                      
    end
    
    % optimization
    for i=1:2

        for n=1:2
            if(i==2 && n ==1)
                dir = -1;
            else
                dir = 1;
            end
            %convert logged data to m and rad
            cal(i).angle(:,n) = dir*convert_iu_to_rad(cal(i).angle(:,n));
            cal(i).pos(:,n)   = convert_iu_to_m(cal(i).pos(:,n));
        end
        
        cal_data = cal(i);
        act_l = eye(i).act(left);
        act_r = eye(i).act(right);
        
        figure; hold on;
        plot(cal(i).angle(:,pitch_idx),'r');
        plot(cal(i).angle(:,yaw_idx),'b');
        
        %calibrate the gimbal geometry
        figure;hold on;
        plot(cal(i).pos(:,right),'b');
        plot(cal(i).pos(:,left),'b');

        global act_meas;
        
        act_meas = cal(i).pos(:,right);
        guess = [eye(i).act(right).cam;eye(i).act(right).neut;eye(i).act(right).dir;eye(i).act(right).link];
        [results] = fminsearch(@min_fxn_geometry, guess);
        [eye(i).act(right)] = parse_calib_result(results, eye(i).act(right));

        act_meas = cal(i).pos(:,left);
        guess = [eye(i).act(left).cam;eye(i).act(left).neut;eye(i).act(left).dir;eye(i).act(left).link];
        [results] = fminsearch(@min_fxn_geometry, guess);
        [eye(i).act(left)] = parse_calib_result(results, eye(i).act(left));
    end
    
end

function [angle,pos,err] = sample_data(sock, ip, msg)
    angle = send_msg_typeb(sock, ip, msg.get_angle, 2);
    pos   = send_msg_typeb(sock, ip, msg.get_pos, 2);
    err   = typecast(send_msg_typeb(sock, ip, msg.get_err, 2),'int16');
end

function [act_state] = check_limit(act_state, pos, err)
    if(abs(err) > 114)%1 mm
        pos = convert_iu_to_m(pos);
        err = convert_iu_to_m(err);
        
        if(act_state.dir < 0 && act_state.max == 0)
            act_state.max = pos;
        elseif(act_state.dir > 0 && act_state.min == 0)
            act_state.min = pos;
        end
        
        act_state.dir = 0;
        act_state.pos_open_loop = pos-err;
    end
end

function [act_state,state] = update_state(act_state,state)

    if(state == 2)
        for i=1:4
            if(act_state(i).pos_open_loop < (act_state(i).max + act_state(i).min)/2)
                act_state(i).dir = 0;
            end
        end
    end
        
    if (act_state(1).dir == 0 && act_state(2).dir == 0 ...
            && act_state(3).dir == 0 && act_state(4).dir == 0)
        if(state==0)
            %at the top, go down
            act_state(1).dir = 1;
            act_state(2).dir  = 1;
            act_state(3).dir  = 1;
            act_state(4).dir  = 1;
            state=1;
        elseif(state==1)
            %both reached the bottom, go halfway
            act_state(1).dir = -1;
            act_state(2).dir  = -1;
            act_state(3).dir  = -1;
            act_state(4).dir  = -1;
            state=2;
        elseif(state==2)
            %reached the middle
            act_state(1).dir = -1;
            act_state(2).dir  = 1;
            act_state(3).dir  = -1;
            act_state(4).dir  = 1;
            state=3;
        elseif(state==3)
            %reached the left
            act_state(1).dir = 1;
            act_state(2).dir  = -1;
            act_state(3).dir  = 1;
            act_state(4).dir  = -1;
            state=4;
        else
            %done
            state=5;
        end
    end
end

function [act] = parse_calib_result(result, act)
    global sample_count;  
    global cal_data;
    global yaw_idx;
    global pitch_idx;
    
    act_cal = zeros(sample_count,1);
    act = struct('axis_id',act.axis_id,'cam',result(1:3),'neut',result(4:6), ...
                 'dir',result(7:9),'link',result(10));

    for i=1:sample_count
        act_cal(i) = inverse_kin_jac(cal_data.angle(i,yaw_idx), ...
                                     cal_data.angle(i,pitch_idx), ...
                                     act);
    end
    plot(act_cal,'r');
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