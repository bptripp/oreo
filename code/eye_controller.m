function [] = eye_controller()

    %close all sockets that may have been left open
    pnet('closeall');

    %% Globals

    global left;
    global right;

    left = 1;
    right = 2;

    %% Calibrate
    recal = input('do you wish to recalibrate?','s');
    eye = struct();
    
    filename = 'calib.mat';
    if exist(filename, 'file') ~= 2 || recal == 'y'
        calibration();
    end

    load(filename);
    neck = struct('angle',[0,0,0],'cappos',[0,0,0]);

    %% Connect to Drives
    eye_conn  = struct('ip', '192.168.2.14', 'port', 51243, 'axis_id',120, 'callback', @parse_response_msg_eye);
    neck_conn = struct('ip', '192.168.2.15', 'port', 51244, 'axis_id',121, 'callback', @parse_response_msg_neck);

    %connect to system, quit if fails
    eye_conn  = connect(eye_conn);
    neck_conn = connect(neck_conn);

    %Set-up clean up function
    cleanup_result = onCleanup(@()cleanup_all(eye_conn, neck_conn));

    if eye_conn.sock == -1 || neck_conn.sock == -1
        return;
    end

    %% Sampling Messages
    global update_msg;
    global eye_poll_msg;
    global neck_poll_msg;

    update_msg = build_msg_typea_group(1,264,int16.empty);
    eye_poll_msg = build_msg_typeb(0, eye_conn.axis_id, 2076, 2);
    neck_poll_msg = build_msg_typeb(0, neck_conn.axis_id, 552, 2);

    %% Check Neck Drive
    % for i =1:3
    %     get_axis_cappos = build_msg_typeb(i, neck_axis_id, 670, 2);
    %     send_msg_typeb(get_axis_cappos);
    %     n=0;
    %     while(neck(i).cappos == 0)
    %         pause(0.1);
    %         n=n+1;
    %         if(n > 10)
    %             display(sprintf('neck axis %d has not found index pulse...exiting',i));
    %             return;
    %         end
    %     end
    % end

    %% Configure Eye Drives

    %set the position control parameters, enable the control loops
    arrayfun(@(axis_id) config_eye_drive(eye_conn, axis_id), 1:4);

    %% Control Loop

    while true
        action = input('enter command:  ', 's');
        if strcmp(action,'exit') == 1
            break;
        elseif strcmp(action,'circle') == 1
            t = 1:400;
            des_yaw = 20*cos(2*pi*t/200);
            des_pitch = 20*sin(2*pi*t/200);
            des_neck = zeros(3,length(t));
            des_neck(1,:) = 20*cos(2*pi*t/200);
            des_eye = [des_yaw; des_pitch];      
            motion(eye_conn, eye, des_eye, neck_conn, neck, des_neck);

        elseif strncmpi(action, 'eye', length(eye)) == 1
            [des_angle_deg,num] = sscanf(action,'eye y:%f p:%f');

            if(num ~= 2)
                disp('Bad input');
                continue;
            end
            
            eye = move_eyes(eye_conn, eye, des_angle_deg);

        elseif strncmpi(action, 'neck', length(neck)) == 1
            [des_angle_deg,num] = sscanf(action,'neck y:%f p:%f r:%f');
            if(num ~= 3)
                disp('Bad input');
                continue;
            end
            neck = move_neck(neck_conn, neck, des_angle_deg);

        else
            disp('Bad Input');
        end

    end

end

%% Misc Helpers
function [] = config_eye_drive(conn, axis_id)

    %reset the target position (tpos) to the current position(apos)
    sta_msg = build_msg_typea(axis_id,11442,552);

    accel = typecast(uint32(8192),'uint16');
    apos_set_axis_accel = build_msg_typea(axis_id,9378,accel);

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

    send_msg_typea(conn, sta_msg);
        
    axis_on(conn, axis_id);

    send_msg_typea(conn, apos_set_axis_accel);
    send_msg_typea(conn, apos_set_axis_pos);
    send_msg_typea(conn, apos_set_axis_cpa);
    send_msg_typea(conn, apos_set_axis_modepp3);
    send_msg_typea(conn, apos_set_axis_tum1);

end

function [] = config_neck_drive(conn, axis_id)

    %reset the target position (tpos) to the current position(apos)
    sta_msg = build_msg_typea(axis_id,11442,552);

    accel = typecast(uint32(8192),'uint16');
    apos_set_axis_accel = build_msg_typea(axis_id,9378,accel);
    
    apos_set_axis_cpa = build_msg_typea(axis_id,22793,[65535,8192]);
    %print_msg(apos_set_axis_1_cpr);

    apos_set_axis_modepp3 = build_msg_typea(axis_id,22793,[49089,34561]);
    %print_msg(apos_set_axis_1_modepp)

    apos_set_axis_tum0 = build_msg_typea(axis_id,22793,[49151,0]);
    %print_msg(apos_set_axis_tum0)

    send_msg_typea(conn, sta_msg);
        
    axis_on(conn, axis_id);

    send_msg_typea(conn, apos_set_axis_accel);
    send_msg_typea(conn, apos_set_axis_pos);
    send_msg_typea(conn, apos_set_axis_cpa);
    send_msg_typea(conn, apos_set_axis_modepp3);
    send_msg_typea(conn, apos_set_axis_tum0);

end

function [] = print_msg(msg)
    disp(dec2hex(msg));
end

function cleanup_all(eye_conn, neck_conn)
    disp('Cleaning up file handles');
    cleanup(eye_conn);
    %cleanup(neck_conn);
    disp('Clean up complete');
end

function [] = cleanup(conn)
    stop_all(conn);
    axis_off(conn, 1);
    axis_off(conn, 2);
    axis_off(conn, 3);
    axis_off(conn, 4);
    disconnect(conn);
end

%% High Level Control Interface
function [] = motion(eye_conn, eye, des_eye, neck_conn, neck, des_neck)
        size = min(length(des_eye(1,:)), length(des_neck(1,:)));

        eye_angle = zeros(size,4);
        neck_angle = zeros(size,3);
        time = zeros(size,1);

        tic;
        for t=1:size
            time(t)=toc;
            eye = move_eyes(eye_conn, eye, des_eye(1:2,t));
            neck = move_neck(neck_conn, neck, des_neck(1:3,t));
            
            eye_angle(t,1) = eye(1).yaw;
            eye_angle(t,2) = eye(1).pitch;
            eye_angle(t,3) = eye(2).pitch;
            eye_angle(t,4) = eye(2).yaw; 

            for ii=1:3
                neck_angle(t,ii) = neck.angle(ii);
            end
            
            pause(0.1);
        end
        
        figure; hold on;
        plot(time,rad2deg(eye_angle));
        plot(time,des_eye);
      
        figure; hold on;
        plot(time,rad2deg(neck_angle));
        plot(time,des_neck);
end

function [eyes] = move_eyes(conn, eyes, des_angle_deg)
    global eye_poll_msg;
    global update_msg;
    global left;
    global right;
    
    des_yaw = deg2rad(des_angle_deg(1));
    des_pitch = deg2rad(des_angle_deg(2));
    
    for ii=1:2
        [r_act_ff,drdyaw,drdpitch]=inverse_kin_jac(des_yaw,des_pitch,eyes(ii).act(right));
        [l_act_ff,dldyaw,dldpitch]=inverse_kin_jac(des_yaw,des_pitch,eyes(ii).act(left));

        %send new linear actuator positions
        set_position(conn, eyes(ii).act(right).axis_id, r_act_ff);
        set_position(conn, eyes(ii).act(left).axis_id, l_act_ff);

        addr_1 = bitshift(eyes(ii).act(right).axis_id,4);
        addr_2 = bitshift(eyes(ii).act(left).axis_id,4);
        eye_poll_msg(3) = addr_1;
        eye_poll_msg(length(eye_poll_msg)) = 105 + addr_1;
        send_msg_typea(conn,eye_poll_msg);
        eye_poll_msg(3) = addr_2;
        eye_poll_msg(length(eye_poll_msg)) = 105 + addr_2;
        
        send_msg_typea(conn,eye_poll_msg);
    end
    
    send_msg_typea(conn, update_msg);
    eyes = udp_read_noblock(conn, eyes);
end

function [neck] = move_neck(conn, neck, des_angle_deg)
    global neck_poll_msg;
    global update_msg;
    
    for ii=1:3       
        des = deg2rad(des_angle_deg(ii));
        %TODO limit here
        set_neck_angle(conn, ii, des);
        addr = bitshift(ii,4);
        neck_poll_msg(3) = addr;
        neck_poll_msg(length(neck_poll_msg)) = 127 + addr;
        send_msg_typea(conn,neck_poll_msg);
    end
    
    send_msg_typea(conn, update_msg);
    neck = udp_read_noblock(conn, neck);
end

%% UDP Connection Management

function [] = udp_write(conn,port,data)
    pnet(conn.sock, 'write', uint8(data));
    pnet(conn.sock, 'writepacket', conn.ip, port);
end

function [result] = udp_read_block(conn)
    size=pnet(conn.sock,'readpacket');
    result=pnet(conn.sock,'read', size,'uint8');
end

function [drive] = udp_read_noblock(conn, drive)
    size=pnet(conn.sock,'readpacket','noblock');
    count = 1;
    while(size > 0 && count < 10)
        data = pnet(conn.sock,'read', size,'uint8');
        drive = conn.callback(data, drive);
        count = count+1;
        size=pnet(conn.sock,'readpacket','noblock');
    end
end

function [conn] = connect(conn)
    %input ip address
    %output, socket bound to local host response port
    conn_port = 30689;
    conn.sock = pnet('udpsocket',conn.port);
    pnet(conn.sock,'setwritetimeout',1);
    pnet(conn.sock,'setreadtimeout',1);

    udp_write(conn, conn_port, 1);
    result = udp_read_block(conn);
    
    if result == 2
        disp('********Connected**********');
    else
        disp('********Connection Failed**********');
        pnet(conn.sock,'close');
        conn.sock = -1;
        return;
    end
    
    udp_write(conn, conn_port, [3,0,194,1,0]);
    result = udp_read_block(conn);
    
    if result == 4
        disp('********Baud Rate Set to 1Mbps**********');
    else
        disp('********Setting Baud Rate Failed**********');
        pnet(conn.sock,'close');
        conn.sock = -1;
        return;
    end
    
    if sync(conn) == -1
        pnet(conn.sock,'close');
        conn.sock = -1;
    end
    
    pnet(conn.sock,'setreadtimeout',0);
end

function [result] = disconnect(conn)

    udp_write(conn, 30689, 5);
    result = udp_read_block(conn);
    if result == 6
        disp('********Disconnected**********');
    else
        disp('********Disconnection Failed**********');
    end
    
    pnet(conn.sock,'close');
    conn.sock = -1;
end

function [result] = sync(conn)
    sync = 255*ones(15,1);
    udp_write(conn, 1700, sync);
    result = udp_read_block(conn);

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

%% Pre Defined Messages

function [] = axis_on(conn,dest_addr) 
    data = int16.empty;    
    msg = build_msg_typea(dest_addr,258,data);
    send_msg_typea(conn, msg);
end

function [] = axis_off(conn,dest_addr) 
    data = int16.empty;      
    msg = build_msg_typea(dest_addr,2,data);
    send_msg_typea(conn, msg);
end

function [] = set_position(conn, axis, pos)
    %pos is position in m
    pos = convert_m_to_iu(pos);
    pos = typecast(int32(pos),'uint16');
    msg = build_msg_typea(axis,9374,pos);
    send_msg_typea(conn, msg);
end

function [] = set_neck_angle(conn, axis, angle)
    %angle is angle in rad
    angle = convert_rad_to_iu(angle);
    angle = typecast(int32(angle),'uint16');
    msg = build_msg_typea(axis,9374,angle);
    send_msg_typea(conn, msg);
end

function [] = stop3(conn, dest_addr)
    data = int16.empty;    
    msg = build_msg_typea(dest_addr,452,data);
    send_msg_typea(conn, msg);
end

function [] = stop_all(conn)
    data = int16.empty;    
    msg = build_msg_typea_group(1,452,data);
    send_msg_typea(conn, msg);
end

function [] = send_msg_typea(conn, msg)
    udp_write(conn,1700,msg);
end

function [] = send_msg_typeb(conn, msg)
    udp_write(conn,1700,msg);
end

%% Build MSG Helpers 

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

%% Parse MSG Responses 

function [result, axis] = parse_response_msg(local_buf)
    
    result = -1;
    axis = -1;
    
    if(length(local_buf)==1 && local_buf(1) == 79)
        %this is an ack, ignore it
        return;
    end
    
    size = local_buf(1)+2;
    
    if( local_buf(size) ~= gen_checksum(local_buf(1:size)))
        disp('CKSUM failed')
        return;
    end

    dst_addr = get_addr(local_buf(2:3));
    optcode = get_16bit_uint(local_buf(4:5));
    axis = get_addr(local_buf(6:7));
    value_addr = get_16bit_uint(local_buf(8:9));

    if(optcode == 46084)
        result = get_16bit_int(local_buf(10:11));
    elseif(optcode == 46085)
        result = get_32bit_int(local_buf(10:13));
    else
        axis = -1;
        result = -1;
    end
    
end

function [eye] = parse_response_msg_eye(data, eye)
    global left;
    global right;
    
    [data, axis] = parse_response_msg(data);
    if axis == 1
        eye(left).yaw = convert_iu_to_rad(data);
    elseif axis == 2
        eye(left).pitch = convert_iu_to_rad(data);
    elseif axis == 3
        eye(right).pitch = convert_iu_to_rad(data);
    elseif axis == 4
        eye(right).yaw = convert_iu_to_rad(data); 
    end
end

function [neck] = parse_response_msg_neck(data, neck)
    [data, axis] = parse_response_msg(data);
    
    if(axis < 1 || axis > 3)
        return;
    end
    
    neck.angle(axis) = convert_iu_to_rad(data);
end

function [pos] = convert_iu_to_m(iu)
    cpr = 2048;
    pole_pitch = 0.018;%in meters
    pos = pole_pitch*double(iu)/cpr;
end

function [rad] = convert_iu_to_rad(iu)
    cpr = 5000;
    rad = pi*double(iu)/(2*cpr);
end

function [iu] = convert_m_to_iu(m)
    cpr = 2048;
    pole_pitch = 0.018;%in meters
    iu = int32(cpr*m/pole_pitch);
end

function [iu] = convert_rad_to_iu(rad)
    cpr = 5000;
    iu = int32((rad*2*cpr)/pi);
end

