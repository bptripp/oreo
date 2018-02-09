#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "macaque.h"

#define COUNTS_PER_POLE	((double)2048)
#define METERS_PER_POLE ((double)0.018)
#define IU_TO_M (METERS_PER_POLE/COUNTS_PER_POLE)
#define M_TO_IU (COUNTS_PER_POLE/METERS_PER_POLE)
#define COUNTS_PER_REV 	((double)5000)
#define COUNTS_PER_REV_YAW 	((double)16*86)
#define IU_TO_RAD (M_PI/(2*COUNTS_PER_REV))
#define RAD_TO_IU (1/IU_TO_RAD)
#define IU_TO_RAD_YAW (M_PI/(2*COUNTS_PER_REV_YAW))
#define RAD_TO_IU_YAW (1/IU_TO_RAD_YAW)

#define SAMPLE_PERIOD_EYE_S	0.0005
#define SAMPLE_PERIOD_NECK_S	0.001
	
#define MPSS_TO_ACCEL_IU		(SAMPLE_PERIOD_EYE_S * SAMPLE_PERIOD_EYE_S * M_TO_IU)
#define ACCEL_IU_TO_MPSS		(1.0 / MPSS_TO_ACCEL_IU)
#define MPS_TO_SPEED_IU		        (SAMPLE_PERIOD_EYE_S * M_TO_IU)
#define SPEED_IU_TO_MPS		        (1.0 / MPS_TO_SPEED_IU)

#define RADPSS_TO_ACCEL_IU		(SAMPLE_PERIOD_NECK_S * SAMPLE_PERIOD_NECK_S * RAD_TO_IU)
#define RADPSS_TO_ACCEL_IU_YAW	(SAMPLE_PERIOD_NECK_S * SAMPLE_PERIOD_NECK_S * RAD_TO_IU_YAW)
#define ACCEL_IU_TO_RADPSS		(1.0 / RADPSS_TO_ACCEL_IU)
#define RADPS_TO_SPEED_IU		(SAMPLE_PERIOD_NECK_S * RAD_TO_IU)
#define RADPS_TO_SPEED_IU_YAW    (SAMPLE_PERIOD_NECK_S * RAD_TO_IU_YAW)
#define SPEED_IU_TO_RADPS		(1.0 / RADPS_TO_SPEED_IU)

#define GETBYTES_FROM_WORD(src,dst_p) (*(uint16_t*)(dst_p) = htons((uint16_t)src))
#define GETBYTES_FROM_LONG(src,dst_p) (*(uint32_t*)(dst_p) = htonl((uint32_t)src))
#define GETWORD_FROM_BYTES(src_p) (ntohs(*(uint16_t*)(src_p)))
#define GETLONG_FROM_BYTES(src_p) (ntohs((*(uint32_t*)src_p)&0xFFFF) | (ntohs((*(uint32_t*)src_p)>>16)<<16))

BOOL ErrorHandler( DWORD fdwCtrlType ) 
{ 

  cleanup();
  
  switch( fdwCtrlType ) 
  { 
    // Handle the CTRL-C signal. 
    case CTRL_C_EVENT: 
      printf( "Ctrl-C event\n\n" );
      Beep( 750, 300 ); 
      return( TRUE );
 
    // CTRL-CLOSE: confirm that the user wants to exit. 
    case CTRL_CLOSE_EVENT: 
      Beep( 600, 200 ); 
      printf( "Ctrl-Close event\n\n" );
      return( TRUE ); 
 
    // Pass other signals to the next handler. 
    case CTRL_BREAK_EVENT: 
      Beep( 900, 200 ); 
      printf( "Ctrl-Break event\n\n" );
      return FALSE; 
 
    case CTRL_LOGOFF_EVENT: 
      Beep( 1000, 200 ); 
      printf( "Ctrl-Logoff event\n\n" );
      return FALSE; 
 
    case CTRL_SHUTDOWN_EVENT: 
      Beep( 750, 500 ); 
      printf( "Ctrl-Shutdown event\n\n" );
      return FALSE; 
 
    default: 
      return FALSE; 
  } 
} 

double convert_iu_to_m(int32_t iu)
{
    return IU_TO_M * iu;
}

int32_t convert_m_to_iu(double m)
{
    return (int32_t)(M_TO_IU * m);
}

double convert_iu_to_rad(int32_t iu)
{
    return IU_TO_RAD * iu;
}

double convert_iu_to_rad_yaw(int32_t iu)
{
    return IU_TO_RAD_YAW * iu;
}

typedef struct rawData
{
    uint8_t id;
    double data;
    double time;
} rawData_t;

typedef struct rawDataLog
{
    uint32_t		index;
    rawData_t           entry[LOG_BUFLEN];
} rawDataLog_t;

typedef struct msg
{
    uint8_t msg[BUFLEN];
    int  size;
} msg_t;

static WSADATA wsa;

static eyeData_t eyeData;
static eyeCalData_t eyeCalData;
static rawDataLog_t eyeLogData;

static neckData_t neckData;
static rawDataLog_t neckLogData;

typedef void(*rxCallbackFxn)(uint16_t, uint16_t, int32_t);
void eyeRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data);
void neckRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data);
static double get_timestamp();

typedef struct devHandle
{
    SOCKET             	sock;
    struct sockaddr_in 	addr;
    int                	addr_len;

    const int          	local_port;
    const char* const  	ip;
    
    int         	runFlag;
    HANDLE      	txThreadHandle;
    HANDLE      	rxThreadHandle;
    char        	rx_buf[BUFLEN];

    uint32_t         	ack_pend;
    uint8_t		force_sync;

    msg_t		cmd_buf[MAX_CMD_PEND];
    uint32_t		cmd_consume_idx;
    uint32_t		cmd_produce_idx;
    uint8_t		host_id;

    rxCallbackFxn	callback;

} devHandle_t;

devHandle_t eye  = {.local_port = EYE_LOCAL_PORT, .ip = EYE_IP, .force_sync=0,
                    .ack_pend = 0, .cmd_consume_idx = 0, .cmd_produce_idx = 0, 
		    .host_id = EYE_HOST_ID, .callback = &eyeRxCallback};

devHandle_t neck = {.local_port = NECK_LOCAL_PORT, .ip = NECK_IP, .force_sync=0,
                    .ack_pend = 0, .cmd_consume_idx = 0, .cmd_produce_idx = 0, 
		    .host_id = NECK_HOST_ID, .callback = &neckRxCallback};

static void add_log_data(rawDataLog_t* log, double time, double data, uint8_t id)
{
    log->entry[log->index].data = data;
    log->entry[log->index].time = time;
    log->entry[log->index].id   = id;
    log->index = (++log->index < LOG_BUFLEN)?log->index:0;	 
}

void neckRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data)
{
    neckData.time = get_timestamp();
    double converted_data;
    uint8_t log_data_id;

    switch (reg_addr)
    {
        case REG_ADDR_APOS & REG_ADDR_MASK:
            converted_data = convert_iu_to_rad(data);
            break;
        default:
            break;
    }

    switch (axis_id)
    {
        case NECK_YAW_AXIS:
            log_data_id = NECK_YAW; 
            neckData.yaw = convert_iu_to_rad_yaw(data);
            break;
        case NECK_PITCH_AXIS:
            log_data_id = NECK_PITCH;
	    neckData.pitch = converted_data;
            break;
        case NECK_ROLL_AXIS:
            log_data_id = NECK_ROLL;
	    neckData.roll = converted_data;
            break;
        default:
            break;
    }

    add_log_data(&neckLogData, neckData.time, converted_data, log_data_id);
}

void eyeRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data)
{

    double converted_data;
    uint8_t log_data_id;
    double time = get_timestamp();
    switch (reg_addr)
    {
        case REG_ADDR_APOS & REG_ADDR_MASK:
            log_data_id = 1;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.pos[axis_id-1] = converted_data;
            break;

        case REG_ADDR_POSERR & REG_ADDR_MASK:
            log_data_id = 2;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.err[axis_id-1] = converted_data;
            break;

        case REG_ADDR_TPOS & REG_ADDR_MASK:
            log_data_id = 3;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.tpos[axis_id-1] = converted_data;
            break;

        case REG_ADDR_APOS2 & REG_ADDR_MASK:
            log_data_id = 0;
            eyeData.time = time;
            converted_data = convert_iu_to_rad(data);

            switch (axis_id)
            {
                case EYE_YAW_LEFT_AXIS:
                    eyeData.yaw[LEFT] = converted_data;
                    break;
                case EYE_YAW_RIGHT_AXIS:
                    eyeData.yaw[RIGHT] = converted_data;
                    break;
                case EYE_PITCH_LEFT_AXIS:
                    eyeData.pitch[LEFT] = converted_data;
                    break;
                case EYE_PITCH_RIGHT_AXIS:
                    eyeData.pitch[RIGHT] = converted_data;
                    break;
                default:
                    break;
            }
            
            break;

        case CAL_RUN_VAR & REG_ADDR_MASK:
            eyeCalData.complete[axis_id-1] = data;
            break;

	    case CAL_APOS2_OFF_VAR & REG_ADDR_MASK:
            eyeData.time = time;
            converted_data = convert_iu_to_rad(data);

            switch (axis_id)
            {
                case EYE_YAW_LEFT_AXIS:
                    eyeData.yaw_offset[LEFT] = converted_data;
                    break;
                case EYE_YAW_RIGHT_AXIS:
                    eyeData.yaw_offset[RIGHT] = converted_data;
                    break;
                case EYE_PITCH_LEFT_AXIS:
                    eyeData.pitch_offset[LEFT] = converted_data;
                    break;
                case EYE_PITCH_RIGHT_AXIS:
                    eyeData.pitch_offset[RIGHT] = converted_data;
                    break;
                default:
                    break;
            }
	    break;

        default:
            break;
    }

    add_log_data(&eyeLogData, time, converted_data, (log_data_id*NUM_EYE_AXIS)+axis_id-1);
}

static uint8_t gen_cksum(uint8_t *msg, uint8_t size)
{
    uint8_t cksum=0;
    uint8_t n=0;
    for(n=0; n < size-1; n++)
    {
        cksum+=msg[n];
    }

    return cksum&0xFF;
}

static uint16_t build_addr(uint8_t axis_id, uint8_t is_host, uint8_t is_group)
{
    uint16_t addr;
    addr = (axis_id&0xFF) << 4;
    addr |= (is_host)?0x0001:0x0000;
    addr |= (is_group)?0x1000:0x0000;
    return addr;
}

static uint8_t get_addr(uint16_t formatted)
{
    uint8_t addr;
    addr = (uint8_t)((formatted&0x0FF0) >> 4);
    return addr;
}

static void parse_response_msg(devHandle_t* dev)
{   
    uint8_t *msg = dev->rx_buf;
    int32_t result = 0;
    uint16_t axis;
    uint16_t reg_addr;

    if(msg == NULL)
    {  
        return;
    }

    int size = msg[MSG_SIZE_OFFSET]+2;
    
    if( msg[size-1] != gen_cksum(msg,size) || size < MSG_TYPEA_BASE)
    {
        return;
    }

    uint16_t dst_addr   = get_addr(GETWORD_FROM_BYTES(&msg[MSG_ADDR_OFFSET]));
    uint16_t optcode    = GETWORD_FROM_BYTES(&msg[MSG_OPT_OFFSET]);

    if (((optcode&MSG_TAKE_OPTCODE_MASK) == (OPT_1WORD_TAKE&MSG_TAKE_OPTCODE_MASK)) 
        && size == MSG_TAKE1WORD_SIZE)
    {
        axis         = optcode&0xFF;      
        reg_addr     = GETWORD_FROM_BYTES(&msg[MSG_TAKE_REG_ADDR_OFFSET]);
        int16_t temp = GETWORD_FROM_BYTES(&msg[MSG_TAKE_DATA_OFFSET]);
        result       = (int32_t)temp;
    }
    else if (((optcode&MSG_TAKE_OPTCODE_MASK) == (OPT_2WORD_TAKE&MSG_TAKE_OPTCODE_MASK))
             && size == MSG_TAKE2WORD_SIZE)
    {
        axis     = optcode&0xFF;
        reg_addr = GETWORD_FROM_BYTES(&msg[MSG_TAKE_REG_ADDR_OFFSET]);	
        result   = GETLONG_FROM_BYTES(&msg[MSG_TAKE_DATA_OFFSET]);
    }	
    else if(optcode == OPT_1WORD_RESP && size == MSG_GIVE1WORD_SIZE)
    {
        axis         = get_addr(GETWORD_FROM_BYTES(&msg[MSG_SEND_ADDR_OFFSET]));
        reg_addr     = GETWORD_FROM_BYTES(&msg[MSG_GIVE_REG_ADDR_OFFSET]);
        int16_t temp = GETWORD_FROM_BYTES(&msg[MSG_GIVE_DATA_OFFSET]);
        result       = (int32_t)temp;
    }
    else if(optcode == OPT_2WORD_RESP && size == MSG_GIVE2WORD_SIZE)
    { 
        axis       = get_addr(GETWORD_FROM_BYTES(&msg[MSG_SEND_ADDR_OFFSET]));
        reg_addr   = GETWORD_FROM_BYTES(&msg[MSG_GIVE_REG_ADDR_OFFSET]);
        result     = GETLONG_FROM_BYTES(&msg[MSG_GIVE_DATA_OFFSET]);
    }

    if(dev->callback!=NULL)
        dev->callback(axis, reg_addr, result);
}

static uint64_t timebase;

static double get_timestamp()
{
    FILETIME tm;
    uint64_t t;
    GetSystemTimePreciseAsFileTime(&tm);
    t = ((uint64_t)tm.dwHighDateTime << 32) | (uint64_t)tm.dwLowDateTime;
    t = t - timebase;
    return (double)t / 10000000.0;
}

DWORD WINAPI ThreadRXFunc(void* data) 
{ 
    devHandle_t* dev = (devHandle_t*)data;
    struct sockaddr_in source_addr;
    int                source_addr_len = sizeof(source_addr);
    int                rx_bytes;
 
    if(!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL))
    {
      printf("Failed to set RX thread priority with error:%d\n", GetLastError());
    }

    while(dev->runFlag)
    {                
        memset(dev->rx_buf,0, BUFLEN);
        if ((rx_bytes = recvfrom(dev->sock, dev->rx_buf, BUFLEN, 0, (struct sockaddr *) &source_addr, &source_addr_len)) == SOCKET_ERROR)
        {
            int last_err = WSAGetLastError();

            if(last_err != WSAETIMEDOUT)
	    {
                printf("recvfrom() failed with error code : %d\n" , last_err);
                break;
            }
        }
	else
	{
            if(dev->force_sync)
            {
                if(rx_bytes == CONN_SYNC_BYTES && dev->rx_buf[CONN_SYNC_BYTES-1] == CONN_SYNC_RESP)
                {
                    dev->force_sync=0;
                    dev->ack_pend = 0;
                }
            }
            else if(dev->rx_buf[0] == MSG_ACK && dev->ack_pend > 0)
	    {
                dev->ack_pend--;
            }
            else
            {
                parse_response_msg(dev);
	    }
	}
    }
}

DWORD WINAPI ThreadTXFunc(void* data) 
{ 
    devHandle_t* dev = (devHandle_t*)data;
    unsigned int index;
    uint8_t sync_cmd[CONN_SYNC_BYTES];
    memset(sync_cmd, CONN_SYNC_BYTE, CONN_SYNC_BYTES);
 
    if(!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL))
    {
      printf("Failed to set TX thread priority with error:%d\n", GetLastError());
    }

    while(dev->runFlag || (dev->cmd_produce_idx - dev->cmd_consume_idx) != 0)
    {   
        if(dev->force_sync)
        { 
            if (sendto(dev->sock, sync_cmd, sizeof(sync_cmd), 0 , (struct sockaddr *) &dev->addr, dev->addr_len) == SOCKET_ERROR)
            {
                printf("sendto() failed with error code : %d\n" , WSAGetLastError());
                break;
            }
            Sleep(100);
        } 
        else if(dev->ack_pend < MAX_ACK_PEND && (dev->cmd_produce_idx - dev->cmd_consume_idx) != 0)
        {  
            index = dev->cmd_consume_idx % MAX_CMD_PEND;

            if (sendto(dev->sock, dev->cmd_buf[index].msg, dev->cmd_buf[index].size, 0 , (struct sockaddr *) &dev->addr, dev->addr_len) == SOCKET_ERROR)
            {
                printf("sendto() failed with error code : %d\n" , WSAGetLastError());
                break;
            }

            dev->cmd_consume_idx++;
            dev->ack_pend++;
        }
        else
        {
	    if(dev->ack_pend > MAX_ACK_PEND)
            {
                //our communication link may have gotten confused
                //resync
                printf("lost too many MSG ACKS, resyncing...\n");
                dev->force_sync=1;
            }
            //yield, ethier we have too many messages pending or nothing to consume
            SwitchToThread();
        }
    }
}

static void sync(devHandle_t* dev)
{
    dev->force_sync=1;
}

static uint16_t connectDev(devHandle_t* dev)
{
    struct sockaddr_in source_addr;
    int                source_addr_len = sizeof(source_addr);

    //setup address structure for connection
    memset(&dev->addr, 0, dev->addr_len);
    dev->addr.sin_family = AF_INET;    
    dev->addr.sin_port = htons(CONN_PORT);
    dev->addr.sin_addr.S_un.S_addr = inet_addr(dev->ip);

    if (sendto(dev->sock, CONN_MSG, sizeof(CONN_MSG), 0, (struct sockaddr *) &dev->addr, dev->addr_len) == SOCKET_ERROR)
    {
        printf("sendto() failed with error code : %d\n" , WSAGetLastError());
        return CONN_ERR;
    }
    
    memset(dev->rx_buf,0, BUFLEN);
    
    if (recvfrom(dev->sock, dev->rx_buf, BUFLEN, 0, (struct sockaddr *) &source_addr, &source_addr_len) == SOCKET_ERROR)
    {
        printf("recvfrom() failed with error code : %d\n" , WSAGetLastError());
        return CONN_ERR;
    }

    if(dev->rx_buf[0] != CONN_RESP)
    {
        return CONN_ERR;
    }

    if (sendto(dev->sock, CONN_CONFIG, sizeof(CONN_CONFIG), 0, (struct sockaddr *) &dev->addr, dev->addr_len) == SOCKET_ERROR)
    {
        printf("sendto() failed with error code : %d\n" , WSAGetLastError());
        return CONN_CONFIG_ERR;
    }

    memset(dev->rx_buf,0, BUFLEN);
    
    if (recvfrom(dev->sock, dev->rx_buf, BUFLEN, 0, (struct sockaddr *) &source_addr, &source_addr_len) == SOCKET_ERROR)
    {
        printf("recvfrom() failed with error code : %d\n" , WSAGetLastError());
        return CONN_CONFIG_ERR;
    }

    if(dev->rx_buf[0] != CONN_CONFIG_RESP)
    {
        return CONN_CONFIG_ERR;
    }

    //setup address structure for communication and commands
    dev->addr.sin_port = htons(COMM_PORT);

    sync(dev);

    return CONN_OK;

}

static uint16_t disconnectDev(devHandle_t* dev)
{
    dev->force_sync = 0;
    struct sockaddr_in source_addr;
    int                source_addr_len = sizeof(source_addr);

    //setup address structure for connection
    memset(&dev->addr, 0, dev->addr_len);
    dev->addr.sin_family = AF_INET;    
    dev->addr.sin_port = htons(CONN_PORT);
    dev->addr.sin_addr.S_un.S_addr = inet_addr(dev->ip);

    if (sendto(dev->sock, DISCONN_MSG, sizeof(DISCONN_MSG), 0, (struct sockaddr *) &dev->addr, dev->addr_len) == SOCKET_ERROR)
    {
        printf("sendto() failed with error code : %d\n" , WSAGetLastError());
        return CONN_ERR;
    }
    
    memset(dev->rx_buf,0, BUFLEN);

    if (recvfrom(dev->sock, dev->rx_buf, BUFLEN, 0, (struct sockaddr *) &source_addr, &source_addr_len) == SOCKET_ERROR)
    {
        printf("recvfrom() failed with error code : %d\n" , WSAGetLastError());
        return CONN_ERR;
    }

    if(dev->rx_buf[0] != DISCONN_RESP)
    {
        return CONN_ERR;
    }

    return CONN_OK;
}

static void shutdownDev(devHandle_t* dev)
{
    uint16_t was_running = dev->runFlag;

    //stop the processing threads
    dev->runFlag = 0;

    if(dev->rxThreadHandle != NULL)
    {
        WaitForSingleObject(dev->rxThreadHandle,SHUTDOWN_TIMEOUT_MS );
        dev->rxThreadHandle = NULL;
    }

    if(dev->txThreadHandle != NULL)
    {
        WaitForSingleObject(dev->txThreadHandle,SHUTDOWN_TIMEOUT_MS );
        dev->txThreadHandle = NULL;
    }

    if(dev->sock != INVALID_SOCKET)
    {
        if(was_running)
        {
            printf("Processing threads complete, disconnecting\n");
            disconnectDev(dev);
        }
        printf("Closing Socket\n");
        closesocket(dev->sock);
        dev->sock = INVALID_SOCKET;
    }    

    dev->force_sync=0;
    dev->ack_pend = 0;
    dev->cmd_consume_idx = 0;
    dev->cmd_produce_idx = 0;
}

static void startDev(devHandle_t* dev)
{
    //incase we are currently running call shutdown first;
    shutdownDev(dev);

    dev->addr_len = sizeof(dev->addr);
    
    //Create a socket
    if((dev->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d\n" , WSAGetLastError());
        return;
    }

    printf("Socket created.\n");

    int timeout = RX_TIMEOUT_MS;
    setsockopt(dev->sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    
    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons( dev->local_port);

    if( bind(dev->sock ,(struct sockaddr *)&local_addr , sizeof(local_addr)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d\n" , WSAGetLastError());
    	closesocket(dev->sock);
        dev->sock = INVALID_SOCKET;
        return;
    }

    printf("Bind Complete.\n");
    uint16_t conn_count=0;
    while (conn_count<5 && connectDev(dev) != CONN_OK)
    {
        conn_count++;
    }
    
    //setup the connection to the technosoft drive
    if(conn_count<5)
    {
        //if we can talk with the drive, start the processing threads
        dev->runFlag=1;
        dev->rxThreadHandle = CreateThread(NULL, 0, ThreadRXFunc, dev, 0, NULL);
        dev->txThreadHandle = CreateThread(NULL, 0, ThreadTXFunc, dev, 0, NULL);
    }
    else
    {
        printf("failed to connect to drive\n");
    }
}

static msg_t* getMsgSlot(devHandle_t* dev)
{
    double base_time = get_timestamp();
    double cur_time;
    while (dev->cmd_produce_idx - dev->cmd_consume_idx == MAX_CMD_PEND)
    {
        cur_time = get_timestamp();
 
        if(cur_time - base_time > SEND_TIMEOUT_S || dev->runFlag == 0)
        { 
            printf("send message failed to get slot\n");
            return NULL;
        }

        // buffer is full, yield
        SwitchToThread();
    }

    msg_t* msg = &dev->cmd_buf[dev->cmd_produce_idx % MAX_CMD_PEND];
    memset(msg,0,BUFLEN);
    return msg;
}

static void buildMsgTypeA(msg_t* msg_s, uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{
    if(words > 2 || msg_s == NULL)
    {
        words=2;
    }

    uint8_t* msg = msg_s->msg;
    uint8_t msg_size = MSG_TYPEA_BASE+(words*2);
    uint16_t dest_addr_formatted = build_addr(axis_id,0,group);

    msg[MSG_SIZE_OFFSET] = msg_size-2; 
    GETBYTES_FROM_WORD(dest_addr_formatted, &msg[MSG_ADDR_OFFSET]);
    GETBYTES_FROM_WORD(optcode, &msg[MSG_OPT_OFFSET]);
   
    if(words == 1)
    {
        GETBYTES_FROM_WORD(0xFFFF&data, &msg[MSG_TYPEA_BASE-1]);
    }
    else if(words == 2)
    {
        GETBYTES_FROM_LONG(data, &msg[MSG_TYPEA_BASE-1]);
    }

    msg[msg_size-1] = gen_cksum(msg, msg_size); 

    msg_s->size = msg_size;
}

static void buildMsgTypeB(msg_t* msg_s, uint8_t axis_id, uint8_t return_addr, uint16_t reg_addr, uint8_t words)
{
    uint8_t* msg = msg_s->msg;
    uint16_t optcode;
    uint16_t dest_addr_formatted = build_addr(axis_id,0,0);
    uint16_t send_addr_formatted = build_addr(return_addr,1,0);

    if (words == 1)
        optcode=OPT_1WORD_REQ;
    else
        optcode=OPT_2WORD_REQ;

    msg[MSG_SIZE_OFFSET] = MSG_TYPEB_BASE-2; 

    GETBYTES_FROM_WORD(dest_addr_formatted, &msg[MSG_ADDR_OFFSET]);
    GETBYTES_FROM_WORD(optcode, &msg[MSG_OPT_OFFSET]);
    GETBYTES_FROM_WORD(send_addr_formatted, &msg[MSG_SEND_ADDR_OFFSET]);
    GETBYTES_FROM_WORD(reg_addr, &msg[MSG_GIVE_REG_ADDR_OFFSET]);

    msg[MSG_TYPEB_BASE-1] = gen_cksum(msg, MSG_TYPEB_BASE);

    msg_s->size = MSG_TYPEB_BASE;
}

static void sendMsgTypeA(devHandle_t* dev, uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{ 
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;

    buildMsgTypeA(msgSlot, axis_id, optcode, data, words, group);
    dev->cmd_produce_idx++;
}

static void sendMsgTypeB(devHandle_t* dev, uint8_t axis_id, uint8_t return_addr, uint16_t reg_addr, uint8_t words)
{    
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;

    buildMsgTypeB(msgSlot, axis_id, return_addr, reg_addr, words);
    dev->cmd_produce_idx++;
}

__declspec(dllexport) void sendMsgTypeAEye(uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{
    sendMsgTypeA(&eye, axis_id, optcode, data, words, group);
}
__declspec(dllexport) void sendMsgTypeBEye(uint8_t axis_id, uint16_t reg_addr, uint8_t words)
{
    sendMsgTypeB(&eye, axis_id, eye.host_id, reg_addr, words);
}
__declspec(dllexport) void sendMsgTypeANeck(uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{
    sendMsgTypeA(&neck, axis_id, optcode, data, words, group);
}
__declspec(dllexport) void sendMsgTypeBNeck(uint8_t axis_id, uint16_t reg_addr, uint8_t words)
{
    sendMsgTypeB(&neck, axis_id, neck.host_id, reg_addr, words);
}

__declspec(dllexport) void start(void)
{

    AllocConsole();

    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);

    printf("****DEBUG WINDOW*****\n");

    if( !SetConsoleCtrlHandler( (PHANDLER_ROUTINE) ErrorHandler, TRUE ) ) 
    {
        printf( "\nERROR: Could not set control handler"); 
        return;
    }

    //Initialise winsock
    printf("\nInitialising Winsock...");

    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Failed. Error Code : %d\n", WSAGetLastError());
        return;
    } 
 
    FILETIME tm;
    GetSystemTimePreciseAsFileTime(&tm);
    timebase = ((uint64_t)tm.dwHighDateTime << 32) | (uint64_t)tm.dwLowDateTime;

    int n;

    if(!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS))
    {
      printf("Failed to set process priority class with error:%d\n", GetLastError());
    } 

    disEyePollData();
    startDev(&eye);

    disNeckPollData();
    startDev(&neck);

    printf("Initialised.\n");
}

__declspec(dllexport) void cleanup(void)
{
    shutdownDev(&eye);
    shutdownDev(&neck);
    WSACleanup();
    printf("Cleanup Complete.\n");
    FreeConsole();
}

__declspec(dllexport) eyeData_t* getEyeData(void)
{
    return &eyeData;
}

__declspec(dllexport) eyeCalData_t* getEyeCalData(void)
{
    return &eyeCalData;
}

__declspec(dllexport) neckData_t* getNeckData(void)
{
    return &neckData;
}

__declspec(dllexport) void enEyePollData(void)
{
    sendMsgTypeAEye(1, OPT_GOTO, MOTION_LOOP_IP, 1, 1);
    return;
}

__declspec(dllexport) void disEyePollData(void)
{
    sendMsgTypeAEye(1, OPT_GOTO, WAIT_LOOP_IP, 1, 1);
    return;
}

__declspec(dllexport) void enNeckPollData(void)
{
    sendMsgTypeANeck(1, OPT_GOTO, MOTION_LOOP_NECK_IP, 1, 1);
    return;
}

__declspec(dllexport) void disNeckPollData(void)
{
    sendMsgTypeANeck(1, OPT_GOTO, WAIT_LOOP_NECK_IP, 1, 1);
    return;
}

#define MAX_FIXED_POINT 32767.999969
#define MIN_FIXED_POINT -32767.999969

static uint32_t getFixedPoint(double value)
{
    if(value >= MAX_FIXED_POINT  || value <= MIN_FIXED_POINT)
    {
        return 0;
    }
    
    int16_t whole = (int16_t)floor(value);
    double fractional = value - whole;
    uint16_t decimal = fractional*0xFFFF;
    uint32_t result = ((uint32_t)decimal << 16) | (whole&0xFFFF);
    return result;
}

static uint32_t getLong(int32_t value)
{
    uint32_t result = ((uint32_t)value << 16) | ((value>>16)&0xFFFF);
    return result;
}

__declspec(dllexport) void setEyePos(uint8_t axis, double pos_m)
{
    uint32_t data = getLong(pos_m*M_TO_IU);
    sendMsgTypeAEye(axis, OPT_CPOS, data, 2, 0);
}

__declspec(dllexport) void setNeckPos(uint8_t axis, double pos_rad)
{
    uint32_t data;
    if (axis == NECK_YAW_AXIS)
        data = getLong(pos_rad*RAD_TO_IU_YAW);
    else
        data = getLong(pos_rad*RAD_TO_IU);
    
    sendMsgTypeANeck(axis, OPT_CPOS, data, 2, 0);
}

__declspec(dllexport) void setNeckSpeed(uint8_t axis, double speed_rps)
{
    uint32_t data;
    
    if(axis == NECK_YAW_AXIS)
        data = getFixedPoint(speed_rps*RADPS_TO_SPEED_IU_YAW);
    else
        data = getFixedPoint(speed_rps*RADPS_TO_SPEED_IU);
    
    sendMsgTypeANeck(axis, OPT_CSPD, data, 2, 0);
}

__declspec(dllexport) void setEyeSpeed(uint8_t axis, double speed_mps)
{
    uint32_t data = getFixedPoint(speed_mps*MPS_TO_SPEED_IU);
    sendMsgTypeAEye(axis, OPT_CSPD, data, 2, 0);
}

__declspec(dllexport) void setNeckAccel(uint8_t axis, double accel_rpss)
{
    uint32_t data;
    
    if(axis == NECK_YAW_AXIS)
        data = getFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU_YAW);
    else
        data = getFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU);
    
    sendMsgTypeANeck(axis, OPT_CACC, data, 2, 0);
}

__declspec(dllexport) void setEyeAccel(uint8_t axis, double accel_mpss)
{
    uint32_t data = getFixedPoint(accel_mpss*MPSS_TO_ACCEL_IU);
    sendMsgTypeAEye(axis, OPT_CACC, data, 2, 0);
}

__declspec(dllexport) void startEyeCal(uint8_t axis, double pos_m)
{
    if(axis <=0 || axis > NUM_EYE_AXIS)
    {
        return;
    }

    uint32_t data;

    setEyePos(axis, pos_m);
    eyeCalData.complete[axis-1] = CAL_RUNNING;

    if(pos_m > 0)
    {
        sendMsgTypeAEye(axis, OPT_GOTO, FOR_CAL_IP, 1, 0);
    }
    else
    {
        sendMsgTypeAEye(axis, OPT_GOTO, REV_CAL_IP, 1, 0);
    }
}