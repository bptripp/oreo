#include <stdint.h>

#define SHUTDOWN_TIMEOUT_MS 	2000
#define RX_TIMEOUT_MS		1000
#define SEND_TIMEOUT_S		1.0
#define BUFLEN 			32  	
#define LOG_BUFLEN		1024
#define MAX_ACK_PEND 		4  //limit how much we pound the drives with polls for data
#define MAX_CMD_PEND 		8  //must be a power of 2

#define CONN_PORT		30689u
#define CONN_MSG		((uint8_t[]){0x01})
#define CONN_RESP		((uint8_t)0x02)
#define CONN_CONFIG	        ((uint8_t[]){0x03,0x00,0xC2,0x01,0x00})//1Mbit config
#define CONN_CONFIG_RESP	((uint8_t)0x04)
#define CONN_ERR		1
#define CONN_CONFIG_ERR		2
#define CONN_SYNC_ERR		3
#define CONN_OK			0
#define CONN_SYNC_RESP		0x0Du
#define CONN_SYNC_BYTES		15
#define CONN_SYNC_BYTE		0xFFu
#define DISCONN_MSG		((uint8_t[]){0x05})
#define DISCONN_RESP		((uint8_t)0x06)

#define COMM_PORT		1700
#define MSG_ACK			((uint8_t)0x4F)

#define NECK_HOST_ID		121
#define NECK_LOCAL_PORT		51244u
#define NECK_IP			"192.168.2.15"

#define EYE_HOST_ID		120
#define EYE_LOCAL_PORT		51243u
#define EYE_IP			"192.168.2.14"

#define MSG_TYPEA_BASE		6
#define MSG_TYPEB_BASE		10
#define MSG_SIZE_OFFSET		0
#define MSG_ADDR_OFFSET		1
#define MSG_OPT_OFFSET		3
#define MSG_SEND_ADDR_OFFSET	5
#define MSG_TAKE_REG_ADDR_OFFSET 5
#define MSG_GIVE_REG_ADDR_OFFSET 7
#define MSG_TAKE_DATA_OFFSET	7
#define MSG_GIVE_DATA_OFFSET	9
#define MSG_TAKE1WORD_SIZE	10
#define MSG_TAKE2WORD_SIZE	12
#define MSG_GIVE1WORD_SIZE	12
#define MSG_GIVE2WORD_SIZE	14
#define MSG_TAKE_OPTCODE_MASK   0xFF00

#define OPT_1WORD_RESP		0xB404u
#define OPT_2WORD_RESP		0xB405u
#define OPT_1WORD_TAKE		0xD400u
#define OPT_2WORD_TAKE		0xD500u
#define OPT_1WORD_REQ		0xB004u
#define OPT_2WORD_REQ		0xB005u
#define OPT_CACC		0x24A2u
#define OPT_CSPD		0x24A0u
#define OPT_CPOS		0x249Eu
#define OPT_GOTO		0x7400u
#define REG_ADDR_APOS		0x0228u
#define REG_ADDR_POSERR		0x022Au
#define REG_ADDR_APOS2		0x081Cu
#define REG_ADDR_TPOS		0x02B2u
#define REG_ADDR_MASK		0x07FFu

//technosoft drive program specific addresses
#define MOTION_LOOP_IP		0x4022u
#define WAIT_LOOP_IP		0x401Bu
#define MOTION_LOOP_NECK_IP	0x403Bu
#define WAIT_LOOP_NECK_IP	0x4034u
#define REV_CAL_IP		0x4061u
#define FOR_CAL_IP		0x4027u
#define CAL_RUN_VAR		0x03B3u
#define CAL_APOS2_OFF_VAR	0x03B0u

//eye drive defines
#define LEFT			0
#define RIGHT			1
#define NUM_EYES		2
#define NUM_EYE_AXIS		(2*(NUM_EYES))

//axis id mapping
#define EYE_YAW_LEFT_AXIS	1
#define EYE_PITCH_LEFT_AXIS	2
#define EYE_PITCH_RIGHT_AXIS	3
#define EYE_YAW_RIGHT_AXIS	4

//log data id
#define EYE_YAW_LEFT		0
#define EYE_PITCH_LEFT		1
#define EYE_YAW_RIGHT		2
#define EYE_PITCH_RIGHT		3
#define EYE_LEFT_LEFT_POS	4
#define EYE_LEFT_RIGHT_POS	5
#define EYE_RIGHT_LEFT_POS	6
#define EYE_RIGHT_RIGHT_POS	7
#define EYE_LEFT_LEFT_POSERR	8
#define EYE_LEFT_RIGHT_POSERR 	9
#define EYE_RIGHT_LEFT_POSERR	10
#define EYE_RIGHT_RIGHT_POSERR	11

//neck drive defines
//axis id mapping
#define NUM_NECK_AXIS		3
#define NECK_YAW_AXIS		1
#define NECK_PITCH_AXIS		2
#define NECK_ROLL_AXIS		3

//log data id
#define NECK_YAW		0
#define NECK_PITCH		1
#define NECK_ROLL		2

#define CAL_COMPLETE		0
#define CAL_RUNNING		1

typedef struct eyeCalData
{
    double time;
    double pos[NUM_EYE_AXIS];
    double err[NUM_EYE_AXIS];
    double tpos[NUM_EYE_AXIS];
    uint16_t complete[NUM_EYE_AXIS];
} eyeCalData_t;

typedef struct eyeData
{
    double yaw[NUM_EYES];
    double yaw_offset[NUM_EYES];
    double pitch[NUM_EYES];
    double pitch_offset[NUM_EYES];
    double time;
} eyeData_t;

typedef struct neckData
{
    double yaw;
    double pitch;
    double roll;
    double time;
} neckData_t;

void sendMsgTypeAEye(uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group);

void sendMsgTypeBEye(uint8_t axis_id, uint16_t reg_addr, uint8_t words);

void sendMsgTypeANeck(uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group);

void sendMsgTypeBNeck(uint8_t axis_id, uint16_t reg_addr, uint8_t words);



neckData_t* getNeckData(void);

void enNeckPollData(void);

void disNeckPollData(void);

void setNeckPos(uint8_t axis, double pos_rad);

void setNeckSpeed(uint8_t axis, double speed_rps);

void setNeckAccel(uint8_t axis, double accel_rpss);


eyeData_t* getEyeData(void);

eyeCalData_t* getEyeCalData(void);

void enEyePollData(void);

void disEyePollData(void);

void setEyePos(uint8_t axis, double pos_m);

void setEyeSpeed(uint8_t axis, double speed_mps);

void setEyeAccel(uint8_t axis, double accel_mpss);

void startEyeCal(uint8_t axis, double pos_m);


void start(void);

void cleanup(void);

