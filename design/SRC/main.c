#include "LIB_Config.h"
#include "Hal_rgb_led.h"  //rgb
#include "Hal_uart.h"
#include "EMSP_API.h"
#include "em380c_hal.h"
#include "hal_infrared.h" //红外线
#include "Hal_key.h"	//按钮
#include "Hal_motor.h" //电机
#include "hal_temp_hum.h" //温湿度
#include <stdio.h>
#include <string.h>
#include <stdlib.h>	

#define	STARTFLAG 0xA1A2A3BB
typedef struct {
	unsigned short ID;
	unsigned int V1;
	unsigned int V2;
	unsigned int V3;
}__attribute__((packed)) sensor_data_frame;

typedef struct {
	unsigned char START_F0;
	unsigned char START_F1;
	unsigned char START_F2;
	unsigned char START_F3;
	unsigned char sensor_num;
	unsigned char heartbeat;
	unsigned char Iotid;
	sensor_data_frame  sensor1;
	sensor_data_frame  sensor2;
	sensor_data_frame  sensor3;
	unsigned char checksum;
}__attribute__((packed)) device_comm_frame;

typedef struct {
unsigned char START_F0;
	unsigned char START_F0;
	unsigned char START_F1;
	unsigned char START_F2;
	unsigned char START_F3;
	unsigned char sensor_num;
	unsigned char heartbeat;
	sensor_data_frame  sensor1;
	sensor_data_frame  sensor2;
	sensor_data_frame  sensor3;
}__attribute__((packed)) receive_comm_frame;

char const SERVER_IP[] = { "172.20.10.4" }; /* Ŀ��IP��ַ */
char const GATEWAY_IP[] = { "172.20.10.1" };
char const WIFI_SSID[] = { "zjd" };
char const WIFI_KEY[] = { "zhoujundi2" };
uint16_t SERVER_PORT = 23344;

char const LOCAL_IP[] = { "10.66.21.0" }; /* ����IP��ַ */

extern int buff_pos;

u8 rcv_buf_from_server[256];
sensor_data_frame sensor_data[3];
//recive_From_Server sensor_data[3];
/* Private variables ---------------------------------------------------------*/
EM380C_parm_TypeDef parm;
u32 version;

void setup_wifi(void)
{
	
	EMW3612_CMD;
	delay_ms(200);
	
	LED_RGB_Control(0, 0, 10);
	delay_ms(200);
	EM380C_Init(BaudRate_115200, WordLength_8b, StopBits_1, Parity_No,
			HardwareFlowControl_None, buffer_128bytes); //if you dont know EM380C's baudrate,you should use	EM380C_scan_Init(...)
	delay_ms(400);
	delay_ms(400);

	while (EM380C_Get_ver(&version) == EM380ERROR)
		;
//		ret = EM380C_Get_ver(&version);
	LED_RGB_Control(10, 0, 0);
	delay_ms(200);

	parm.wifi_mode = STATION;
	strcpy((char*) parm.wifi_ssid, WIFI_SSID);
	strcpy((char*) parm.wifi_wepkey, WIFI_KEY);
	parm.wifi_wepkeylen = strlen(WIFI_SSID);

	strcpy((char*) parm.local_ip_addr, LOCAL_IP);
	strcpy((char*) parm.remote_ip_addr, SERVER_IP);
	strcpy((char*) parm.gateway, GATEWAY_IP);
	strcpy((char*) parm.net_mask, "255.255.255.0");
	parm.portH = SERVER_PORT >> 8;
	parm.portL = SERVER_PORT & 0xff;
	parm.connect_mode = TCP_Client;
	parm.use_dhcp = DHCP_Enable;
	parm.use_udp = TCP_mode;
	parm.UART_buadrate = BaudRate_115200;
	parm.DMA_buffersize = buffer_64bytes;
	parm.use_CTS_RTS = HardwareFlowControl_None;
	parm.parity = Parity_No;
	parm.data_length = WordLength_8b;
	parm.stop_bits = StopBits_1;

	parm.io1 = None;
	strcpy((char*) parm.wpa_key, WIFI_KEY);
	parm.security_mode = Auto;

	while (EM380C_Set_Config(&parm) == EM380ERROR)
		;									 //EMSP API, set EM380C status
	delay_ms(400);
	LED_RGB_Control(0, 10, 0);

	while (EM380C_Startup() == EM380ERROR)
		;
	delay_ms(400);
	LED_RGB_Control(5, 5, 5);

//	EM380C_Reset();	
	delay_ms(400);	
	EMW3612_DAT;
	delay_ms(400);
}


void solveRcv(sensor_data_frame sensordate){
		if(sensordate.ID==1){
			//RGB
			int R=sensordate.V1;
			int G=sensordate.V2;
			int B=sensordate.V3;
			LED_RGB_Control(R,G,B);
		}
		else if(sensordate.ID==2){
			//温湿度
		
		}
		else if(sensordate.ID==3){
			//电机
			int status=sensordate.V1;
			Motor_status(status);
		}	
		else if(sensordate.ID==4){
			//红外线
		}
}

unsigned short calcheckcode(unsigned char *dat)
{
	int n;
	short chk = 0;

	for (n = 0; n < sizeof(device_comm_frame) - 2; n++) {
		chk += *dat;
		dat++;
	}
	return chk;
}
unsigned short calcheckcode2(unsigned char *dat)
{
	int n;
	short chk = 0;

	for (n = 0; n < sizeof(receive_comm_frame) - 2; n++) {
		chk += *dat;
		dat++;
	}
	return chk;
}
int process_rcv_data()
{
	int datalen;
	vs8 ret = 1;
	short mychecksum;
	//device_comm_frame *prcv_buf = (device_comm_frame *)rcv_buf_from_server;
	receive_comm_frame *prcv_buf = (receive_comm_frame *)rcv_buf_from_server;
   datalen = UART_receive_buf(rcv_buf_from_server);
	
	if (!((rcv_buf_from_server[0] == 0xa1) &&
		(rcv_buf_from_server[1] == 0xa2) &&
		(rcv_buf_from_server[2] == 0xa3) &&
		(rcv_buf_from_server[3] == 0xbb))){
		ret = 2;
		goto done;
	}
	mychecksum=prcv_buf->checksum;
	if (calcheckcode2(rcv_buf_from_server) != mychecksum) {	// check sum error
		ret = 3;
		goto done;
	}
	
	// data ok
	memcpy(sensor_data, (u8 *)&prcv_buf->sensor1, sizeof(sensor_data_frame) * (prcv_buf->sensor_num));
	ret = 1;

done:
	return ret;
}

void Hal_Init(void)
{
	UARTx_Init();
	RGB_LED_Init();
//	printf("UART2 Init OK\r\n");
}

int main(void)
{
	int cnt = 1;
	device_comm_frame tcpip_data;
	

	system_init();
	Hal_Init();
	setup_wifi();
	
	tcpip_data.START_F0 = (STARTFLAG >> 24) & 0xff;
	tcpip_data.START_F1 = (STARTFLAG >> 16) & 0xff;
	tcpip_data.START_F2 = (STARTFLAG >> 8) & 0xff;
	tcpip_data.START_F3 = (STARTFLAG >> 0) & 0xff;
	
	tcpip_data.sensor_num = 3;
		tcpip_data.Iotid=26;
		tcpip_data.sensor1.ID=1;
		tcpip_data.sensor2.ID=2;
		tcpip_data.sensor3.ID=3;
		tcpip_data.sensor1.V3=0;
		tcpip_data.sensor3.V2=0;
		tcpip_data.sensor3.V3=0;
	
	while (cnt++) {
	tcpip_data.heartbeat++;
		
		enable_irq();
		DHT11_Read_Data((u8 *)&v1,(u8*)&v2);
		
		LED_RGB_Control(R,G,B);
		
		tcpip_data.sensor1.V1=v1;
		tcpip_data.sensor1.V2=v2;
		
		tcpip_data.sensor2.V1=R;
		tcpip_data.sensor2.V2=G;
		tcpip_data.sensor2.V3=B;
		
		IR_Handle();
		if(bInfrared){
		tcpip_data.sensor3.V1=1;
		}
		else{
		tcpip_data.sensor3.V1=0;
		}
		//sensor_data.ID=1;
		//sensor_data.V1=1;
		//sensor_data.V2=2;
		//sensor_data.V3=3;
		tcpip_data.checksum = calcheckcode((unsigned char *) &tcpip_data);
		//EM380C_Send_Data((uint8_t *) &tcpip_data, sizeof(device_comm_frame));
		UART_send_buf((uint8_t *)&tcpip_data,sizeof(device_comm_frame));
		//UART_send_buf((uint8_t *)&sensor_data,sizeof(sensor_data_frame));
		
		
		delay_ms(200);
		if (process_rcv_data() == 1){	//OK	
				solveRcv(sensor_data[0]);
				solveRcv(sensor_data[1]);
				solveRcv(sensor_data[2]);
				//LED_RGB_Control(255, 0, 0);
		}
		else if(process_rcv_data() == 2){
			//头部校验出错
				LED_RGB_Control(0, 255, 0);
		}
		else if(process_rcv_data() == 3){
			//校验码出错
				LED_RGB_Control(0, 0, 255);
		}
		else{	//ERRORW
			;
		}
		
		//switch (cnt % 3) {
		//case 0:
		//	LED_RGB_Control(10, 0, 0);
		//	break;
		//case 1:
		//	LED_RGB_Control(0, 10, 0);
		//	break;
		//case 2:
		//	LED_RGB_Control(0, 0, 10);
		//	break;
		//default:
		//	break;
		//}
	}
}

/*-------------------------------END OF FILE-------------------------------*/

