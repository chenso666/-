/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include "string.h"
/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT.h"
#include "iocc2530.h"
#include "stdlib.h"
#include <stdio.h>
#include "hal_adc.h"
#include "delay.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
char temp1[]="";
char temp2[]="";
char TxBuf[5];

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint16 humData;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
extern void LCD_CLS(void);
extern void LCD_P16x16Ch(unsigned char x, unsigned char y, unsigned char N);
uint16 ReadSoilData( void );
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
uint16 ReadSoilData(void)
{
    uint16 reading = 0;
    
    /* 启用土壤湿度传感器对应的通道（P0_7） */
    ADCCFG |= 0x80;  // P0_7, 土壤湿度传感器对应的通道
    
    /* 写入寄存器，启动转换 */
    ADCCON3 = 0x86;  // AIN7 (P0_7) 64抽取率，7位ENOB
    
    /* 等待转换完成 */
    while (!(ADCCON1 & 0x80));
    
    /* 禁用通道 */
    ADCCFG &= (0x80 ^ 0xFF);  // 通过按位异或禁用
    
    /* 读取结果 */
    reading = ADCL;
    reading |= (int16)(ADCH << 8); 
    
    reading >>= 8;  // 右移8位，获得土壤湿度传感器的结果
    return reading;
}

void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  P1SEL &= ~0X03; //初始化LED  
  P1DIR |= 0X03;
  


 // P0SEL &= ~0xC0; //初始化P0_7和P0_6
 // P0DIR |= 0xC0;
  HalLcdInit();
  LCD_CLS();
  HalLcdWriteString("WELCOM to CC2530", 2);

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  MT_UartInit();  //串口初始化
  MT_UartRegisterTaskID(task_id); //利用语句App_TaskID = taskID将UART注册在应用层上，若发生串口事件，则
//在应用层上处理，在后面的第17步执行函数MT_UartProcessZToolData会用到

  
 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}


/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
            SampleApp_MessageMSGCB( MSGpkt );
         
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            
          }
          else
          {
            // Device is no longer in the network
          }
          break;
          
        case CMD_SERIAL_MSG: //串口收到数据后由 MT_UART 层传递过来的/数据，编译时不定
                               //义 MT相关内容
           SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
           break;
          
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {

  uint16 humData = ReadSoilData();
  float soilHumidity = ((255 - (float)humData) / 255) * 100;
  char humidityStr[32];
  sprintf(humidityStr, "%.2f", soilHumidity); 
  AF_DataRequest(&SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                 SAMPLEAPP_PERIODIC_CLUSTERID,
                 strlen(humidityStr),
                 (uint8*)humidityStr,
                 &SampleApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS);
    
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );
    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */

void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
   if ( keys & HAL_KEY_SW_6 )//S1
  {
    
    
    
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

  }
  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;
  char temp_str[11]; // 足够大的缓冲区存放温度字符串
  uint8 lens = pkt->cmd.DataLength;
  if (lens > 10) lens = 10; // 防止溢出
  memcpy(temp_str, pkt->cmd.Data, lens);
  temp_str[lens] = '\0'; // 添加字符串终止符
  float temp = atof(temp_str); // 转换
  
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      HalUARTWrite(0, "TEMP:", 5); 
      HalUARTWrite(0, pkt->cmd.Data,pkt->cmd.DataLength);
      HalUARTWrite(0,"\n",1 );
      if(temp >= 28.0f){
                 //驱动风扇
                 P0SEL &= ~0x30;  // 清空位4和位5
                 P0DIR |= 0x30;   // 设置为输出模式
                 P0_5=1;
                 P0_4=0;
              //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,2500 );
      }else if(temp < 28.0f){               
                 P0_5=1;
                 P0_4=1;
            // osal_stop_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
         
      }
      break;
      
    case SAMPLEAPP_FLASH_CLUSTERID:
                char light_str[10];
                 uint8 data_len = pkt->cmd.DataLength;
    // 确保不溢出缓冲区
    if (data_len >= sizeof(light_str)) {
        data_len = sizeof(light_str) - 1;
    }
    
    memcpy(light_str, pkt->cmd.Data, data_len);
    light_str[data_len] = '\0';  // 强制终止字符串
    
                HalUARTWrite(0, "LIGHT:", 6); 
                HalUARTWrite(0, light_str, strlen(light_str));
                HalUARTWrite(0, "\n", 1);
                int lightValue = atoi(light_str);  // 字符串转整数
                 
                static uint8 motorState = 255;  // 初始为未定义状态（防止第一次不动作）
                if (lightValue > 100&& motorState != 1) { 
                  P1SEL &= ~0x0C;  
                  P1DIR |= 0x0C;   
                  P1_3 = 1;        
                  P1_2 = 0;        
                  Delay_ms(5000);
                  P1_3 = 1;        
                  P1_2 = 1;  
                   HalLedOnOff (HAL_LED_1,HAL_LED_MODE_ON);
                   motorState = 1; 
                } else if(lightValue <= 100&& motorState != 0) {
                  P1_3 = 0;        
                  P1_2 = 1;        
                  Delay_ms(5000);
                  P1_3 = 1;        
                  P1_2 = 1;  
                  HalLedOnOff (HAL_LED_1,HAL_LED_MODE_OFF);
                  motorState = 0; 
                }
      break;
  }

}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  HalUARTWrite(0, (uint8 *)"PeriodicMsg\r\n", strlen("PeriodicMsg\r\n"));

  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
{
    uint8 i,len,*str=NULL; //len 用来存放实际数据的长度（单位为字节）
    str=cmdMsg->msg; //指向记录数据长度那个字节
    len=*str; //msg 里的第 1 个字节代表后面的数据长度
}

