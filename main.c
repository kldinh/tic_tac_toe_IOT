//*****************************************************************************
// Copyright (C) 2014 Texas Instruments Incorporated
//
// All rights reserved. Property of Texas Instruments Incorporated.
// Restricted rights to use, duplicate or disclose this code are
// granted through contract.
// The program may not be used without the written permission of
// Texas Instruments Incorporated or against the terms and conditions
// stipulated in the agreement under which this program has been supplied,
// and under no circumstances can it be used with non-TI connectivity device.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     -   MQTT Client
// Application Overview -   This application acts as a MQTT client and connects
//                          to the IBM MQTT broker, simultaneously we can
//                          connect a web client from a web browser. Both
//                          clients can inter-communicate using appropriate
//                          topic names.
//
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_MQTT_Client
// or
// docs\examples\CC32xx_MQTT_Client.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup mqtt_client
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdlib.h>

// simplelink includes
#include "simplelink.h"

// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "rom_map.h"
#include "prcm.h"
#include "uart.h"
#include "timer.h"
#include <stdio.h>
#include <string.h>


// common interface includes
#include "network_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "button_if.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "common.h"
#include "utils.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

#include "sl_mqtt_client.h"

#include "hw_common_reg.h"
#include "spi.h"
#include "rom.h"
#include "pin_mux_config.h"
#include "glcdfont.h"
#include "gpio.h"
#include "systick.h"

// application specific includes



#define APPLICATION_VERSION 	"1.1.1"

/*Operate Lib in MQTT 3.1 mode.*/
#define MQTT_3_1_1              false /*MQTT 3.1.1 */
#define MQTT_3_1                true /*MQTT 3.1*/

#define WILL_TOPIC              "Client"
#define WILL_MSG                "Client Stopped"
#define WILL_QOS                QOS1
#define WILL_RETAIN             false

/*Defining Broker IP address and port Number*/
//#define SERVER_ADDRESS          "messagesight.demos.ibm.com"
// Enter your AWS Endpoint address as the SERVER ADDRESS
#define SERVER_ADDRESS          "A2JG3817MNXU4K.iot.us-west-2.amazonaws.com"
//#define PORT_NUMBER             1883
#define PORT_NUMBER             8883

#define MAX_BROKER_CONN         1

#define SERVER_MODE             MQTT_3_1_1
/*Specifying Receive time out for the Receive task*//*Specifying Receive time out for the Receive task*/
#define RCV_TIMEOUT             30

/*Background receive task priority*/
#define TASK_PRIORITY           3

/* Keep Alive Timer value*/
#define KEEP_ALIVE_TIMER        25

/*Clean session flag*/
#define CLEAN_SESSION           true

/*Retain Flag. Used in publish message. */
#define RETAIN                  0

/*Defining Publish Topic*/
#define PUB_TOPIC_FOR_SW3       "/cc3200/ButtonPressEvtSw3"
#define PUB_TOPIC_FOR_SW2       "/cc3200/ButtonPressEvtSw2"
#define PUB_TOPIC				"/cc3200/WriteString2"

/*Defining Number of topics*/
#define TOPIC_COUNT             3

/*Defining Subscription Topic Values*/
#define TOPIC1                  "/cc3200/ToggleLEDCmdL1"
#define TOPIC2                  "/cc3200/ToggleLEDCmdL2"
#define TOPIC3                  "/cc3200/ToggleLEDCmdL3"
#define TOPIC4					"/cc3200/WriteString1"


/*Defining QOS levels*/
#define QOS0                    0
#define QOS1                    1
#define QOS2                    2

/*Spawn task priority and OSI Stack Size*/
#define OSI_STACK_SIZE          2048
#define UART_PRINT              Report

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                10   /* Current Date */
#define MONTH               03     /* Month 1-12 */
#define YEAR                2016  /* Current year */
#define HOUR                18    /* Time - hours */
#define MINUTE              0    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define	BLACK           0x0000
#define	BLUE            0x001F
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define	RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

#define SPI_IF_BIT_RATE  100000

#define PLAYER 'X'
#define OPPONENT 'O'

int time; //used for the waiting period inbetween T9 texting
extern int cursor_x; //for led screen placement
extern int cursor_y; //for led screen placement
char input; // only used in lab 2
char input2;// gave our UART connection the char character
char* cString;// part of lab 2
int g_iCounter;// part of lab 2
int iStringLength;// part of lab 2
int MAX_STRING_LENGTH;// part of lab 2
unsigned char test;//not used
int i = 4; //for coordinates of LED for DrawChar
int j = 4; //for coordinates of LED for DrawChar
int k = 6;//for coordinates of LED for DrawChar
int flag;//to know when got into the interrupt
int a; //used as a counter
int c;//used as a counter
int d = 6; //for coordinates of LED for DrawChar
int g = 0; //for coordinates of LED for DrawChar
int t; //used as a counter
int o; //used as a counter
int l; //used as a counter
int r; //used as a counter

int n = 75;
int m = 6;

int new_button_flag = 0; //used to see if a new button was pressed
int previous_button; //remembers what the preivous button pressed was
int prev_flag = 0;
int space_flag = 0;


char letter; //stores the letter you want to text into "letter"
int text; //cycles through the 3 or 4 letters on each button
int new_flag;//see if a button was pressed
int button;//to tell which button was pressed
int my_turn; // is 1 when it is the player's turn
char winner; // gives winner when a row is matched; reset board and change score
char recv_char; // character received over MQTT
char recv_flag = 0; // indicates a character has been received over MQTT

int my_score = 0;
int their_score = 0;


int start_read = 0; //used for the correct systick value when overlaps
unsigned long diff; //gets the difference between systick clock
unsigned long tick_arr [16];
unsigned long counter, curr_tick, prev_tick = 0; //used to find values of systick
int count = 0; //used for count
int button_flag;//to tell when to draw the text
unsigned long final[16]; //what we give into the UART
int button_pressed;


//the signal values of each button
unsigned long one[16] = {2,2,1,1,1,1,1,1,2,1,1,1,1,2,1,1};
unsigned long two[16] = {2,2,1,1,1,1,1,1,1,2,1,1,1,2,1,1};
unsigned long three[16] = {2,2,1,1,1,1,1,1,2,2,1,1,1,2,1,1};
unsigned long four[16] = {2,2,1,1,1,1,1,1,1,1,2,1,1,2,1,1};
unsigned long five[16] = {2,2,1,1,1,1,1,1,2,1,2,1,1,2,1,1};
unsigned long six[16] = {2,2,1,1,1,1,1,1,1,2,2,1,1,2,1,1};
unsigned long seven[16] = {2,2,1,1,1,1,1,1,2,2,2,1,1,2,1,1};
unsigned long eight[16] = {2,2,1,1,1,1,1,1,1,1,1,2,1,2,1,1};
unsigned long nine[16] = {2,2,1,1,1,1,1,1,2,1,1,2,1,2,1,1};
unsigned long zero[16] = {2,2,1,1,1,1,1,1,1,1,1,1,1,2,1,1};
unsigned long right[16] = {2,2,1,1,1,1,1,1,1,2,2,2,2,1,1,1};
unsigned long left[16] = {2,2,1,1,1,1,1,1,2,2,2,2,2,1,1,1};
unsigned long enter[16] = {2,2,1,1,1,1,1,1,1,2,1,2,1,1,1,1};


char print[30];//tells LED what to print
char receive[30];//the values UART recives

float p = 3.1415926;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;

typedef struct connection_config{
    SlMqttClientCtxCfg_t broker_config;
    void *clt_ctx;
    unsigned char *client_id;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    int num_topics;
    char *topic[TOPIC_COUNT];
    unsigned char qos[TOPIC_COUNT];
    SlMqttWill_t will_params;
    bool is_connected;
}connect_config;

typedef enum events
{
    PUSH_BUTTON_SW2_PRESSED,
    PUSH_BUTTON_SW3_PRESSED,
    BROKER_DISCONNECTION
}osi_messages;

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
          long pay_len, bool dup,unsigned char qos, bool retain);
static void sl_MqttEvt(void *app_hndl,long evt, const void *buf,
                       unsigned long len);
static void sl_MqttDisconnect(void *app_hndl);
void pushButtonInterruptHandler2();
void pushButtonInterruptHandler3();
void ToggleLedState(ledEnum LedNum);
void TimerPeriodicIntHandler(void);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
void BoardInit(void);
static void DisplayBanner(char * AppName);
void MqttClient(void *pvParameters);

static int set_time();
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#ifdef USE_FREERTOS
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#endif

char *security_file_list[] = {"/cert/private.der", "/cert/client.der", "/cert/rootCA.der"};  //Order: Private Key, Certificate File, CA File, DH Key (N/A)
SlDateTime g_time;

unsigned short g_usTimerInts;
/* AP Security Parameters */
SlSecParams_t SecurityParams = {0};

/*Message Queue*/
OsiMsgQ_t g_PBQueue;

/* connection configuration */
connect_config usr_connect_config[] =
{
    {
        {
            {
                (SL_MQTT_NETCONN_URL|SL_MQTT_NETCONN_SEC),
                SERVER_ADDRESS,
                PORT_NUMBER,
				SL_SO_SEC_METHOD_TLSV1_2,	//Method (TLS1.2)
				SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA,	//Cipher
				3,							//number of files
				security_file_list 			//name of files
            },
            SERVER_MODE,
            true,
        },
        NULL,
        "kevinasdf",
        NULL,
        NULL,
        true,
        KEEP_ALIVE_TIMER,
        {Mqtt_Recv, sl_MqttEvt, sl_MqttDisconnect},
        TOPIC_COUNT,
        {TOPIC1, TOPIC2, TOPIC4},
        {QOS1, QOS1, QOS1},
        /*{WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},*/
		NULL,
        false
    }
};

/* library configuration */
SlMqttClientLibCfg_t Mqtt_Client={
    1882,
    TASK_PRIORITY,
    30,
    true,
    (long(*)(const char *, ...))UART_PRINT
};

/*Publishing topics and messages*/
const char *pub_topic_sw2 = PUB_TOPIC_FOR_SW2;
const char *pub_topic_sw3 = PUB_TOPIC_FOR_SW3;
const char *pub_topic = PUB_TOPIC;
unsigned char *data_sw2={"Push button sw2 is pressed on CC32XX device"};
unsigned char *data_sw3={"Push button sw3 is pressed on CC32XX device"};

void *app_hndl = (void*)usr_connect_config;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//! Defines Mqtt_Pub_Message_Receive event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives a Publish Message from the broker.
//!
//!\param[out]     topstr => pointer to topic of the message
//!\param[out]     top_len => topic length
//!\param[out]     payload => pointer to payload
//!\param[out]     pay_len => payload length
//!\param[out]     retain => Tells whether its a Retained message or not
//!\param[out]     dup => Tells whether its a duplicate message or not
//!\param[out]     qos => Tells the Qos level
//!
//!\return none
//****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
                       long pay_len, bool dup,unsigned char qos, bool retain)
{
    
    char *output_str=(char*)malloc(top_len+1);
    memset(output_str,'\0',top_len+1);
    strncpy(output_str, (char*)topstr, top_len);
    output_str[top_len]='\0';
    int write_flag = 0;


    if(strncmp(output_str,TOPIC1, top_len) == 0)
    {
        ToggleLedState(LED1);
    }
    else if(strncmp(output_str,TOPIC2, top_len) == 0)
    {
        ToggleLedState(LED2);
    }
    else if(strncmp(output_str,TOPIC3, top_len) == 0)
    {
        ToggleLedState(LED3);
    }
    else if(strncmp(output_str,TOPIC4, top_len) == 0)
    {
       	write_flag = 1;
    }

    UART_PRINT("\n\rPublish Message Received");
    UART_PRINT("\n\rTopic: ");
    UART_PRINT("%s",output_str);
    free(output_str);
    UART_PRINT(" [Qos: %d] ",qos);
    if(retain)
      UART_PRINT(" [Retained]");
    if(dup)
      UART_PRINT(" [Duplicate]");
    
    output_str=(char*)malloc(pay_len+1);
    memset(output_str,'\0',pay_len+1);
    strncpy(output_str, (char*)payload, pay_len);
    output_str[pay_len]='\0';
    UART_PRINT("\n\rData is: ");
    UART_PRINT("%s",(char*)output_str);
    UART_PRINT("\n\r");

    if(write_flag == 1)
            {
        		recv_char = output_str[0];
        		recv_flag = 1;
        	/*    		for(i = 0; i < strlen(output_str); i++)
        		{
        			if(m>115)
        				{
        					m = 0;
        					n = n + 10;
        				}

        			drawChar(m, n, output_str[i], WHITE, BLACK, 1);

        			m = m + 6;
        		}
      */
        		write_flag = 0;
            }


    free(output_str);
    
    return;
}

static void SystickIntHandler()
{



}

//****************************************************************************
//! Defines sl_MqttEvt event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives an ack(whenever user is in non-blocking mode) or encounters an error.
//!
//! param[out]      evt => Event that invokes the handler. Event can be of the
//!                        following types:
//!                        MQTT_ACK - Ack Received 
//!                        MQTT_ERROR - unknown error
//!                        
//!  
//! \param[out]     buf => points to buffer
//! \param[out]     len => buffer length
//!       
//! \return none
//****************************************************************************
static void
sl_MqttEvt(void *app_hndl, long evt, const void *buf,unsigned long len)
{
    int i;
    switch(evt)
    {
      case SL_MQTT_CL_EVT_PUBACK:
        UART_PRINT("PubAck:\n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      case SL_MQTT_CL_EVT_SUBACK:
        UART_PRINT("\n\rGranted QoS Levels are:\n\r");
        
        for(i=0;i<len;i++)
        {
          UART_PRINT("QoS %d\n\r",((unsigned char*)buf)[i]);
        }
        break;
        
      case SL_MQTT_CL_EVT_UNSUBACK:
        UART_PRINT("UnSub Ack \n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      default:
        break;
  
    }
}

//****************************************************************************
//
//! callback event in case of MQTT disconnection
//!
//! \param app_hndl is the handle for the disconnected connection
//!
//! return none
//
//****************************************************************************
static void
sl_MqttDisconnect(void *app_hndl)
{
    connect_config *local_con_conf;
    osi_messages var = BROKER_DISCONNECTION;
    local_con_conf = app_hndl;
    sl_ExtLib_MqttClientUnsub(local_con_conf->clt_ctx, local_con_conf->topic,
                              TOPIC_COUNT);
    UART_PRINT("disconnect from broker %s\r\n",
           (local_con_conf->broker_config).server_info.server_addr);
    local_con_conf->is_connected = false;
    sl_ExtLib_MqttClientCtxDelete(local_con_conf->clt_ctx);

    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&var,OSI_NO_WAIT);

}

//****************************************************************************
//
//! Push Button Handler1(GPIOS2). Press push button2 (GPIOSW2) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//!    event publish messages
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void pushButtonInterruptHandler2()
{
    osi_messages var = PUSH_BUTTON_SW2_PRESSED;
    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&var,OSI_NO_WAIT);
}

//****************************************************************************
//
//! Push Button Handler3(GPIOS3). Press push button3 (GPIOSW3) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//!    event publish messages
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void pushButtonInterruptHandler3()
{
    osi_messages var = PUSH_BUTTON_SW3_PRESSED;
    //
    // write message indicating exit from sending loop
    //
    osi_MsgQWrite(&g_PBQueue,&var,OSI_NO_WAIT);

}

//****************************************************************************
//
//!    Toggles the state of GPIOs(LEDs)
//!
//! \param LedNum is the enumeration for the GPIO to be toggled
//!
//!    \return none
//
//****************************************************************************
void ToggleLedState(ledEnum LedNum)
{
    unsigned char ledstate = 0;
    switch(LedNum)
    {
    case LED1:
        ledstate = GPIO_IF_LedStatus(MCU_RED_LED_GPIO);
        if(!ledstate)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        }
        else
        {
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
        }
        break;
    case LED2:
        ledstate = GPIO_IF_LedStatus(MCU_ORANGE_LED_GPIO);
        if(!ledstate)
        {
            GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
        }
        else
        {
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
        }
        break;
    case LED3:
        ledstate = GPIO_IF_LedStatus(MCU_GREEN_LED_GPIO);
        if(!ledstate)
        {
            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        }
        else
        {
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
        }
        break;
    default:
        break;
    }
}

//*****************************************************************************
//
//! Periodic Timer Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void
TimerPeriodicIntHandler(void)
{
    unsigned long ulInts;

    //
    // Clear all pending interrupts from the timer we are
    // currently using.
    //
    ulInts = MAP_TimerIntStatus(TIMERA0_BASE, true);
    MAP_TimerIntClear(TIMERA0_BASE, ulInts);

    //
    // Increment our interrupt counter.
    //
    g_usTimerInts++;
    if(!(g_usTimerInts & 0x1))
    {
        //
        // Off Led
        //
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    }
    else
    {
        //
        // On Led
        //
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    }
}

//****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void LedTimerConfigNStart()
{
    //
    // Configure Timer for blinking the LED for IP acquisition
    //
    Timer_IF_Init(PRCM_TIMERA0,TIMERA0_BASE,TIMER_CFG_PERIODIC,TIMER_A,0);
    Timer_IF_IntSetup(TIMERA0_BASE,TIMER_A,TimerPeriodicIntHandler);
    Timer_IF_Start(TIMERA0_BASE,TIMER_A,100);
}

//****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void LedTimerDeinitStop()
{
    //
    // Disable the LED blinking Timer as Device is connected to AP
    //
    Timer_IF_Stop(TIMERA0_BASE,TIMER_A);
    Timer_IF_DeInit(TIMERA0_BASE,TIMER_A);

}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
    //
    // Set vector table base
    //
    #if defined(ccs)
        IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}
  
//*****************************************************************************
//
//! Task implementing MQTT client communication to other web client through
//!    a broker
//!
//! \param  none
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt library and set up MQTT connection configurations
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \return None
//!
//*****************************************************************************
void MqttClient(void *pvParameters)
{
    
    long lRetVal = -1;
    int iCount = 0;
    int iNumBroker = 0;
    int iConnBroker = 0;
    osi_messages RecvQue;
    
    connect_config *local_con_conf = (connect_config *)app_hndl;

    //
    // Configure LED
    //
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    //
    // Reset The state of the machine
    //
    Network_IF_ResetMCUStateMachine();

    //
    // Start the driver
    //
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
       UART_PRINT("Failed to start SimpleLink Device\n\r",lRetVal);
       LOOP_FOREVER();
    }

    // switch on Green LED to indicate Simplelink is properly up
    GPIO_IF_LedOn(MCU_ON_IND);

    // Start Timer to blink Red LED till AP connection
    LedTimerConfigNStart();

    // Initialize AP security params
    SecurityParams.Key = (signed char *)SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    //
    // Connect to the Access Point
    //
    lRetVal = Network_IF_ConnectAP(SSID_NAME, SecurityParams);
    if(lRetVal < 0)
    {
       UART_PRINT("Connection to an AP failed\n\r");
       LOOP_FOREVER();
    }

    //
    // Disable the LED blinking Timer as Device is connected to AP
    //
    LedTimerDeinitStop();

    //
    // Switch ON RED LED to indicate that Device acquired an IP
    //
    GPIO_IF_LedOn(MCU_IP_ALLOC_IND);

    set_time();
     if(lRetVal < 0)
         {
             UART_PRINT("Unable to set time in the device");
     }

    UtilsDelay(20000000);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
   
    //
    // Register Push Button Handlers
    //
    Button_IF_Init(pushButtonInterruptHandler2,pushButtonInterruptHandler3);
    
    //
    // Initialze MQTT client lib
    //
    lRetVal = sl_ExtLib_MqttClientInit(&Mqtt_Client);
    if(lRetVal != 0)
    {
        // lib initialization failed
        UART_PRINT("MQTT Client lib initialization failed\n\r");
        LOOP_FOREVER();
    }
    
    /******************* connection to the broker ***************************/
    iNumBroker = sizeof(usr_connect_config)/sizeof(connect_config);
    if(iNumBroker > MAX_BROKER_CONN)
    {
        UART_PRINT("Num of brokers are more then max num of brokers\n\r");
        LOOP_FOREVER();
    }

    while(iCount < iNumBroker)
    {
        //create client context
        local_con_conf[iCount].clt_ctx =
        sl_ExtLib_MqttClientCtxCreate(&local_con_conf[iCount].broker_config,
                                      &local_con_conf[iCount].CallBAcks,
                                      &(local_con_conf[iCount]));

        //
        // Set Client ID
        //
        sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                            SL_MQTT_PARAM_CLIENT_ID,
                            local_con_conf[iCount].client_id,
                            strlen((char*)(local_con_conf[iCount].client_id)));

        //
        // Set will Params
        //
        if(local_con_conf[iCount].will_params.will_topic != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                    SL_MQTT_PARAM_WILL_PARAM,
                                    &(local_con_conf[iCount].will_params),
                                    sizeof(SlMqttWill_t));
        }

        //
        // setting username and password
        //
        if(local_con_conf[iCount].usr_name != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_USER_NAME,
                                local_con_conf[iCount].usr_name,
                                strlen((char*)local_con_conf[iCount].usr_name));

            if(local_con_conf[iCount].usr_pwd != NULL)
            {
                sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_PASS_WORD,
                                local_con_conf[iCount].usr_pwd,
                                strlen((char*)local_con_conf[iCount].usr_pwd));
            }
        }

        //
        // connectin to the broker
        //
        if((sl_ExtLib_MqttClientConnect((void*)local_con_conf[iCount].clt_ctx,
                            local_con_conf[iCount].is_clean,
                            local_con_conf[iCount].keep_alive_time) & 0xFF) != 0)
        {
            UART_PRINT("\n\rBroker connect fail for conn no. %d \n\r",iCount+1);
            
            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            
            break;
        }
        else
        {
            UART_PRINT("\n\rSuccess: conn to Broker no. %d\n\r ", iCount+1);
            local_con_conf[iCount].is_connected = true;
            iConnBroker++;
        }

        //
        // Subscribe to topics
        //

        if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
                                   local_con_conf[iCount].topic,
                                   local_con_conf[iCount].qos, TOPIC_COUNT) < 0)
        {
            UART_PRINT("\n\r Subscription Error for conn no. %d\n\r", iCount+1);
            UART_PRINT("Disconnecting from the broker\r\n");
            sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
            local_con_conf[iCount].is_connected = false;
            
            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            iConnBroker--;
            break;
        }
        else
        {
            int iSub;
            UART_PRINT("Client subscribed on following topics:\n\r");
            for(iSub = 0; iSub < local_con_conf[iCount].num_topics; iSub++)
            {
                UART_PRINT("%s\n\r", local_con_conf[iCount].topic[iSub]);
            }
        }
        iCount++;
    }

    if(iConnBroker < 1)
    {
        //
        // no succesful connection to broker
        //
        goto end;
    }

    drawFastHLine(10, 60, 115, YELLOW);
       drawFastHLine(10, 93, 115, YELLOW); //draws line in middle to tell printing out what UART recieved

       drawFastVLine(45, 30, 115, YELLOW);
       drawFastVLine(90, 30, 115, YELLOW);
       char* tictac = "Tic-Tac-Toe";

    		int z = 30;
    		int b;
       for(b = 0; b < 11; b++)
       {
    	drawChar(z,6, tictac[b], WHITE, BLACK, 1);
    	z = z + 6;
       }

    iCount = 0;

    count = 0;
    	prev_tick = 0;
    	new_button_flag = 0;
    	winner = 0;
    	button_pressed = 0;
    	if (PLAYER == 'X')
    		my_turn = 1;
    	else
    		my_turn = 0;

    SysTickPeriodSet(16777216);        //setting systick beggining values to 16777216
       SysTickIntRegister(SystickIntHandler);         //registering the interrupt
       SysTickEnable();//enabling systick

       int x, y;
              char box [3][3];
              for(x = 0; x < 3; x++)
           	   for(y = 0; y < 3; y++)
           		   box[x][y] = 0;

           char num_char;
           char *send_char = (char*)malloc(1);
           int turns = 0;

           drawChar(0,6, PLAYER, GREEN, BLACK, 1);
           drawChar(5,6, ':', GREEN, BLACK, 1);
           drawChar(10,6, '0', GREEN, BLACK, 1);
           drawChar(16,6, '0', GREEN, BLACK, 1);
           drawChar(100,6, OPPONENT, RED, BLACK, 1);
           drawChar(105,6, ':', RED, BLACK, 1);
           drawChar(110,6, '0', RED, BLACK, 1);
           drawChar(116,6, '0', RED, BLACK, 1);



               for(;;)
            {
            	//printf("button_flag: %d\n", button_flag);
        			if (winner != 0) // winner found
        			{
        				if (button_flag == 1) // wait for enter button to reset board
        				{
        					button_flag = 0;
        					for(k = 0; k < 16; k++)
        					{
        						if(tick_arr[k] < 90000)
        								final[k] = 1;
        						else
        							final[k] = 2;

        						//printf("%lu ", tick_arr[k]);
        						//printf("%lu ", final[k]);

        					}
        					if(memcmp(final, enter, sizeof(final)) == 0)
        						{
        							for(x = 0; x < 3; x++)
        							   for(y = 0; y < 3; y++)
        								   box[x][y] = 0;

        							drawChar(20, 35, ' ', WHITE, BLACK, 3);
        							drawChar(60, 35, ' ', WHITE, BLACK, 3);
        							drawChar(100, 35, ' ', WHITE, BLACK, 3);
        							drawChar(20, 67, ' ', WHITE, BLACK, 3);
        							drawChar(60, 67, ' ', WHITE, BLACK, 3);
        							drawChar(100, 67, ' ', WHITE, BLACK, 3);
        							drawChar(20, 100, ' ', WHITE, BLACK, 3);
        							drawChar(60, 100, ' ', WHITE, BLACK, 3);
        							drawChar(100, 100, ' ', WHITE, BLACK, 3);

        							if (winner == PLAYER)
        								my_score++;
        							else if (winner == OPPONENT)
        								their_score++;
        						   drawChar(10,6, ('0' + my_score/10), GREEN, BLACK, 1);
        						   drawChar(16,6, ('0' + my_score%10), GREEN, BLACK, 1);
        						   drawChar(110,6, ('0' + their_score/10), RED, BLACK, 1);
        						   drawChar(116,6, ('0' + their_score%10), RED, BLACK, 1);

        /*
        							fillScreen(BLACK);
        							z = 30;
        							for(b = 0; b < 11; b++)
        							{
        								drawChar(z,6, tictac[b], WHITE, BLACK, 1);
        								z = z + 6;
        							}
        							drawFastHLine(10, 60, 115, YELLOW);
        							drawFastHLine(10, 93, 115, YELLOW); //draws line in middle to tell printing out what UART recieved
        							drawFastVLine(45, 30, 115, YELLOW);
        							drawFastVLine(90, 30, 115, YELLOW);
        */
        							if (PLAYER == 'X')
        								my_turn = 1;
        							else
        								my_turn = 0;
        							turns = 0;
        							winner = 0;
        						}
        				}
        			}
        			else if (my_turn)
        			{
        				//when a button is pressed, store the signals into final array
        				// compare values in final array to set array values to see what button pressed
        				//tell next function which button was pressed
        				if (button_flag == 1)
        				{

        					button_flag = 0;
        				//	drawChar(10, 10, 'l', WHITE, BLACK, 5);
        					//for (k = 0; k < 16; k++)
        						//printf("%lu, ", tick_arr[k]);
        					for(k = 0; k < 16; k++)
        					{
        						if(tick_arr[k] < 90000)
        								final[k] = 1;
        						else
        							final[k] = 2;

        						//printf("%lu ", tick_arr[k]);
        						//printf("%lu ", final[k]);

        					}


        					if(memcmp(final, one, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '1' \n");
        						if (box[0][0] == 0)
        						{
        							drawChar(20, 35, PLAYER, WHITE, BLACK, 3);
        							box[0][0] = PLAYER;
        							button_pressed = 1;
        							*send_char = '1';
        							sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        							                    pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, two, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '2' \n");
        						if (box[0][1] == 0)
        						{
        							box[0][1] = PLAYER;
        							drawChar(60, 35, PLAYER, WHITE, BLACK, 3);
        							button_pressed = 1;
        							*send_char = '2';
        							sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        							                    pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, three, sizeof(final)) == 0)
        					{
        						 printf("Pressed button: '3' \n");
        						if (box[0][2] == 0)
        						{
        							 box[0][2] = PLAYER;
        							 drawChar(100, 35, PLAYER, WHITE, BLACK, 3);
        							 button_pressed = 1;
        							 *send_char = '3';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        									 	 	   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, four, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '4' \n");
        						if (box[1][0] == 0)
        						{
        							box[1][0] = PLAYER;
        							drawChar(20, 67, PLAYER, WHITE, BLACK, 3);
        							button_pressed = 1;
        							*send_char = '4';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        											   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, five, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '5' \n");
        						if (box[1][1] == 0)
        						{
        							box[1][1] = PLAYER;
        							drawChar(60, 67, PLAYER, WHITE, BLACK, 3);
        							button_pressed = 1;
        							*send_char = '5';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        									 	 	   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, six, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '6' \n");
        						if (box[1][2] == 0)
        						{
        							box[1][2] = PLAYER;
        							drawChar(100, 67, PLAYER, WHITE, BLACK, 3);
        							button_pressed = 1;
        							*send_char = '6';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        									 	 	   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, seven, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '7' \n");
        						if (box[2][0] == 0)
        						{
        							 box[2][0] = PLAYER;
        							drawChar(20, 100, PLAYER, WHITE, BLACK, 3);
        							button_pressed = 1;
        							*send_char = '7';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        									 	 	   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, eight, sizeof(final)) == 0)
        					{
        						printf("Pressed button: '8' \n");
        						if (box[2][1] == 0)
        						{
        							 box[2][1] = PLAYER;
        							drawChar(60, 100, PLAYER, WHITE, BLACK, 3);
        							button_pressed = 1;
        							*send_char = '8';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        									 	 	   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, nine, sizeof(final)) == 0)
        					{
        					   printf("Pressed button: '9' \n");
        						if (box[2][2] == 0)
        						{
        						   box[2][2] = PLAYER;
        						   drawChar(100, 100, PLAYER, WHITE, BLACK, 3);
        						   button_pressed = 1;
        						   *send_char = '9';
        							 sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
        									 	 	   pub_topic,send_char,strlen((char*)send_char),QOS1,RETAIN);
        						}
        					}
        					if(memcmp(final, zero, sizeof(final)) == 0)
        					{
        					   printf("Pressed button: '0' \n");
        					   new_flag = 1;
        					   button = 0;
        					}
        					if(memcmp(final, right, sizeof(final)) == 0)
        					{
        						printf("Pressed button: 'Right' \n");
        						new_flag = 1;
        						button = 10;

        					}
        					if(memcmp(final, left, sizeof(final)) == 0)
        					{
        					   printf("Pressed button: 'Left' \n");
        					   new_flag = 1;
        					   button = 20;
        					}

        					if(memcmp(final, enter, sizeof(final)) == 0)
        					{
        					   printf("Pressed button: 'Enter' \n");
        					   new_flag = 1;
        					   button = 21;
        					}

        					printf("\n");
        					//fillScreen(BLACK);
        				}
        			}
        			else // opponent's turn
        			{
        				if (recv_flag == 1)
        				{
        					switch (recv_char) {
        						case '1':
        							box[0][0] = OPPONENT;
        							drawChar(20, 35, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '2':
        							box[0][1] = OPPONENT;
        							drawChar(60, 35, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '3':
        							box[0][2] = OPPONENT;
        							drawChar(100, 35, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '4':
        							box[1][0] = OPPONENT;
        							drawChar(20, 67, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '5':
        							box[1][1] = OPPONENT;
        							drawChar(60, 67, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '6':
        							box[1][2] = OPPONENT;
        							drawChar(100, 67, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '7':
        							box[2][0] = OPPONENT;
        							drawChar(20, 100, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '8':
        							box[2][1] = OPPONENT;
        							drawChar(60, 100, OPPONENT, WHITE, BLACK, 3);
        						break;
        						case '9':
        							box[2][2] = OPPONENT;
        							drawChar(100, 100, OPPONENT, WHITE, BLACK, 3);
        						break;
        					}
        					button_pressed = 1;
        					recv_flag = 0;
        				}
        			}

        			if (button_pressed)
        			{

        				button_pressed = 0;
        				// check horizontal
        				if((box[0][0] == PLAYER && box[0][1] == PLAYER && box[0][2] == PLAYER) ||
        						(box[0][0] == OPPONENT && box[0][1] == OPPONENT && box[0][2] == OPPONENT))
        				{
        					drawChar(20, 35, box[0][0], GREEN, BLACK, 3);
        					drawChar(60, 35, box[0][0], GREEN, BLACK, 3);
        					drawChar(100, 35, box[0][0], GREEN, BLACK, 3);
        					winner = box[0][0];
        				}

        				if((box[1][0] == PLAYER && box[1][1] == PLAYER && box[1][2] == PLAYER) ||
        						(box[1][0] == OPPONENT && box[1][1] == OPPONENT && box[1][2] == OPPONENT))
        				{
        					drawChar(20, 67, box[1][0], GREEN, BLACK, 3);
        					drawChar(60, 67, box[1][0], GREEN, BLACK, 3);
        					drawChar(100, 67, box[1][0], GREEN, BLACK, 3);
        					winner = box[1][0];
        				}

        				if((box[2][0] == PLAYER && box[2][1] == PLAYER && box[2][2] == PLAYER) ||
        						(box[2][0] == OPPONENT && box[2][1] == OPPONENT && box[2][2] == OPPONENT))
        				{
        					drawChar(20, 100, box[2][0], GREEN, BLACK, 3);
        					drawChar(60, 100, box[2][0], GREEN, BLACK, 3);
        					drawChar(100, 100, box[2][0], GREEN, BLACK, 3);
        					winner = box[2][0];
        				}
        				// check vertical
        				if((box[0][0] == PLAYER && box[1][0] == PLAYER && box[2][0] == PLAYER) ||
        						(box[0][0] == OPPONENT && box[1][0] == OPPONENT && box[2][0] == OPPONENT))
        				{
        					drawChar(20, 35, box[0][0], GREEN, BLACK, 3);
        					drawChar(20, 67, box[0][0], GREEN, BLACK, 3);
        					drawChar(20, 100, box[0][0], GREEN, BLACK, 3);
        					winner = box[0][0];
        				}

        				if((box[0][1] == PLAYER && box[1][1] == PLAYER && box[2][1] == PLAYER) ||
        						(box[0][1] == OPPONENT && box[1][1] == OPPONENT && box[2][1] == OPPONENT))
        				{
        					drawChar(60, 35, box[0][1], GREEN, BLACK, 3);
        					drawChar(60, 67, box[0][1], GREEN, BLACK, 3);
        					drawChar(60, 100, box[0][1], GREEN, BLACK, 3);
        					winner = box[0][1];
        				}

        				if((box[0][2] == PLAYER && box[1][2] == PLAYER && box[2][2] == PLAYER) ||
        						(box[0][2] == OPPONENT && box[1][2] == OPPONENT && box[2][2] == OPPONENT))
        				{
        					drawChar(100, 35, box[0][2], GREEN, BLACK, 3);
        					drawChar(100, 67, box[0][2], GREEN, BLACK, 3);
        					drawChar(100, 100, box[0][2], GREEN, BLACK, 3);
        					winner = box[0][2];
        				}
        				// check diagonals
        				if((box[0][0] == PLAYER && box[1][1] ==PLAYER &&  box[2][2] == PLAYER) ||
        						(box[0][0] == OPPONENT && box[1][1] == OPPONENT && box[2][2] == OPPONENT))
        				{
        					drawChar(20, 35, box[0][0], GREEN, BLACK, 3);
        					drawChar(60, 67, box[0][0], GREEN, BLACK, 3);
        					drawChar(100, 100, box[0][0], GREEN, BLACK, 3);
        					winner = box[0][0];
        				}

        				if((box[0][2] == PLAYER && box[1][1] == PLAYER && box[2][0] == PLAYER) ||
        						(box[0][2] == OPPONENT && box[1][1] == OPPONENT && box[2][0] == OPPONENT))
        				{
        					drawChar(100, 35, box[0][2], GREEN, BLACK, 3);
        					drawChar(60, 67, box[0][2], GREEN, BLACK, 3);
        					drawChar(20, 100, box[0][2], GREEN, BLACK, 3);
        					winner = box[0][2];
        				}
        				turns++;
        				if (turns == 9 && winner == 0)
        				{
        					drawChar(20, 35, box[0][0], RED, BLACK, 3);
        					drawChar(60, 35, box[0][1], RED, BLACK, 3);
        					drawChar(100, 35, box[0][2], RED, BLACK, 3);
        					drawChar(20, 67, box[1][0], RED, BLACK, 3);
        					drawChar(60, 67, box[1][1], RED, BLACK, 3);
        					drawChar(100, 67, box[1][2], RED, BLACK, 3);
        					drawChar(20, 100, box[2][0], RED, BLACK, 3);
        					drawChar(60, 100, box[2][1], RED, BLACK, 3);
        					drawChar(100, 100, box[2][2], RED, BLACK, 3);
        					winner = 'z';
        				}
        				my_turn = !my_turn;
        			}

        /*
                osi_MsgQRead( &g_PBQueue, &RecvQue, OSI_WAIT_FOREVER);

                if(PUSH_BUTTON_SW2_PRESSED == RecvQue)
                {
                    Button_IF_EnableInterrupt(SW2);
                    //
                    // send publish message
                    //
                    sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                            pub_topic_sw2,data_sw2,strlen((char*)data_sw2),QOS1,RETAIN);
                    UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
                    UART_PRINT("Topic: %s\n\r",pub_topic_sw2);
                    UART_PRINT("Data: %s\n\r",data_sw2);
                }
                else if(PUSH_BUTTON_SW3_PRESSED == RecvQue)
                {
                    Button_IF_EnableInterrupt(SW3);
                    //
                    // send publish message
                    //
                    sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                            pub_topic_sw3,data_sw3,strlen((char*)data_sw3),QOS1,RETAIN);
                    UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
                    UART_PRINT("Topic: %s\n\r",pub_topic_sw3);
                    UART_PRINT("Data: %s\n\r",data_sw3);
                }
                else if(BROKER_DISCONNECTION == RecvQue)
                {
                    iConnBroker--;
                    if(iConnBroker < 1)
                    {
                        //
                        // device not connected to any broker
                        //
                        goto end;
                    }
                }
        */
            }
end:
    //
    // Deinitializating the client library
    //
    sl_ExtLib_MqttClientExit();
    UART_PRINT("\n\r Exiting the Application\n\r");
    
    LOOP_FOREVER();
}

static void GPIOIntHandler()
{
	GPIOIntDisable(GPIOA3_BASE, 0x10);
	if (GPIOPinRead(GPIOA3_BASE, 0x10) == 0x10) // rising edge
	{
		if (start_read == 0)
		{
			curr_tick = SysTickValueGet();
			if (prev_tick > curr_tick)
				diff = prev_tick - curr_tick;
			else
				diff = 16777216 - curr_tick + prev_tick;
			//printf("%lu\n", diff);
			//prev_tick = curr_tick;
			if (650000 < diff && diff < 700000)
				start_read = 1;
		}

	}

	else // falling edge
	{
		if (start_read == 0)
		{
			prev_tick = SysTickValueGet();
		}
		else
		{
			if (count == 0)
			{
				prev_tick = SysTickValueGet();
				count++;
			}
			else
			{
				curr_tick = SysTickValueGet();
				if (prev_tick > curr_tick)
					diff = prev_tick - curr_tick;
				else
					diff = 16777216 - curr_tick + prev_tick;
				//printf("%i\n", diff);
					tick_arr[count - 1] = diff;
					if (count == 16) {
						count = 0;
						start_read = 0;
						button_flag = 1;
					}
					else
						count++;
				//}
				prev_tick = curr_tick;
			}
		}
	}
	GPIOIntClear(GPIOA3_BASE, 0x10);
	GPIOIntEnable(GPIOA3_BASE, 0x10);

}



void main()
{ 
    long lRetVal = -1;
    //
    // Initialize the board configurations
    //
    BoardInit();

    //
    // Pinmux for UART
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    //
    // Display Application Banner
    //
    DisplayBanner("MQTT_Client");

    //
    // Start the SimpleLink Host
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
        MAP_SPIReset(GSPI_BASE);
        MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                            SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                            (SPI_SW_CTRL_CS |
                            SPI_4PIN_MODE |
                            SPI_TURBO_OFF |
                            SPI_CS_ACTIVELOW |
                            SPI_WL_8));
        MAP_SPIEnable(GSPI_BASE);
        Adafruit_Init();
        fillScreen(BLACK);

        ClearTerm();

        UARTEnable(UARTA0_BASE);

                GPIOIntRegister(GPIOA3_BASE, GPIOIntHandler);
                GPIOIntTypeSet(GPIOA3_BASE, 0x10, GPIO_BOTH_EDGES);
                GPIOIntEnable(GPIOA3_BASE, GPIO_INT_PIN_4);

    //
    // Start the MQTT Client task
    //
    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(osi_messages),10);
    lRetVal = osi_TaskCreate(MqttClient,
                            (const signed char *)"Mqtt Client App",
                            OSI_STACK_SIZE, NULL, 2, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    //
    // Start the task scheduler
    //
    osi_start();
}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time()
{
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}
