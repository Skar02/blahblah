/**
 * Handles the application, of controlling motor speed, through threads.
 *
 * @authors Josefine Nyholm, Nils Kiefer, Arunraghavendren Senthil Kumar
 * 
 * @cite https://arm-software.github.io/CMSIS-RTX/latest/rtos2_tutorial.html#rtos2_tutorial_threads
 * @cite T. Martin and M. Rogers, The designer's guide to the cortex-m processor family, Second edition. 2016.
 */

#include "main.h" 
#include "application.h" 
#include "controller.h"
#include "peripherals.h"
#include "cmsis_os2.h"
#include "socket.h"
#include "shared_protocol.h"

//#define PERIOD_CTRL 10 dfhgg clanker code claker code

/* Global variables ----------------------------------------------------------*/

int32_t reference, velocity, control;
uint32_t millisec;


static osThreadId_t main_id, rxtx_id, actuate_id; //< Defines thread IDs fdfg dfg df g


// rxtx mumbo jumbo
int8_t  sock;
uint8_t sn;
uint8_t status;
int8_t  ret;
uint8_t server_ip[4] = {192, 168, 0, 10};

static volatile uint8_t session_active = 0U; //Tracks status of the section
static volatile uint32_t last_ctrl_ms = 0; //This is our watchdog timer that triggers motor stop if stale connection

enum																			// The diferent flags for the different cases. The threads act according to which flag woke them up
{
    FLAG_SESSION_START = 0x01U,
    FLAG_SESSION_STOP = 0x02U,
    FLAG_NEW_CONTROL = 0x04U
};



//Function & Thread declaration
static void init_threads(void);
static void app_main(void *arg);
static void app_rxtx(void *arg);
static void app_actuate(void *arg);

static int32_t send_all(uint8_t socket_num, const void *buf, uint16_t len);
static int32_t recv_all(uint8_t socket_num, void *buf, uint16_t len, uint32_t timeout_ms);

// Defines attributes for main_id and app_main()
static const osThreadAttr_t threadAttr_main = {
	.name       = "app_main",
	.stack_size = 128*4,         // Application_Loop call + waiting for flags, small call-stack => 256 bytes, with margin => 512 bytes
	.priority   = osPriorityNormal
};

//Defines attributes for rxtx_id and app_rxtx()
static const osThreadAttr_t threadAttr_rxtx = {
	.name       = "app_rxtx",
	.stack_size = 128*4,       // ~24 bytes local variables, ~32 bytes RTOS-functions, ~232 bytes function calls, call-stack + safety ~100 bytes
	.priority   = osPriorityBelowNormal
};

//Defines attributes for actuate_id and app_actuate()
static const osThreadAttr_t threadAttr_actuate = {
	.name       = "app_actuate",
	.stack_size = 128*2,              // ~8 bytes local variables, ~100-150 bytes RTOS-function, call-stack + safety ~100 bytes
	.priority   = osPriorityHigh
};


// Helper functions that assist with the communication
static int32_t send_all(uint8_t sn, const void *buf, uint16_t len)  //sn-socket number, *buf pointer to data we want to send, len is no of bytes we want to send
{
    const uint8_t *p = (const uint8_t *)buf; 												//creates a byte pointer, move through buffer byte by byte
    uint16_t sent = 0;																							// tracks how may bytes have been succesfully set so far

    while (sent < len)
    {
        int32_t n = send(sn, (uint8_t *)(p + sent), (uint16_t)(len - sent)); //p+sent is the first unsent byte in buffer
        if (n <= 0) {
            return n;   																										 // <0 error, =0 closed
        }          
        sent = (uint16_t)(sent + (uint16_t)n);
    }
    return (int32_t)sent;																										 //returns total bytes sent
}

static int32_t recv_all(uint8_t sn, void *buf, uint16_t len, uint32_t timeout_ms) //sn-socket number, *buf pointer to data we want to receive, len is no of bytes we want to receive, timeout_ms is just out period
{
    uint8_t *p = (uint8_t *)buf;
    uint16_t recvd = 0U;
		uint32_t wait_start_ms = Main_GetTickMillisec();


    while (recvd < len)
    {
				uint16_t rx_size = getSn_RX_RSR(sn); //Check the incoming size
			
				if (rx_size >0U){
					
					int32_t n = recv(sn, (uint8_t *)(p + recvd), (uint16_t)(len - recvd));
					if (n <= 0){
						return -1;          // error or closed
					} 
					recvd = (uint16_t)(recvd + (uint16_t)n);
					wait_start_ms = Main_GetTickMillisec();  //time control signal succesfully received
          continue;
				}
				
				uint8_t status = 0U;
				getsockopt(sn, SO_STATUS, &status);
				if (status != SOCK_ESTABLISHED)
						return -1;

				if ((Main_GetTickMillisec() - wait_start_ms) >= timeout_ms) { //Watchdog
            return 0;
        }
			
        osDelay(1);
    }
    return 1;
}

//Functions 
 

void Application_Setup()  //Initializes global variables, motor, controller and threads, 
													//Ethernet is enabled in our preprocessor for this target so this is called in main.c (the if ethernet enabled block)
{
  
  reference = 0;					// Initialise global variables
  velocity  = 0;
  control   = 0;
  millisec  = 0;
	
  Peripheral_GPIO_EnableMotor(); // Initialise hardware
  Controller_Reset();            // Initialize controller	
	
	osKernelInitialize();
	init_threads();                // Initializes threads
	osKernelStart();
}


void Application_Loop()		//App_main basically runs just this, keeps application running
 {
	 
	 for (;;)
	 {
    // Create socket
    sock = socket(0, Sn_MR_TCP, 0, 0); //cleint gets auto-assigned to any available free port
    if (sock < 0) {
        for(;;) osDelay(1000);
    }
    sn = (uint8_t)sock;

    // Connect
    ret = connect(sn, server_ip, 5000); //client connects to the server port, TCP handshake initiated here (client syn to server, server replies, client ack)
		if (ret != SOCK_OK){                //check if client is succesfulyl connecting, if not retry setup
			close(sn);
			osDelay(500);
			continue;
		}
		
		uint32_t start_ms = Main_GetTickMillisec();
		
		while (1){
			getsockopt(sn, SO_STATUS, &status);								//get status of the connection, if connected break out
			
			if (status == SOCK_ESTABLISHED){ 
					break;
			}
			
			if ((Main_GetTickMillisec() - start_ms) > 3000) {  //if connection takes longer than 3000ms
                close(sn);															 //close socket
                osDelay(500);														 //arbitary delay to allow for proper closure
                break;      														 // retry from top
      }

      osDelay(10);
		}

    if (status != SOCK_ESTABLISHED)
            continue;
		
		//succesfully connnected//
		
		session_active = 1;  													     //section is active
		last_ctrl_ms = Main_GetTickMillisec();  		 	     //Resetting watchdog timer
		osThreadFlagsSet(rxtx_id, FLAG_SESSION_START);     //kickstart rxtx thread
		osThreadFlagsSet(actuate_id, FLAG_SESSION_START);  //kickstart actuate thread
		
    
		osThreadFlagsWait(FLAG_SESSION_STOP, osFlagsWaitAny, osWaitForever); // Connected session, wait for flag stop
		
		//session stopped//
		
		session_active = 0; 						//session is no longer active
		control = 0;										//reset control value to zero
		Peripheral_PWM_ActuateMotor(0); //ensure motor is not running

    close(sn);
		
		
		do {														//temporary delay while socket is still open, break out when sock is closed and go back to start
    getsockopt(sn, SO_STATUS, &status);
    osDelay(5);
		} while (status != SOCK_CLOSED);
		
		
		}
 }


 //Initializes threads
static void init_threads(void)
{
	main_id = osThreadNew(app_main, NULL, &threadAttr_main);
	rxtx_id = osThreadNew(app_rxtx, NULL, &threadAttr_rxtx);
	actuate_id = osThreadNew(app_actuate, NULL, &threadAttr_actuate);
}

__NO_RETURN static void app_main(void *arg)
{
	for(;;)
	{
		Application_Loop();
	}
}




static void app_rxtx(void *arg)
{	
	for(;;)
	{
			osThreadFlagsWait(FLAG_SESSION_START, osFlagsWaitAny, osWaitForever); //Flag is cleared here, thread awakened
			while (session_active == 1){
				
				uint32_t cycle_start_ms = Main_GetTickMillisec(); //Start of cycle
				
				 //Measure new Velocity
        Sample_t s;
				velocity = Peripheral_Encoder_CalculateVelocity(s.timestamp);
        s.timestamp = Main_GetTickMillisec();
        s.velocity  = velocity;
				
				//Send measurement. If failed go into this if block 
        if (send_all(sn, &s, (uint16_t)sizeof(s)) <= 0) {
            break;
				}
				
			 //Receive control
			  int32_t n = recv_all(sn, (uint8_t*)&control, sizeof(control),PERIOD_CTRL);
				
				if (n != 1){  //Failed control signal receive, likely some disconnect
						break;
				}
				
				last_ctrl_ms = Main_GetTickMillisec();
				osThreadFlagsSet(actuate_id, FLAG_NEW_CONTROL);
				
				
				uint32_t now_ms = Main_GetTickMillisec();
				uint32_t elapsed_ms = now_ms - cycle_start_ms;
				
				if (elapsed_ms < PERIOD_CTRL) {   //you dont want to have a ping pong effect where you keep sending and receiving 
						osDelay(PERIOD_CTRL - elapsed_ms);
				}
				
				
			}
			session_active = 0;
			osThreadFlagsSet(actuate_id, FLAG_SESSION_STOP);
      osThreadFlagsSet(main_id, FLAG_SESSION_STOP);
	}
}

__NO_RETURN static void app_actuate(void *arg)
{for(;;)
	{
				osThreadFlagsWait(FLAG_SESSION_START, osFlagsWaitAny, osWaitForever);
				Peripheral_PWM_ActuateMotor(0);
				
				while (session_active == 1){
						uint32_t flags = osThreadFlagsWait(FLAG_NEW_CONTROL | FLAG_SESSION_STOP, osFlagsWaitAny, osWaitForever);
					
            if ((flags & osFlagsError) == 0U) {
                if ((flags & FLAG_SESSION_STOP) != 0U) {
                    break;
                }
                if ((flags & FLAG_NEW_CONTROL) != 0U) {
                    Peripheral_PWM_ActuateMotor(control);
                }
            }

            if ((Main_GetTickMillisec() - last_ctrl_ms) > PERIOD_CTRL) {  //watchdog
                Peripheral_PWM_ActuateMotor(0);
								break;
            }
        }

        Peripheral_PWM_ActuateMotor(0);
		
				
		
	}
}