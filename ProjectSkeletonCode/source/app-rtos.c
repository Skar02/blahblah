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

/* Global variables ----------------------------------------------------------*/

 int32_t reference, velocity, control;
 uint32_t millisec;
static osThreadId_t main_id, ctrl_id, ref_id; //< Defines thread IDs
static osTimerId_t ctrl_timer, ref_timer;     //< Defines callback timers

/* Function/Thread declaration -----------------------------------------------*/

static void timerCallback(void *arg); // Callback timer function
static void init_virtualTimers(void);

static void init_threads(void);
static void app_main(void *arg);
static void app_ctrl(void *arg);
static void app_ref(void *arg);

/**
 * Defines attributes for main_id and app_main()
 */
static const osThreadAttr_t threadAttr_main = {
	.name       = "app_main",
	.stack_size = 128*4,         // Application_Loop call + waiting for flags, small call-stack => 256 bytes, with margin => 512 bytes
	.priority   = osPriorityNormal
};

/**
 * Defines attributes for ctrl_id and app_ctrl()
 */
static const osThreadAttr_t threadAttr_ctrl = {
	.name       = "app_ctrl",
	.stack_size = 128*4,       // ~24 bytes local variables, ~32 bytes RTOS-functions, ~232 bytes function calls, call-stack + safety ~100 bytes
	.priority   = osPriorityHigh
};

/**
 * Defines attributes for ref_id and app_ref()
 */
static const osThreadAttr_t threadAttr_ref = {
	.name       = "app_ref",
	.stack_size = 128*2,              // ~8 bytes local variables, ~100-150 bytes RTOS-function, call-stack + safety ~100 bytes
	.priority   = osPriorityBelowNormal
};

/* Functions -----------------------------------------------------------------*/
 
/**
 * Initializes global variables, motor, controller and threads
 */
void Application_Setup()
{
  // Reset global variables
  reference = 2000;
  velocity  = 0;
  control   = 0;
  millisec  = 0;
	
  Peripheral_GPIO_EnableMotor(); // Initialise hardware
  Controller_Reset();            // Initialize controller	
	
	osKernelInitialize();
	init_threads();                // Initializes threads
	
	osKernelStart();
}

/**
 * Keeps the application waiting
 */
void Application_Loop()
 {
   osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
 }
 
 /* Virtual Timer Functions --------------------------------------------------*/
 
/**
 * Initializes the threads virtual timers to call the callback function periodically.
 */
static void init_virtualTimers(void)
{
	ctrl_timer = osTimerNew(timerCallback, osTimerPeriodic, ctrl_id, NULL); // Sets a periodic timer for app_ctrl to call the callback function
	ref_timer  = osTimerNew(timerCallback, osTimerPeriodic, ref_id, NULL);  // Sets a periodic timer for app_ref to call the callback function
	
	uint32_t tickDelay_ctrl = (PERIOD_CTRL * osKernelGetTickFreq()) / 1000; // Calculates amount of ticks representing the required period in ms
	uint32_t tickDelay_ref  = (PERIOD_REF * osKernelGetTickFreq()) / 1000;  // Calculates amount of ticks representing the required period in ms
	
	// Starts and specifies timing in system ticks
	osTimerStart(ctrl_timer, tickDelay_ctrl);
	osTimerStart(ref_timer, tickDelay_ref);
}

/**
 * Sets the correct thread flag depending on input parameted thread ID.
 *
 * @param arg - Thread ID
 */
static void timerCallback(void *arg)
{
  osThreadFlagsSet((osThreadId_t)arg, 0x01);					// Flags the correct thread through thread ID
}
 
/* Thread Functions ----------------------------------------------------------*/

/**
 * Initializes threads
 */
static void init_threads(void)
{
	main_id = osThreadNew(app_main, NULL, &threadAttr_main);
	ctrl_id = osThreadNew(app_ctrl, NULL, &threadAttr_ctrl);
	ref_id  = osThreadNew(app_ref, NULL, &threadAttr_ref);
}

/**
 * Calls Application_loop() indefinetly
 *
 * @param arg - Thread argument
 */
__NO_RETURN static void app_main(void *arg)
{
	init_virtualTimers();			  	 // Initializes and starts virtual timers
	for(;;)
	{
		Application_Loop();
	}
}

/* Thread Functions with osDelay()--------------------------------------------*/

/**
 * Samples encoder, calculates the control signal and applies it to the motor every 10 ms, using osDelay().
 *
 * @param arg - Thread argument
 */
/*
__NO_RETURN static void app_ctrl(void *arg)
{
	uint32_t tickDelay = (PERIOD_CTRL * osKernelGetTickFreq()) / 1000; // Calculates amount of ticks representing the required period in ms
	
	for(;;)
	{
		millisec = Main_GetTickMillisec();
		
		velocity = Peripheral_Encoder_CalculateVelocity(millisec);            // Calculate motor velocity
		control  = Controller_PIController(&reference, &velocity, &millisec); // Calculate control signal
		
		Peripheral_PWM_ActuateMotor(control); // Apply control signal to motor
		
		uint32_t runTimeTicks = Main_GetTickMillisec() - millisec;
		
		if(runTimeTicks < tickDelay)
			osDelay(tickDelay - runTimeTicks);
		else
			osDelay(1);
	}
}
*/

/**
 * Toggles the direction of the reference every 4000 ms using osDelay().
 *
 * @param arg - Thread argument
 */
/*
__NO_RETURN static void app_ref(void *arg)
{
	uint32_t tickDelay = (PERIOD_REF * osKernelGetTickFreq()) / 1000; // Calculates amount of ticks representing the required period in ms
	
	for(;;)
	{
		reference = -reference; // Flip reference
		osDelay(tickDelay); 	  // Wait for next sample
	}
}
*/

/* Thread Functions with Flags -----------------------------------------------*/

/**
 * Samples encoder, calculates the control signal and applies it to the motor every 10 ms, using osThreadFlagsWait().
 *
 * @param arg - Thread argument
 */
__NO_RETURN static void app_ctrl(void *arg)
{	
	for(;;)
	{
		osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); // Wait for next sample flagging (flag gets rinsed automatically)
		
		millisec = Main_GetTickMillisec();
		
		velocity = Peripheral_Encoder_CalculateVelocity(millisec);            // Calculate motor velocity
		control  = Controller_PIController(&reference, &velocity, &millisec); // Calculate control signal
		
		Peripheral_PWM_ActuateMotor(control); // Apply control signal to motor
	}
}

/**
 * Toggles the direction of the reference every 4000 ms using osThreadFlagsWait().
 *
 * @param arg - Thread argument
 */
__NO_RETURN static void app_ref(void *arg)
{
	for(;;)
	{
		osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); // Wait for next sample flagging (flag gets rinsed automatically)
		reference = -reference;                                 // Flip reference
	}
}