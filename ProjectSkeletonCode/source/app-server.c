/**
 * Task 3 server:
 * - Generates reference square-wave
 * - Receives Sample_t from client
 * - Calculates PI control
 * - Sends control back to client
 */

#include "main.h"
#include "application.h"
#include "controller.h"
#include "peripherals.h"
#include "cmsis_os2.h"
#include "socket.h"
#include "shared_protocol.h"

/* Global variables ----------------------------------------------------------*/
int32_t reference, velocity, control;
uint32_t millisec;

static int8_t sock = -1;
static uint8_t sn = 0U;

static osThreadId_t main_id, ctrl_id, ref_id;
static osTimerId_t ref_timer;

static volatile uint8_t session_active = 0U;

enum
{
    FLAG_SESSION_START = 0x01U,
    FLAG_SESSION_STOP = 0x02U,
    FLAG_REF_TICK = 0x04U
};

/* Function/Thread declaration -----------------------------------------------*/
static void timerCallback(void *arg);
static void init_virtualTimers(void);

static void init_threads(void);
static void app_main(void *arg);
static void app_ctrl(void *arg);
static void app_ref(void *arg);

static int32_t send_all(uint8_t socket_num, const void *buf, uint16_t len);
static int32_t recv_all(uint8_t socket_num, void *buf, uint16_t len, uint32_t timeout_ms);

static const osThreadAttr_t threadAttr_main = {
    .name = "app_main",
    .stack_size = 128U * 4U,
    .priority = osPriorityBelowNormal
};

static const osThreadAttr_t threadAttr_ctrl = {
    .name = "app_ctrl",
    .stack_size = 128U * 4U,
    .priority = osPriorityNormal
};

static const osThreadAttr_t threadAttr_ref = {
    .name = "app_ref",
    .stack_size = 128U * 2U,
    .priority = osPriorityHigh
};

//Helper functions that assist with the communication
static int32_t send_all(uint8_t socket_num, const void *buf, uint16_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint16_t sent = 0U;

    while (sent < len) {
        int32_t n = send(socket_num, (uint8_t *)(p + sent), (uint16_t)(len - sent));
        if (n <= 0) {
            return n;
        }
        sent = (uint16_t)(sent + (uint16_t)n);
    }

    return (int32_t)sent;
}

static int32_t recv_all(uint8_t sn, void *buf, uint16_t len, uint32_t timeout_ms)
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

/* Functions -----------------------------------------------------------------*/
void Application_Setup(void)
{
    reference = 2000;
    velocity = 0;
    control = 0;
    millisec = 0;
    session_active = 0U;

    Controller_Reset();

    osKernelInitialize();
    init_virtualTimers();
    init_threads();
    osKernelStart();
}

void Application_Loop(void)
{
    for (;;) {
        uint32_t tickDelay_ref = (PERIOD_REF * osKernelGetTickFreq()) / 1000U;
        if (tickDelay_ref == 0U) {   //if in the even we have a refernce period smaller than tick resolution
            tickDelay_ref = 1U;
        }

        sock = socket(0, Sn_MR_TCP, 5000, 0); //local port number for server
        if (sock < 0) {
            osDelay(200U);
            continue;
        }
        sn = (uint8_t)sock;

        if (listen(sn) != SOCK_OK) { //check if server is succesfully listneing, if it isnt, retry setup     
            close(sn);
            osDelay(200U);
            continue;
        }

        {
            uint8_t connected = 0U;
            for (;;) {
            uint8_t status = 0U;
            getsockopt(sn, SO_STATUS, &status);
            if (status == SOCK_ESTABLISHED) {
                connected = 1U;
                break;
            }
            if (status == SOCK_CLOSED) {
                break;
            }
            osDelay(5U);
        }
            if (connected == 0U) {
                (void)close(sn);
                osDelay(200U);
                continue;
            }
        }

        session_active = 1U;
        reference = 2000;
        velocity = 0;
        control = 0;
        millisec = Main_GetTickMillisec();
        Controller_Reset();

        osTimerStart(ref_timer, tickDelay_ref);
        osThreadFlagsSet(ctrl_id, FLAG_SESSION_START);
        osThreadFlagsSet(ref_id, FLAG_SESSION_START);

        osThreadFlagsWait(FLAG_SESSION_STOP, osFlagsWaitAny, osWaitForever);

        session_active = 0U;
        osThreadFlagsSet(ref_id, FLAG_SESSION_STOP);
        osTimerStop(ref_timer);
        Controller_Reset();
				
        close(sn);
        osDelay(100U);
		}
}

/* Virtual Timer Functions ---------------------------------------------------*/
static void init_virtualTimers(void)
{
    ref_timer = osTimerNew(timerCallback, osTimerPeriodic, NULL, NULL);
}

static void timerCallback(void *arg)
{
    osThreadFlagsSet(ref_id, FLAG_REF_TICK);
}

/* Thread functions ----------------------------------------------------------*/
static void init_threads(void)
{
    main_id = osThreadNew(app_main, NULL, &threadAttr_main);
    ctrl_id = osThreadNew(app_ctrl, NULL, &threadAttr_ctrl);
    ref_id = osThreadNew(app_ref, NULL, &threadAttr_ref);
}

__NO_RETURN static void app_main(void *arg)
{
    for (;;){
				Application_Loop();
		}
}

__NO_RETURN static void app_ctrl(void *arg)
{
    (void)arg;

    for (;;) {
        osThreadFlagsWait(FLAG_SESSION_START, osFlagsWaitAny, osWaitForever);

        while (session_active == 1) {
            Sample_t s;
            int32_t recv_result = recv_all(sn, &s, (uint16_t)sizeof(s), (uint32_t)(2U * PERIOD_CTRL));

            if (recv_result != 1) {
                break;
            }

            velocity = s.velocity;
            millisec = s.timestamp;
            control = Controller_PIController(&reference, &velocity, &millisec);

            if (send_all(sn, &control, (uint16_t)sizeof(control)) != (int32_t)sizeof(control)) {
                break;
            }
        }

        session_active = 0U;
        osThreadFlagsSet(main_id, FLAG_SESSION_STOP);
    }
}

__NO_RETURN static void app_ref(void *arg)
{
    (void)arg;

    for (;;) {
        osThreadFlagsWait(FLAG_SESSION_START, osFlagsWaitAny, osWaitForever);

        while (session_active == 1 ) {
            uint32_t flags = osThreadFlagsWait(FLAG_REF_TICK | FLAG_SESSION_STOP, osFlagsWaitAny, osWaitForever);
            if ((flags & osFlagsError) != 0U) {
                continue;
            }

            if (((flags & FLAG_SESSION_STOP) != 0U) || (session_active == 0U)) {
                break;
            }

            if ((flags & FLAG_REF_TICK) != 0U) {
                reference = -reference;
            }
        }
    }
}