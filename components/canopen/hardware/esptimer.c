#include <stdio.h>
#include <stdlib.h>
#include "esptimer.h"
#include "../inc/timer.h"
#include "../inc/data.h"
#include "../inc/sysdep.h"
#include "config.h"
// Canfestivals includes
#include "../inc/can.h"
#include "../inc/esp32xxx/canfestival.h"
#include "../inc/esp32xxx/applicfg.h"
#include "../dictionary/esp32xxx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"

TIMEVAL last_set_val = 0;
TIMEVAL last_counter_val = 0;
TIMEVAL elapsed_time = 0;


static gptimer_handle_t gptimer = NULL;

static bool IRAM_ATTR co_realtime_task(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
	uint32_t status;
	status = taskENTER_CRITICAL_FROM_ISR();
	last_counter_val = 0;
	elapsed_time = 0;
	TimeDispatch();
	taskEXIT_CRITICAL_FROM_ISR(status);
    return 0;
}


void esptimer_init(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 100000, // 0.1MHz, 1 tick=10us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = co_realtime_task,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = TIMEVAL_MAX, // period = 1s
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}


void start_callback(CO_Data* d, UNS32 id)
{    
}

// Initializes the timer, turn on the interrupt and put the interrupt time to zero
void initTimer(void)
{
	// this is needed for correct canfestival virtual timer management start
	SetAlarm(NULL, 0, start_callback, 0, 0);
}


void setTimer(TIMEVAL value)
{
	uint32_t timer = 0; // Copy the value of the running timer 

    gptimer_get_raw_count(gptimer,&timer);
	//gptimer_stop(gptimer);
	elapsed_time += timer - last_counter_val;
	last_counter_val = TIMEVAL_MAX - value;
	gptimer_set_raw_count(gptimer, last_counter_val);
    //gptimer_start(gptimer);
}


TIMEVAL getElapsedTime(void)
{
	uint32_t timer = 0; // Copy the value of the running timer 

    gptimer_get_raw_count(gptimer,&timer);
	//gptimer_stop(gptimer);
  
	if(timer < last_counter_val)
	  timer += TIMEVAL_MAX;
  
	TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
    //gptimer_start(gptimer);
  
	return elapsed;
}