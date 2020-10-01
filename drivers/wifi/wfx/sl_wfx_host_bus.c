/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#define LOG_MODULE_NAME wfx
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_DECLARE(LOG_MODULE_NAME);

#include "sl_wfx.h"
#include "sl_wfx_registers.h"
#include "sl_wfx_host_cfg.h"

#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <kernel.h>

#include "sl_wfx_task.h"
#include "sl_wfx_host.h"


struct k_poll_signal sl_wfx_bus_event_flag_rx = K_POLL_SIGNAL_INITIALIZER(sl_wfx_bus_event_flag_rx);
struct k_poll_signal sl_wfx_bus_event_flag_tx = K_POLL_SIGNAL_INITIALIZER(sl_wfx_bus_event_flag_tx);
struct k_poll_signal sl_wfx_bus_event_wake = K_POLL_SIGNAL_INITIALIZER(sl_wfx_bus_event_wake);

K_SEM_DEFINE(wfx_send_confirm, 0, 1);
sl_status_t wfx_send_result;
wfx_frame_q_item wfx_bus_tx_frame;

static sl_status_t receive_frames()
{
	sl_status_t result;
	uint16_t control_register = 0;

	do {
		result = sl_wfx_receive_frame(&control_register);

		SL_WFX_ERROR_CHECK(result);
	} while ((control_register & SL_WFX_CONT_NEXT_LEN_MASK) != 0);
error_handler:
	return result;
}

static void wfx_bus_task(int unused1, int unused2, int unused3)
{
	sl_status_t result;

	struct k_poll_event events[3] = {
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sl_wfx_bus_event_flag_rx),
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sl_wfx_bus_event_flag_tx),
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sl_wfx_bus_event_wake)
	};


	while (1) {
#ifdef CONFIG_WIFI_WFX_SLEEP
#ifdef CONFIG_WIFI_WFX_BUS_SPI
		if (GPIO_PinInGet(SL_WFX_HOST_CFG_SPI_WIRQPORT, SL_WFX_HOST_CFG_SPI_WIRQPIN)) // wfx messages pending
#else
		if (GPIO_PinInGet(SL_WFX_HOST_CFG_WIRQPORT, SL_WFX_HOST_CFG_WIRQPIN))
#endif
		{
			printk("Siggie\n");
			k_poll_signal_raise(&sl_wfx_bus_event_flag_rx, 1);
		}
#endif
		printk("Polling\n");
		k_poll(events, 3, K_FOREVER);
		if (events[0].state == K_POLL_STATE_SIGNALED) {
			printk("RX Event\n");
			events[0].signal->signaled = false;
			events[0].state = K_POLL_STATE_NOT_READY;
			result = receive_frames();
			printk("gotframes\n");

			// If PowerSave && notTxNow && nousedbuffers			

			if ((sl_wfx_context->state & SL_WFX_POWER_SAVE_ACTIVE)
		            && !sl_wfx_context->used_buffers && events[1].state != K_POLL_STATE_SIGNALED) {
		            	printk("Back to Sleep: %d\n", GPIO_PinInGet(SL_WFX_HOST_CFG_WIRQPORT, SL_WFX_HOST_CFG_WIRQPIN));
		            	sl_wfx_context->state |= SL_WFX_SLEEPING;
				result = sl_wfx_host_set_wake_up_pin(0);
			}

			// something about pin for wakeup if sleeping
#ifdef CONFIG_WIFI_WFX_BUS_SDIO
			sl_wfx_host_enable_platform_interrupt();
#endif
		}
		if (events[1].state == K_POLL_STATE_SIGNALED) {
			events[1].signal->signaled = false;
			events[1].state = K_POLL_STATE_NOT_READY;
			printk("TX Event\n");
			
			wfx_send_result = sl_wfx_send_ethernet_frame(wfx_bus_tx_frame.frame,
                                            wfx_bus_tx_frame.data_length,
                                            wfx_bus_tx_frame.interface,
                                            wfx_bus_tx_frame.priority);
			printk("Sent %d bytes: %d\n", wfx_bus_tx_frame.data_length, wfx_send_result);
		
			k_sem_give(&wfx_send_confirm);
                                            
		}
		if (events[2].state == K_POLL_STATE_SIGNALED) {
			events[2].signal->signaled = false;
			events[2].state = K_POLL_STATE_NOT_READY;
		}

	}
}


// TODO: How do I find max thread size properly?
K_THREAD_DEFINE(wfx_bus_tid, 4096,
		wfx_bus_task, NULL, NULL, NULL,
		5, 0, 0);
