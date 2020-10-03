/**************************************************************************//**
 * Copyright 2019, Silicon Laboratories Inc.
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
#ifndef SL_WFX_HOST_H
#define SL_WFX_HOST_H
#ifdef __cplusplus
extern "C" {
#endif

void sl_wfx_process(void);

#ifdef CONFIG_WIFI_WFX_SLEEP
sl_status_t sl_wfx_host_switch_to_wirq(void);
#endif
#ifdef __cplusplus
}
#endif

extern struct k_poll_signal sl_wfx_bus_event_flag_rx;
extern struct k_poll_signal sl_wfx_bus_event_flag_tx;
extern struct k_poll_signal sl_wfx_bus_event_wake;

extern sl_status_t wfx_send_result;
extern struct k_sem wfx_send_confirm;
typedef struct {
  sl_wfx_send_frame_req_t *frame;
  uint32_t data_length;
  sl_wfx_interface_t interface;
  uint8_t priority;
} wfx_frame_q_item;
extern wfx_frame_q_item wfx_bus_tx_frame;

sl_status_t sl_wfx_host_enable_sdio(void);
sl_status_t sl_wfx_host_disable_sdio(void);

#endif
