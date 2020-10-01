/***************************************************************************//**
 * @file
 * @brief WFX FMAC driver host implementation
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
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <kernel.h>
#include <init.h>
#include <debug/stack.h>
#include <device.h>
#include <string.h>
#include <errno.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/net_l2.h>
#include <net/net_context.h>
#include <net/net_offload.h>
#include <net/wifi_mgmt.h>
#include <net/ethernet.h>
#include <sys/printk.h>
#include <string.h>
#include <logging/log.h>

#include "sl_wfx.h"
#include "sl_wfx_host_api.h"
#include "sl_wfx_host_cfg.h"

#include "em_core.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"
#if   defined(SLEXP802X)
#include "brd8022a_pds.h"
#include "brd8023a_pds.h"
#elif defined(EFM32GG11B820F2048GM64) // WGM160PX22KGA2
#include "brd4001a_pds.h"
#else
#error "WFX200 EXP board type must be specified"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* Firmware include */
#include "sl_wfx_wf200_C0.h"
#include "sl_wfx_host.h"

static sl_wfx_context_t wifi;
//+sl_wfx_context_t sl_wfx_context;
//sl_wfx_context_t sl_wfx_context;

#define SL_WFX_EVENT_MAX_SIZE  512
#define SL_WFX_EVENT_LIST_SIZE 1
#define SL_WFX_MAX_STATIONS    8
#define SL_WFX_MAX_SCAN_RESULTS 50

volatile uint8_t wf200_interrupt_event = 0;

static uint16_t control_register = 0;

struct {
	uint32_t wf200_firmware_download_progress;
	uint8_t waited_event_id;
	uint8_t posted_event_id;
	struct device *dev;
	struct net_if *iface;
	scan_result_cb_t scan_cb;

} host_context;

#ifdef CONFIG_WIFI_WFX_SLEEP
extern sl_wfx_context_t wifi;
#endif

static void connectTest();
                            
K_MUTEX_DEFINE(wfx_mutex);
K_SEM_DEFINE(wf200_confirmation, 0, 1);

void sl_wfx_host_received_frame_callback(sl_wfx_received_ind_t *rx_buffer)
{
	uint16_t len = rx_buffer->body.frame_length;;
	uint8_t *buffer = (uint8_t *)&(rx_buffer->body.frame[rx_buffer->body.frame_padding]);
	struct net_pkt *rx_frame = NULL;
	int res;

	rx_frame = net_pkt_rx_alloc_with_buffer(host_context.iface, len, AF_UNSPEC, 0, K_NO_WAIT);
	if (rx_frame == NULL) {
		LOG_ERR("Failed to allocate rx_frame");
		return;
	}
	if (net_pkt_write(rx_frame, buffer, len) < 0) {
		LOG_ERR("Failed to append rx packet");
		net_pkt_unref(rx_frame);
		return;
	}

	res = net_recv_data(host_context.iface, rx_frame);
	if (res < 0) {
		LOG_ERR("Failed to enqueue rx packet");
		net_pkt_unref(rx_frame);
	}
}


/* WF200 host callbacks */
void sl_wfx_connect_callback(uint8_t *mac, uint32_t status);
void sl_wfx_disconnect_callback(uint8_t *mac, uint16_t reason);
void sl_wfx_start_ap_callback(uint32_t status);
void sl_wfx_stop_ap_callback(void);
void sl_wfx_scan_result_callback(sl_wfx_scan_result_ind_body_t *scan_result);
void sl_wfx_scan_complete_callback(uint32_t status);
void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t *frame);
void sl_wfx_client_connected_callback(uint8_t *mac);
void sl_wfx_ap_client_disconnected_callback(uint32_t status, uint8_t *mac);

/**************************************************************************//**
 * WFX FMAC driver host interface initialization
 *****************************************************************************/
sl_status_t sl_wfx_host_init(void)
{
	host_context.wf200_firmware_download_progress = 0;
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get firmware data
 *****************************************************************************/
sl_status_t sl_wfx_host_get_firmware_data(const uint8_t **data, uint32_t data_size)
{
	*data = &sl_wfx_firmware[host_context.wf200_firmware_download_progress];
	host_context.wf200_firmware_download_progress += data_size;
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get firmware size
 *****************************************************************************/
sl_status_t sl_wfx_host_get_firmware_size(uint32_t *firmware_size)
{
	*firmware_size = sizeof(sl_wfx_firmware);
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get PDS data
 *****************************************************************************/
sl_status_t sl_wfx_host_get_pds_data(const char **pds_data, uint16_t index)
{
#ifdef SLEXP802X
	// Manage dynamically the PDS in function of the chip connected
	if (strncmp("WFM200", (char *)sl_wfx_context->wfx_opn, 6) == 0) {
		*pds_data = pds_table_brd8023a[index];
	} else {
		*pds_data = pds_table_brd8022a[index];
	}
#else
	*pds_data = pds_table_brd4001a[index];
#endif
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get PDS size
 *****************************************************************************/
sl_status_t sl_wfx_host_get_pds_size(uint16_t *pds_size)
{
#ifdef SLEXP802X
	// Manage dynamically the PDS in function of the chip connected
	if (strncmp("WFM200", (char *)sl_wfx_context->wfx_opn, 6) == 0) {
		*pds_size = SL_WFX_ARRAY_COUNT(pds_table_brd8023a);
	} else {
		*pds_size = SL_WFX_ARRAY_COUNT(pds_table_brd8022a);
	}
#else
	*pds_size = SL_WFX_ARRAY_COUNT(pds_table_brd4001a);
#endif
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Deinit host interface
 *****************************************************************************/
sl_status_t sl_wfx_host_deinit(void)
{
	return SL_STATUS_OK;
}

uint8_t hugething[4096];


/**************************************************************************//**
 * Allocate buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_allocate_buffer(void **buffer,
					sl_wfx_buffer_type_t type,
					uint32_t buffer_size)
{
	SL_WFX_UNUSED_PARAMETER(type);

	*buffer = k_malloc(buffer_size);

	if (*buffer == NULL) {
		return SL_STATUS_FAIL;
	} else {
		return SL_STATUS_OK;
	}
}

/**************************************************************************//**
 * Free host buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_free_buffer(void *buffer, sl_wfx_buffer_type_t type)
{
	SL_WFX_UNUSED_PARAMETER(type);
	k_free(buffer);
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set reset pin low
 *****************************************************************************/
sl_status_t sl_wfx_host_hold_in_reset(void)
{
	GPIO_PinOutClear(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set wakeup pin status
 *****************************************************************************/
sl_status_t sl_wfx_host_set_wake_up_pin(uint8_t state)
{
	printk("SetWakeUp %d\n", state);
	if (state > 0) {
		sl_wfx_host_enable_sdio();
		GPIO_PinOutSet(SL_WFX_HOST_CFG_WUP_PORT, SL_WFX_HOST_CFG_WUP_PIN);
	} else {
		sl_wfx_host_disable_sdio();
		GPIO_PinOutClear(SL_WFX_HOST_CFG_WUP_PORT, SL_WFX_HOST_CFG_WUP_PIN);
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_reset_chip(void)
{
	// Pull it low for at least 1 ms to issue a reset sequence
	GPIO_PinOutClear(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);
	// Delay for 10ms
	k_sleep(K_MSEC(10));

	// Hold pin high to get chip out of reset
	GPIO_PinOutSet(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);
	// Delay for 3ms
	k_sleep(K_MSEC(3));
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_wake_up(void)
{
	// TODO: Seriously
	printk("Presleep\n");
//	k_busy_wait(3); // K_MSEC(3));
//	k_yield();
	k_sleep(K_MSEC(3));
	printk("Postsleep\n");
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait(uint32_t wait_time)
{
	k_sleep(K_MSEC(wait_time));
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_setup_waited_event(uint8_t event_id)
{
	host_context.waited_event_id = event_id;
	host_context.posted_event_id = 0;

	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_confirmation(uint8_t confirmation_id,
					      uint32_t timeout_ms,
					      void **event_payload_out)
{
	int64_t tmo = k_uptime_get() + timeout_ms;
	
	while (1) {
		int64_t left = tmo - k_uptime_get();

		int r = k_sem_take(&wf200_confirmation, left <= 0 ? K_NO_WAIT : K_MSEC(left));
		if (r == -EBUSY)
			return SL_STATUS_TIMEOUT;
		else if (r != 0)
			return SL_STATUS_FAIL;
			
		if (confirmation_id == host_context.posted_event_id) {
			if ( event_payload_out != NULL ) {
				*event_payload_out = sl_wfx_context->event_payload_buffer;
			}
			return SL_STATUS_OK;
		}
	}
}

void sl_wfx_process(void)
{
	if (wf200_interrupt_event == 1) {
		// Reset the control register value
		control_register = 0;

		do {
			wf200_interrupt_event = 0;
			sl_wfx_receive_frame(&control_register);
		} while ((control_register & SL_WFX_CONT_NEXT_LEN_MASK) != 0);

#ifdef CONFIG_WIFI_WFX_BUS_SDIO
		sl_wfx_host_enable_platform_interrupt();
#endif
	}
}

/**************************************************************************//**
 * Called when the driver needs to lock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_lock(void)
{
	if (k_mutex_lock(&wfx_mutex, K_FOREVER) != 0) 
		return SL_STATUS_FAIL;

	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to unlock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_unlock(void)
{
	k_mutex_unlock(&wfx_mutex);
	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to post an event
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_post_event(sl_wfx_generic_message_t *event_payload)
{
	switch (event_payload->header.id) {
	/******** INDICATION ********/
	case SL_WFX_CONNECT_IND_ID:
	{
		sl_wfx_connect_ind_t *connect_indication = (sl_wfx_connect_ind_t *) event_payload;
		sl_wfx_connect_callback(connect_indication->body.mac, connect_indication->body.status);
		break;
	}
	case SL_WFX_DISCONNECT_IND_ID:
	{
		sl_wfx_disconnect_ind_t *disconnect_indication = (sl_wfx_disconnect_ind_t *) event_payload;
		sl_wfx_disconnect_callback(disconnect_indication->body.mac, disconnect_indication->body.reason);
		break;
	}
	case SL_WFX_START_AP_IND_ID:
	{
		sl_wfx_start_ap_ind_t *start_ap_indication = (sl_wfx_start_ap_ind_t *) event_payload;
		sl_wfx_start_ap_callback(start_ap_indication->body.status);
		break;
	}
	case SL_WFX_STOP_AP_IND_ID:
	{
		sl_wfx_stop_ap_callback();
		break;
	}
	case SL_WFX_RECEIVED_IND_ID:
	{
		sl_wfx_received_ind_t *ethernet_frame = (sl_wfx_received_ind_t *) event_payload;
		if (ethernet_frame->body.frame_type == 0) {
			sl_wfx_host_received_frame_callback(ethernet_frame);
		}
		break;
	}
	case SL_WFX_SCAN_RESULT_IND_ID:
	{
		sl_wfx_scan_result_ind_t *scan_result = (sl_wfx_scan_result_ind_t *) event_payload;
		sl_wfx_scan_result_callback(&scan_result->body);
		break;
	}
	case SL_WFX_SCAN_COMPLETE_IND_ID:
	{
		sl_wfx_scan_complete_ind_t *scan_complete = (sl_wfx_scan_complete_ind_t *) event_payload;
		sl_wfx_scan_complete_callback(scan_complete->body.status);
		break;
	}
	case SL_WFX_AP_CLIENT_CONNECTED_IND_ID:
	{
		sl_wfx_ap_client_connected_ind_t *client_connected_indication = (sl_wfx_ap_client_connected_ind_t *) event_payload;
		sl_wfx_client_connected_callback(client_connected_indication->body.mac);
		break;
	}
	case SL_WFX_AP_CLIENT_REJECTED_IND_ID:
	{
		break;
	}
	case SL_WFX_AP_CLIENT_DISCONNECTED_IND_ID:
	{
		sl_wfx_ap_client_disconnected_ind_t *ap_client_disconnected_indication = (sl_wfx_ap_client_disconnected_ind_t *) event_payload;
		sl_wfx_ap_client_disconnected_callback(ap_client_disconnected_indication->body.reason, ap_client_disconnected_indication->body.mac);
		break;
	}
	case SL_WFX_GENERIC_IND_ID:
	{
		sl_wfx_generic_ind_t *generic_status = (sl_wfx_generic_ind_t *) event_payload;
		sl_wfx_generic_status_callback(generic_status);
		break;
	}
	case SL_WFX_EXCEPTION_IND_ID:
	{
		sl_wfx_exception_ind_t *firmware_exception = (sl_wfx_exception_ind_t *) event_payload;
		LOG_ERR("firmware exception %u", firmware_exception->body.reason);
		break;
	}
	case SL_WFX_ERROR_IND_ID:
	{
		sl_wfx_error_ind_t *firmware_error = (sl_wfx_error_ind_t *) event_payload;
		LOG_ERR("firmware error %u", firmware_error->body.type);
		break;
	}
	/******** CONFIRMATION ********/
	case SL_WFX_SEND_FRAME_CNF_ID:
	{
		break;
	}
	}

	if (host_context.waited_event_id == event_payload->header.id) {
		/* Post the event in the queue */
		memcpy(sl_wfx_context->event_payload_buffer,
		       (void *) event_payload,
		       event_payload->header.length);
		host_context.posted_event_id = event_payload->header.id;
		k_sem_give(&wf200_confirmation);
	}

	return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to transmit a frame
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_transmit_frame(void *frame, uint32_t frame_len)
{
	return sl_wfx_data_write(frame, frame_len);
}

/**************************************************************************//**
 * Callback for individual scan result
 *****************************************************************************/
void sl_wfx_scan_result_callback(sl_wfx_scan_result_ind_body_t *scan_result)
{
	if (host_context.scan_cb != NULL) {
		struct wifi_scan_result result;

		memcpy(result.ssid, scan_result->ssid_def.ssid, WIFI_SSID_MAX_LEN);
		result.ssid_length = strnlen(result.ssid, WIFI_SSID_MAX_LEN);
		result.channel = scan_result->channel;

		result.rssi = ((int32_t)scan_result->rcpi - 220) / 2;

		if (scan_result->security_mode.psk) {
			result.security = WIFI_SECURITY_TYPE_PSK;
		} else if (scan_result->security_mode.eap) {
			LOG_WRN("Access point %s uses EAP, cannot return scan result", scan_result->ssid_def.ssid);
			return;
		} else {
			result.security = WIFI_SECURITY_TYPE_NONE;
		}

		host_context.scan_cb(host_context.iface, 0, &result);
	}
}

/**************************************************************************//**
 * Callback for scan complete
 *****************************************************************************/
void sl_wfx_scan_complete_callback(uint32_t status)
{
	if (host_context.scan_cb != NULL) {
		host_context.scan_cb(host_context.iface, 0, NULL);
		host_context.scan_cb = NULL;
	}

	(void)(status);
}

/**************************************************************************//**
 * Callback when station connects
 *****************************************************************************/
void sl_wfx_connect_callback(uint8_t *mac, uint32_t status)
{
	switch (status) {
	case WFM_STATUS_SUCCESS:
	{
		LOG_DBG("Connected to %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		sl_wfx_context->state |= SL_WFX_STA_INTERFACE_CONNECTED;
//      lwip_set_sta_link_up();

#ifdef CONFIG_WIFI_WFX_SLEEP
		if (!(wifi.state & SL_WFX_AP_INTERFACE_UP)) {
			// Enable the power save
			sl_wfx_set_power_mode(WFM_PM_MODE_DTIM, 0);
			sl_wfx_enable_device_power_save();
		}
#endif
		break;
	}
	case WFM_STATUS_NO_MATCHING_AP:
	{
		LOG_WRN("Connection failed, access point not found");
		break;
	}
	case WFM_STATUS_CONNECTION_ABORTED:
	{
		LOG_WRN("Connection aborted");
		break;
	}
	case WFM_STATUS_CONNECTION_TIMEOUT:
	{
		LOG_WRN("Connection timeout");
//		connectTest();
		break;
	}
	case WFM_STATUS_CONNECTION_REJECTED_BY_AP:
	{
		LOG_WRN("Connection rejected by the access point");
		break;
	}
	case WFM_STATUS_CONNECTION_AUTH_FAILURE:
	{
		LOG_WRN("Connection authentication failure");
		break;
	}
	default:
	{
		LOG_WRN("Connection attempt error");
		break;
	}
	}
}

/**************************************************************************//**
 * Callback for station disconnect
 *****************************************************************************/
void sl_wfx_disconnect_callback(uint8_t *mac, uint16_t reason)
{
	(void)(mac);
	LOG_DBG("Disconnected: %u", reason);
	sl_wfx_context->state &= ~SL_WFX_STA_INTERFACE_CONNECTED;
//  lwip_set_sta_link_down();
}

/**************************************************************************//**
 * Callback for AP started
 *****************************************************************************/
void sl_wfx_start_ap_callback(uint32_t status)
{
	if (status == 0) {
		LOG_DBG("AP started");
		sl_wfx_context->state |= SL_WFX_AP_INTERFACE_UP;
//    lwip_set_ap_link_up();

#ifdef CONFIG_WIFI_WFX_SLEEP
		// Power save always disabled when SoftAP mode enabled
		sl_wfx_set_power_mode(WFM_PM_MODE_ACTIVE, 0);
		sl_wfx_disable_device_power_save();
#endif
	} else {
		LOG_ERR("AP start failed");
	}
}

/**************************************************************************//**
 * Callback for AP stopped
 *****************************************************************************/
void sl_wfx_stop_ap_callback(void)
{
//  dhcpserver_clear_stored_mac ();
	LOG_DBG("SoftAP stopped");
	sl_wfx_context->state &= ~SL_WFX_AP_INTERFACE_UP;
//  lwip_set_ap_link_down();

#ifdef CONFIG_WIFI_WFX_SLEEP
	if (wifi.state & SL_WFX_STA_INTERFACE_CONNECTED) {
		// Enable the power save
		sl_wfx_set_power_mode(WFM_PM_MODE_PS, 0);
		sl_wfx_enable_device_power_save();
	}
#endif
}

/**************************************************************************//**
 * Callback for client connect to AP
 *****************************************************************************/
void sl_wfx_client_connected_callback(uint8_t *mac)
{
	LOG_DBG("Client connected, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
	       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**************************************************************************//**
 * Callback for client rejected from AP
 *****************************************************************************/
void sl_wfx_ap_client_rejected_callback(uint32_t status, uint8_t *mac)
{
   LOG_DBG("Client rejected, reason: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        (int)status, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
/*
   struct eth_addr mac_addr;
   memcpy(&mac_addr, mac, SL_WFX_BSSID_SIZE);
   //  dhcpserver_remove_mac(&mac_addr);
 */
}

/**************************************************************************//**
 * Callback for AP client disconnect
 *****************************************************************************/
void sl_wfx_ap_client_disconnected_callback(uint32_t status, uint8_t *mac)
{
   LOG_DBG("Client disconnected, reason: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
         (int)status, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

/*
   struct eth_addr mac_addr;
   memcpy(&mac_addr, mac, SL_WFX_BSSID_SIZE);
   //  dhcpserver_remove_mac(&mac_addr);
 */
}

/**************************************************************************//**
 * Callback for generic status received
 *****************************************************************************/
void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t *frame)
{
	(void)(frame);
	LOG_DBG("Generic status received");
}

/**************************************************************************//**
 * Called when the driver is considering putting the WFx in
 * sleep mode
 * @returns SL_WIFI_SLEEP_GRANTED to let the WFx go to sleep,
 * SL_WIFI_SLEEP_NOT_GRANTED otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_sleep_grant(sl_wfx_host_bus_transfer_type_t type,
				    sl_wfx_register_address_t address,
				    uint32_t length)
{
	(void)(type);
	(void)(address);
	(void)(length);

	return SL_STATUS_WIFI_SLEEP_GRANTED;
}

#if SL_WFX_DEBUG_MASK
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
void sl_wfx_host_log(const char *string, ...)
{
	va_list valist;
                                struct log_msg_ids src_level = {               
                                        .level = LOG_LEVEL_DBG,                       
                                        .domain_id = CONFIG_LOG_DOMAIN_ID,     
                                        .source_id = LOG_CURRENT_MODULE_ID()                  
                                };                                             
                                
	va_start(valist, string);
	vprintk(string, valist);
//	log_generic(src_level, string, valist, LOG_STRDUP_CHECK_EXEC);
	va_end(valist);
}
#else
void sl_wfx_host_log(const char *string, ...)
{
	ARG_UNUSED(string);
}
#endif
#endif

static void wfx_iface_init(struct net_if *iface)
{
	host_context.iface = iface;
	net_if_set_link_addr(iface, wifi.mac_addr_0.octet, sizeof(wifi.mac_addr_0), NET_LINK_ETHERNET);
	ethernet_init(iface);
}


static enum ethernet_hw_caps wfx_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return (ETHERNET_AUTO_NEGOTIATION_SET | ETHERNET_LINK_10BASE_T |
		ETHERNET_LINK_100BASE_T | ETHERNET_DUPLEX_SET);
}

static int wfx_send(const struct device *dev, struct net_pkt *pkt)
{
	uint16_t pkt_len = net_pkt_get_len(pkt);
//	sl_status_t result;
	sl_wfx_send_frame_req_t *tx_buffer = NULL;
	int res = 0;
	
//	uint16_t pad_len = 0;
	
//	if (total_len < 60) {
//		pad_len = 60-total_len;
//	}
	
	uint16_t alloc_len = SL_WFX_ROUND_UP(sizeof(sl_wfx_send_frame_req_t) + pkt_len, SL_WFX_SDIO_BLOCK_SIZE);

	tx_buffer = k_malloc(alloc_len);
	if (tx_buffer == NULL) {
		LOG_ERR("Failed to allocate buffer for tx packet");
		res = -EIO;
		goto error;
	}

	memset(tx_buffer, 0, sizeof(sl_wfx_send_frame_req_t));

	if (net_pkt_read(pkt, tx_buffer->body.packet_data, pkt_len)) {
		LOG_ERR("Failed to read packet into buffer");
		res = -EIO;
		goto error;
	}

//	for(int i = 0; i < pad_len; ++i) {
//		 tx_buffer->body.packet_data[total_len + i] = 0;
//	}

	wfx_bus_tx_frame.frame = tx_buffer;
	wfx_bus_tx_frame.data_length = pkt_len;
	wfx_bus_tx_frame.interface = SL_WFX_STA_INTERFACE;
	wfx_bus_tx_frame.priority = 0;
	wfx_send_result = 0;
	
	k_poll_signal_raise(&sl_wfx_bus_event_flag_tx, 1);
	
	if (k_sem_take(&wfx_send_confirm, K_FOREVER) != 0) {
		LOG_ERR("Failed to wait for send confirmation");
		res = -EIO;
		goto error;
	}
	
	if (wfx_send_result != SL_STATUS_OK) {
		LOG_ERR("Failed to tx packet: %d", wfx_send_result);
		res = -EIO;
		goto error;
	}
	
/*
	result = sl_wfx_send_ethernet_frame(tx_buffer, pkt_len, SL_WFX_STA_INTERFACE, 0);
	if (result != SL_STATUS_OK) {
		LOG_ERR("Failed to tx packet: %d", result);
		res = -EIO;
		goto error;
	}
*/

error:
	if (tx_buffer != NULL)
		k_free(tx_buffer);

	return res;
}

static int wfx_mgmt_scan(const struct device *dev, scan_result_cb_t cb)
{
	if (host_context.scan_cb) {
		return -EALREADY;
	}

	if (cb == NULL) {
		return -EIO;
	}

	host_context.scan_cb = cb;
	sl_status_t status = sl_wfx_send_scan_command(WFM_SCAN_MODE_ACTIVE,
						      NULL,
						      0,
						      NULL,
						      0,
						      NULL,
						      0,
						      NULL);

	if (status != SL_STATUS_OK) {
		host_context.scan_cb = NULL;
		return -EIO;
	}

	return 0;
}

static int wfx_mgmt_connect(const struct device *dev,
			    struct wifi_connect_req_params *params)
{
	sl_wfx_security_mode_t security_mode;
	uint16_t channel;

	if (params->security == WIFI_SECURITY_TYPE_PSK) {
		security_mode = WFM_SECURITY_MODE_WPA2_WPA1_PSK;
	} else if (params->security == WIFI_SECURITY_TYPE_NONE) {
		security_mode = WFM_SECURITY_MODE_OPEN;
	} else {
		LOG_ERR("Unknown security mode when connecting");
		return -EIO;
	}

	if (params->channel == WIFI_CHANNEL_ANY) {
		channel = 0;
	} else {
		channel = params->channel;
	}
	
	LOG_DBG("Connecting to %s(%d) on %d\n", params->ssid, params->ssid_length, channel);
	
	sl_status_t status = sl_wfx_send_join_command(params->ssid, params->ssid_length,
						      NULL, channel, security_mode, 0, 0,
						      params->psk, params->psk_length,
						      NULL, 0);

	return (status == SL_STATUS_OK) ? 0 : -EIO;
	
//TODO	call back stuff here, I think
}

static int wfx_mgmt_disconnect(const struct device *device)
{

	sl_status_t status = sl_wfx_send_disconnect_command();

	if (status == SL_STATUS_WIFI_WRONG_STATE) {
		return -EALREADY;
	}
	if (status != SL_STATUS_OK) {
		return -EIO;
	}
	return 0;
}

static const struct net_wifi_mgmt_offload wfx_api = {
	.ethernet.iface_api.init = wfx_iface_init,
	.ethernet.get_capabilities = wfx_get_capabilities,
	.ethernet.send = wfx_send,
	.scan = wfx_mgmt_scan,
	.connect = wfx_mgmt_connect,
	.disconnect = wfx_mgmt_disconnect,
};

static int wfx_init(const struct device *dev)
{
	host_context.dev = dev;

	return 0;
}

static void wfx_boot_task(int unused1, int unused2, int unused3)
{
	LOG_ERR("TASKY");

	sl_status_t status = sl_wfx_init(&wifi);
	LOG_DBG("FMAC Driver version %s", FMAC_DRIVER_VERSION_STRING);

	switch (status) {
	case SL_STATUS_OK:
		wifi.state = SL_WFX_STARTED;
		LOG_DBG("WF200 Firmware version %d.%d.%d",
		       wifi.firmware_major,
		       wifi.firmware_minor,
		       wifi.firmware_build);
		LOG_INF("WF200 initialization successful");
		break;
	case SL_STATUS_WIFI_INVALID_KEY:
		LOG_ERR("Failed to init WF200: Firmware keyset invalid");
		break;
	case SL_STATUS_WIFI_FIRMWARE_DOWNLOAD_TIMEOUT:
		LOG_ERR("Failed to init WF200: Firmware download timeout");
		break;
	case SL_STATUS_TIMEOUT:
		LOG_ERR("Failed to init WF200: Poll for value timeout");
		break;
	case SL_STATUS_FAIL:
		LOG_ERR("Failed to init WF200: Error");
		break;
	default:
		LOG_ERR("Failed to init WF200: Unknown error");
	}

#ifdef CONFIG_WIFI_WFX_SLEEP
#ifdef CONFIG_WIFI_WFX_BUS_SDIO
  status = sl_wfx_host_switch_to_wirq();
#endif
#endif
	connectTest();

}


static void connectTest() {
	struct wifi_connect_req_params params;
	params.security = WIFI_SECURITY_TYPE_PSK;
	params.channel = WIFI_CHANNEL_ANY;
	params.psk = "jubeltorsk";
	params.psk_length = strlen(params.psk);
	params.ssid = "dev";
	params.ssid_length = strlen(params.ssid);
	wfx_mgmt_connect(host_context.dev, &params);
}

ETH_NET_DEVICE_INIT(wfx, CONFIG_WIFI_WFX_NAME, wfx_init,
		    device_pm_control_nop, &host_context, NULL,
		    CONFIG_ETH_INIT_PRIORITY, &wfx_api, NET_ETH_MTU);

// TODO: How do I find max thread size properly?
K_THREAD_DEFINE(wfx_boot_tid, 4096,
                wfx_boot_task, NULL, NULL, NULL,
                5, 0, 0);
                