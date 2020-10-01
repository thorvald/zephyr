#define LOG_MODULE_NAME wfx
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_DECLARE(LOG_MODULE_NAME);

#include "sl_wfx.h"
#include "sl_wfx_host_api.h"

#include "em_device.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"
#include "sdiodrv.h"
#include "sdio.h"

#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/gpio.h>
#include <power/power.h>

#include "sl_wfx_host_cfg.h"
#include "sl_wfx_host.h"

// #define SL_WFX_SDIO_BLOCK_SIZE 64
#define SDIO_ACTION_COMPLETION_TIMEOUT_MS     5000

extern uint8_t wf200_interrupt_event;

static void com_evt_callback(SDIODRV_Event_t evt, uint32_t error);
static void sdio_irq_callback(void);

static SDIODRV_Init_t sdiodrv_init = {
	.instance = SDIO,
	.freq = 50000000,
	.portLocationClk = 0,
	.portLocationCmd = 0,
	.portLocationCd = 0,
	.portLocationWp = 0,
	.portLocationDat = 0,
	.clockSource = cmuSelect_AUXHFRCO,
	.transferWidth = SDIO_TRANSFER_WIDTH_4BIT
};

static SDIODRV_Handle_t sdiodrv_handle;
static SDIODRV_Callbacks_t sdiodrv_callbacks;

static uint16_t rca;
static bool sdio_enabled = false;
static volatile bool sdio_error = false;
static volatile bool action_done = false;
static SDIODRV_Event_t waited_evt = 0;
#ifdef CONFIG_WIFI_WFX_SLEEP
static bool useWIRQ = false;
#endif

/****************************************************************************************************//**
 *                                     sl_wfx_host_init_bus()
 *
 * @brief    Initializes the communications bus.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_init_bus(void)
{
	sl_status_t status = SL_STATUS_FAIL;
	int res;

	res = SDIODRV_Init(&sdiodrv_handle, &sdiodrv_init);
	if (res == 0) {
		res = SDIODRV_DeviceInitAndIdent(&sdiodrv_handle, &rca);
		if (res == 0) {
			res = SDIODRV_SelectCard(&sdiodrv_handle, rca);
			if (res == 0) {
				sdiodrv_callbacks.comEvtCb = com_evt_callback;
				sdiodrv_callbacks.cardInterruptCb = sdio_irq_callback;
				SDIODRV_RegisterCallbacks(&sdiodrv_callbacks);

				sl_wfx_host_enable_sdio();
				status = SL_STATUS_OK;
			}
		}
	}

	return status;
}

/****************************************************************************************************//**
 *                                     sl_wfx_host_deinit_bus()
 *
 * @brief    De-initializes the communications bus.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_deinit_bus(void)
{
	sl_status_t status = SL_STATUS_FAIL;
	int res;

	res = SDIODRV_DeInit(&sdiodrv_handle);
	if (res == 0) {
		status = SL_STATUS_OK;
	}

	return status;
}

static void com_evt_callback(SDIODRV_Event_t evt, uint32_t error)
{
//	printk("EVT %x %x %x\n", evt, error, waited_evt);
	if ((evt & SDIODRV_EVENT_CARD_INTERRUPT) != 0) {
		k_poll_signal_raise(&sl_wfx_bus_event_flag_rx, 1);
	}

	if ((evt & waited_evt) != 0) {
		action_done = true;
		if (((evt & SDIODRV_EVENT_COM_ERROR) != 0)
		    || (error != SDIODRV_ERROR_NONE)) {
			sdio_error = true;
		}
	}
}

static sl_status_t prep_action_wait(SDIODRV_Event_t evt)
{
	if (waited_evt != 0)
		return SL_STATUS_FAIL;
		
	waited_evt = evt;
	sdio_error = false;
	action_done = false;
	
	return SL_STATUS_OK;
}

static sl_status_t wait_action_completion(uint32_t timeout_ms)
{
	uint64_t tmo = k_uptime_get() + timeout_ms;
	sl_status_t status = SL_STATUS_TIMEOUT;

	// FIXME overlap after running almost 50days
	while (k_uptime_get() < tmo) {
		if (action_done) {
			status = SL_STATUS_OK;
			break;
		} else if (sdio_error) {
			status = SL_STATUS_FAIL;
			break;
		}
		k_yield();
	}

	waited_evt = 0;
	return status;
}

sl_status_t sl_wfx_host_sdio_transfer_cmd52(sl_wfx_host_bus_transfer_type_t type, uint8_t function, uint32_t address, uint8_t *buffer)
{
	prep_action_wait(SDIODRV_EVENT_CMD_COMPLETE);

	if (type == SL_WFX_BUS_READ) {
		SDIODRV_IOReadWriteDirect(&sdiodrv_handle, SDIODRV_IO_OP_READ, function, address, buffer);
	} else {
		SDIODRV_IOReadWriteDirect(&sdiodrv_handle, SDIODRV_IO_OP_WRITE, function, address, buffer);
	}

	// Wait for the operation completion
	sl_status_t status = wait_action_completion(SDIO_ACTION_COMPLETION_TIMEOUT_MS);
	if (status != SL_STATUS_OK) {
		SDIODRV_Abort(&sdiodrv_handle, function);
	}
	return status;
}

sl_status_t sl_wfx_host_sdio_transfer_cmd53(sl_wfx_host_bus_transfer_type_t type, uint8_t function, uint32_t address, uint8_t *buffer, uint16_t buffer_length)
{
	uint32_t dummy_data;
	uint16_t block_count;
	uint8_t *buf_tmp = buffer;
	
	prep_action_wait(SDIODRV_EVENT_TRANS_COMPLETE);

	// Clear the processing flags
	sdio_error = false;
	action_done = false;

	// Ensure a valid buffer address for each operations (for SDIO DMA).
	if (buffer == NULL) {
		buf_tmp = (uint8_t *)&dummy_data;
	}

	if (buffer_length >= 512) {
		block_count = (buffer_length / SL_WFX_SDIO_BLOCK_SIZE) + (((buffer_length % SL_WFX_SDIO_BLOCK_SIZE) == 0) ? 0 : 1);

		SDIO_ConfigureTransfer(sdiodrv_handle.init.instance, SL_WFX_SDIO_BLOCK_SIZE, block_count);

		if (type == SL_WFX_BUS_READ) {
			SDIODRV_IOReadWriteExtendedBlocks(&sdiodrv_handle, SDIODRV_IO_OP_READ, function, address, block_count, buf_tmp);
		} else {
			SDIODRV_IOReadWriteExtendedBlocks(&sdiodrv_handle, SDIODRV_IO_OP_WRITE, function, address, block_count, buf_tmp);
		}
	} else {

		SDIO_ConfigureTransfer(sdiodrv_handle.init.instance, buffer_length, 0);

		if (type == SL_WFX_BUS_READ) {
			SDIODRV_IOReadWriteExtendedBytes(&sdiodrv_handle, SDIODRV_IO_OP_READ, function, address, buffer_length, buf_tmp);
		} else {
			SDIODRV_IOReadWriteExtendedBytes(&sdiodrv_handle, SDIODRV_IO_OP_WRITE, function, address, buffer_length, buf_tmp);
		}
	}

	// Wait for the operation completion
	sl_status_t status = wait_action_completion(SDIO_ACTION_COMPLETION_TIMEOUT_MS);
	if (status != SL_STATUS_OK) {
		SDIODRV_Abort(&sdiodrv_handle, function);
	}
	return status;
}

sl_status_t sl_wfx_host_sdio_enable_high_speed_mode(void)
{
	SDIODRV_EnableHighSpeed(&sdiodrv_handle, 1);
	return SL_STATUS_OK;
}

static void sdio_irq_callback(void)
{
//	LOG_INF("InterruptBack");
	k_poll_signal_raise(&sl_wfx_bus_event_flag_rx, 1);
	wf200_interrupt_event = 1;
}

/****************************************************************************************************//**
 *                                  sl_wfx_host_enable_platform_interrupt()
 *
 * @brief    Enable interrupts on the host side.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_enable_platform_interrupt(void)
{
#ifdef CONFIG_WIFI_WFX_SLEEP
	if (useWIRQ) {
		printk("intEnable\n");
		// GPIO_ExtIntConfig(WFX_HOST_CFG_WIRQPORT, WFX_HOST_CFG_WIRQPIN, WFX_HOST_CFG_IRQ, true, false, true);
		return SL_STATUS_OK;
	} else
#endif
	{
		SDIODRV_EnableInterrupts(&sdiodrv_handle,
					 SDIO_IEN_CARDINTSEN | SDIO_IFENC_CARDINTEN,
					 1);
	}
	return SL_STATUS_OK;
}

/****************************************************************************************************//**
 *                                 sl_wfx_host_disable_platform_interrupt()
 *
 * @brief    Disable interrupts on the host side.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_disable_platform_interrupt(void)
{
#ifdef CONFIG_WIFI_WFX_SLEEP
	if (useWIRQ) {
		printk("IntDiable\n");
		GPIO_IntDisable(SL_WFX_HOST_CFG_IRQ);
		return SL_STATUS_OK;
	} else
#endif
	{
		SDIODRV_EnableInterrupts(&sdiodrv_handle,
					 SDIO_IEN_CARDINTSEN | SDIO_IFENC_CARDINTEN,
					 0);
	}
	return SL_STATUS_OK;
}

#ifdef CONFIG_WIFI_WFX_SLEEP

static struct gpio_callback irq_cb_data;

static void irq_pin_triggered(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("IntTrigger %x\n", pins);
	 k_poll_signal_raise(&sl_wfx_bus_event_flag_rx, 1);
	printk("Raised\n");
}

sl_status_t sl_wfx_host_switch_to_wirq(void)
{
	uint32_t value32;
	
	printk("SwitchWirq\n");
	
	const struct device *irqpin = device_get_binding("GPIO_A");
	if (irqpin == NULL) {
		printk("NoPin\n");
		return SL_STATUS_FAIL;
	}
	
	// High or low active?
	int ret = gpio_pin_configure(irqpin, SL_WFX_HOST_CFG_WIRQPIN, GPIO_INPUT);
	if (ret != 0) {
		printk("Config\n");
		return SL_STATUS_FAIL;
	}
	
	ret = gpio_pin_interrupt_configure(irqpin, SL_WFX_HOST_CFG_WIRQPIN, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("IntConfig\n");
		return SL_STATUS_FAIL;
	}
	
	gpio_init_callback(&irq_cb_data, irq_pin_triggered, BIT(SL_WFX_HOST_CFG_WIRQPIN));
	gpio_add_callback(irqpin, &irq_cb_data);

/*	
	

	GPIO_ExtIntConfig(SL_WFX_HOST_CFG_WIRQPORT,
			  SL_WFX_HOST_CFG_WIRQPIN,
			  SL_WFX_HOST_CFG_IRQ,
			  true,
			  false,
			  true);
*/

	printk("CB added\n");

	sl_wfx_reg_read_32(SL_WFX_CONFIG_REG_ID, &value32);
	value32 |= (1 << 15);
	sl_wfx_reg_write_32(SL_WFX_CONFIG_REG_ID, value32);
	useWIRQ = true;
	printk("Returning\n");
	return SL_STATUS_OK;
}
#endif

sl_status_t sl_wfx_host_enable_sdio(void)
{
  if (sdio_enabled == false) {
    printk("Enabling\n");
#ifdef CONFIG_SYS_PM_STATE_LOCK    
	sys_pm_ctrl_disable_state(SYS_POWER_STATE_SLEEP_2);
#endif
    CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
    // Re-enable SDIO clock
    CMU_ClockEnable(cmuClock_SDIO, true);
    while ((CMU->STATUS & CMU_STATUS_SDIOCLKENS) == 0UL) ;
    SDIO->CLOCKCTRL |= SDIO_CLOCKCTRL_INTCLKEN;
    while ((SDIO->CLOCKCTRL & SDIO_CLOCKCTRL_INTCLKSTABLE) == 0) ;
    SDIO->CLOCKCTRL |= SDIO_CLOCKCTRL_SDCLKEN;
    sdio_enabled = true;
    printk("Enabled\n");
  }
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_disable_sdio(void)
{
  if (sdio_enabled == true) {
     printk("Disabling\n");
    // Disable internal SDIO peripheral clock
    SDIO->CLOCKCTRL &= ~(_SDIO_CLOCKCTRL_SDCLKEN_MASK | _SDIO_CLOCKCTRL_INTCLKEN_MASK);
    // Disable MCU clock tree SDIO clock
    CMU_ClockEnable(cmuClock_SDIO, false);
    while ((CMU->STATUS & CMU_STATUS_SDIOCLKENS) != 0) ;
    CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
    sdio_enabled = false;
#ifdef CONFIG_SYS_PM_STATE_LOCK    
	sys_pm_ctrl_enable_state(SYS_POWER_STATE_SLEEP_2);
#endif
  }
  return SL_STATUS_OK;
}

// MAKE THE ABOVE WORK, since we need interrupts to wake up somehow
// Need SDIO interrupt until firmware fully loaded

