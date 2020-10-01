/*
 * Copyright (c) 2019 Interay Solutions B.V.
 * Copyright (c) 2019 Oane Kingma
 * Copyright (c) 2020 Thorvald Natvig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <power/power.h>
#ifdef CONFIG_WIFI_WFX
#include <sl_wfx.h>
#endif
#include "em_cmu.h"
#include "em_rmu.h"
#include "board.h"

#define SL_WFX_HOST_CFG_RESET_PORT          gpioPortF
#define SL_WFX_HOST_CFG_RESET_PIN           12

#define SL_WFX_HOST_CFG_WUP_PORT            gpioPortE
#define SL_WFX_HOST_CFG_WUP_PIN             4
#define SL_WFX_HOST_CFG_WIRQPORT            gpioPortA                    /* WIRQ port*/
#define SL_WFX_HOST_CFG_WIRQPIN             8                            /* WIRQ pin */
#define SL_WFX_HOST_CFG_IRQ                 10
#define LP_CLK_PORT                         gpioPortA
#define LP_CLK_PIN                          12


static int efm32gg_slwstk6121a_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	
#ifdef CONFIG_ETH_GECKO
	const struct device *cur_dev;

	/* Configure ethernet reference clock */
	cur_dev = device_get_binding(ETH_REF_CLK_GPIO_NAME);
	if (!cur_dev) {
		printk("Ethernet reference clock gpio port was not found!\n");
		return -ENODEV;
	}

	gpio_pin_configure(cur_dev, ETH_REF_CLK_GPIO_PIN, GPIO_OUTPUT);
	gpio_pin_set(cur_dev, ETH_REF_CLK_GPIO_PIN, 0);

	/* enable CMU_CLK2 as RMII reference clock */
	CMU->CTRL |= CMU_CTRL_CLKOUTSEL2_HFXO;
	CMU->ROUTELOC0 = (CMU->ROUTELOC0 & ~_CMU_ROUTELOC0_CLKOUT2LOC_MASK) |
			 (ETH_REF_CLK_LOCATION << _CMU_ROUTELOC0_CLKOUT2LOC_SHIFT);
	CMU->ROUTEPEN |= CMU_ROUTEPEN_CLKOUT2PEN;
#endif /* CONFIG_ETH_GECKO */

#ifdef CONFIG_SYS_PM_STATE_LOCK
//  sys_pm_ctrl_disable_state(SYS_POWER_STATE_SLEEP_2);
  sys_pm_ctrl_disable_state(SYS_POWER_STATE_SLEEP_3);
#endif

  CMU_ClockEnable(cmuClock_SDIOREF, true);

  CMU_AUXHFRCOFreqSet(cmuAUXHFRCOFreq_48M0Hz);
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
  
  // Configure WF200 reset pin.
  GPIO_PinModeSet(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN, gpioModePushPull, 0);
  // Configure WF200 WUP pin.
  GPIO_PinModeSet(SL_WFX_HOST_CFG_WUP_PORT, SL_WFX_HOST_CFG_WUP_PIN, gpioModePushPull, 0);
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
      // GPIO used as IRQ
  GPIO_PinModeSet(SL_WFX_HOST_CFG_WIRQPORT,  SL_WFX_HOST_CFG_WIRQPIN,  gpioModeInputPull,  0);
  // SDIO Pull-ups
  GPIO_PinModeSet(gpioPortD,  0,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  1,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  2,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  3,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  5,  gpioModeDisabled,  1);

  //WF200 LF CLK
  CMU->CTRL      |= CMU_CTRL_CLKOUTSEL0_LFXO;
  CMU->ROUTEPEN  |= CMU_ROUTEPEN_CLKOUT0PEN;
  CMU->ROUTELOC0 |= CMU_ROUTELOC0_CLKOUT0LOC_LOC5;
  GPIO_PinModeSet(LP_CLK_PORT,  LP_CLK_PIN,  gpioModePushPull,  0);

	printf("Banana\n");

/*
sl_wfx_context_t wifi;
	
	sl_status_t status = sl_wfx_init(&wifi);
  printf("FMAC Driver version    %s\r\n", FMAC_DRIVER_VERSION_STRING);

  switch (status) {
    case SL_STATUS_OK:
      wifi.state = SL_WFX_STARTED;
      printf("WF200 Firmware version %d.%d.%d\r\n",
             wifi.firmware_major,
             wifi.firmware_minor,
             wifi.firmware_build);
      printf("WF200 initialization successful\r\n");
      break;
    case SL_STATUS_WIFI_INVALID_KEY:
      printf("Failed to init WF200: Firmware keyset invalid\r\n");
      break;
    case SL_STATUS_WIFI_FIRMWARE_DOWNLOAD_TIMEOUT:
      printf("Failed to init WF200: Firmware download timeout\r\n");
      break;
    case SL_STATUS_TIMEOUT:
      printf("Failed to init WF200: Poll for value timeout\r\n");
      break;
    case SL_STATUS_FAIL:
      printf("Failed to init WF200: Error\r\n");
      break;
    default:
      printf("Failed to init WF200: Unknown error\r\n");
  }
*/    

	return 0;
}

/* needs to be done after GPIO driver init and device tree available */
SYS_INIT(efm32gg_slwstk6121a_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
