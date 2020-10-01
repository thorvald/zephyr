# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_WFX)
  zephyr_include_directories(./)

  zephyr_sources(
    sl_wfx_host_bus.c
    sl_wfx_host.c
    )
  zephyr_sources_ifdef(CONFIG_WIFI_WFX_SECURE_LINK
    sl_wfx_host_crypto.c
    )
  zephyr_sources_ifdef(CONFIG_WIFI_WFX_BUS_SDIO
    sl_wfx_host_sdio.c
  )
  zephyr_sources_ifdef(CONFIG_WIFI_WFX_BUS_SPI
    sl_wfx_host_spi.c
  )

endif()

