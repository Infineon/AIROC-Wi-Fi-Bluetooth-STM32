/*******************************************************************************
* \file cybt_platform_internal.h
*
* \brief
* Defines variables specific to 20829 platform.
*
********************************************************************************/

#ifndef CYBT_PLATFORM_INTERNAL_H
#define CYBT_PLATFORM_INTERNAL_H

#include "cyabs_rtos.h"

#ifdef FPGA_TEST_PLATFORM
#define PATCH_DOWNLOAD_FN    NULL
#define GET_TRNG_FN          NULL
#else
#define PATCH_DOWNLOAD_FN    bt_post_reset_cback
#endif /* FPGA_TEST_PLATFORM */

#define BLESS_CONTROLLER     WICED_FALSE

#define LOCK                             (true)
#define UNLOCK                           (false)
#define CONTROLLER_SLEEP(enable)         (Cy_BTSS_PowerDep(enable))

#define ENABLE_SLEEP_MODE()              (bt_enable_sleep_mode())

#if(RELBUF_DELAY_IN_US > 0)
#define RELBUF_DELAY()  Cy_SysLib_DelayUs(RELBUF_DELAY_IN_US)
#else
#define RELBUF_DELAY()
#endif

void cybt_platform_btss_get_trng(uint8_t *p_rand, uint8_t *p_len);

#endif // CYBT_PLATFORM_INTERNAL_H

