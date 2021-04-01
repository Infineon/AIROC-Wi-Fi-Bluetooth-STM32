/***************************************************************************//**
* \file stm32_cyhal_lptimer.c
*
* \brief
* Provides a high level interface for interacting with the STM32 Low-Power Timer.
* This interface abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
*
*******************************************************************************
* \copyright
* Copyright 2021 Cypress Semiconductor Corporation
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cmsis_compiler.h"
#include "cyhal_system.h"
#include "cyhal_lptimer.h"
#include "stm32_cyhal_lptimer_ex.h"

#if defined(HAL_LPTIM_MODULE_ENABLED)

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
*      Private macros
*******************************************************************************/

#define LPTIMER_16BIT

#define _CYHAL_LPTIMER_MAX_INSTANCES            (2U)
#define _CYHAL_LPTIMER_MIN_DELAY_TICKS          (3U)       /* minimum amount of lfclk cycles of that
                                                              LPTIMER can delay for. */

#define _CYHAL_LPTIMER_MAX_DELAY_TICKS          (0xfff0UL) /* ~36hours, Not set to 0xffffffff to
                                                              avoid C0 and C1 both overflowing */

/* consider LPTIMER being driven by 32768Hz and 32kHz */
#define _CYHAL_LPTIMER_CLK_FREQ_HZ              (32768U)

/* Adjust for period to be exactly 2 sec */
#if (_CYHAL_LPTIMER_CLK_FREQ_HZ == 32768U)
    #define _CYHAL_LPTIMER_MAX_PERIOD           (65535U)
#elif (_CYHAL_LPTIMER_CLK_FREQ_HZ == 32000U)
    #define _CYHAL_LPTIMER_MAX_PERIOD           (63999U)
#endif

#define _CYHAL_LPTIMER_MAX_COMPARE _CYHAL_LPTIMER_MAX_PERIOD


/*******************************************************************************
*      Private types
*******************************************************************************/

typedef struct stm32_cyhal_lptimer_structs_t
{
    LPTIM_HandleTypeDef*            hlptim;
    cyhal_lptimer_t*                obj;
    uint32_t                        occupied;
} stm32_cyhal_lptimer_structs_t;


/*******************************************************************************
*      Private globals
*******************************************************************************/

static stm32_cyhal_lptimer_structs_t _cyhal_lptimer_structs[_CYHAL_LPTIMER_MAX_INSTANCES] =
    { 0 }; /* init with zeros */


//--------------------------------------------------------------------------------------------------
// stm32_cypal_lptimer_hw_init
//--------------------------------------------------------------------------------------------------
uint32_t stm32_cypal_lptimer_hw_init(LPTIM_HandleTypeDef* hlptim)
{
    stm32_cyhal_lptimer_structs_t* struct_ptr;
    for (uint32_t i = 0; i < _CYHAL_LPTIMER_MAX_INSTANCES; i++)
    {
        /* create a pool of available handles */
        struct_ptr = &_cyhal_lptimer_structs[i];
        if (NULL == struct_ptr->hlptim)
        {
            /* store a handle */
            struct_ptr->hlptim = hlptim;
            return 0; /* success */
        }
    }
    return 1; /* all structs are occupied already */
}


//--------------------------------------------------------------------------------------------------
// _stm32_cypal_lptimer_get_irqn
//--------------------------------------------------------------------------------------------------
cy_rslt_t _stm32_cypal_lptimer_get_irqn(LPTIM_HandleTypeDef* hlptim, IRQn_Type* irqn)
{
    cy_rslt_t ret = CY_RSLT_SUCCESS;

    if (irqn == NULL)
    {
        ret = CY_RSLT_TYPE_ERROR;
    }
    else

    #if defined (LPTIM1)
    if (hlptim->Instance == LPTIM1)
    {
        *irqn = LPTIM1_IRQn;
    }
    else
    #endif

    #if defined (LPTIM2)
    if (hlptim->Instance == LPTIM2)
    {
        *irqn = LPTIM2_IRQn;
    }
    else
    #endif

    #if defined (LPTIM3)
    if (hlptim->Instance == LPTIM3)
    {
        *irqn = LPTIM3_IRQn;
    }
    else
    #endif

    #if defined (LPTIM4)
    if (hlptim->Instance == LPTIM4)
    {
        *irqn = LPTIM4_IRQn;
    }
    else
    #endif

    #if defined (LPTIM5)
    if (hlptim->Instance == LPTIM5)
    {
        *irqn = LPTIM5_IRQn;
    }
    else
    #endif
    {
        ret = CY_RSLT_TYPE_ERROR;
        assert_param(false); /* wrong instance */
    }

    return ret;
}


//--------------------------------------------------------------------------------------------------
// _cyhal_lptimer_compare_match_callback
//--------------------------------------------------------------------------------------------------
void _cyhal_lptimer_compare_match_callback(LPTIM_HandleTypeDef* hlptim)
{
    stm32_cyhal_lptimer_structs_t* struct_ptr;
    for (uint32_t i=0; i < _CYHAL_LPTIMER_MAX_INSTANCES; i++)
    {
        struct_ptr = &_cyhal_lptimer_structs[i];

        if ((hlptim == struct_ptr->hlptim) && (struct_ptr->obj->event_enable))
        {
            cyhal_lptimer_t* obj = struct_ptr->obj;
            ((cyhal_lptimer_event_callback_t)obj->callback)(obj->callback_arg,
                                                            CYHAL_LPTIMER_COMPARE_MATCH);
        }
    }
}


//--------------------------------------------------------------------------------------------------
// _cyhal_lptimer_autoreload_match_callback
//--------------------------------------------------------------------------------------------------
void _cyhal_lptimer_autoreload_match_callback(LPTIM_HandleTypeDef* hlptim)
{
    for (uint32_t i=0; i < _CYHAL_LPTIMER_MAX_INSTANCES; i++)
    {
        if (hlptim == _cyhal_lptimer_structs[i].hlptim)
        {
            _cyhal_lptimer_structs[i].obj->cur_time_in_lpticks += _CYHAL_LPTIMER_MAX_PERIOD;
        }
    }
}


#if defined(LPTIMER_16BIT)
/* Implementing 16-bit version first */

//--------------------------------------------------------------------------------------------------
// _cyhal_lptimer_set_match_common
//--------------------------------------------------------------------------------------------------
static uint32_t _cyhal_lptimer_set_match_common(cyhal_lptimer_t* obj, uint32_t ticks)
{
    uint32_t critical_section = cyhal_system_critical_section_enter();

    if (ticks > _CYHAL_LPTIMER_MAX_COMPARE)
    {
        ticks = _CYHAL_LPTIMER_MAX_COMPARE;
    }

    __HAL_LPTIM_COMPARE_SET(obj->hlptimer, ticks);

    cyhal_system_critical_section_exit(critical_section);

    return CY_RSLT_SUCCESS;
}


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_set_match
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_lptimer_set_match(cyhal_lptimer_t* obj, uint32_t ticks)
{
    return _cyhal_lptimer_set_match_common(obj, ticks);
}


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_set_delay
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_lptimer_set_delay(cyhal_lptimer_t* obj, uint32_t delay)
{
    if (delay <= _CYHAL_LPTIMER_MIN_DELAY_TICKS)
    {
        delay = _CYHAL_LPTIMER_MIN_DELAY_TICKS;
    }
    if (delay > _CYHAL_LPTIMER_MAX_DELAY_TICKS)
    {
        delay = _CYHAL_LPTIMER_MAX_DELAY_TICKS;
    }

    return _cyhal_lptimer_set_match_common(obj, cyhal_lptimer_read(obj) + delay);
}


#endif /* defined(LPTIMER_16BIT) */


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_init
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_lptimer_init(cyhal_lptimer_t* obj)
{
    cy_rslt_t                      rslt = CY_RSLT_SUCCESS;
    stm32_cyhal_lptimer_structs_t* struct_ptr;
    IRQn_Type                      irqn;

    /* Check the parameters */
    assert_param(NULL != obj);

    obj->hlptimer = NULL;
    obj->cur_time_in_lpticks = 0u;

    for (uint32_t i = 0; i < _CYHAL_LPTIMER_MAX_INSTANCES; i++)
    {
        struct_ptr = &_cyhal_lptimer_structs[i];
        if (struct_ptr->occupied == 0)
        {   /* make obj refer an added handle  */
            obj->hlptimer        = struct_ptr->hlptim;
            struct_ptr->occupied = 1;
            struct_ptr->obj = obj;
            break;
        }
    }

    /* INFO: cyhal_lptimer_init() suggests IRQ initialization */
    if ((NULL != obj->hlptimer) &&
        (HAL_LPTIM_STATE_RESET != obj->hlptimer->State))
    {   /*
         * LPTIMER is initialized
         * LPTIMER is enabled
         *
         * It also contains a call to HAL_LPTIM_MspInit where:
         * Clock is enabled
         *
         * LPTIMER IRQs are NOT enabled here
         */
        if (HAL_LPTIM_Init(obj->hlptimer) != HAL_OK)
        {
            cyhal_lptimer_free(obj);
            rslt = CY_RSLT_TYPE_ERROR;
        }

        /* Enable interrupt */
        rslt = _stm32_cypal_lptimer_get_irqn(obj->hlptimer, &irqn);
        if (rslt == CY_RSLT_SUCCESS)
        {
            HAL_NVIC_SetPriority(irqn, 0, 0);
            HAL_NVIC_EnableIRQ(irqn);
        }

        /* Start LPtimer */
        if (rslt == CY_RSLT_SUCCESS)
        {
            HAL_StatusTypeDef hal_ret;
            /* Set ARR to MAX(16bit) */
            /* Set CMP to MAX(16bit) */
            hal_ret = HAL_LPTIM_TimeOut_Start_IT(obj->hlptimer,
                                                 _CYHAL_LPTIMER_MAX_PERIOD,
                                                 _CYHAL_LPTIMER_MAX_COMPARE);

            /* Register  Auto-reload match callback */
            (void)HAL_LPTIM_RegisterCallback(obj->hlptimer, HAL_LPTIM_AUTORELOAD_MATCH_CB_ID,
                                             &_cyhal_lptimer_autoreload_match_callback);

            /* Enable Autoreload write complete interrupt */
            __HAL_LPTIM_ENABLE_IT(obj->hlptimer, LPTIM_IER_ARRMIE);

            rslt = (hal_ret == HAL_OK) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
        }
    }
    else
    {
        rslt = CY_RSLT_TYPE_ERROR;
    }
    return rslt;
}


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_free
//--------------------------------------------------------------------------------------------------
void cyhal_lptimer_free(cyhal_lptimer_t* obj)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    (void)HAL_LPTIM_TimeOut_Stop_IT(obj->hlptimer);
    /*
     * LPTIMER is de-initialized
     * LPTIMER is disabled
     *
     * It also contains a call to HAL_LPTIM_MspDeInit where:
     * LPTIMER IRQs are disabled
     * Clock is disabled
     */
    (void)HAL_LPTIM_DeInit(obj->hlptimer);

    stm32_cyhal_lptimer_structs_t* struct_ptr;
    for (uint32_t i=0; i < _CYHAL_LPTIMER_MAX_INSTANCES; i++)
    {
        struct_ptr = &_cyhal_lptimer_structs[i];
        if (struct_ptr->hlptim == obj->hlptimer)
        {
            struct_ptr->occupied = 0;
            break;
        }
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_read
//--------------------------------------------------------------------------------------------------
uint32_t cyhal_lptimer_read(const cyhal_lptimer_t* obj)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    /* Read current tick */
    uint32_t  lpticks = HAL_LPTIM_ReadCounter(obj->hlptimer);
    return obj->cur_time_in_lpticks + lpticks;
}


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_register_callback
//--------------------------------------------------------------------------------------------------
void cyhal_lptimer_register_callback(cyhal_lptimer_t* obj, cyhal_lptimer_event_callback_t callback,
                                     void* callback_arg)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    /* store of callback pointer */
    obj->callback = (void*)callback;

    /* store of callback arg */
    obj->callback_arg = callback_arg;
}


//--------------------------------------------------------------------------------------------------
// cyhal_lptimer_enable_event
//--------------------------------------------------------------------------------------------------
void cyhal_lptimer_enable_event(cyhal_lptimer_t* obj, cyhal_lptimer_event_t event,
                                uint8_t intr_priority, bool enable)
{
    (void)event;
    IRQn_Type irqn;

    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(event == CYHAL_LPTIMER_COMPARE_MATCH);

    obj->event_enable = enable;

    if (enable)
    {
        (void)_stm32_cypal_lptimer_get_irqn(obj->hlptimer, &irqn);
        /* Update priority */
        HAL_NVIC_SetPriority(irqn, intr_priority, 0);

        /* Register  Compare match callback */
        (void)HAL_LPTIM_RegisterCallback(obj->hlptimer, HAL_LPTIM_COMPARE_MATCH_CB_ID,
                                         &_cyhal_lptimer_compare_match_callback);
    }
    else
    {
        /* Un-register Compare match callback */
        (void)HAL_LPTIM_UnRegisterCallback(obj->hlptimer, HAL_LPTIM_COMPARE_MATCH_CB_ID);
    }
}


#if defined(__cplusplus)
}
#endif

#endif /* (HAL_LPTIM_MODULE_ENABLED) */
