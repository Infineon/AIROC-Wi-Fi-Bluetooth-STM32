/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
/****************************************************************************
**
** Name:         wiced_data_types.h
**
** Description:  AIROC data types header file for VS2010 projects
**
** Copyright (c) Cypress Semiconductor
**
******************************************************************************/

#ifndef WICED_DATA_TYPES_H
#define WICED_DATA_TYPES_H

#include <stdint.h>
#include <string.h>

/**
 *  @ingroup     gentypes
 *
 *  @{
 */

#ifndef WICED_FALSE
#define WICED_FALSE 0   /**< AIROC false */
#endif // !WICED_FALSE
#ifndef WICED_TRUE
#define WICED_TRUE  1   /**< AIROC true */
#endif // !WICED_TRUE

#ifndef FALSE
#define FALSE 0         /**< false */
#endif

#ifndef TRUE
#define TRUE  1         /**< true */
#endif

#ifdef __ARM__
#define WICED_BT_STRUCT_PACKED  __packed() struct               /**< packed structure */
#define WICED_BT_UNION_PACKED  __packed() union                 /**< packed union */
#elif defined(TOOLCHAIN_gnu) || defined(COMPILER_GNU)
#define WICED_BT_STRUCT_PACKED struct __attribute__((packed))   /**< packed structure */
#define WICED_BT_UNION_PACKED  union  __attribute__((packed))   /**< packed union */
#else
#define WICED_BT_STRUCT_PACKED  struct                          /**< packed structure */
#define WICED_BT_UNION_PACKED   union                           /**< packed union */
#endif

/** Surpress Warnings */
#define WICED_SUPPRESS_WARNINGS(m) if((m)){;}

/* Suppress unused variable warning */
#ifndef UNUSED_VARIABLE
/** Unused Variable */
#define UNUSED_VARIABLE(x) /*@-noeffect@*/ ( (void)(x) ) /*@+noeffect@*/
#endif

/**  To prevent complier to optimize with LDM and STM instructions */
#define WICED_MEMCPY(a, b, c)       memcpy((void*)(a), (const void*)(b), c)     /**< AIROC Memory copy*/
#define WICED_MEMSET(a, b, c)       memset((void*)(a), b, c)                    /**< AIROC Memory set */
#define WICED_MEMMOVE(a, b, c)      memmove((void*)(a), (const void*)(b), c)    /**< AIROC Memory move*/
#define WICED_MEMCMP(a, b, c)       memcmp((void*)(a), (const void*)(b), c)     /**< AIROC Memory compare*/

/** MACRO to convert an address into a 4 byte buffer*/
#define ADDRESS_TO_BUFFER(b,a)  uintptr_t addr=(uintptr_t)a; for(int i = 0; i < 4; i++) {b[3-i] = (addr >> (i * 8)) & 0xFF;}

/** AIROC Boolean */
typedef unsigned int   wiced_bool_t;

/** Function prototypes to lock and unlock (typically using a mutex). The context
** pointer may be NULL, depending on implementation.
*/
typedef struct {
    void* p_lock_context;                           /**< lock context pointer */
    void (*pf_lock_func)(void * p_lock_context);    /**< Lock function pointer */
    void (*pf_unlock_func)(void * p_lock_context);  /**< Unlock function pointer */
}wiced_bt_lock_t;

 /**
  * Exception callback for stack, controller & porting layer exceptions:
  *
  * Invoked by porting layer on stack/controller/porting layer exceptions and critical unrecoverable errors
  *
  * @param[in] code    : Exception code - Numerical value of an exception
  *                      (See CYBT_STACK_BASE_EXCEPTION in wiced_bt_stack_platform.h for stack exceptions
  *                      See CYBT_CONTROLLER_BASE_EXCEPTION & CYBT_PORTING_BASE_EXCEPTION in cybt_platform_config.h
  *                      for controller and porting layer exceptions)
  * @param[in] ptr     : Pointer to the exception data
  * @param[in] length  : Length of the exception data
  *
  * @return void
  */
typedef void (*pf_wiced_exception)(uint16_t code, void* ptr, uint32_t length);
/**@} gentypes */

/**
 *  @ingroup     app_utils
 *
 *  @{
 */

/**
* Serialized function prototype
*
*/
typedef void (*wiced_bt_serialized_app_func_t)(void *param);
/**@} app_utils */

#endif
