/******************************************************************************
 * File Name:   heap_usage.c
 *
 * Description: This file contains the code for printing heap usage.
 *              Supports only GCC_ARM compiler. Define PRINT_HEAP_USAGE for
 *              printing the heap usage numbers.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 *******************************************************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

/* ARM compiler also defines __GNUC__ */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
#include <malloc.h>
#endif /* #if defined (__GNUC__) && !defined(__ARMCC_VERSION) */


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define TO_KB(size_bytes)  ((float)(size_bytes)/1024)


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: print_heap_usage
********************************************************************************
* Summary:
* Prints the available heap and utilized heap by using mallinfo().
*
*******************************************************************************/
void print_heap_usage(char* msg)
{
    /* ARM compiler also defines __GNUC__ */
    #if defined(PRINT_HEAP_USAGE) && defined (__GNUC__) && !defined(__ARMCC_VERSION)
    struct mallinfo mall_info = mallinfo();

    extern uint8_t __HeapBase;  /* Symbol exported by the linker. */
    extern uint8_t __HeapLimit; /* Symbol exported by the linker. */

    uint8_t* heap_base = (uint8_t*)&__HeapBase;
    uint8_t* heap_limit = (uint8_t*)&__HeapLimit;
    uint32_t heap_size = (uint32_t)(heap_limit - heap_base);

    printf("\r\n\n********** Heap Usage **********\r\n");
    printf(msg);
    printf("\r\nTotal available heap        : %" PRIu32 " bytes/%.2f KB\r\n", heap_size,
           TO_KB(heap_size));

    printf("Maximum heap utilized so far: %u bytes/%.2f KB, %.2f%% of available heap\r\n",
           mall_info.arena, TO_KB(mall_info.arena), ((float)mall_info.arena * 100u)/heap_size);

    printf("Heap in use at this point   : %u bytes/%.2f KB, %.2f%% of available heap\r\n",
           mall_info.uordblks, TO_KB(
               mall_info.uordblks), ((float)mall_info.uordblks * 100u)/heap_size);

    printf("********************************\r\n\n");
    #endif /* #if defined(PRINT_HEAP_USAGE) && defined (__GNUC__) && !defined(__ARMCC_VERSION) */
}


/* [] END OF FILE */
