/******************************************************************************
 * File Name:   publisher_task.c
 *
 * Description: This file contains the task that sets up the user button GPIO
 *              for the publisher and publishes MQTT messages on the topic
 *              'MQTT_PUB_TOPIC' to control a device that is actuated by the
 *              subscriber task. The file also contains the ISR that notifies
 *              the publisher task about the new device state to be published.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"

/* Task header files */
#include "publisher_task.h"
#include "mqtt_task.h"
#include "subscriber_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cybsp_types.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Interrupt priority for User Button Input. */
#define USER_BTN_INTR_PRIORITY          (7)

/* The maximum number of times each PUBLISH in this example will be retried. */
#define PUBLISH_RETRY_LIMIT             (10)

/* A PUBLISH message is retried if no response is received within this
 * time (in milliseconds).
 */
#define PUBLISH_RETRY_MS                (1000)

/* Queue length of a message queue that is used to communicate with the
 * publisher task.
 */
#define PUBLISHER_TASK_QUEUE_LENGTH     (3u)

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void publisher_init(void);
static void publisher_deinit(void);
static void isr_button_press(void* callback_arg, cyhal_gpio_event_t event);
void print_heap_usage(char* msg);

/******************************************************************************
 * Global Variables
 *******************************************************************************/
/* FreeRTOS task handle for this task. */
TaskHandle_t publisher_task_handle;

/* Handle of the queue holding the commands for the publisher task */
QueueHandle_t publisher_task_q;

/* Structure to store publish message information. */
cy_mqtt_publish_info_t publish_info =
{
    .qos       = (cy_mqtt_qos_t)MQTT_MESSAGES_QOS,
    .topic     = MQTT_PUB_TOPIC,
    .topic_len = (sizeof(MQTT_PUB_TOPIC) - 1),
    .retain    = false,
    .dup       = false
};

/* Structure that stores the callback data for the GPIO interrupt event. */
cyhal_gpio_callback_data_t cb_data =
{
    .callback     = isr_button_press,
    .callback_arg = NULL
};

#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#define PUBLISH_MSG_TIMEOUT 10 * 1000
static cy_timer_t publish_msg_timer;
static void publish_msg_timeout(void* data);


/******************************************************************************
* Function Name: publisher_task
******************************************************************************
* Summary:
*  Task that sets up the user button GPIO for the publisher and publishes
*  MQTT messages to the broker. The user button init and deinit operations,
*  and the MQTT publish operation is performed based on commands sent by other
*  tasks and callbacks over a message queue.
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  void
*
******************************************************************************/
void publisher_task(void* pvParameters)
{
    /* Status variable */
    cy_rslt_t result;

    publisher_data_t publisher_q_data;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* To avoid compiler warnings */
    (void)pvParameters;

    /* Initialize and set-up the user button GPIO. */
    publisher_init();

    /* Create a message queue to communicate with other tasks and callbacks. */
    publisher_task_q = xQueueCreate(PUBLISHER_TASK_QUEUE_LENGTH, sizeof(publisher_data_t));

    if (cy_rtos_init_timer(&publish_msg_timer, CY_TIMER_TYPE_ONCE,
                           (cy_timer_callback_t)publish_msg_timeout,
                           (cy_timer_callback_arg_t)NULL) == CY_RSLT_SUCCESS)
    {
        if (cy_rtos_start_timer(&publish_msg_timer, PUBLISH_MSG_TIMEOUT) == CY_RSLT_SUCCESS)
        {
            printf("start publish timer\n");
        }
        else
        {
            printf("fail to start publish timer\n");
        }
    }
    else
    {
        printf("fail to init publish timer\n");
    }

    while (true)
    {
        /* Wait for commands from other tasks and callbacks. */
        if (pdTRUE == xQueueReceive(publisher_task_q, &publisher_q_data, portMAX_DELAY))
        {
            switch (publisher_q_data.cmd)
            {
                case PUBLISHER_INIT:
                {
                    /* Initialize and set-up the user button GPIO. */
                    publisher_init();
                    break;
                }

                case PUBLISHER_DEINIT:
                {
                    /* Deinit the user button GPIO and corresponding interrupt. */
                    publisher_deinit();
                    break;
                }

                case PUBLISH_MQTT_MSG:
                {
                    /* Publish the data received over the message queue. */
                    publish_info.payload = publisher_q_data.data;
                    publish_info.payload_len = strlen(publish_info.payload);

                    printf("\nPublisher: Publishing '%s' on the topic '%s'\n",
                           (char*)publish_info.payload, publish_info.topic);

                    result = cy_mqtt_publish(mqtt_connection, &publish_info);

                    if (result != CY_RSLT_SUCCESS)
                    {
                        printf("  Publisher: MQTT Publish failed with error 0x%0X.\n\n",
                               (int)result);

                        /* Communicate the publish failure with the the MQTT
                         * client task.
                         */
                        mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
                        xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
                    }

                    print_heap_usage("publisher_task: After publishing an MQTT message");
                    break;
                }
            }
        }
    }
}


/******************************************************************************
* Function Name: publisher_init
******************************************************************************
* Summary:
*  Function that initializes and sets-up the user button GPIO pin along with
*  its interrupt.
*
* Parameters:
*  void
*
* Return:
*  void
*
******************************************************************************/
static void publisher_init(void)
{
    /* Initialize the user button GPIO and register interrupt on falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, true);

    printf("\nPress the user button (SW2) to publish \"%s\"/\"%s\" on the topic '%s'...\n",
           MQTT_DEVICE_ON_MESSAGE, MQTT_DEVICE_OFF_MESSAGE, publish_info.topic);
}


/******************************************************************************
* Function Name: publisher_deinit
******************************************************************************
* Summary:
*  Cleanup function for the publisher task that disables the user button
*  interrupt and deinits the user button GPIO pin.
*
* Parameters:
*  void
*
* Return:
*  void
*
******************************************************************************/
static void publisher_deinit(void)
{
    /* Deregister the ISR and disable the interrupt on the user button. */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, false);
    cyhal_gpio_free(CYBSP_USER_BTN);
}


/******************************************************************************
* Function Name: isr_button_press
******************************************************************************
* Summary:
*  GPIO interrupt service routine. This function detects button
*  presses and sends the publish command along with the data to be published
*  to the publisher task over a message queue. Based on the current device
*  state, the publish data is set so that the device state gets toggled.
*
* Parameters:
*  void *callback_arg : pointer to variable passed to the ISR (unused)
*  cyhal_gpio_event_t event : GPIO event type (unused)
*
* Return:
*  void
*
******************************************************************************/
static void isr_button_press(void* callback_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    publisher_data_t publisher_q_data;

    /* To avoid compiler warnings */
    (void)callback_arg;
    (void)event;

    /* Assign the publish command to be sent to the publisher task. */
    publisher_q_data.cmd = PUBLISH_MQTT_MSG;

    /* Assign the publish message payload so that the device state toggles. */
    if (current_device_state == DEVICE_ON_STATE)
    {
        publisher_q_data.data = (char*)MQTT_DEVICE_OFF_MESSAGE;
    }
    else
    {
        publisher_q_data.data = (char*)MQTT_DEVICE_ON_MESSAGE;
    }

    /* Send the command and data to publisher task over the queue */
    xQueueSendFromISR(publisher_task_q, &publisher_q_data, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/***************************************************************************************************
 * publish_msg_timeout
 **************************************************************************************************/
static void publish_msg_timeout(void* data)
{
    publisher_data_t publisher_q_data;

    /* Assign the publish command to be sent to the publisher task. */
    publisher_q_data.cmd = PUBLISH_MQTT_MSG;

    /* Assign the publish message payload so that the device state toggles. */
    if (current_device_state == DEVICE_ON_STATE)
    {
        publisher_q_data.data = (char*)MQTT_DEVICE_OFF_MESSAGE;
    }
    else
    {
        publisher_q_data.data = (char*)MQTT_DEVICE_ON_MESSAGE;
    }

    /* Send the command and data to publisher task over the queue */
    xQueueSend(publisher_task_q, &publisher_q_data, portMAX_DELAY);

    cy_rtos_start_timer(&publish_msg_timer, PUBLISH_MSG_TIMEOUT);
}


/* [] END OF FILE */
