/**
  ******************************************************************************
  * @file      dlib_lock_glue.c
  * @author    STMicroelectronics
  * @brief     Implementation of IAR DLib lock interface
  *
  * @details
  * This file implements locking glue necessary to protect C library
  * functions and initialization of local static objects in C++.
  * Lock strategies are defined in stm32_lock.h that implements
  * different level of thread-safety.
  *
  * For more information about which C functions
  * need which of these lowlevel functions
  * please consult the IAR C/C++ Development Guide
  * and DLib_Threads.h
  *
  * @see http://ftp.iar.se/WWWfiles/arm/webic/doc/EWARM_DevelopmentGuide.ENU.pdf
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#ifndef __ICCARM__
#error "dlib_lock_glue.c" should be used with IAR DLib only
#endif /* __ICCARM__ */

/* Includes ------------------------------------------------------------------*/
#include <DLib_Threads.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32_lock.h"

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Global Error_Handler
  */
__WEAK void Error_Handler(void)
{
  /* Not used if it exists in project */
  while (1);
}

/* Private typedef -----------------------------------------------------------*/
struct __lock
{
  uint8_t initialized; /**< Flag to indicate that lock is initialized */
  LockingData_t lock_data; /**< The locking data */
};

/* Private macros ------------------------------------------------------------*/
/** Convert pointer to pointer to instance of struct __lock */
#define STM32_GET_LOCK_PTR(iar_Rmtx_ptr) ((struct __lock *) *(iar_Rmtx_ptr))

/** See struct __lock definition */
#define STM32_LOCK_PARAMETER(lock_ptr) (&(lock_ptr)->lock_data)

/** See struct __lock definition */
#define STM32_LOCK_INITIALIZED(lock_ptr) ((lock_ptr)->initialized)

/** Size of array */
#define STM32_LOCK_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/* Private variables ---------------------------------------------------------*/
/** Maximum system locks allowed by DLib */
static __no_init struct __lock static_system_lock[_MAX_LOCK];

/** Lock for static_system_lock array */
static LockingData_t static_system_list_lock = LOCKING_DATA_INIT;

#ifdef FOPEN_MAX
/** Maximum file locks allowed by DLib */
static __no_init struct __lock static_file_lock[FOPEN_MAX];

/** Lock for static_file_lock array */
static LockingData_t static_file_list_lock = LOCKING_DATA_INIT;
#endif /* FOPEN_MAX */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Thread safety Initialization, called before main
  */
__attribute__ ((constructor)) void __dlib_thread_safety_init()
{
  uint32_t index;

  /* Mark all system locks as not initialized */
  stm32_lock_acquire(&static_system_list_lock);
  for (index = 0; index < STM32_LOCK_ARRAY_SIZE(static_system_lock); index++)
  {
    STM32_LOCK_INITIALIZED(&static_system_lock[index]) = 0;
  }
  stm32_lock_release(&static_system_list_lock);

  /* Mark all file locks as not initialized */
#ifdef FOPEN_MAX
  stm32_lock_acquire(&static_file_list_lock);
  for (index = 0; index < STM32_LOCK_ARRAY_SIZE(static_file_lock); index++)
  {
    STM32_LOCK_INITIALIZED(&static_file_lock[index]) = 0;
  }
  stm32_lock_release(&static_file_list_lock);
#endif /* FOPEN_MAX */

  /* Initialize system and file locks */
  __iar_Initlocks();

  /* Run the C++ global object constructors */
#ifdef __cplusplus
  extern void __iar_dynamic_initialization();
  __iar_dynamic_initialization();
#endif /* __cplusplus */
}

/**
  * @defgroup __iar_system_Mtx_Functions IAR system locks
  * @{
  */

/**
  * @brief Initialize a system lock
  * @param lock The lock
  */
void __iar_system_Mtxinit(__iar_Rmtx *lock)
{
  uint32_t index;

  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_system_list_lock);
  for (index = 0; index < STM32_LOCK_ARRAY_SIZE(static_system_lock); index++)
  {
    if (STM32_LOCK_INITIALIZED(&static_system_lock[index]) == 0)
    {
      *lock = &static_system_lock[index];
      STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) = 1;
      stm32_lock_init(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
      stm32_lock_release(&static_system_list_lock);
      return;
    }
  }

  /* Not enough mutexes, this should never happen */
  STM32_LOCK_BLOCK();

  /* Release of static_system_list_lock, not possible since STM32_LOCK_BLOCK is invoked */
}

/**
  * @brief Lock a system lock
  * @param lock The lock
  */
void __iar_system_Mtxlock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_system_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    stm32_lock_acquire(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_system_list_lock);
}

/**
  * @brief Unlock a system lock
  * @param lock The lock
  */
void __iar_system_Mtxunlock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_system_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    stm32_lock_release(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_system_list_lock);
}

/**
  * @brief Destroy a system lock
  * @param lock The lock
  */
void __iar_system_Mtxdst(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_system_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) = 0;
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_system_list_lock);
}

/**
  * @}
  */

/**
  * @defgroup __iar_file_Mtx_Functions IAR file locks
  * @{
  */

/**
  * @brief Initialize a file lock
  * @param lock The lock
  */
void __iar_file_Mtxinit(__iar_Rmtx *lock)
{
#ifdef FOPEN_MAX
  uint32_t index;

  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_file_list_lock);
  for (index = 0; index < STM32_LOCK_ARRAY_SIZE(static_file_lock); index++)
  {
    if (STM32_LOCK_INITIALIZED(&static_file_lock[index]) == 0)
    {
      *lock = &static_file_lock[index];
      STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) = 1;
      stm32_lock_init(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
      stm32_lock_release(&static_file_list_lock);
      return;
    }
  }

  /* Not enough mutexes, this should never happen */
  STM32_LOCK_BLOCK();

  /* Release of static_file_lock, not possible since STM32_LOCK_BLOCK is invoked */
#else
  STM32_LOCK_UNUSED(lock);

  /* Not enough mutexes, this should never happen */
  STM32_LOCK_BLOCK();
#endif /* FOPEN_MAX */
}

/**
  * @brief Lock a file lock
  * @param lock The lock
  */
void __iar_file_Mtxlock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

#ifdef FOPEN_MAX
  stm32_lock_acquire(&static_file_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    stm32_lock_acquire(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_file_list_lock);
#else
  STM32_LOCK_BLOCK();
#endif /* FOPEN_MAX */
}

/**
  * @brief Unlock a file lock
  * @param lock The lock
  */
void __iar_file_Mtxunlock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

#ifdef FOPEN_MAX
  stm32_lock_acquire(&static_file_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    stm32_lock_release(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_file_list_lock);
#else
  STM32_LOCK_BLOCK();
#endif /* FOPEN_MAX */
}

/**
  * @brief Destroy a file lock
  * @param lock The lock
  */
void __iar_file_Mtxdst(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

#ifdef FOPEN_MAX
  stm32_lock_acquire(&static_file_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) = 0;
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_file_list_lock);
#else
  STM32_LOCK_BLOCK();
#endif /* FOPEN_MAX */
}

/**
  * @}
  */

/**
  * @defgroup __iar_dynamic_Mtx_Functions IAR C++ dynamic locks
  * @{
  */

/**
  * @brief Initialize a C++ dynamic lock
  * @param lock The lock
  */
void __iar_Initdynamiclock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  *lock = malloc(sizeof(LockingData_t));
  if (*lock != NULL)
  {
    stm32_lock_init(*lock);
  }
  else
  {
    /* Cannot allocate memory for a new mutex */
    STM32_LOCK_BLOCK();
  }
}

/**
  * @brief Lock a C++ dynamic lock
  * @param lock The lock
  */
void __iar_Lockdynamiclock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  if (*lock != NULL)
  {
    stm32_lock_acquire(*lock);
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
}

/**
  * @brief Unlock a C++ dynamic lock
  * @param lock The lock
  */
void __iar_Unlockdynamiclock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  if (*lock != NULL)
  {
    stm32_lock_release(*lock);
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
}

/**
  * @brief Destroy a C++ dynamic lock
  * @param lock The lock
  */
void __iar_Dstdynamiclock(__iar_Rmtx *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  free(*lock);
  *lock = NULL;
}

/**
  * @}
  */
