/* generated configuration header file - do not edit */
#pragma once
/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include "bsp_api.h"
#endif
#ifndef CMSIS_device_header
#define CMSIS_device_header "bsp_api.h"
#endif /* CMSIS_device_header */




#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      1
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       (1000)
#define configMAX_PRIORITIES                     ( 5 )
#define configMINIMAL_STACK_SIZE                 (256)
#define configTOTAL_HEAP_SIZE                    (0x30000)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 0
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
//#define configRECORD_STACK_HIGH_ADDRESS          1
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
#define configUSE_CO_ROUTINES                    0
//#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                3
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             256

/* The following flag must be enabled only when using newlib */
#define configUSE_NEWLIB_REENTRANT           0

/* CMSIS-RTOS V2 flags */
#define configUSE_OS2_THREAD_SUSPEND_RESUME  1
#define configUSE_OS2_THREAD_ENUMERATE       1
#define configUSE_OS2_EVENTFLAGS_FROM_ISR    1
#define configUSE_OS2_THREAD_FLAGS           1
#define configUSE_OS2_TIMER                  1
#define configUSE_OS2_MUTEX                  1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_vTaskDelayUntil              1
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1
#define INCLUDE_xTimerPendFunctionCall       0
//#define INCLUDE_xQueueGetMutexHolder         1
#define INCLUDE_uxTaskGetStackHighWaterMark  0
#define INCLUDE_xTaskGetCurrentTaskHandle    1
#define INCLUDE_eTaskGetState                0
#define INCLUDE_xTaskGetIdleTaskHandle       0
#define INCLUDE_xEventGroupSetBitFromISR     1
#define INCLUDE_xTaskAbortDelay              1
#define INCLUDE_xTaskGetHandle               0
#define INCLUDE_xTaskResumeFromISR           1

#define configUSE_TICKLESS_IDLE (0)
#define configUSE_MALLOC_FAILED_HOOK (0)
#define configUSE_DAEMON_TASK_STARTUP_HOOK (0)
#define configMINIMAL_SECURE_STACK_SIZE (256U)  /* Unused in RZ microprocessor port. */
#define configUSE_STATS_FORMATTING_FUNCTIONS (0)
#define configIDLE_SHOULD_YIELD (1)
#define configUSE_TASK_NOTIFICATIONS (1)
#define configUSE_ALTERNATIVE_API (0U)
#define configCHECK_FOR_STACK_OVERFLOW (0)
#define configUSE_QUEUE_SETS (0)
#define configUSE_TIME_SLICING (0)
#define configENABLE_BACKWARD_COMPATIBILITY (0)
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS (5)
#define configSTACK_DEPTH_TYPE uint32_t
#define configAPPLICATION_ALLOCATED_HEAP (0)
#define configGENERATE_RUN_TIME_STATS (0)
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY (30)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY ((1))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
#define configMAX_API_CALL_INTERRUPT_PRIORITY (configMAX_SYSCALL_INTERRUPT_PRIORITY)
#define configASSERT( x ) if (!(x)) {__BKPT(0);}
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS (0)
#define configPRINT_STRING(x)                      (printf(x))
#define configLOGGING_INCLUDE_TIME_AND_TASK_NAME   (0)
#define configLOGGING_MAX_MESSAGE_LENGTH           (192)
#define configUNIQUE_INTERRUPT_PRIORITIES          (32)
