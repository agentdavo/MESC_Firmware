/* generated thread source file - do not edit */
#include "blinky_thread.h"

#if 0
                static StaticTask_t blinky_thread_memory;
                #if defined(__ARMCC_VERSION)           /* AC6 compiler */
                static uint8_t blinky_thread_stack[1024] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #else
                static uint8_t blinky_thread_stack[1024] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.blinky_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #endif
                #endif
                TaskHandle_t blinky_thread;
                void blinky_thread_create(void);
                static void blinky_thread_func(void * pvParameters);
                void rtos_startup_err_callback(void * p_instance, void * p_data);
                void rtos_startup_common_init(void);
extern uint32_t g_fsp_common_thread_count;

                const rm_freertos_port_parameters_t blinky_thread_parameters =
                {
                    .p_context = (void *) NULL,
                };

                void blinky_thread_create (void)
                {
                    /* Increment count so we will know the number of threads created in the FSP Configuration editor. */
                    g_fsp_common_thread_count++;

                    /* Initialize each kernel object. */
                    

                    #if 0
                    blinky_thread = xTaskCreateStatic(
                    #else
                    BaseType_t blinky_thread_create_err = xTaskCreate(
                    #endif
                        blinky_thread_func,
                        (const char *)"Blinky Thread",
                        1024/4, // In words, not bytes
                        (void *) &blinky_thread_parameters, //pvParameters
                        1,
                        #if 0
                        (StackType_t *)&blinky_thread_stack,
                        (StaticTask_t *)&blinky_thread_memory
                        #else
                        & blinky_thread
                        #endif
                    );

                    #if 0
                    if (NULL == blinky_thread)
                    {
                        rtos_startup_err_callback(blinky_thread, 0);
                    }
                    #else
                    if (pdPASS != blinky_thread_create_err)
                    {
                        rtos_startup_err_callback(blinky_thread, 0);
                    }
                    #endif
                }
                static void blinky_thread_func (void * pvParameters)
                {
                    /* Initialize common components */
                    rtos_startup_common_init();

                    /* Initialize each module instance. */
                    

                    /* Enter user code for this thread. Pass task handle. */
                    blinky_thread_entry(pvParameters);
                }
