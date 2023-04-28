#include <MESCbat.h>
#include <MESCfoc.h>
#include <MESCprofile.h>
#include <MESCspeed.h>
#include <MESCtemp.h>

#include <HAL/MESC_HAL.h>

#include <Tasks/init.h>
#include <bsp_api.h>
#include <cmsis_os.h>
#include <hal_data.h>

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void StartDefaultTask(void *argument)
{
  //MX_USB_DEVICE_Init(); // TODO: Should be implemented for RZT
  while(true)
  {
    //HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); // TODO: Should be implemented for RZT
    osDelay(200);
  }
}

int main(void)
{
  g_hal_init();

#ifdef IC_TIMER
  MESC_IC_Init(IC_TIMER);
#endif

#ifdef USE_ENCODER
  HAL_SPI_Init(&hspi3);
#endif

  bat_init(PROFILE_DEFAULT);
  speed_init(PROFILE_DEFAULT);
  // Initialise user Interface
  //ui_init( PROFILE_DEFAULT );

  Delay(1);

  mtr[0].mtimer = &htim1;
  mtr[0].stimer = &htim2;
  temp_init(PROFILE_DEFAULT);
  motor_init(PROFILE_DEFAULT);
  MESCInit(&mtr[0]);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  init_system();

  osKernelStart();

  while (1)
  {
  }
  return 0;
}

#if 0
#if configSUPPORT_STATIC_ALLOCATION
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) BSP_WEAK_REFERENCE;
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize) BSP_WEAK_REFERENCE;

/* If configSUPPORT_STATIC_ALLOCATION is set to 1, the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t**  ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize)
{
  /* If the buffers to be provided to the Idle task are declared inside this
   * function then they must be declared static - otherwise they will be allocated on
   * the stack and so not exists after this function exits. */
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

  /* Pass out a pointer to the StaticTask_t structure in which the Idle
   * task's state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task's stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
   * Note that, as the array is necessarily of type StackType_t,
   * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* If configSUPPORT_STATIC_ALLOCATION is set to 1, the application must provide an
 * implementation of vApplicationGetTimerTaskMemory() to provide the memory that is
 * used by the RTOS daemon/time task. */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t**  ppxTimerTaskStackBuffer,
                                    uint32_t*      pulTimerTaskStackSize)
{
  /* If the buffers to be provided to the Timer task are declared inside this
   * function then they must be declared static - otherwise they will be allocated on
   * the stack and so not exists after this function exits. */
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[ configMINIMAL_STACK_SIZE ];

  /* Pass out a pointer to the StaticTask_t structure in which the Idle
   * task's state will be stored. */
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

  /* Pass out the array that will be used as the Timer task's stack. */
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;

  /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
   * Note that, as the array is necessarily of type StackType_t,
   * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulTimerTaskStackSize = configMINIMAL_STACK_SIZE;
}
#endif
#endif