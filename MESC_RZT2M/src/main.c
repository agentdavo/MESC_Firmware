#include <MESCbat.h>
#include <MESCfoc.h>
#include <MESCprofile.h>
#include <MESCspeed.h>
#include <MESCtemp.h>

#include <HAL/MESC_HAL.h>

#include <Tasks/init.h>

#include <hal_data.h>

TaskHandle_t defaultTaskHandle = NULL;

extern MESC_hal* getHalForMotor0();
extern MESC_hal* getHalForMotor1();

void StartDefaultTask(void* argument)
{
  (void)argument;
  UsbDeviceInit();
  while(true)
  {
    //ToggleLed();
    vTaskDelay(200);
  }
}


MESC_motor mtr[NUM_MOTORS];

int main(void)
{
  g_hal_init();

  bat_init(PROFILE_DEFAULT);
  speed_init(PROFILE_DEFAULT);
  temp_init(PROFILE_DEFAULT);
  motor_init(PROFILE_DEFAULT);
  // Initialise user Interface
  //ui_init(PROFILE_DEFAULT);

  Delay(1);

  MESCInit(&mtr[0], getHalForMotor0());
  MESCInit(&mtr[1], getHalForMotor1());

  uint8_t returnCode = xTaskCreate(StartDefaultTask, "defaultTask", 256 * 4, NULL, tskIDLE_PRIORITY, defaultTaskHandle);

  init_system();

  vTaskStartScheduler();

  // TODO: Tjis task can be suppressed by config freertos special idle hook
  while (1)
  {
    vTaskDelay(configTICK_RATE_HZ);
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