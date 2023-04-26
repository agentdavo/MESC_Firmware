#include "comms_thread.h"

void comms_thread_entry(void* pvParameters)
{
  FSP_PARAMETER_NOT_USED(pvParameters);

  while (1)
    {
      vTaskDelay(1);
    }
}
