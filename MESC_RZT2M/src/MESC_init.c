#include "hal_data.h"

#include "MESC_init.h"

#include "MESCbat.h"
#include "MESCflash.h"
#include "MESCprofile.h"
#include "MESCmotor.h"
#include "MESCspeed.h"
#include "MESCtemp.h"
#include "MESCuart.h"
#include "MESCui.h"
#include "MESCmotor_state.h"

fsp_err_t mesc_init(void)
{
  fsp_err_t err = FSP_SUCCESS;

  bat_init( PROFILE_DEFAULT );
  speed_init( PROFILE_DEFAULT );
  temp_init( PROFILE_DEFAULT );
  motor_init( PROFILE_DEFAULT );
  MESCInit(&mtr[0]);
  MESCInit(&mtr[1]);
  return err;
}
