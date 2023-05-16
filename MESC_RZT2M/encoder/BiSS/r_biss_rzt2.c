#include "r_biss_rzt2_if.h"

//#include "iodefine_biss.h"

#include "bissc/r_bissc_rzt2_if.h"

#include <ecl/r_ecl_rzt2_if.h>

#include <stdio.h>

// BiSS Encoder I/F Version
#define BISS_VERSION            (0x00010000u)
#define BISS_VER_ERR            (0xFFFFFFFFu)

// BiSS ID
#define BISS_ID_0               (R_ECL_CH_0)
#define BISS_ID_1               (R_ECL_CH_1)

typedef enum biss_state_e
{
    BISS_STATE_CLOSE = 0,
    BISS_STATE_C_OPEN
} biss_state_t;

// Status of BiSS Common
static biss_state_t biss_state[BISS_ID_NUM] =
{
    BISS_STATE_CLOSE,
    BISS_STATE_CLOSE
};

static  int32_t biss_id_tbl[BISS_ID_NUM] =
{
    BISS_ID_0,
    BISS_ID_1
};

static int32_t biss_id_to_index(const int32_t id)
{
  int32_t index;
  for (index = 0; index < BISS_ID_NUM; index++)
    {
      if (id == biss_id_tbl[index])
        {
          break;
        }
    }
  return index;
}

r_biss_err_t R_BISS_Open(const int32_t id, const r_biss_type_t type, r_biss_info_t* pinfo)
{
    int32_t index = biss_id_to_index(id);
    if ((BISS_ID_NUM <= index) ||
        (NULL == pinfo) ||
        (R_BISS_TYPE_C != type))
    {
        return R_BISS_ERR_INVALID_ARG;
    }

    if (BISS_STATE_CLOSE != biss_state[index])
    { // check BiSS Status
        return R_BISS_ERR_ACCESS;
    }

    r_biss_err_t result = R_BISSC_Open(index, pinfo);
    if (R_BISS_SUCCESS == result)
    {
        biss_state[index] = BISS_STATE_C_OPEN;
    }
    return result;
}

r_biss_err_t R_BISS_Close(const int32_t id)
{
    int32_t index = biss_id_to_index(id);
    if (BISS_ID_NUM <= index)
    {
        return R_BISS_ERR_INVALID_ARG;
    }
    if (biss_state[index] == BISS_STATE_C_OPEN)
    {
        r_biss_err_t result = R_BISSC_Close(index);
        if (R_BISS_SUCCESS == result)
        {
            biss_state[index] = BISS_STATE_CLOSE;
        }
        return result;
    }
    return R_BISS_SUCCESS;
}

uint32_t R_BISS_GetVersion(const r_biss_type_t type)
{
    return (R_BISS_TYPE_CMN == type)
               ? BISS_VERSION
               : (R_BISS_TYPE_C == type)
                     ? R_BISSC_GetVersion()
                     : BISS_VER_ERR;
}

r_biss_err_t R_BISS_Control(const int32_t id, const r_biss_cmd_t cmd, void *const pbuf)
{
    int32_t index = biss_id_to_index(id);
    if (BISS_ID_NUM <= index)
    {
        return R_BISS_ERR_INVALID_ARG;
    }
    return (BISS_STATE_C_OPEN == biss_state[index]) ? R_BISSC_Control(index, cmd, pbuf) : R_BISS_ERR_ACCESS;
}



