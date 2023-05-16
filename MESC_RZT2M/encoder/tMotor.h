#pragma once

// Activity Status Flags
#define ACT_MotionComplete  0x0001      // Motion completed
#define ACT_ServoOn         0x0002      // Servo Control is ON
#define ACT_PowerOn         0x0004      // PWM Amplifier enabled
#define ACT_PosCaptured     0x0008      // Position captured on external trigger

#define ACT_Reserved        0x0010      //
#if 1	// EtherCAT
#define ACT_Homed           0x0020
#else // ethercat
#define ACT_Reserved2       0x0020      //
#endif // ethercat
#define ACT_PVTWatermark    0x0040	    /* PVT FIFO Buffer emptied below the watermark */
#define ACT_Aligned         0x0080      /* Phasing completed */

#define ACT_Busy            0x0100      /* Firmware is busy (moving, homing or phasing) */
#define ACT_OverCurrent     0x0200      /* Current overload is registered */
#define ACT_Inhibit         0x0400      /* External Inhibit Input is troggered */
#define ACT_PVTEmpty        0x0800      /* PVT buffer depleted - interpolation starved */

#define ACT_MtOverTemp      0x1000      /* Motor Overheating is detected */
#define ACT_AmpFault        0x2000      /* PWM Amplifier Failure */
#define ACT_PosError        0x4000      /* Position Error Exceeded */
#define ACT_WrapAround      0x8000      /* Position Counter wrwaparound */

typedef union {
  struct {
    short Low;
    short High;
  } Reg16;
  long     Reg32;
} TReg32;

struct t_motor
{
  long eeprom_addr;
  long enc_timeout;
  long real_res;
  int16_t real_rot;
  uint16_t enc_status;
  uint16_t act_state;
  uint16_t enc_open;
  volatile TReg32 captured_pos;
  long pos_offset;
  volatile long index_pos;
};