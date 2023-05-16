#pragma once

// general params
#define NUM_MOTORS                    2
#define HAS_PHASE_SENSORS
#define DEFAULT_SENSOR_MODE           MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE         MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE         MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE         MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE         MOTOR_SENSOR_MODE_HFI

// comms inferface
#define MESC_UART_USB                 MESC_USB
#define HW_UART huart3

//#define USE_DEADSHORT //This can be used in place of the phase sensors for startup from running.
#define DEADSHORT_CURRENT 30.0f	//When recovering from tracking phase without phase sensors, the
                                 //deadshort function will short the phases
                                 //until the current exceeds this value. At this point, it calculates the Vd Vq and phase angle
                                 //Don't set too high, after 9PWM periods, it will run the calc and start the motor regardless.
                                 //This seems to work best with a higher current bandwidth (~10krads-1) and using the non-linear observer centering.
                                 //Broadly incompatible with the flux observer
                                 //Only works for forward direction presently
                                 //^^WIP, not completely stable yet

// motor_profile
#define MAX_MOTOR_PHASE_CURRENT       120.0f
#define DEFAULT_MOTOR_POWER           500.0f
#define DEFAULT_MOTOR_PP              10

#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD        2.0f

//#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS                    10
#define ENCODER_E_OFFSET              25000
#define POLE_ANGLE                    (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#define FASTLED     //GPIOC
#define FASTLEDIO   //GPIO_PIN_12
#define FASTLEDIONO //12
#define SLOWLED     //GPIOC
#define SLOWLEDIO   //GPIO_PIN_9
#define SLOWLEDIONO //9

//#define SAFE_START_DEFAULT 0

#define LOGGING

//GPIO for IC timer //These actually have to be timer compatible pins and
//you must have done something (anything) with the timer in CUBEMX to make it generate the config files
//#define IC_TIM_GPIO //GPIOB
//#define IC_TIM_PIN  //GPIO_PIN_6
//#define IC_TIM_IONO //6
//#define IC_TIMER htim4 //This must be TIM2-TIM5. Untested with other timers
//Assign a use for the input capture timer
//#define IC_TIMER_RCPWM
//#define IC_TIMER_ENCODER

//#define KILLSWITCH_GPIO //GPIOB
//#define KILLSWITCH_PIN //GPIO_PIN_3
//#define KILLSWITCH_IONO //3

#ifdef ENCODER_DIR_REVERSED
#define POLE_PAIRS_VALUE (-POLE_PAIRS)
#else
#define POLE_PAIRS_VALUE (POLE_PAIRS)
#endif

#define DEFAULT_MOTOR_Ld              0.000006f
#define DEFAULT_MOTOR_Lq              0.000006f
#define DEFAULT_MOTOR_R               0.000006f

/////////////////////Related to OBSERVER//////////////////////////////
//#define DONT_USE_FLUX_LINKAGE_OBSERVER //This tracks the flux linkage in real time, and the detection algorithm relies on it.
#define MAX_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*2.0f //Sets the limits for tracking.
#define MIN_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*0.7f//Faster convergence with closer start points
#define FLUX_LINKAGE_GAIN 10.0f * sqrtf(DEFAULT_FLUX_LINKAGE)//*(DEFAULT_FLUX_LINKAGE*DEFAULT_FLUX_LINKAGE)*PWM_FREQUENCY

//#define USE_NONLINEAR_OBSERVER_CENTERING //This is not a preferred option, since it relies on gain tuning and instability,
//which is precisely what the original observer intended to avoid.
//Also, incompatible with flux linkage observer for now...
#define NON_LINEAR_CENTERING_GAIN     5000.0f
#define USE_CLAMPED_OBSERVER_CENTERING //Pick one of the two centering methods... preferably this one

// hw_setup
#define ABS_MAX_PHASE_CURRENT 120.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.00033f
#define OPGAIN 10.5f
#define R_VBUS_TOP 100000.0f
#define R_VBUS_BOTTOM 3300.0f
#define SHUNT_POLARITY -1.0f

// input_vars
#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 70.0f
//#define MIN_IQ_REQUEST -10
#define IC_PULSE_MAX 2100
#define IC_PULSE_MIN 900
#define IC_PULSE_MID 1500
#define IC_PULSE_DEADZONE 100
#define IC_DURATION_MAX 25000
#define IC_DURATION_MIN 15000
#define ADC1MIN 1200
#define ADC1MAX 2700
#define ADC2MIN 1200
#define ADC2MAX 4095
#define ADC1_POLARITY 1.0f
#define ADC2_POLARITY -1.0f

//#define DEFAULT_CONTROL_MODE MOTOR_CONTROL_MODE_TORQUE
//#define ADC1OOR 4094

// MESCfoc
#define PWM_FREQUENCY              20000
#define CUSTOM_DEADTIME            800
#define SLOW_LOOP_FREQUENCY        100
#define DEADTIME_COMP
#define DEADTIME_COMP_V            10
#define OVERMOD_DT_COMP_THRESHOLD  100
#define MAX_MODULATION             0.95f
#define I_MEASURE                  20.0f
#define V_MEASURE                  4.0f
#define ERPM_MEASURE               3000.0f
#define DEADSHORT_CURRENT          30.0f

//#define USE_HFI
#define HFI_VOLTAGE                4.0f
#define HFI_TEST_CURRENT           10.0f
#define HFI_THRESHOLD              3.0f
#define HFI45
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

#define CURRENT_BANDWIDTH          1000.0f
#define ADC_OFFSET_DEFAULT         2048.0f
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE

// MESCFOC
#define USE_ENCODER
#define ENCODER_E_OFFSET           25000
#define INTERPOLATE_V7_ANGLE
#define SEVEN_SECTOR
// #define USE_FIELD_WEAKENING
#define USE_FIELD_WEAKENINGV2
#define DEADTIME_COMP
// #define OVERMOD_DT_COMP_THRESHOLD

//#define USE_LR_OBSERVER

// #define FASTLED GPIOC
// #define FASTLEDIO GPIO_PIN_12
// #define FASTLEDIONO 12
// #define SLOWLED GPIOC
// #define SLOWLEDIO GPIO_PIN_9
// #define SLOWLEDIONO 9

#define HAS_PHASE_SENSORS //This refers to VOLTAGE sensing on phase, not current!

#define SLOWTIM_SCALER 2

//#define MISSING_UCURRSENSOR //You can run two current sensors ONLY if they are phase sensors.
//#define MISSING_VCURRSENSOR //Running this with low side sensors may result in fire.
//#define MISSING_WCURRSENSOR //Also requires that the third ADC is spoofed in the getRawADC(void) function in MESChw_setup.c to avoid trips



////////////////////USER DEFINES//////////////////
///////////////////RCPWM//////////////////////
#define IC_DURATION_MAX 25000
#define IC_DURATION_MIN 15000

#define IC_PULSE_MAX 2100
#define IC_PULSE_MIN 900
#define IC_PULSE_MID 1500

#define IC_PULSE_DEADZONE 100


/////////////////ADC///////////////
#define  ADC1MIN 1200
#define  ADC1MAX 2700
#define  ADC2MIN 1200
#define  ADC2MAX 4095

#define ADC1_POLARITY 1.0f
#define ADC2_POLARITY -1.0f

#ifndef DEFAULT_INPUT
#define DEFAULT_INPUT	0b1001 //0b...wxyz where w is UART, x is RCPWM, y is ADC2 z is ADC1
#endif

//Use the Ebike Profile tool
#define USE_PROFILE

#ifndef FIELD_WEAKENING_CURRENT
#define FIELD_WEAKENING_CURRENT 10.0f //This does not set whether FW is used, just the default current
#endif

#ifndef FIELD_WEAKENING_THRESHOLD
#define FIELD_WEAKENING_THRESHOLD 0.8f
#endif

/////////////////////Related to CIRCLE LIMITATION////////////////////////////////////////
//#define USE_SQRT_CIRCLE_LIM //Use for high PWM frequency (less clock cycles) or try if stability issues seen with Vd favouring option (unlikely)
#define USE_SQRT_CIRCLE_LIM_VD //Use for Field weakening

//#define USE_MTPA

/////////////////////Related to ONLINE PARAMETER ESTIMATION//////////////////////////////
#ifndef LR_OBS_CURRENT
#define LR_OBS_CURRENT 0.1*MAX_IQ_REQUEST 	//Inject this much current into the d-axis at the slowloop frequency and observe the change in Vd and Vq
                                             //Needs to be a small current that does not have much effect on the running parameters.
#endif

//Inputs
#define GET_THROTTLE_INPUT 	_motor->Raw.ADC_in_ext1 = 0.99f*_motor->Raw.ADC_in_ext1 + 0.01f*hadc1.Instance->JDR3;  // Throttle for MP2 with F405 pill
//#define GET_THROTTLE_INPUT2 	_motor->Raw.ADC_in_ext2 = 0.99f*_motor->Raw.ADC_in_ext2 + 0.01f*hadc1.Instance->JDR3;  // Throttle for MP2 with F405 pill

#define GET_FETU_T 			_motor->Raw.MOSu_T = 	0.99f * _motor->Raw.MOSu_T + 0.01f*ADC2_buffer[3] //Temperature on PB1
#define GET_MOTOR_T _motor->Raw.Motor_T = ADC1_buffer[4]

// TODO: Should be moved to HAL
#define MESC_GPIO_HALL //GPIOC

//extern TIM_HandleTypeDef htim7;
//extern SPI_HandleTypeDef hspi3;
//#define debugtim htim7

// TODO: Should be moved to HAL
#define getHallState(...) 0//((MESC_GPIO_HALL->IDR >> 6) & 0x7)

/* Temperature parameters */
#define MESC_PROFILE_TEMP_R_F     10000.0f
#define MESC_PROFILE_TEMP_SCHEMA  TEMP_SCHEMA_R_F_ON_R_T
#define MESC_PROFILE_TEMP_SH_BETA 3437.864258f
#define MESC_PROFILE_TEMP_SH_R    0.098243f
#define MESC_PROFILE_TEMP_SH_R0   10000.0f

#define FLASH_BASE 0x00000000