#ifndef MESCHW_RZ_T2M_H_
#define MESCHW_RZ_T2M_H_

// general params
#define NUM_MOTORS                    2
#define HAS_PHASE_SENSORS
#define DEFAULT_SENSOR_MODE           MOTOR_SENSOR_MODE_ENCODER

// m1 adc macros
#define M1_VOL_B    ADC_CHANNEL_0
#define M1_VOL_U    ADC_CHANNEL_1
#define M1_VOL_V    ADC_CHANNEL_2
#define M1_VOL_W    ADC_CHANNEL_3
#define M1_CUR_U    ADC_CHANNEL_4
#define M1_CUR_V    ADC_CHANNEL_5
#define M1_CUR_W    ADC_CHANNEL_6
#define M1_MOST_U   ADC_CHANNEL_7
#define M1_MOST_V   ADC_CHANNEL_8
#define M1_MOST_W   ADC_CHANNEL_9
#define M1_THROT    ADC_CHANNEL_10
#define M1_TEMP     ADC_CHANNEL_11

// m1 adc macros
#define M2_VOL_B    ADC_CHANNEL_0
#define M2_VOL_U    ADC_CHANNEL_1
#define M2_VOL_V    ADC_CHANNEL_2
#define M2_VOL_W    ADC_CHANNEL_3
#define M2_CUR_U    ADC_CHANNEL_4
#define M2_CUR_V    ADC_CHANNEL_5
#define M2_CUR_W    ADC_CHANNEL_6
#define M2_MOST_U   ADC_CHANNEL_7
#define M2_MOST_V   ADC_CHANNEL_8
#define M2_MOST_W   ADC_CHANNEL_9
#define M2_THROT    ADC_CHANNEL_10
#define M2_TEMP     ADC_CHANNEL_11

// comms inferface
#define MESC_UART_USB                 MESC_USB
#define HW_UART huart3

// motor_profile
#define MAX_MOTOR_PHASE_CURRENT       120.0f
#define DEFAULT_MOTOR_POWER           500.0f
#define DEFAULT_MOTOR_PP              10
#define POLE_ANGLE (65536/POLE_PAIRS)
#define DEFAULT_MOTOR_Ld              0.000006f
#define DEFAULT_MOTOR_Lq              0.000006f
#define DEFAULT_MOTOR_R               0.000006f
#define MIN_FLUX_LINKAGE              0.000006f
#define MAX_FLUX_LINKAGE              0.000006f
#define FLUX_LINKAGE_GAIN             0.000006f
#define NON_LINEAR_CENTERING_GAIN     0.000006f

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
#define HFI_VOLTAGE                4.0f
#define HFI_TEST_CURRENT           10.0f
#define HFI_THRESHOLD              3.0f
#define CURRENT_BANDWIDTH          1000.0f
#define ADC_OFFSET_DEFAULT         2048.0f

// MESCFOC
#define USE_ENCODER
#define ENCODER_E_OFFSET           25000
#define INTERPOLATE_V7_ANGLE
#define SEVEN_SECTOR
// #define USE_FIELD_WEAKENING
#define USE_FIELD_WEAKENINGV2
#define DEADTIME_COMP
// #define OVERMOD_DT_COMP_THRESHOLD

// #define FASTLED GPIOC
// #define FASTLEDIO GPIO_PIN_12
// #define FASTLEDIONO 12
// #define SLOWLED GPIOC
// #define SLOWLEDIO GPIO_PIN_9
// #define SLOWLEDIONO 9

#endif /* MESC_RZT_ */
