
#ifndef USER_FOC_H
#define USER_FOC_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "user_vars.h"
#include <stdbool.h>

//=============================================================================
// Incremental Build options for System check-out
//=============================================================================
#define DMC_LEVEL_1     1   //Y 50% duty, offset calibration and verify phase shift
#define DMC_LEVEL_2     2   //Y open loop control to check sensing signals
#define DMC_LEVEL_3     3   //Y closed current loop to check the hardware settings
#define DMC_LEVEL_4     4   //Y Parameters identification and run with InstaSPIN-FOC

#define DMC_BUILDLEVEL  DMC_LEVEL_4

//*****************************************************************************
//
//! \brief Defines the ADC data
//
//*****************************************************************************
typedef struct _HAL_ADCData_t_
{
    float32_t VdcBus_V;         //!< the dc bus voltage values
		float32_t Temperture;        
		float32_t potentiomete;
	
    MATH_Vec3 I_A;              //!< the current values
    MATH_Vec3 V_V;              //!< the voltage values

//#if defined(MOTOR1_ISBLDC)
//    MATH_Vec2 Idc1_A;           //!< the dc-link current values
//    float32_t offset_Idc_ad;    //!< the motor current offset value
//#elif defined(MOTOR1_DCLINKSS)
//    MATH_Vec2 Idc1_A;           //!< the dc-link current up values
//    MATH_Vec2 Idc2_A;           //!< the dc-link current down values
//    float32_t offset_Idc_ad;    //!< the motor current offset value
//#endif  // MOTOR1_ISBLDC | MOTOR1_DCLINKSS

//#if !defined(MOTOR1_ISBLDC) && !defined(MOTOR1_DCLINKSS)  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
//    MATH_Vec3 offset_I_ad;      //!< the motor current offset value
//#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    MATH_Vec3 offset_V_sf;      //!< the motor voltage offset factor

    float32_t current_sf;       //!< the current scale factor, amps/cnt
    float32_t voltage_sf;       //!< the phase voltage scale factor, volts/cnt
    float32_t dcBusvoltage_sf;  //!< the phase voltage scale factor, volts/cnt
		
		int32_t offset_I_ad[3]; 
		int32_t offset_I_ad_sum[3];

#if defined(CMD_POT_EN)
    uint16_t  potAdc;           //!< the ADC result from POT
#endif  // CMD_POT_EN
} HAL_ADCData_t;

//*****************************************************************************
//
//! \brief Defines the PWM data
//
//*****************************************************************************
typedef struct _HAL_PWMData_t_
{
    MATH_Vec3 Vabc_pu;     //!< the PWM time-durations for each motor phase

    uint16_t  cmpValue[3];
    uint16_t  deadband;
    uint16_t  noiseWindow;
    uint16_t  adcSample;
    uint16_t  minCMPValue;
    uint16_t  period;
    uint16_t  socCMP;
    bool      flagEnablePwm;
} HAL_PWMData_t;


extern HAL_ADCData_t AdcData;
extern HAL_PWMData_t PwmData;

extern void RCM_Conv(void);
extern void TaskTime_mainLoop(void);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of USER_FOC_H definition
