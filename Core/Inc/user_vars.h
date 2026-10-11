
#ifndef USER_VARS_H
#define USER_VARS_H

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

#include <stdint.h>
#include <stdbool.h>

/* Ported: the reference used CMSIS-DSP (arm_math.h) only for float32_t and
 * arm_sin/cos. We define float32_t locally and route trig through the CORDIC
 * accelerator (see foc_hw.c). */
#ifndef FLOAT32_T_DEFINED
#define FLOAT32_T_DEFINED
typedef float float32_t;
#endif

typedef enum
{
  notvalid,
  ongoing,
  valid
} RCM_status_t;

// State machine typedef for motor running status
typedef enum
{
    MOTOR_STOP_IDLE      = 0,
    MOTOR_FAULT_STOP     = 1,
    MOTOR_BRAKE_STOP     = 2,
    MOTOR_NORM_STOP      = 3,
    MOTOR_CHARGE         = 4,
    MOTOR_SEEK_POS       = 5,
    MOTOR_ALIGNMENT      = 6,
    MOTOR_IPD_HFI        = 7,
    MOTOR_OL_START       = 8,
    MOTOR_CL_RUNNING     = 9,
    MOTOR_CTRL_RUN       = 10,
    MOTOR_RUN_FWC        = 11,
    MOTOR_IDENTIFICATION = 12,
    MOTOR_RS_RECALC      = 13
} MOTOR_Status_e;

//*****************************************************************************
//
//! \brief Defines a two element vector
//
//*****************************************************************************
typedef struct _MATH_Vec2_
{
    float32_t value[2];
} MATH_Vec2;

typedef MATH_Vec2 MATH_vec2;

//*****************************************************************************
//
//! \brief Defines a three element vector
//
//*****************************************************************************
typedef struct _MATH_Vec3_
{
    float32_t value[3];
} MATH_Vec3;

typedef MATH_Vec3 MATH_vec3;

//*****************************************************************************
//
//! \brief Defines the trajectory (TRAJ) object
//
//*****************************************************************************
typedef struct _TRAJ_Obj_
{
    float32_t targetValue;  //!< the target value for the trajectory
    float32_t intValue;     //!< the intermediate value along the trajectory
    float32_t minValue;     //!< the minimum value for the trajectory generator
    float32_t maxValue;     //!< the maximum value for the trajectory generator
    float32_t maxDelta;     //!< the maximum delta value for the trajectory
                          //!< generator
} TRAJ_Obj;

//*****************************************************************************
//
//! \brief Defines the TRAJ handle
//
//*****************************************************************************
typedef struct _TRAJ_Obj_ *TRAJ_Handle;

//! \brief Defines the angle generator (ANGLE_COMP) object
//!
typedef struct _VS_FREQ_Obj_
{
    float32_t   maxVsMag_pu;
    float32_t   Freq;          //!< Input: Input Frequency (pu)
    float32_t   LowFreq;       //!< Parameter: Low Frequency (pu)
    float32_t   HighFreq;       //!< Parameter: High Frequency at rated voltage (pu)
    float32_t   MaxFreq;        //!< Parameter: Maximum Frequency (pu)
    float32_t   VoltMin;        //!< Parameter: Voltage at low Frequency range (pu)
    float32_t   VoltMax;        //!< Parameter: Rated voltage (pu)
    float32_t   VfSlope;        //!< Variable
    float32_t   Vs_out;         //!< Output: Output voltage (pu)
    MATH_Vec2   Vdq_gain;       //!< Variable
    MATH_Vec2   Vdq_out;        //!< Output: Output voltage (pu)
} VS_FREQ_Obj;

//! \brief Defines the VS_FREQ_obj handle
//!
typedef struct _VS_FREQ_Obj_  *VS_FREQ_Handle;

//*****************************************************************************
//
//! \brief Defines the PI controller object
//
//*****************************************************************************
typedef struct _PI_Obj_
{
    float32_t Kp;                //!< the proportional gain for the PI controller
    float32_t Ki;                //!< the integral gain for the PI controller
    float32_t Ui;                //!< the integrator start value for the PI
                               //!< controller
    float32_t refValue;          //!< the reference input value
    float32_t fbackValue;        //!< the feedback input value
    float32_t ffwdValue;         //!< the feedforward input value
    float32_t outMin;            //!< the minimum output value allowed for the PI
                               //!< controller
    float32_t outMax;            //!< the maximum output value allowed for the PI
                               //!< controller
} PI_Obj;

//*****************************************************************************
//
//! \brief Defines the PI handle
//
//*****************************************************************************
typedef struct _PI_Obj_ *PI_Handle;

//! \brief Enumeration for svpwm mode
//!
typedef enum
{
  SVM_COM_C = 0,                //!< SVPWM common SVM mode
  SVM_MIN_C = 1,                //!< DPWM minimum SVM mode
  SVM_MAX_C = 2,                //!< DPWM maximum SVM
  SVM_DQ_S  = 3                 //!< SVPWM standard DQ mode
} SVM_Mode_e;

//*****************************************************************************
//
//! \brief Defines the Space Vector Generator (SVGEN) object
//
//*****************************************************************************
typedef struct _SVGEN_Obj_
{
    float32_t oneOverDcBus_invV;    //!< The inverse DC bus voltage value, 1/V
    uint16_t sector;                //!< the sector value of space vector
    SVM_Mode_e svmMode;
} SVGEN_Obj;

//*****************************************************************************
//
//! \brief Defines the SVGEN handle
//
//*****************************************************************************
typedef struct _SVGEN_Obj_ *SVGEN_Handle;

//! \brief Defines the ESMO controller object
//!
typedef struct _ESMO_Obj_
{
    float32_t scaleFreq_Hz;   //

    float32_t speed_sf;     // speed PU scale factor
    float32_t voltage_sf;   // voltage PU scale factor
    float32_t current_sf;   // current PU scale factor

    float32_t Ts;           // sampling period in sec
    float32_t base_wTs;     // base_wTs = BASE_FREQ * T ,for lib
    float32_t filterFc_sf;  // smo filter frequency coefficient

    float32_t Fdsmopos;     // motor dependent plant matrix
    float32_t Fqsmopos;     // motor dependent plant matrix
    float32_t Gdsmopos;     // motor dependent control gain
    float32_t Gqsmopos;     // motor dependent control gain
    float32_t Kslf;         // sliding control filter gain
    float32_t E0;           // estimated bemf threshold

    float32_t Kslide;       // sliding control gain
    float32_t KslideMax;    // sliding control gain, maximum value
    float32_t KslideMin;    // sliding control gain, minimum value

	float32_t Valpha;       // stationary d-axis phase voltage (pu)
	float32_t Vbeta;        // stationary q-axis phase voltage (pu)

    float32_t EstIalpha;    // estimated stationary alfa-axis stator current
    float32_t EstIbeta;     // estimated stationary beta-axis stator current

	float32_t Ealpha;       // stationary alfa-axis back EMF
	float32_t Ebeta;        // stationary beta-axis back EMF

	int32_t Ealphaq31;       // stationary alfa-axis back EMF
	int32_t Ebetaq31;        // stationary beta-axis back EMF
	
	float32_t Zalpha;       // stationary alfa-axis sliding control
	float32_t Zbeta;        // stationary beta-axis sliding control

    float32_t Ed;
    float32_t Eq;
    float32_t Eq_mag;

    float32_t thetaOffset_rad;
    float32_t thetaElec_rad;
		int16_t		thetaElec;
			float32_t thetaOffset;
			
	float32_t thetaErr;
	float32_t thetaErrSF;
	float32_t theta;
	float32_t thetaPll;
	float32_t thetaPll2;
	float32_t thetaDelta;
    float32_t offsetSF;     // Scale factor
    float32_t thetaEst;

    float32_t speedRef;
    float32_t speedEst;
    float32_t speedFlt;

	float32_t pll_Out;          // controller output
	float32_t pll_Umax;         // upper saturation limit
	float32_t pll_Umin;         // lower saturation limit
	float32_t pll_ui;           // integral term

	float32_t pll_Kp;           // proportional gain
	float32_t pll_KpMax;        // maximum proportional gain
	float32_t pll_KpMin;        // minimum proportional loop gain
	float32_t pll_KpSF;         // proportional gain coefficient
	float32_t pll_Ki;           // integral gain

	float32_t lpf_b0;           // Low Pass Filter Param b0
	float32_t lpf_a1;           // Low Pass Filter Param a1

	float32_t lpfFc_Hz;         // Low Pass Filter desired cut off frequency (Hz)
} ESMO_Obj;


//! \brief Defines the ESMO handle
//!
typedef struct _ESMO_Obj_ *ESMO_Handle;

typedef struct _SPDFR_obj_
{
    float32_t  theta;           // Input: reference set-point
    float32_t  thetaPrev;       // Input: feedback
    float32_t  K2;              // Parameter: proportional loop gain
    float32_t  K3;              // Parameter: integral gain
    float32_t  scaleFreq;       // Parameter:
    float32_t  speed_pu;        // Output: frequency
    float32_t  speed_Hz;        // Output: frequency
} SPDFR_Obj;

//! \brief Defines the SPDFR handle
//!
typedef struct _SPDFR_obj_ *SPDFR_Handle;

//--------------------------------------------------------------------------------------
typedef struct _MOTOR_Vars_t_
{
  bool ADCRegularLocked;    /*!< This flag is set when regular conversions are locked. Specific to G4XX. */     /* Cut 2.2 patch*/
	bool offset_complete;
	
	bool flagRunIdentAndOnLine;
	bool enableSpeedCtrl;
  bool enableCurrentCtrl;
	bool flagSetupController;
  bool flagEnableSpeedCtrl;
  bool flagEnableCurrentCtrl;
	
	bool flagCmdRpmOrHz;                // true-rpm, false-Hz

	uint16_t offset_I_ad_cnt; 
	
	uint16_t counterSpeed;
	uint16_t counterTrajSpeed;	
	
	uint16_t stateRunTimeCnt;
	
	uint32_t ISRCount;
	
	uint16_t timerBase_1ms;
	uint16_t timerCnt_5ms;
  uint16_t timerCnt_1s;
		
	uint16_t counterLEDC;       //!< Counter used to divide down a timer base for
                                //!< visually blinking an LED

  uint16_t counterLEDB;       //!< Counter used to divide down a timer base for
                                //!< visually blinking an LED

  uint16_t timeWaitLEDB;      //
		
	uint16_t alignTimeDelay;
  uint16_t forceRunTimeDelay;
  uint16_t startupTimeDelay;
  uint16_t flyingStartTimeDelay;
  uint16_t fwcTimeDelay;
		
	float32_t direction;                    // 1.0f->forward, -1.0f->reserve
	
	float32_t IdRated_A;              //!< Defines the Id rated current value, A
  float32_t IsRef_A;
  float32_t IsSet_A;
  float32_t Is_ffwd_A;
	
	float32_t Is_A;
  float32_t Vs_V;
  float32_t VsRef_pu;
  float32_t VsRef_V;
  float32_t VsMax_V;              // The maximum stator voltage magnitude, V
	float32_t maxVsMag_pu;
	float32_t oneOverDcBus_inv;	
	               
	float32_t speedRef_rpm;                 // Speed target value, rpm
	float32_t speed_rpm;                    // Speed feedback value, rpm
	float32_t rpm2Hz_sf;                    // Convert rpm to Hz
	float32_t hz2Rpm_sf;                    // convert Hz to rpm

  float32_t maxFrequency_Hz;
	float32_t speedRef_Hz;                  // Speed target value, Hz
	float32_t speed_int_Hz;                 // Speed reference value, Hz
	float32_t speed_Hz;                     // Speed feedback value, Hz
	float32_t speed_reg_Hz;                 // Speed feedback value, Hz
	float32_t speed_rps;
	
	float32_t ctrlPeriod_sec;     //!< Defines the controller execution     //!< period, sec
	float32_t freq_Hz;            //!< the freq input value
	float32_t angleDeltaFactor;   //!< predetermined factor for use in angle compensation calculation
	float32_t angleDelta_rad;     //!< the angle delta value
	float32_t angle_rad;          //!< the angle output value
	
  float32_t angleGen_rad;	      // the rotor angle from Generator modules  
	float32_t hElAngle;           // the rotor angle from FOC modules 
	
	float32_t fluxCurrent_A;
	float32_t alignCurrent_A;
	float32_t startCurrent_A;
	float32_t maxCurrent_A;
	float32_t brakingCurrent_A;
		
	float32_t speedStart_Hz;
  float32_t speedForce_Hz;
	float32_t speedAbs_Hz;
  float32_t speedFilter_Hz;
  float32_t speedFlyingStart_Hz;
	
  float32_t accelerationMax_Hzps;
  float32_t accelerationStart_Hzps;
		
	float32_t frswPos_sf;
	float32_t angleDelayed_sf;
			
	MATH_Vec2 Iab_A;                // the alpha/beta current values, A
	MATH_Vec2 Idq_in_A;    		// the d&q axis current are converter from 3-phase sampling input current of motor
	MATH_Vec2 Vdq_out_V;            // the output control voltage on d&q axis
  MATH_Vec2 Vab_out_V;            // the output control voltage on alpha&beta axis
	    
  MATH_Vec2 IdqRef_A;      // the reference current on d&q rotation axis	    
  MATH_Vec2 Idq_set_A;     //!< the reference current on d&q rotation axis        
  MATH_Vec2 Idq_out_A;     // the reference output current on d&q rotation axis
		
	TRAJ_Obj trajHandle_spd;        // the handle for the speed reference trajectory
	VS_FREQ_Obj VsFreqHandle;       //!< the handles for Vs per Freq for open loop control
	
	PI_Obj piHandle_Id;
	PI_Obj piHandle_Iq;
	PI_Obj piHandle_spd;
	
  float32_t anglePLLComp_rad;       // the rotor angle delay compensation value
  float32_t anglePLL_rad;           // the rotor angle from PLL modules
  float32_t speedPLL_Hz;            // the speed from PLL module
	float32_t angleFOC_rad;
	
  MOTOR_Status_e motorState;
		
	//!< the handle for the esmo object
	ESMO_Obj esmoHandle;
	
	//!< the handle for the speedfr object
  SPDFR_Obj spdfrHandle;
		
	SVGEN_Obj svgenHandle;
	
} MOTOR_Vars_t;

typedef struct _MOTOR_Vars_t_ *MOTOR_Handle;

//*****************************************************************************
//
//! \brief Defines a structure for the user parameters
//
//*****************************************************************************
typedef struct _USER_Params_
{
    float32_t dcBus_nominal_V;            //!< Defines the nominal DC bus
                                        //!< voltage, V
    float32_t pwmPeriod_usec;             //!< Defines the Pulse Width Modulation
                                        //!< (PWM) period, usec
	  float32_t ctrlPeriod_sec;             //!< Defines the controller execution
                                        //!< period, sec
    float32_t voltage_sf;                 //!< Defines the voltage scale factor
                                        //!< for the system
    float32_t current_sf;                 //!< Defines the current scale factor
                                        //!< for the system
	  uint16_t motor_numPolePairs;          //!< Defines the number of pole pairs
                                        //!< for the motor
    float32_t motor_Rs_Ohm;               //!< Defines the alpha stator
                                        //!< resistance, Ohm
    float32_t motor_Ls_d_H;               //!< Defines the direct stator
                                        //!< inductance, H
    float32_t motor_Ls_q_H;               //!< Defines the quadrature stator
                                        //!< inductance, H
    float32_t maxFrequency_Hz;          //!< Defines the maximum frequency value,
                                        //!< Hz
	  float32_t BWc_rps;                    //!< Defines the bandwidth of the
                                        //!< current controllers, rad/sec
    float32_t ctrlFreq_Hz;                //!< Defines the controller frequency,
                                        //!< Hz
    float32_t Vd_sf;                      //!< Defines the Vd scale factor to
                                        //!< prevent a Vd only component for
                                        //!< the Vdq vector
    float32_t maxVsMag_V;                 //!< Defines the maximum stator voltage
                                        //!< magnitude, V
    float32_t maxCurrent_A;               //!< Defines the maximum current value,
                                        //!< A																				
	  int_least16_t numCtrlTicksPerSpeedTick;
                                        //!< Defines the number of controller
                                        //!< clock ticks per speed controller
                                        //!< clock tick
    int_least16_t numCtrlTicksPerCurrentTick;
                                        //!< Defines the number of controller
                                        //!< clock ticks per current controller
                                        //!< clock tick
} USER_Params;
// **************************************************************************
//! \brief Defines the USER_Params handle
//!
typedef struct _USER_Params_ *userParams_Handle;

extern USER_Params   userParamsHandle;
extern MOTOR_Vars_t MOTOR_vars;

extern void MOTOR_vars_Init(void);
extern void ESMO_updateFilterParams(ESMO_Handle handle);
extern void ESMO_updatePLLParams(ESMO_Handle handle);
extern void ESMO_resetParams(ESMO_Handle handle);
extern void ESMO_resetPLL(ESMO_Handle handle);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of USER_VARS_H definition
