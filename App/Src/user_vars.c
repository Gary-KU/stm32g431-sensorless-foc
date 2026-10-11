
#include "main.h"
#include "user_vars.h"
#include "user_motor.h"
#include "user_mtr.h"
#include "user_foc.h"
#include <math.h>

#define __sqrtf(x) sqrtf(x)

USER_Params   userParamsHandle;
MOTOR_Vars_t MOTOR_vars;

void USER_setMotor1Params(userParams_Handle handle);
void ESMO_updateFilterParams(ESMO_Handle handle);
void ESMO_updatePLLParams(ESMO_Handle handle);
void ESMO_resetParams(ESMO_Handle handle);
void ESMO_resetPLL(ESMO_Handle handle);
void ESMO_init(ESMO_Handle handle,const USER_Params *pUserParams);
void SPDFR_setParams(SPDFR_Handle handle, const USER_Params *pUserParams);
void MOTOR_vars_Init(void);

//------------------------------------------------------------------------------
void USER_setMotor1Params(userParams_Handle handle)
{
	  USER_Params *objUser = (USER_Params *)handle;
	
		objUser->dcBus_nominal_V = USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;
	
		objUser->pwmPeriod_usec = USER_M1_CTRL_PERIOD_usec;
		objUser->ctrlPeriod_sec = USER_M1_CTRL_PERIOD_sec;
	
		objUser->current_sf = USER_M1_CURRENT_SF;
		objUser->voltage_sf = USER_M1_VOLTAGE_SF;
	
		objUser->motor_numPolePairs = USER_MOTOR1_NUM_POLE_PAIRS;
		objUser->motor_Rs_Ohm = g_foc_rs_ohm;
		objUser->motor_Ls_d_H = g_foc_ls_h;
		objUser->motor_Ls_q_H = g_foc_ls_h;
	
		objUser->maxFrequency_Hz = USER_MOTOR1_FREQ_MAX_Hz;
	
	  objUser->numCtrlTicksPerSpeedTick = USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK;
	
	  objUser->BWc_rps = MATH_TWO_PI * (float32_t)200.0f;
	
		objUser->ctrlFreq_Hz =  PWM_FREQUENCY;
		
		objUser->numCtrlTicksPerCurrentTick = 1;
		
		objUser->Vd_sf = USER_M1_VD_SF;
		
		objUser->maxVsMag_V =USER_M1_MAX_VS_MAG_PU*USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;
		
		objUser->maxCurrent_A =USER_MOTOR1_MAX_CURRENT_A;
}

void setupControllers(MOTOR_Handle handle,const USER_Params *pUserParams)
{
	    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
			USER_Params *objUser = (USER_Params *)(pUserParams);
	
	    float32_t Ls_d_H = objUser->motor_Ls_d_H;
			float32_t Ls_q_H = objUser->motor_Ls_q_H;

			float32_t Rs_Ohm = objUser->motor_Rs_Ohm;
			float32_t RdoverLd_rps = Rs_Ohm / Ls_d_H;
			float32_t RqoverLq_rps = Rs_Ohm / Ls_q_H;
	
	    float32_t BWc_rps = objUser->BWc_rps;
			float32_t currentCtrlPeriod_sec =
                (float32_t)objUser->numCtrlTicksPerCurrentTick /
                objUser->ctrlFreq_Hz;
	
	    float32_t outMax_V = objUser->Vd_sf * objUser->maxVsMag_V;
	
	    float32_t Kp_Id = 0.25f * Ls_d_H * BWc_rps;
			float32_t Ki_Id =  RdoverLd_rps * currentCtrlPeriod_sec;

			float32_t Kp_Iq = 0.25f * Ls_q_H * BWc_rps;
			float32_t Ki_Iq =  RqoverLq_rps * currentCtrlPeriod_sec;
			
			// set the Id controller
			//PI_setGains(obj->piHandle_Id, Kp_Id, Ki_Id);
			obj->piHandle_Id.Kp =Kp_Id;
			obj->piHandle_Id.Ki =Ki_Id;
			//PI_setUi(obj->piHandle_Id, 0.0f);
			obj->piHandle_Id.Ui =0.0f;
			//PI_setRefValue(obj->piHandle_Id, 0.0f);
			obj->piHandle_Id.refValue =0.0f;
			//PI_setFbackValue(obj->piHandle_Id, 0.0f);
			obj->piHandle_Id.fbackValue =0.0f;
			//PI_setFfwdValue(obj->piHandle_Id, 0.0f);
			obj->piHandle_Id.ffwdValue =0.0f;
			//PI_setMinMax(obj->piHandle_Id, -outMax_V, outMax_V);
			obj->piHandle_Id.outMin = -outMax_V;
			obj->piHandle_Id.outMax = outMax_V;
			
			// set the Iq controller
			//PI_setGains(obj->piHandle_Iq, Kp_Iq, Ki_Iq);
			obj->piHandle_Iq.Kp =Kp_Iq;
			obj->piHandle_Iq.Ki =Ki_Iq;
			//PI_setUi(obj->piHandle_Iq, 0.0f);
			obj->piHandle_Iq.Ui =0.0f;
			//PI_setRefValue(obj->piHandle_Iq, 0.0f);
			obj->piHandle_Iq.refValue =0.0f;
			//PI_setFbackValue(obj->piHandle_Iq, 0.0f);
			obj->piHandle_Iq.fbackValue =0.0f;
			//PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
			obj->piHandle_Iq.ffwdValue =0.0f;
			//PI_setMinMax(obj->piHandle_Iq, 0.0f, 0.0f);
			obj->piHandle_Iq.outMin = 0.0f;
			obj->piHandle_Iq.outMax = 0.0f;
			
			float32_t Kp_spd1 = 2.5f * objUser->maxCurrent_A / objUser->maxFrequency_Hz;
      float32_t Ki_spd1 = 5.0f * objUser->maxCurrent_A * objUser->ctrlPeriod_sec;

      //PI_setGains(obj->piHandle_spd, Kp_spd1, Ki_spd1);
			obj->piHandle_spd.Kp =Kp_spd1;
			obj->piHandle_spd.Ki =Ki_spd1;
			
			obj->piHandle_spd.refValue =0.0f;
			obj->piHandle_spd.fbackValue =0.0f;
			obj->piHandle_spd.ffwdValue =0.0f;
			
			obj->piHandle_spd.outMin = -objUser->maxCurrent_A;
			obj->piHandle_spd.outMax = objUser->maxCurrent_A;
}

void ESMO_updateFilterParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    // TempVarLpf = Fc *2 * PI * Ts
    float32_t tempVarLpf = obj->lpfFc_Hz * MATH_TWO_PI * obj->Ts;

    // a1 = 1/(1+ iqTempVarLpf)
    obj->lpf_a1 = 1.0f / (1.0f + tempVarLpf);

    // b0 = 1 - a1
    obj->lpf_b0 = 1.0f - obj->lpf_a1;

    //
    obj->Kslf = obj->filterFc_sf * MATH_TWO_PI * obj->base_wTs;

    return;
}

//------------------------------------------------------------------------------
void ESMO_updatePLLParams(ESMO_Handle handle)
{
	ESMO_Obj *obj = (ESMO_Obj *)handle;

	float32_t speedRefAbs = fabsf(obj->speedRef);

	obj->pll_Kp = obj->pll_KpMin + obj->pll_KpSF * speedRefAbs;
	obj->pll_Kp = MATH_sat(obj->pll_Kp, obj->pll_KpMax, obj->pll_KpMin);
}

//! \brief     Resets PLL integration for the ESMO controller
//! \param[in] handle    The ESMO controller handle
void ESMO_resetPLL(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_ui = 0.0f;
    obj->pll_Out = 0.0f;

    return;
}

//------------------------------------------------------------------------------
void ESMO_resetParams(ESMO_Handle handle)
{
	ESMO_Obj *obj = (ESMO_Obj *)handle;

	obj->pll_ui = 0.0f;
	obj->pll_Out = 0.0f;
	
	obj->speedEst = 0.0f;
	obj->speedFlt = 0.0f;
	
	obj->theta = 0.0f;
	obj->thetaErr = 0.0f;
	obj->thetaPll = 0.0f;
	obj->thetaEst = 0.0f;
	
	obj->Ed = 0.0f;
	obj->Eq = 0.0f;
	
	obj->Kslide = obj->KslideMin;
	obj->pll_Kp = obj->pll_KpMin;

    return;
}

void ESMO_init(ESMO_Handle handle,const USER_Params *pUserParams)
{
	ESMO_Obj *obj = (ESMO_Obj *)handle;
	
	//ESMO_setKslideParams
	obj->KslideMax = USER_MOTOR1_KSLIDE_MAX;
	obj->KslideMin = USER_MOTOR1_KSLIDE_MIN;	
	//ESMO_setPLLParams
	obj->pll_KpMax = USER_MOTOR1_PLL_KP_MAX;
	obj->pll_KpMin = USER_MOTOR1_PLL_KP_MIN;
	obj->pll_KpSF = USER_MOTOR1_PLL_KP_SF;
	//ESMO_setPLLKi
	obj->pll_Ki = USER_MOTOR1_PLL_KI;   // Optional
	
	obj->E0 = USER_MOTOR1_BEMF_THRESHOLD;	
	obj->offsetSF = USER_MOTOR1_THETA_OFFSET_SF;
	obj->filterFc_sf = USER_MOTOR1_BEMF_KSLF_FC_SF;
	obj->lpfFc_Hz = USER_MOTOR1_SPEED_LPF_FC_Hz;
	
	// set the ESMO controller parameters
	obj->scaleFreq_Hz = pUserParams->maxFrequency_Hz;
	
	obj->speed_sf = 1.0f / pUserParams->maxFrequency_Hz;;
	obj->voltage_sf = (MATH_ONE_OVER_THREE / 4096.0f) / pUserParams->voltage_sf;
	obj->current_sf = (1.0f / 4096.0f) / pUserParams->current_sf;
	
	obj->Ts = pUserParams->ctrlPeriod_sec;
  obj->base_wTs = obj->Ts * obj->scaleFreq_Hz;
  obj->thetaDelta = obj->scaleFreq_Hz * obj->Ts;
  obj->thetaErrSF = MATH_ONE_OVER_TWO_PI;
	
	obj->Kslide = obj->KslideMin;

	obj->pll_Kp = obj->pll_KpMin;
	obj->pll_ui   = 0.0f;
	obj->pll_Ki   = g_foc_pll_ki;   /* was 0 in the reference (relied on CORDIC) */
	obj->pll_Umax = 1.0f;       // The maximum value in Per-unit format, don't change
	obj->pll_Umin = -1.0f;      // The minimum value in Per-unit format, don't change

	obj->EstIalpha = 0.0f;
	obj->EstIbeta = 0.0f;
	
	obj->Zalpha = 0.0f;
	obj->Zbeta = 0.0f;

	obj->Ealpha = 0.0f;
	obj->Ebeta = 0.0f;
	float32_t motor_Rs = pUserParams->motor_Rs_Ohm;
	
	float32_t baseVIs = (pUserParams->voltage_sf * obj->Ts) / pUserParams->current_sf;

  obj->Fdsmopos = 1.0f - (motor_Rs / pUserParams->motor_Ls_d_H * obj->Ts);
  obj->Fqsmopos = 1.0f - (motor_Rs / pUserParams->motor_Ls_q_H * obj->Ts);
  obj->Gdsmopos = baseVIs / pUserParams->motor_Ls_d_H;
  obj->Gqsmopos = baseVIs / pUserParams->motor_Ls_q_H;
	
  ESMO_updateFilterParams(handle);

  return;	
}

//------------------------------------------------------------------------------
void SPDFR_setParams(SPDFR_Handle handle, const USER_Params *pUserParams)
{
    SPDFR_Obj *obj = (SPDFR_Obj *)handle;
    float32_t K1;

    obj->scaleFreq = pUserParams->maxFrequency_Hz;

    // Initialize the Speed module for speed calculation from sensor
    K1 = 1 / (pUserParams->maxFrequency_Hz * pUserParams->ctrlPeriod_sec);

    // Low-pass cut-off frequency
    obj->K2 = 1.0f / (1.0f + pUserParams->ctrlPeriod_sec * (MATH_TWO_PI  * 10.0f));
    obj->K3 = (1.0f - obj->K2) * K1;

    return;
} // end of ESMO_run() function

void MOTOR_vars_Init(void)
{
	MOTOR_vars.ADCRegularLocked = false;	
	
	MOTOR_vars.flagRunIdentAndOnLine = false;
	MOTOR_vars.enableSpeedCtrl = false;
  MOTOR_vars.enableCurrentCtrl = false;
	MOTOR_vars.flagEnableCurrentCtrl = true;
	MOTOR_vars.flagEnableSpeedCtrl = true;
	
	MOTOR_vars.direction = 1.0f;          // 1.0f->forward, -1.0f->reserve
	MOTOR_vars.ISRCount = 0;
	
	MOTOR_vars.stateRunTimeCnt =0;
	
	USER_setMotor1Params(&userParamsHandle);
	
	AdcData.dcBusvoltage_sf = USER_M1_VOLTAGE_SF;
	AdcData.current_sf = USER_M1_CURRENT_SF;
	
	MOTOR_vars.maxFrequency_Hz = USER_MOTOR1_FREQ_MAX_Hz;
	MOTOR_vars.speed_int_Hz = 0.0f;
	MOTOR_vars.speedFilter_Hz = 0.0f;
	MOTOR_vars.oneOverDcBus_inv = 0.0f;	
	MOTOR_vars.hElAngle = 0.0f; 
	MOTOR_vars.VsMax_V = 0.0f;	// The maximum stator voltage magnitude, V
	MOTOR_vars.maxVsMag_pu = USER_M1_MAX_VS_MAG_PU;
	
	MOTOR_vars.ctrlPeriod_sec = USER_M1_CTRL_PERIOD_sec;
	MOTOR_vars.angleDeltaFactor = MATH_TWO_PI*MOTOR_vars.ctrlPeriod_sec; //ctrlPeriod_sec * MATH_TWO_PI;  MATH_TWO_PI= 65536;          
  MOTOR_vars.angleDelta_rad = 0.0f;
  MOTOR_vars.angle_rad = 0.0f;
	
	MOTOR_vars.IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;
	
	MOTOR_vars.fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
  MOTOR_vars.alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
  MOTOR_vars.startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
  MOTOR_vars.maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;
		
	MOTOR_vars.speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;
	MOTOR_vars.speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;
	
	MOTOR_vars.accelerationMax_Hzps = USER_MOTOR1_ACCEL_MAX_Hzps;
  MOTOR_vars.accelerationStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;
		
	MOTOR_vars.trajHandle_spd.targetValue = 0.0f;
	MOTOR_vars.trajHandle_spd.intValue = 0.0f;	
	MOTOR_vars.trajHandle_spd.maxValue =USER_MOTOR1_FREQ_MAX_Hz;    
	MOTOR_vars.trajHandle_spd.minValue =-USER_MOTOR1_FREQ_MAX_Hz;   
	MOTOR_vars.trajHandle_spd.maxDelta =USER_M1_MAX_ACCEL_Hzps*MOTOR_vars.ctrlPeriod_sec; //maxAccel_Hzps * ctrlPeriod_sec

  MOTOR_vars.VsFreqHandle.MaxFreq = USER_MOTOR1_FREQ_MAX_Hz;
	MOTOR_vars.VsFreqHandle.LowFreq = USER_MOTOR1_FREQ_LOW_Hz;
	MOTOR_vars.VsFreqHandle.HighFreq = USER_MOTOR1_FREQ_HIGH_Hz;
	MOTOR_vars.VsFreqHandle.VoltMin = USER_MOTOR1_VOLT_MIN_V;
  MOTOR_vars.VsFreqHandle.VoltMax = USER_MOTOR1_VOLT_MAX_V;
	MOTOR_vars.VsFreqHandle.VfSlope = (MOTOR_vars.VsFreqHandle.VoltMax - MOTOR_vars.VsFreqHandle.VoltMin)/\
																					(MOTOR_vars.VsFreqHandle.HighFreq - MOTOR_vars.VsFreqHandle.LowFreq);
  MOTOR_vars.VsFreqHandle.maxVsMag_pu = USER_M1_MAX_VS_MAG_PU;
	MOTOR_vars.VsFreqHandle.Vdq_gain.value[0] = 0.3f; 
	MOTOR_vars.VsFreqHandle.Vdq_gain.value[1] = \
										__sqrtf(MOTOR_vars.VsFreqHandle.maxVsMag_pu * MOTOR_vars.VsFreqHandle.maxVsMag_pu -\
                      MOTOR_vars.VsFreqHandle.Vdq_gain.value[0] * MOTOR_vars.VsFreqHandle.Vdq_gain.value[0]);
	MOTOR_vars.VsFreqHandle.Vs_out =0;
	MOTOR_vars.VsFreqHandle.Vdq_out.value[0] = 0.0f;
	MOTOR_vars.VsFreqHandle.Vdq_out.value[1] = 0.0f;
	
	MOTOR_vars.Idq_set_A.value[0] = 0.0f;
	MOTOR_vars.Idq_set_A.value[1] = 1.0f;
			
//float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
//          Kp = 0.02*240/30 = 0.16;
//float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;
//          Ki = MATH_TWO_PI * (float32_t)5.0f /16000 =0.002
	MOTOR_vars.piHandle_Id.Kp =0.16f;      //32; q15
	MOTOR_vars.piHandle_Id.Ki =0.002f;     //3; q15
	
	MOTOR_vars.piHandle_Iq.Kp =0.16f;      //32; q15
	MOTOR_vars.piHandle_Iq.Ki =0.002f;     //3; q15

	// clear integral outputs of the controllers
	MOTOR_vars.piHandle_Id.refValue = 0.0f;
	MOTOR_vars.piHandle_Iq.refValue = 0.0f;
	MOTOR_vars.piHandle_spd.refValue = 0.0f;

	MOTOR_vars.piHandle_Id.Ui = 0.0f;
	MOTOR_vars.piHandle_Iq.Ui = 0.0f;
	MOTOR_vars.piHandle_spd.Ui = 0.0f;

    // clear current references
  MOTOR_vars.Idq_out_A.value[0] = 0.0f;
  MOTOR_vars.Idq_out_A.value[1] = 0.0f;

  MOTOR_vars.IdRated_A = 0.0f;
  MOTOR_vars.IsRef_A = 0.0f;

  // set the ESMO controller parameters
	ESMO_init(&MOTOR_vars.esmoHandle,&userParamsHandle);
	
	// set the spdfr parameters
  SPDFR_setParams(&MOTOR_vars.spdfrHandle,&userParamsHandle);
		
	MOTOR_vars.frswPos_sf = 0.6f;
	
  MOTOR_vars.angleDelayed_sf = 0.5f * MATH_TWO_PI * USER_M1_CTRL_PERIOD_sec;
	
  MOTOR_vars.alignTimeDelay = (uint16_t)(PWM_FREQUENCY * 2.0f);      // 2.0s
  MOTOR_vars.forceRunTimeDelay = (uint16_t)(PWM_FREQUENCY * 1.0f);   // 1.0s
  MOTOR_vars.startupTimeDelay = (uint16_t)(PWM_FREQUENCY * 2.0f);    // 2.0s
	
  MOTOR_vars.flagCmdRpmOrHz = false;     // the speed command is rpm
  MOTOR_vars.rpm2Hz_sf = userParamsHandle.motor_numPolePairs / 60.0f;
  MOTOR_vars.hz2Rpm_sf = 60.0f / userParamsHandle.motor_numPolePairs;
	
	setupControllers(&MOTOR_vars,&userParamsHandle);	
	
//PwmData.deadband = deadband;
//PwmData.noiseWindow = noiseWindow;
//PwmData.adcSample = adcSample;
//PwmData.minCMPValue = deadband + noiseWindow + adcSample;
	PwmData.minCMPValue = 200;
}
