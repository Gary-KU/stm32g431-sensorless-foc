
#include "main.h"
#include "user_vars.h"
#include "user_motor.h"
#include "user_mtr.h"
#include "foc_hw.h"
#include "stm32g4xx_ll_cordic.h"
#include <math.h>

/* CORDIC-backed trig instead of CMSIS-DSP arm_sin_f32 / arm_cos_f32. */
#define arm_sin_f32(a)  FOC_Sin(a)
#define arm_cos_f32(a)  FOC_Cos(a)
#define __sqrtf(x)      sqrtf(x)

/* Ported from the reference user_foc.c so the FOC app layer does not need it. */
float32_t MATH_incrAngle(const float32_t angle_rad, const float32_t angleDelta_rad)
{
  float32_t a = angle_rad + angleDelta_rad;
  a = (a > MATH_PI) ? a - MATH_TWO_PI : a;
  a = (a < -MATH_PI) ? a + MATH_TWO_PI : a;
  return a;
}

/* CORDIC FUNCTION: PHASE q1.31 (Electrical Angle computation) */
#define CORDIC_CONFIG_PHASE     (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
         LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 |\
         LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/**
  * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf alpha and beta.
  * @param  wBemf_alfa_est estimated Bemf alpha on the stator reference frame.
  *         wBemf_beta_est estimated Bemf beta on the stator reference frame.
  * @retval int16_t rotor electrical angle (s16degrees).
  */

/**
  * @brief  This function codify a floating point number into the relative 32bit integer.
  * @param  float Floating point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
__weak uint32_t MCM_floatToIntBit( float_t x ) //cstat !MISRAC2012-Dir-4.6_a
{
  const uint32_t *pInt;
  pInt = (uint32_t *)(&x); //cstat !MISRAC2012-Rule-11.3
  return (*pInt);
}
//*****************************************************************************
//
//! \brief     Saturates the input value between the minimum and maximum values
//!
//! \param[in] in   The input value
//!
//! \param[in] max  The maximum value allowed
//!
//! \param[in] min  The minimum value allowed
//!
//! \return    The saturated value
//
//*****************************************************************************
float32_t MATH_sat(const float32_t in, const float32_t max, const float32_t min)
{
    float32_t out = in;

//    if(in < min)
//    {
//        out = min;
//    }
//    else if(in > max)
//    {
//        out = max;
//    }

    out = (out > max) ? max : out;
    out = (out < min) ? min : out;

    return(out);
} // end of MATH_sat() function

void CLARKE_run(const MATH_Vec3 *pInVec, MATH_Vec2 *pOutVec)
{

    uint_least8_t numSensors = 2;

    float32_t alpha_sf ;
    float32_t beta_sf ;
	
    // initialize the Clarke transform module for current
    if(3 == numSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;         
        beta_sf  = MATH_ONE_OVER_SQRT_THREE;      		
    }
    else if(2 == numSensors)
    {
        alpha_sf = 1.0f;
        beta_sf	 = MATH_ONE_OVER_SQRT_THREE;        
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }
    //
    // Perform the Clarke transform for either 2 or 3 sensor setups
    //
    if(numSensors == 3)
    {
        pOutVec->value[0] =((pInVec->value[0] * (float32_t)2.0f) - (pInVec->value[1] + pInVec->value[2])) * alpha_sf;
        pOutVec->value[1] = (pInVec->value[1] - pInVec->value[2]) * beta_sf;
    }
    else if(numSensors == 2)
    {
        pOutVec->value[0] = pInVec->value[0] * alpha_sf;
        pOutVec->value[1] = (pInVec->value[0] + (pInVec->value[1] * (float32_t)2.0f)) * beta_sf;
    }
		
    return;
} // end of CLARKE_run() function

void PARK_run(const MATH_Vec2 *pInVec, MATH_Vec2 *pOutVec,float32_t Theta)
{
	float32_t sinTh = arm_sin_f32(Theta);
  float32_t cosTh = arm_cos_f32(Theta);

	float32_t value_0 = pInVec->value[0];
  float32_t value_1 = pInVec->value[1];

	pOutVec->value[0] = (value_0 * cosTh) + (value_1 * sinTh);
	pOutVec->value[1] = (value_1 * cosTh) - (value_0 * sinTh);
	
  return;
} // end of PARK_run() function

void IPARK_run(const MATH_Vec2 *pInVec, MATH_Vec2 *pOutVec,float32_t Theta)
{
	float32_t sinTh = arm_sin_f32(Theta);
  float32_t cosTh = arm_cos_f32(Theta);

  float32_t value_0 = pInVec->value[0];
  float32_t value_1 = pInVec->value[1];
	
	pOutVec->value[0] = (value_0 * cosTh) - (value_1 * sinTh);
	pOutVec->value[1] = (value_1 * cosTh) + (value_0 * sinTh);

  return;
} // end of IPARK_run() function

void ANGLE_GEN_run(const float32_t freq_Hz)
{
	
    MOTOR_vars.angleDelta_rad = freq_Hz * MOTOR_vars.angleDeltaFactor;

    // increment the angle
    MOTOR_vars.angle_rad += MOTOR_vars.angleDelta_rad;

	  if(MOTOR_vars.angle_rad > MATH_PI)
    {
        MOTOR_vars.angle_rad -= MATH_TWO_PI;
    }
    else if(MOTOR_vars.angle_rad < -MATH_PI)
    {
        MOTOR_vars.angle_rad += MATH_TWO_PI;
    }
		
    return;
} // end of ANGLE_GEN_run()


void TRAJ_run(float32_t targetValue)
{
    float32_t error; 
	
		error = targetValue - MOTOR_vars.trajHandle_spd.intValue;
	
		MOTOR_vars.trajHandle_spd.intValue +=MATH_sat(error,MOTOR_vars.trajHandle_spd.maxDelta,-MOTOR_vars.trajHandle_spd.maxDelta);

		MOTOR_vars.trajHandle_spd.intValue =MATH_sat(MOTOR_vars.trajHandle_spd.intValue,MOTOR_vars.trajHandle_spd.maxValue,MOTOR_vars.trajHandle_spd.minValue);
}// end of TRAJ_run() function

void VS_FREQ_run(const float32_t Freq_pu)
{
	  if(Freq_pu < 0.0f)
		{
			MOTOR_vars.VsFreqHandle.Freq =-Freq_pu;
		}
		else
		{
			MOTOR_vars.VsFreqHandle.Freq = Freq_pu;
		}
		
    if(MOTOR_vars.VsFreqHandle.Freq <= MOTOR_vars.VsFreqHandle.LowFreq)
    {
        MOTOR_vars.VsFreqHandle.Vs_out = MOTOR_vars.VsFreqHandle.VoltMin;
    }
    else if(MOTOR_vars.VsFreqHandle.Freq >= MOTOR_vars.VsFreqHandle.HighFreq)
    {
        MOTOR_vars.VsFreqHandle.Vs_out = MOTOR_vars.VsFreqHandle.VoltMax;
    }
    else
    {
        MOTOR_vars.VsFreqHandle.Vs_out = MOTOR_vars.VsFreqHandle.VoltMin + MOTOR_vars.VsFreqHandle.VfSlope * (MOTOR_vars.VsFreqHandle.Freq - MOTOR_vars.VsFreqHandle.LowFreq);
    }

    MOTOR_vars.VsFreqHandle.Vdq_out.value[0] =  MOTOR_vars.VsFreqHandle.Vs_out *  MOTOR_vars.VsFreqHandle.Vdq_gain.value[0];

    if( Freq_pu >  0.0f)
    {
         MOTOR_vars.VsFreqHandle.Vdq_out.value[1] =  MOTOR_vars.VsFreqHandle.Vs_out *  MOTOR_vars.VsFreqHandle.Vdq_gain.value[1];
    }
    else
    {
         MOTOR_vars.VsFreqHandle.Vdq_out.value[1] = - MOTOR_vars.VsFreqHandle.Vs_out *  MOTOR_vars.VsFreqHandle.Vdq_gain.value[1];
    }

    return;
} // end of VS_FREQ_run()

void PI_run_series(PI_Handle handle, const  float32_t refValue,
              const float32_t fbackValue, const float32_t ffwdValue,
              float32_t *pOutValue)
{
    float32_t Error;
    float32_t Kp = handle->Kp;
    float32_t Ki = handle->Ki;
    float32_t Up;
    float32_t Ui = handle->Ui;
    float32_t outMax = handle->outMax;
    float32_t outMin = handle->outMin;

    Error = refValue - fbackValue;

    // Compute the proportional output
    Up = Kp * Error;

    // Compute the integral output with saturation
//  Ui = __fsat(Ui + (Ki * Up), outMax, outMin);
	
		Ui = MATH_sat(Ui + (Ki * Up), outMax, outMin);
		
    handle->Ui =Ui;
		handle->refValue =refValue;
		handle->fbackValue =fbackValue;
		handle->ffwdValue =ffwdValue;

    // Saturate the output
//  *pOutValue = __fsat(Up + Ui + ffwdValue, outMax, outMin);

		*pOutValue = MATH_sat(Up + Ui + ffwdValue, outMax, outMin);
		
    return;
} // end of PI_run_series() function


void SVGEN_run(SVGEN_Handle handle,const MATH_Vec2 *pVab_V, MATH_Vec3 *pVabc_pu)
{
	float32_t Vmax_pu = 0,Vmin_pu = 0,Vcom_pu;
	float32_t oneOverDcBus_invV = handle->oneOverDcBus_invV;
	SVM_Mode_e svmMode = SVM_COM_C;   //SVGEN_getMode(handle);
	
  float32_t Valpha_pu = pVab_V->value[0] * oneOverDcBus_invV;
  float32_t Vbeta_pu = pVab_V->value[1] * oneOverDcBus_invV;
	
  float32_t Va_tmp = (float32_t)(0.5f) * Valpha_pu;
  float32_t Vb_tmp = MATH_SQRTTHREE_OVER_TWO * Vbeta_pu;

  float32_t Va_pu = Valpha_pu;
	
// -0.5*Valpha + sqrt(3)/2 * Vbeta
  float32_t Vb_pu = -Va_tmp + Vb_tmp;

// -0.5*Valpha - sqrt(3)/2 * Vbeta
  float32_t Vc_pu = -Va_tmp - Vb_tmp;
	
	// Find Vmax and Vmin
	if(Va_pu > Vb_pu)
	{
			Vmax_pu = Va_pu;
			Vmin_pu = Vb_pu;
	}
	else
	{
			Vmax_pu = Vb_pu;
			Vmin_pu = Va_pu;
	}

	if(Vc_pu > Vmax_pu)
	{
			Vmax_pu = Vc_pu;
	}
	else if(Vc_pu < Vmin_pu)
	{
			Vmin_pu = Vc_pu;
	}

	// Compute Vcom = 0.5*(Vmax+Vmin)
  Vcom_pu = 0.5f * (Vmax_pu + Vmin_pu);
	
	if(svmMode == SVM_COM_C)
	{
			// Subtract common-mode term to achieve SV modulation
			pVabc_pu->value[0] = (Va_pu - Vcom_pu);
			pVabc_pu->value[1] = (Vb_pu - Vcom_pu);
			pVabc_pu->value[2] = (Vc_pu - Vcom_pu);
	}
	else if(svmMode == SVM_MIN_C)
	{
			pVabc_pu->value[0] = (Va_pu - Vmin_pu) - 0.5f;
			pVabc_pu->value[1] = (Vb_pu - Vmin_pu) - 0.5f;
			pVabc_pu->value[2] = (Vc_pu - Vmin_pu) - 0.5f;
	}
	else if(svmMode == SVM_MAX_C)
	{
			pVabc_pu->value[0] = (Va_pu - Vmax_pu) + 0.5f;
			pVabc_pu->value[1] = (Vb_pu - Vmax_pu) + 0.5f;
			pVabc_pu->value[2] = (Vc_pu - Vmax_pu) + 0.5f;
	}
	
//	pVabc_pu->value[2] = Vcom_pu;
	return;
}

//! \brief     Set the SPDFR controller
//! \param[in] handle   The SPDFR controller handle
void SPDFR_run(SPDFR_Handle handle, float32_t theta)
{
    SPDFR_Obj *obj = (SPDFR_Obj *)handle;
    float32_t error;

    obj->theta  = theta * MATH_ONE_OVER_TWO_PI;

    // error cal
    error = obj->theta - obj->thetaPrev;

    if(error < -0.5f)
    {
        error += 1.0f;
    }
    else if(error > 0.5f)
    {
        error -= 1.0f;
    }

    // Update the electrical angle
    obj->thetaPrev = obj->theta;

    // Low-pass filter
    obj->speed_pu = (obj->K2 * obj->speed_pu) + (obj->K3 * error);

    // Saturate the output
    obj->speed_pu = MATH_sat(obj->speed_pu, 1.0f, -1.0f);

    // Change motor speed for pu to rpm value
    obj->speed_Hz = obj->scaleFreq * obj->speed_pu;

    return;
}

//! \brief     Update Kslide for the ESMO controller
//! \param[in] handle      The ESMO controller handle
void ESMO_updateKslide(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    if(obj->Kslide < obj->KslideMax)
    {
        obj->Kslide += 0.000002f;
    }

    return;
}

void ESMO_run(ESMO_Handle handle,float32_t Vdcbus, MATH_vec3 *pVabc_pu, MATH_vec2 *pIabVec)
{
		ESMO_Obj *obj = (ESMO_Obj *)handle;

    // Scale the incomming modulation functions with the DC bus voltage value
    // and calculate the 3 Phase voltages
    float32_t Vtemp   = Vdcbus * obj->voltage_sf;

    float32_t VphaseA = Vtemp *
            (pVabc_pu->value[0] * 2.0f - pVabc_pu->value[1] - pVabc_pu->value[2]);

    float32_t VphaseB = Vtemp *
            (pVabc_pu->value[1] * 2.0f - pVabc_pu->value[0] - pVabc_pu->value[2]);

    // Voltage transformation (a,b,c)  ->  (Alpha,Beta)
    obj->Valpha = VphaseA;
    obj->Vbeta = (VphaseA + VphaseB * 2.0f) * MATH_ONE_OVER_SQRT_THREE;

    // Sliding mode current observer
    float32_t ValphaError = obj->Valpha - obj->Ealpha - obj->Zalpha;
    float32_t VbetaError  = obj->Vbeta - obj->Ebeta - obj->Zbeta;

    obj->EstIalpha = obj->Gdsmopos * ValphaError + obj->Fdsmopos * obj->EstIalpha;
    obj->EstIbeta  = obj->Gqsmopos * VbetaError  + obj->Fqsmopos * obj->EstIbeta;

    //  Current errors
    float32_t IalphaError = obj->EstIalpha - pIabVec->value[0] * obj->current_sf;
    float32_t IbetaError  = obj->EstIbeta  - pIabVec->value[1] * obj->current_sf;

    // Sliding control calculator
    obj->Zalpha = MATH_sat(IalphaError, obj->E0, -obj->E0) * obj->Kslide;
    obj->Zbeta  = MATH_sat(IbetaError,  obj->E0, -obj->E0) * obj->Kslide;

    //  Sliding control filter -> back EMF calculator
    obj->Ealpha = obj->Ealpha + obj->Kslf * (obj->Zalpha - obj->Ealpha);
    obj->Ebeta  = obj->Ebeta  + obj->Kslf * (obj->Zbeta  - obj->Ebeta);
		
//		obj->thetaOffset_rad = obj->speedRef * obj->offsetSF + 0.005f;
//    obj->thetaElec_rad = __atan2(obj->Ealpha, obj->Ebeta) + obj->thetaOffset_rad;
		
//		obj->Ealphaq31 = obj->Ealpha *2147483647;
//		obj->Ebetaq31  = obj->Ebeta *2147483647;		
//		obj->thetaElec_rad = (float32_t)obj->thetaElec/32767 ;	
//		
//		obj->thetaOffset_rad = obj->speedRef * obj->offsetSF + 0.005f;
//		obj->thetaElec_rad = obj->thetaElec_rad *MATH_PI + obj->thetaOffset_rad;
//		
//    if(obj->thetaElec_rad > MATH_PI)
//    {
//        obj->thetaElec_rad -= MATH_TWO_PI;
//    }
//    else if(obj->thetaElec_rad < (-MATH_PI))
//    {
//        obj->thetaElec_rad += MATH_TWO_PI;
//    }
		
		
//    float32_t thetaOffset = __atan2puf32((obj->speedRef * obj->offsetSF), obj->Kslf);
//    obj->thetaPll  = obj->theta - thetaOffset;

//    float32_t pllSine   = __sinpuf32(obj->thetaPll);
//    float32_t pllCosine = __cospuf32(obj->thetaPll);
		
    
		/* Use the PLL's own estimate as the demodulation angle. The reference added
		 * a CORDIC-derived thetaOffset here to cancel the PLL steady-state phase
		 * lag (its integral term was disabled); that path is unreliable on this
		 * board and lost lock above ~24 Hz. With pll_Ki enabled the II-type PLL
		 * needs no such compensation. */
		obj->thetaPll = obj->thetaEst;
		if(obj->thetaPll > MATH_PI)
    {
        obj->thetaPll -= MATH_TWO_PI;
    }
    else if(obj->thetaPll < (-MATH_PI))
    {
        obj->thetaPll += MATH_TWO_PI;
    }
    float32_t pllSine   = arm_sin_f32(obj->thetaPll);
    float32_t pllCosine = arm_cos_f32(obj->thetaPll);
		
    obj->Ed = obj->Ealpha * pllCosine + obj->Ebeta  * pllSine;
    obj->Eq = obj->Ebeta  * pllCosine - obj->Ealpha * pllSine;
    obj->Eq_mag = __sqrtf(obj->Ealpha * obj->Ealpha + obj->Ebeta * obj->Ebeta);

    //0.1591549431 = 1/6.28 (1/(2*PI())
    float32_t thetaErrSF = obj->thetaErrSF;

    if(obj->Eq >= 0.0f)
    {
        thetaErrSF = -obj->thetaErrSF;
    }

    /* Guard the division: Eq_mag passes through zero whenever the estimated
     * back-EMF vector crosses the origin (hand-over, transients). The reference
     * divides unconditionally, which yields NaN and poisons the whole angle
     * (saturated current -> overcurrent trip). */
    if (obj->Eq_mag > 1.0e-6f) {
      obj->thetaErr = obj->Ed * thetaErrSF / obj->Eq_mag;
    } else {
      obj->thetaErr = 0.0f;
    }

    // integral term (clamped: without this the PLL winds up while the BEMF is
    // still tiny during align / I-F start and the speed estimate saturates)
    obj->pll_ui = MATH_sat((obj->pll_Ki * obj->thetaErr) + obj->pll_ui,
                           obj->pll_Umax, obj->pll_Umin);

    // control output
    obj->pll_Out = MATH_sat((obj->pll_Kp * obj->thetaErr + obj->pll_ui),
                          obj->pll_Umax, obj->pll_Umin);

    obj->speedEst = (obj->pll_Out + obj->speedEst) * 0.5f;

    // low pass filter for estimation speed
    obj->speedFlt = obj->lpf_b0 * obj->pll_Out + obj->lpf_a1 * obj->speedFlt;

    // speed integration to get the rotor angle
    obj->theta = obj->theta + obj->speedFlt * obj->thetaDelta;

    if(obj->theta > 1.0f)
    {
        obj->theta -= 1.0f;
    }
    else if(obj->theta < -1.0f)
    {
        obj->theta += 1.0f;
    }

    obj->thetaEst = obj->theta * MATH_TWO_PI;

    if(obj->thetaEst > MATH_PI)
    {
        obj->thetaEst -= MATH_TWO_PI;
    }
    else if(obj->thetaEst < (-MATH_PI))
    {
        obj->thetaEst += MATH_TWO_PI;
    }

	return;
} // end of ESMO_run() function
