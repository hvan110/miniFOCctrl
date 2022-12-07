#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"

typedef int int32_T;
typedef float real32_T;
typedef double real64_T;

#define PWM_TIM_PULSE_TPWM  (CKTIM/(PWM_FREQ))
#define Udc 24.0f
#define PI					3.14159265358979f
#define ADC_REF_V                   (float)(3.3f)
#define SAMPLE_RES                  (double)(0.01)
#define AMP_GAIN                    (double)(18.0)
#define SAMPLE_CURR_CON_FACTOR      (double)(ADC_REF_V/4095.0/AMP_GAIN/SAMPLE_RES/10.0)
	
#define R_HIGH  200.0F
#define R_LOW		10.0F
#define SAMPLE_VOL_CON_FACTOR 	 ADC_REF_V /4095.0F / (R_LOW/(R_LOW+R_HIGH))

#define FOC_PERIOD (1.0f / (real32_T)PWM_FREQ)
#define SPEED_PID_PERIOD (1.0f*(float)(PID_SPEED_SAMPLING_TIME+1)/2000.0f)

#define PID_SPEED_SAMPLING_500us      0     // min 500usec
#define PID_SPEED_SAMPLING_1ms        1
#define PID_SPEED_SAMPLING_2ms        3     // (3+1)*500usec = 2msec
#define PID_SPEED_SAMPLING_5ms        9		// (9+1)*500usec = 5msec		
#define PID_SPEED_SAMPLING_10ms       19	// (19+1)*500usec = 10msec
#define PID_SPEED_SAMPLING_20ms       39	// (39+1)*500usec = 20msec
#define PID_SPEED_SAMPLING_127ms      255   // max (255-1)*500us = 127 ms

#define POSI_TIME PID_SPEED_SAMPLING_5ms
#define SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_1ms)
#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))
#define PID_SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_2ms)
#define Ref_Signal_Time PID_SPEED_SAMPLING_1ms
#define DISPLAY_TIME PID_SPEED_SAMPLING_10ms
#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz

#define PWM2_MODE 0
#define PWM1_MODE 1
#define TNOISE_NS 1550  
#define TRISE_NS 1550    
#define SAMPLING_TIME_NS   700  
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 168uL)/1000uL) 
#define TNOISE (u16)((((u16)(TNOISE_NS)) * 168uL)/1000uL)
#define TRISE (u16)((((u16)(TRISE_NS)) * 168uL)/1000uL)
#define TDEAD (u16)((DEADTIME_NS * 168uL)/1000uL)
#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif
#define TW_AFTER ((u16)(((DEADTIME_NS+MAX_TNTR_NS)*168uL)/1000ul))
#define TW_BEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*168uL)/1000ul))+1)


#define Start_V 3.5f//¶¨ÒåÆô¶¯Á¦¾Ø
#define cpr (float)(2.0f*PI)
#define Max_Vbus 24.0F

typedef struct
{
  real32_T Ia;
  real32_T Ib;
  real32_T Ic;
}CURRENT_ABC_DEF;

typedef struct
{
  real32_T Ialpha;
  real32_T Ibeta;
}CURRENT_ALPHA_BETA_DEF;

typedef struct
{
  real32_T Valpha;
  real32_T Vbeta;
}VOLTAGE_ALPHA_BETA_DEF;

typedef struct
{
  real32_T Cos;
  real32_T Sin;
}TRANSF_COS_SIN_DEF;

typedef struct
{
  real32_T Id;
  real32_T Iq;
}CURRENT_DQ_DEF;

typedef struct
{
  real32_T Vd;
  real32_T Vq;
}VOLTAGE_DQ_DEF;

typedef struct
{
  real32_T P_Gain;
  real32_T I_Gain;
  real32_T D_Gain;
  real32_T B_Gain;
  real32_T Max_Output;
  real32_T Min_Output;
  real32_T I_Sum;
}CURRENT_PID_DEF;

typedef struct
{
  real32_T P_Gain;
  real32_T I_Gain;
  real32_T D_Gain;
  real32_T B_Gain;
  real32_T Max_Output;
  real32_T Min_Output;
  real32_T I_Sum;
}SPEED_PID_DEF;

typedef struct
{
  real32_T P_Gain;
  real32_T I_Gain;
  real32_T D_Gain;
  real32_T B_Gain;
  real32_T Max_Output;
  real32_T Min_Output;
  real32_T I_Sum;
}POSI_PID_DEF;


typedef enum 
{
IDLE, INIT, START, RUN, STOP, BRAKE, WAIT, FAULT
} SystStatus_t;

void shut_pwm(void);
float get_DC_BUS(void);
float get_angle(void);
float GetMotor_Speed_RPM(void);
float GetMotorPreSpeed(void);
void board_config(void);
void foc_algorithm_step(void);
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp);
void Angle_To_Cos_Sin(real32_T angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp);
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp);
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp);
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,real32_T Udc_temp,real32_T Tpwm_temp);
real32_T ENC_Get_Electrical_Angle(void);
CURRENT_ABC_DEF get_Iab(void);
void Current_PID_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,CURRENT_PID_DEF* current_pid_temp);
void Speed_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,SPEED_PID_DEF* current_pid_temp);
void Posi_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,POSI_PID_DEF* current_pid_temp);
void PID_Init(void);
void Start_Up(void);
real32_T ENC_Get_Mechanical_Angle_X(void);
real32_T get_motor_rpm(void);
void LED_DIR(u32 ms);
float get_angle_X(void);
void set_posi(float posi);
void set_speed(float speed_rpm);

#endif


