#include "motor.h"
#include "arm_math.h"
#include "ref_signal_in.h"
#include "Sensor_SPI.h"
#include "user_fcn.h"


real32_T ID_REF = 0.0F;
real32_T IQ_REF = 0.0F;


real32_T D_PI_I = -4000.0F; // about Rs/3 * PWM_FREQ -1050
real32_T D_PI_KB = -100.0F;//-50
real32_T D_PI_LOW_LIMIT = -24.0F;
real32_T D_PI_P = -4.0F; //about Ls/3 * PWM_FREQ    -1.5
real32_T D_PI_UP_LIMIT = 24.0F;

real32_T Q_PI_I = -4000.0F;
real32_T Q_PI_KB = -100.0F;
real32_T Q_PI_LOW_LIMIT = -24.0F;
real32_T Q_PI_P = -4.0F;
real32_T Q_PI_UP_LIMIT = 24.0F;

real32_T SPEED_PI_I = 0.05F;
real32_T SPEED_PI_KB = 0.005f;
real32_T SPEED_PI_LOW_LIMIT = -5.0F;
real32_T SPEED_PI_P = -0.001F;
real32_T SPEED_PI_UP_LIMIT = 5.0F;

real32_T POSI_PI_I = 0.0F;
real32_T POSI_PI_D = 0.0f;
real32_T POSI_PI_LOW_LIMIT = -200.0F;
real32_T POSI_PI_P = 150.0F; 
real32_T POSI_PI_UP_LIMIT = 200.0F;

 u32 hPhaseA_OffSet=2037;//2037
 u32 hPhaseB_OffSet=2030;//2030
 u32 hPhaseC_OffSet=2033;//2033
int32_T sector;
u8 State;
CURRENT_ABC_DEF Current_Iabc;
CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
TRANSF_COS_SIN_DEF Transf_Cos_Sin;
CURRENT_DQ_DEF Current_Idq; 
VOLTAGE_DQ_DEF Voltage_DQ;
CURRENT_PID_DEF Current_D_PID;
CURRENT_PID_DEF Current_Q_PID;
SPEED_PID_DEF Speed_Pid;
POSI_PID_DEF  Posi_Pid;
u8 PWM4Direction=PWM2_MODE;
real32_T Speed_Ref = 0.0F;
real32_T Speed_Pid_Out;  
real32_T Speed_Fdk;

real32_T Posi_Ref = 0.0;
real32_T Posi_Pid_Out;
real32_T Posi_Fdk;

float encoder_offset;
int circle_num=0;
/* extern */
extern float ref_in;
extern float ref_out;
extern float Uq;
extern float Kpp;
extern float wr_test;
extern float ref_out;
//encoder
static u16 hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
static u16 hPosiMeas_Timebase = POSI_TIME;
volatile u8 bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
extern  u16 hdisplay_Timebase;
real32_T cnt;
volatile u8 ref_Signal_tim_500us = Ref_Signal_Time;
//speed
float velocity;
static float FullRotationOffset;
static float angle_data,d_angle;
static float angle_data_prev;
float current_angle;
float angle_prev;

void board_config(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,PWM_PERIOD-1);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
 
	PID_Init();
	Start_Up();
	


	
	
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)//ADC回调函数
{
	if(State==RUN)
		foc_algorithm_step();	
}





void HAL_SYSTICK_Callback(void)
{
	
	//500us if update, write HAL_InitTick()  1000U->2000U (1ms->500us)
	
	/*speed check*/
	 if(hSpeedMeas_Timebase_500us !=0)  
  {								
    hSpeedMeas_Timebase_500us--;	  
  }									  
  else
  {
    hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
		GetMotorPreSpeed();
	}
	/*speed loop cacl*/
	if (bPID_Speed_Sampling_Time_500us != 0 ) 
  {
    bPID_Speed_Sampling_Time_500us --;
  }
	else
  { 
    bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
		Speed_Fdk = velocity;	
    Speed_Pid_Calc(Speed_Ref,Speed_Fdk,&Speed_Pid_Out,&Speed_Pid);
				
  }
	
	/*posi cacl*/
	 if(hPosiMeas_Timebase !=0)  
  {								
    hPosiMeas_Timebase--;	  
  }									  
  else
  {
    hPosiMeas_Timebase = POSI_TIME;
		Posi_Fdk = get_angle_X();
		Posi_Pid_Calc(Posi_Ref,Posi_Fdk,&Posi_Pid_Out,&Posi_Pid);
	}
	
	
	/*get Sine Wave*/
	if(State==RUN)
	{
		if(ref_Signal_tim_500us !=0)
		{
			ref_Signal_tim_500us--;
		}
		else
		{
			ref_Signal_tim_500us=Ref_Signal_Time;
			ref_Core();
			LED_DIR(1000);
		}
	}
	else if(State==FAULT)
		LED_DIR(80);
	
	/*PID Speed-loop Ref*/
	
	Speed_Ref = 200.0;
//	set_speed(0.0);

		
	  
}



/*FOC Core part 10Khz*/
void foc_algorithm_step(void) 
{

	
	Clarke_Transf(get_Iab(),&Current_Ialpha_beta);
	Angle_To_Cos_Sin(ENC_Get_Electrical_Angle()-encoder_offset,&Transf_Cos_Sin); //when use openloop, angle=cnt;ENC_Get_Electrical_Angle()-encoder_offset
	Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);
	 
//	/*PID Control part*/
	IQ_REF = Speed_Pid_Out;
//	IQ_REF = kg_f_control(0.01);
	ID_REF = 0.0F;
	Current_PID_Calc(ID_REF,Current_Idq.Id,&Voltage_DQ.Vd,&Current_D_PID);   
  Current_PID_Calc(IQ_REF,Current_Idq.Iq,&Voltage_DQ.Vq,&Current_Q_PID); 
	
	Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta); 
	SVPWM_Calc(Voltage_Alpha_Beta,Udc,PWM_TIM_PULSE_TPWM);

}
void Start_Up(void)
{
	Voltage_DQ.Vd=Start_V;
  Voltage_DQ.Vq=0.0f;
	Angle_To_Cos_Sin(0.0f,&Transf_Cos_Sin); 
	Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta); 
	SVPWM_Calc(Voltage_Alpha_Beta,Udc,PWM_TIM_PULSE_TPWM);	
	HAL_Delay(300);
	encoder_offset = ENC_Get_Electrical_Angle();
	HAL_Delay(20);
	Voltage_DQ.Vd=0.0f;
  Voltage_DQ.Vq=0.0f;
	Angle_To_Cos_Sin(0.0f,&Transf_Cos_Sin); 
	Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta); 
	SVPWM_Calc(Voltage_Alpha_Beta,Udc,PWM_TIM_PULSE_TPWM);
	State=RUN;
}




void set_speed(float speed_rpm)
{
	Speed_Ref=speed_rpm;
}

void set_posi(float posi)
{
	Posi_Ref = posi;
	set_speed(Posi_Pid_Out);
}

float get_DC_BUS(void)
{
	float temp;
	HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,50);
	temp = HAL_ADC_GetValue(&hadc1);
	temp = temp*SAMPLE_VOL_CON_FACTOR;
	
	if(temp<Udc*0.95 || temp>Udc*1.05)
	{
		shut_pwm();
		State=FAULT;
	}
	return temp;

}
void shut_pwm(void)
{
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
}

/**********************************************************************************************************
get Theta
**********************************************************************************************************/ 
real32_T ENC_Get_Electrical_Angle(void)
{
	real32_T temp;
	temp=((real32_T)SPI_AS5048A_ReadData()/ 16384.0f * cpr) *POLE_PAIR_NUM ;
	while(temp>cpr)
		temp-=cpr;
	return temp;
}
//get mech Angle
float get_angle(void)
{
	float temp;
	temp=(real32_T)SPI_AS5048A_ReadData()/ 16384.0f * cpr ;
	return temp;
}

float get_angle_X(void)
{
	float temp;
	temp=(real32_T)SPI_AS5048A_ReadData()/ 16384.0f * cpr + circle_num*cpr;
	return temp;
}



u8 first_m=0;

float GetMotorPreSpeed(void)
{
	angle_data=get_angle();
	d_angle=angle_data-angle_data_prev;
	if(ABS(d_angle)>0.8*cpr)
	{
		if(d_angle>0)
			circle_num--;
		else
			circle_num++;
	}
	
  current_angle=angle_data;
	velocity=(current_angle-angle_prev);
	
	if(velocity<-PI)velocity=velocity+cpr;
	else if(velocity>PI)velocity=velocity-cpr;
	
	
	angle_prev=current_angle;
	angle_data_prev=angle_data;
	if(first_m==0)
	{
		first_m=1;
		return 0.0f;
	}
	else
	{
		velocity=velocity*30.0f/PI*SPEED_SAMPLING_FREQ;
	return velocity;
	}
}
float GetMotor_Speed_RPM(void)
{
	float rpm;
	rpm=velocity*30.0f/PI*SPEED_SAMPLING_FREQ;
	
	return rpm;
}


/**********************************************************************************************************
get Iab
**********************************************************************************************************/ 
CURRENT_ABC_DEF get_Iab(void)
{
	CURRENT_ABC_DEF Iab_temp;
	Iab_temp.Ia =(int16_t)((int16_t)hPhaseA_OffSet-(int16_t)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1)) * SAMPLE_CURR_CON_FACTOR;
	Iab_temp.Ib =(int16_t)((int16_t)hPhaseB_OffSet-(int16_t)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2)) * SAMPLE_CURR_CON_FACTOR;
	Iab_temp.Ic = -Iab_temp.Ia - Iab_temp.Ib;
	return (Iab_temp);
}


void LED_DIR(u32 ms)
{
	static u32 cnt=0;
	cnt++;
	if(cnt%ms==0)
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	
	if(cnt>=U32_MAX)
		cnt=0;
	
}
void PID_Init(void)
{
	// Id- loop
	Current_D_PID.P_Gain = D_PI_P;
  Current_D_PID.I_Gain = D_PI_I;
  Current_D_PID.B_Gain = D_PI_KB;
  Current_D_PID.Max_Output = D_PI_UP_LIMIT;
  Current_D_PID.Min_Output = D_PI_LOW_LIMIT;
  Current_D_PID.I_Sum = 0.0f;
  //Iq- loop
  Current_Q_PID.P_Gain = Q_PI_P;
  Current_Q_PID.I_Gain = Q_PI_I;
  Current_Q_PID.B_Gain = Q_PI_KB;
  Current_Q_PID.Max_Output = Q_PI_UP_LIMIT;
  Current_Q_PID.Min_Output = Q_PI_LOW_LIMIT;
  Current_Q_PID.I_Sum = 0.0f;
	//Speed loop
	Speed_Pid.P_Gain = SPEED_PI_P;
  Speed_Pid.I_Gain = SPEED_PI_I;
  Speed_Pid.B_Gain = SPEED_PI_KB;
  Speed_Pid.Max_Output = SPEED_PI_UP_LIMIT;
  Speed_Pid.Min_Output = SPEED_PI_LOW_LIMIT;
  Speed_Pid.I_Sum = 0.0f;
	//Posi loop
	Posi_Pid.P_Gain = POSI_PI_P;
	Posi_Pid.D_Gain = POSI_PI_D;
	Posi_Pid.I_Gain = POSI_PI_I;
	Posi_Pid.Max_Output = POSI_PI_UP_LIMIT;
	Posi_Pid.Min_Output = POSI_PI_LOW_LIMIT;
	Posi_Pid.I_Sum = 0.0f;
	
	
	
}

/***************************************
current PID 
B is Integral windup gain ,
usually, it is about I gain 
***************************************/

void Current_PID_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,CURRENT_PID_DEF* current_pid_temp)
{
  real32_T error;
  real32_T temp;
  error = ref_temp - fdb_temp;
  temp = current_pid_temp->P_Gain * error + current_pid_temp->I_Sum;
  if (temp > current_pid_temp->Max_Output) 
  {
    *out_temp = current_pid_temp->Max_Output;
  } 
  else if (temp < current_pid_temp->Min_Output) 
  {
    *out_temp = current_pid_temp->Min_Output;
  } 
  else 
  {
    *out_temp = temp;
  }
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * error) *FOC_PERIOD;
}

void Speed_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,SPEED_PID_DEF* current_pid_temp)
{

  real32_T error;
  real32_T temp;

  error =  ref_temp - fdb_temp;             //2*pi的作用是 单位转换   Hz转换为rad/s

  temp = (error + current_pid_temp->I_Sum) * current_pid_temp->P_Gain;

 
  if (temp > current_pid_temp->Max_Output) {
    *out_temp = current_pid_temp->Max_Output;
  } else if (temp < current_pid_temp->Min_Output) {
    *out_temp = current_pid_temp->Min_Output;
  } else {
    *out_temp = temp;
  }
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain* error) * SPEED_PID_PERIOD;
}


void Posi_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,POSI_PID_DEF* current_pid_temp)
{

  real32_T error,error_1;
  real32_T temp;

  error =  ref_temp - fdb_temp;             

  temp =  current_pid_temp->P_Gain*error + current_pid_temp->D_Gain*(error-error_1) + current_pid_temp->I_Sum;

 
  if (temp > current_pid_temp->Max_Output) {
    *out_temp = current_pid_temp->Max_Output;
  } else if (temp < current_pid_temp->Min_Output) {
    *out_temp = current_pid_temp->Min_Output;
  } else {
    *out_temp = temp;
  }
	error_1 = error;
  current_pid_temp->I_Sum +=  current_pid_temp->I_Gain*error ;
}





/**********************************************************************************************************
Clarke变换，输入Ia,Ib，得到Ialpha和Ibeta
**********************************************************************************************************/ 
/***************************************
功能：Clark变换
形参：三相电流以及alpha_beta电流
说明：由三相互差120度变换到两相互差90度
***************************************/
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp)
{
  Current_alpha_beta_temp->Ialpha = (Current_abc_temp.Ia - (Current_abc_temp.Ib + Current_abc_temp.Ic) * 0.5F) * 2.0F / 3.0F;
  Current_alpha_beta_temp->Ibeta = (Current_abc_temp.Ib - Current_abc_temp.Ic) * 0.866025388F * 2.0F / 3.0F;
}
/***************************************
功能：COS_SIN值计算
形参：角度以及COS_SIN结构体
说明：COS_SIN值计算
***************************************/
void Angle_To_Cos_Sin(real32_T angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp)
{
  cos_sin_temp->Cos = arm_cos_f32(angle_temp);
  cos_sin_temp->Sin = arm_sin_f32(angle_temp);
}
/***************************************
功能：PARK变换
形参：alpha_beta电流、COS_SIN值、DQ轴电流
说明：交流变直流
***************************************/
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp)
{
  current_dq_temp->Id = current_alpha_beta_temp.Ialpha * cos_sin_temp.Cos + current_alpha_beta_temp.Ibeta * cos_sin_temp.Sin;
  current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * cos_sin_temp.Sin + current_alpha_beta_temp.Ibeta * cos_sin_temp.Cos;
}
/***************************************
功能：反PARK变换
形参：DQ轴电压、COS_SIN值、alpha_beta电压
说明：直流变交流
***************************************/
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp)
{
  v_alpha_beta_temp->Valpha = cos_sin_temp.Cos * v_dq_temp.Vd - cos_sin_temp.Sin * v_dq_temp.Vq;
  v_alpha_beta_temp->Vbeta  = cos_sin_temp.Sin * v_dq_temp.Vd + cos_sin_temp.Cos * v_dq_temp.Vq;
}



/***************************************
功能：SVPWM计算
形参：alpha_beta电压以及母线电压、定时器周期
说明：根据alpha_beta电压计算三相占空比
***************************************/
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,real32_T Udc_temp,real32_T Tpwm_temp)
{
  
  real32_T Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
	u16 Tcmp4 ;
	real32_T  hDeltaDuty;
  sector = 0;
  Tcmp1 = 0.0F;
  Tcmp2 = 0.0F;
  Tcmp3 = 0.0F;
	Tcmp4 = 0;//计算采样时机
  if (v_alpha_beta_temp.Vbeta > 0.0F) {
    sector = 1;
  }
  
  if ((1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 4;
  }
  
  switch (sector) {
  case 1:
    Tx = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 2:
    Tx = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    break;
    
  case 3:
    Tx = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    break;
    
  case 4:
    Tx = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    Ty = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 5:
    Tx = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    Ty = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
    
  default:
    Tx = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
  }
  
  f_temp = Tx + Ty;
  if (f_temp > Tpwm_temp) {
    Tx /= f_temp;
    Ty /= (Tx + Ty);
  }
  
  Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
  Tb = Tx / 2.0F + Ta;
  Tc = Ty / 2.0F + Tb;
  switch (sector) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
	
		if((u16)(PWM_PERIOD-(u16)Tcmp1)>TW_AFTER)
		{
			Tcmp4=PWM_PERIOD-1;
		}
		else
		{
			hDeltaDuty = (u16)(Tcmp1 - Tcmp2);
			if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp1)*2) 
			{
					Tcmp4 = (u16)Tcmp1 - TW_BEFORE; // Ts before Phase A 
			}
			else
			{
				Tcmp4 = (u16)Tcmp1 + TW_BEFORE;
				if (Tcmp4 >= PWM_PERIOD)
				{        
					PWM4Direction=PWM1_MODE;
					Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
				}
			}
		}
		
    break;
    
  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
	
		if((u16)(PWM_PERIOD-(u16)Tcmp2)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp2 - Tcmp1);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp2)*2) 
				{
						Tcmp4 = (u16)Tcmp2 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp2 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}
		
    break;
    
  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
	
		if((u16)(PWM_PERIOD-(u16)Tcmp2)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp2 - Tcmp3);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp2)*2) 
				{
						Tcmp4 = (u16)Tcmp2 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp2 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}
		
    break;
    
  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
		if((u16)(PWM_PERIOD-(u16)Tcmp3)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp3 - Tcmp2);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp3)*2) 
				{
						Tcmp4 = (u16)Tcmp3 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp3 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}	
    break;
    
  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
		if((u16)(PWM_PERIOD-(u16)Tcmp3)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp3 - Tcmp1);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp3)*2) 
				{
						Tcmp4 = (u16)Tcmp3 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp3 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}	
    break;
    
  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
			if((u16)(PWM_PERIOD-(u16)Tcmp1)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp1 - Tcmp3);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp1)*2) 
				{
						Tcmp4 = (u16)Tcmp1 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp1 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}
    break;
  }
	
	if (PWM4Direction == PWM2_MODE)
  {
    //Set Polarity of CC4 High
    TIM1->CCER &= 0xDFFF;    
  }
  else
  {
    //Set Polarity of CC4 Low
    TIM1->CCER |= 0x2000;
  }
	
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(u16)Tcmp1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(u16)Tcmp2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(u16)Tcmp3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,(u16)Tcmp4);

}

