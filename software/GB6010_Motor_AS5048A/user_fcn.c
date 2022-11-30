#include "user_fcn.h"
#include "usart.h"
#include "arm_math.h"
#include "lcd_init.h"
#include "lcd.h"
#include "ugui.h"
#include "tim.h" 

 u16 hdisplay_Timebase = DISPLAY_TIME;
extern u8 State;
extern real32_T Speed_Fdk;
extern real32_T Posi_Ref;
extern real32_T Posi_Fdk;
extern real32_T Speed_Ref;
extern CURRENT_DQ_DEF Current_Idq; 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数
{	
	if(htim->Instance == TIM5)
	{
		get_DC_BUS();
		control_cmd();
		printf("{plotter:%.4f}\n",Posi_Fdk); 
	}
}



extern real32_T Posi_Ref ;
void control_cmd(void)
{
	u8 temp[8];
	HAL_UART_Receive(&huart1,temp,8,1);
	if(temp[0]==0xfe)
	{
		if(temp[1]==0x00)//speed
			Speed_Ref=temp[2];
		if(temp[1]==0x01)
			Posi_Ref=temp[2];
	}
}

float muti_error;
void muti_switch(void)
{
	


	
}

/*
when Kp=0.01--0.02 it is a mech spring
when Kp=0.00002 it is like weight loss
*/
float Kp=0.02f;
float L=12;
float mass=13;
float init_angle=0.0f;
float kg_f_control(float target)
{
	float target_torque;
	target_torque = Kp*target*L*mass*arm_cos_f32(-get_angle()-init_angle);
	return target_torque;
}



void gui_init(void)
{
	LCD_Init();
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
}
void dis_info_core(void)
{
	if(State==RUN)
		LCD_ShowString(0,0,"RUN",RED,WHITE,24,0);
	else
		LCD_ShowString(0,0,"FAULA",RED,WHITE,24,0);
	
	LCD_ShowString(100,0,"DC-BUS",WHITE,BLACK,16,0);
	LCD_ShowFloatNum1(90,20,get_DC_BUS(),4,RED,BLACK,16);
	LCD_ShowString(135,14,"V",GREEN,BLACK,24,0);
	
	LCD_ShowString(0,68,"Position:",WHITE,BLACK,12,0);
	LCD_ShowFloatNum1(80,68,Posi_Fdk,5,WHITE,BLACK,12);
	LCD_ShowString(0,54,"Speed:",WHITE,BLACK,12,0);
	LCD_ShowFloatNum1(80,54,Speed_Fdk,5,WHITE,BLACK,12);
	LCD_ShowString(0,42,"Current:",WHITE,BLACK,12,0);
	LCD_ShowFloatNum1(80,42,Current_Idq.Iq,5,WHITE,BLACK,12);
	
}




