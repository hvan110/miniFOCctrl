#include "ref_signal_in.h"
#include "math.h"
#include "arm_math.h"

float hPosi_ref=2000;
float counter;	
float thr_test;
float wr_test;
float ref_time=5000.0F;//ms



void ref_Core(void)
{
	if(counter<=ref_time)
			{
				counter=counter/1000.0f;
				thr_test = hPosi_ref/2.0 *  arm_sin_f32(pi/(ref_time/1000.0f) * counter-pi/2.0f) + hPosi_ref/2.0;
				wr_test  = hPosi_ref/2.0*pi/(ref_time/1000.0f)  *arm_cos_f32(pi/(ref_time/1000.0f) * counter-pi/2.0);

				counter = counter * 1000;
			}
			else
			{
				thr_test=hPosi_ref;
				wr_test = 0;
			}			
			counter+=1.0f;			
			if(counter>=600000)
				counter=0;
}


///*ref signal deal*/
//float ref_in=400.0f;
//float ref_out;
//float low_pass_fator=1000.0f;//low pass fator
//float speed_ref_temp1;
//float speed_ref_temp2,speed_ref_temp3,speed_ref_temp3_1;
//void ref_deal_core(void)
//{
//	speed_ref_temp1 = ref_in-speed_ref_temp3_1;
//	speed_ref_temp2 = speed_ref_temp1 / low_pass_fator;
//	speed_ref_temp3 = speed_ref_temp2 + speed_ref_temp3_1;
//	ref_out = speed_ref_temp3;
//	speed_ref_temp3_1 = speed_ref_temp3;
//}
