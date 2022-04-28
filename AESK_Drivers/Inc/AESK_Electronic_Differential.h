/*
 * AESK_Electronic_Differensial.h
 *
 *  Created on: 10 Mar 2021
 *      Author: Ahmet
 */

#ifndef INC_AESK_ELECTRONIC_DIFFERENTIAL_H_
#define INC_AESK_ELECTRONIC_DIFFERENTIAL_H_

#include "main.h"
#include "Can_Lyra_Header.h"
#include "AESK_VCU_General.h"
#include "math.h"

#define PI							3.141592f
#define RPM_TO_KMH_COEF				0.105183f
#define INCREMENT					0.1f
#define MIN_INTEGRAL_VAL			-20
#define MAX_INTEGRAL_VAL			45
#define HALF_AXEL_LENGTH			30.0f
#define MAX_ADC_VAL					2500
#define MIN_ADC_VAL					500
//kp -> 11.0f ki-> 11.0f kd-> 0.0043f
#define SP_KP						0.05f						//0.5f bir önceki değer
#define SP_KI						0.05f
#define SP_KD						0.00f
#define E_DIF_KD					0.0f

#define DT							0.01f
#define HALF_AXLE					129.87f	//Örnek
#define TORQUE_SET_PROTECTION		20
#define SET_VEL_LOW_PASS_COEF		0.4f
#define SPEED_PID_POS_LIMIT			40
#define SPEED_PID_NEG_LIMIT			-20	//-110
#define ANTI_WIND_UP_CONST			5



typedef struct
{
	float kp_f32;
	float ki_f32;
	float kd_f32;
	float half_axle_len;
	float l_motor_torque_f32;
	float r_motor_torque_f32;
	float target_set_vel;
	float integral_f32;
	float integral_last_f32;
	float integral_sum_f32;
	float max_pos_out;
	float min_neg_out;
	float regen_coef_f32;
	float control_input_coef_f32;
	uint8_t r_wheel_target_vel_u8;
	uint8_t l_wheel_target_vel_u8;
	uint8_t r_wheel_vel_u8;
	uint8_t l_wheel_vel_u8;
}electronic_dif_param_s;
electronic_dif_param_s electronic_dif_param;



void AESK_Electronic_Dif_Init(electronic_dif_param_s* parameters, float kp, float ki, float kd,  float regen_coef);	//Elektronik Diferansiyel katsayilari setlenir.
uint8_t AESK_Target_Speed_Calc(float turn_radius, float half_axle_len, int16_t str_ang, uint8_t actual_velocity);	//Hedef teker hizlarini hesaplamak icin kullanilir.
float AESK_Ramp_Function(float increment, uint16_t velocity, uint16_t target_vel);	//Speed PI fonksiyonuna girisin yavas ve yumusak bir sekilde verilmesini saglar.
int16_t AESK_Speed_PI(float neg_out_sat, float pos_out_sat, float target_rpm, float act_rpm, float dt);	//Kullanicidan alinan input hiz PI bloguna sokularak tork cikisi verir.
void Torque_Distribution(uint16_t adc_val, int16_t torque); //Tork dagitimi



#endif /* INC_AESK_ELECTRONIC_DIFFERENTIAL_H_ */
