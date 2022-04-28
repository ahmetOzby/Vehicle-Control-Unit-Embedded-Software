#include <AESK_Electronic_Differential.h>


void AESK_Electronic_Dif_Init(electronic_dif_param_s* parameters, float kp, float ki, float kd,  float regen_coef)
{
	parameters->kp_f32 = kp;
	parameters->ki_f32 = ki;
	parameters->kd_f32 = kd;


	//PID Limit
	parameters->max_pos_out = SPEED_PID_POS_LIMIT;
	parameters->min_neg_out = SPEED_PID_NEG_LIMIT;

	//MCU Speed & Torque Limit Init
	lyra_can_data.tx.mcu.speed_limit_u16 = SPEED_LIMIT;
	lyra_can_data.tx.mcu.torque_limit_u8 = TORQUE_LIMIT;


	//Regen Coefs
	parameters->regen_coef_f32 = regen_coef;
	electronic_dif_param.control_input_coef_f32 = CONTROL_INPUT;
}

uint8_t AESK_Target_Speed_Calc(float turn_radius, float half_axle_len, int16_t str_ang, uint8_t act_vel)
{
	float angular_speed;
	angular_speed = act_vel / turn_radius;
    return (uint8_t)((turn_radius - half_axle_len) * angular_speed);
}


int16_t AESK_Speed_PI(float neg_out_sat, float pos_out_sat, float target_rpm, float act_rpm, float dt)
{
	//static int16_t temp_out_val = 0;
	static int8_t last_err = 0;
	float err = target_rpm - act_rpm;
	float der = (err - last_err) / dt;
	int16_t out;
	float p_response = electronic_dif_param.kp_f32 * err;
	float d_response = electronic_dif_param.kd_f32 * der;
	float pd_response = p_response + d_response;

	if (pd_response < pos_out_sat && pd_response > neg_out_sat)
	{
		electronic_dif_param.integral_f32 += (err * dt);
		electronic_dif_param.integral_sum_f32 = electronic_dif_param.ki_f32 * electronic_dif_param.integral_f32;
		electronic_dif_param.integral_sum_f32 = (electronic_dif_param.integral_sum_f32 < MIN_INTEGRAL_VAL) ? MIN_INTEGRAL_VAL : electronic_dif_param.integral_sum_f32;
		electronic_dif_param.integral_sum_f32 = (electronic_dif_param.integral_sum_f32 > MAX_INTEGRAL_VAL) ? MAX_INTEGRAL_VAL : electronic_dif_param.integral_sum_f32;
	}


	if((target_rpm == 0) && abs(vcu_data.actual_velocity_i8 <= 1))
	{
		electronic_dif_param.integral_sum_f32 = 0.0;
		electronic_dif_param.integral_f32 = 0.0;
	}

	else if(lyra_can_data.tx.mcu.states_union.states_t.ignition == OFF && (vcu_data.actual_velocity_i8 <= 2) && (vcu_data.actual_velocity_i8 >= -2))
	{
		electronic_dif_param.integral_sum_f32 = 0.0;
		electronic_dif_param.integral_f32 = 0.0;
	}

	out = (int16_t)(pd_response + electronic_dif_param.integral_sum_f32 + 0.5f);

	if ((out > pos_out_sat) || (out < neg_out_sat))
	{
		out = (out > pos_out_sat) ? pos_out_sat : neg_out_sat;
		electronic_dif_param.integral_f32 = electronic_dif_param.integral_last_f32;
	}



	//Out Saturation
//	out = (out > pos_out_sat) ? pos_out_sat : out;
//	out = (out < neg_out_sat) ? neg_out_sat : out;

	last_err = err;
	electronic_dif_param.integral_last_f32 = electronic_dif_param.integral_f32;
	//Anti Wind Up
	/*out = (temp_out_val + ANTI_WIND_UP_CONST) < out ? (temp_out_val + ANTI_WIND_UP_CONST) : out;
	out = (temp_out_val - ANTI_WIND_UP_CONST) > out ? (temp_out_val - ANTI_WIND_UP_CONST) : out;*/


	//Regen
	if(vcu_data.fake_brake == ON)
	{
		//PID OUT Saturation
		electronic_dif_param.max_pos_out = AESK_Ramp_Function(electronic_dif_param.regen_coef_f32, electronic_dif_param.max_pos_out, 0);
		electronic_dif_param.min_neg_out = -electronic_dif_param.max_pos_out;

		if(abs(vcu_data.actual_velocity_i8 < 2))
		{
			electronic_dif_param.integral_f32 = 0;
			electronic_dif_param.integral_sum_f32 = 0;
			lyra_can_data.tx.mcu.torque_set_i16 = 0;
		}
	}
	return out;
}

float AESK_Ramp_Function(float increment, uint16_t velocity, uint16_t target_velocity)
{
	return target_velocity * increment + velocity * (1.0f - increment);
}

/*void Wheel_Speed_Calc(double r_present_vel, double l_present_vel, float* r_calculated_vel, float* l_calculated_vel, double rack_slide_angle)
{
	double got_omer = -400 * pow(rack_slide_angle, 4) + 308000 * pow(rack_slide_angle, 3) - 62897240 * pow(rack_slide_angle, 2) + 1388787400 * rack_slide_angle + 65938452899;
	double divinded = -7090477625 + rack_slide_angle * 407396150 + pow(rack_slide_angle, 2) * (-2887500) + 5000 * pow(rack_slide_angle, 3) + 8950 * pow(got_omer, 0.5f);
	double divisor = (100 * pow(rack_slide_angle, 2) - 38500 * rack_slide_angle + 3833789) * 3780;
	inner_wheel_degree =  asin(divinded / divisor) / PI * 180 - 17.6;

	if(inner_wheel_degree == 0)
	{
		inner_wheel_degree = 0.001;
	}

	double turn_rad = abs(TRACK_MM / tan(abs(inner_wheel_degree)) + AXLE_MM / 2.0f);

	//double turn_rad = 3200.0f;
	double r_angular_vel = (r_present_vel / 3600.0f / (turn_rad / 1000000.0f));
	double l_angular_vel = (l_present_vel / 3600.0f / (turn_rad / 1000000.0f));

	if(rack_slide_angle <= 25)
	{
		*r_calculated_vel = (3.6f * (turn_rad / 1000.0f + HALF_AXLE / 1000.0f) * r_angular_vel);
		*l_calculated_vel = (3.6f * (turn_rad / 1000.0f - HALF_AXLE / 1000.0f) * l_angular_vel);
	}

	else
	{
		*r_calculated_vel = (3.6f * (turn_rad / 1000.0f - HALF_AXLE / 1000.0f) * r_angular_vel);
		*l_calculated_vel = (3.6f * (turn_rad / 1000.0f + HALF_AXLE / 1000.0f) * l_angular_vel);
	}
}
*/


/*uint16_t ADC_Filter(uint16_t unfiltered_adc_val)
{
	static uint64_t sum = 0;
	static uint8_t ctr = 0;
	electronic_dif_param.temp_filtered_value = unfiltered_adc_val;
	sum += unfiltered_adc_val;
	ctr++;

	if(ctr == 100)
	{
		uint16_t return_val = sum / 100;
		ctr = 0;
		sum = 0;
		electronic_dif_param.temp_filtered_value = return_val;
		return return_val;
	}
	return electronic_dif_param.temp_filtered_value;
}*/


float Steering_Angle_Proccess(uint16_t unfiltered_str_angle)
{
	static uint16_t temp_adc_val = 0;
	float filtered_adc_val = AESK_Ramp_Function(0.001f, temp_adc_val, unfiltered_str_angle);
	temp_adc_val = unfiltered_str_angle;
	return filtered_adc_val / 4096 * 50;
}

void Torque_Distribution(uint16_t adc_val, int16_t torque)
{
	float scaled_adc_val = adc_val - MIDDLE_POINT_ADC_VAL - MIN_STEERING_ADC_VAL;

	//Sol
	if(adc_val <= MIDDLE_POINT_ADC_VAL -150)
	{
		//lyra_can_data.tx.mcu.torque_set_i16 = torque / 2 + 2.0f * abs(scaled_adc_val) / MIDDLE_POINT_ADC_VAL;
		//lyra_can_data.tx.mcu.torque_set_2_i16 = torque - 4.5f * abs(scaled_adc_val) / 2047.0f;
		lyra_can_data.tx.mcu.torque_set_i16 = torque - 2.50f * abs(abs(scaled_adc_val)) / (MAX_STEERING_ADC_VAL - MIN_STEERING_ADC_VAL);
		if(lyra_can_data.tx.mcu.torque_set_i16 < 0)
		{
			lyra_can_data.tx.mcu.torque_set_i16 = 0;
		}
	}

	//Sag
	else if(adc_val >= MIDDLE_POINT_ADC_VAL + 150)
	{
		//lyra_can_data.tx.mcu.torque_set_2_i16 = torque / 2 - 2.0f * scaled_adc_val / MIDDLE_POINT_ADC_VAL;
		lyra_can_data.tx.mcu.torque_set_2_i16 = -torque + 2.50f * abs(MAX_STEERING_ADC_VAL - abs(scaled_adc_val)) / (MAX_STEERING_ADC_VAL - MIN_STEERING_ADC_VAL);
		if(lyra_can_data.tx.mcu.torque_set_2_i16 > 0)
		{
			lyra_can_data.tx.mcu.torque_set_2_i16 = 0;
		}
	}

	else
	{
		lyra_can_data.tx.mcu.torque_set_i16 = torque;
		lyra_can_data.tx.mcu.torque_set_2_i16 = -torque;
	}
}
