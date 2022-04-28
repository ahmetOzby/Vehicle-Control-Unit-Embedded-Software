/*
 * AESK_General.c
 *
 *  Created on: Mar 8, 2021
 *      Author: Ahmet
 */

#include <AESK_VCU_General.h>
#include "main.h"
#include "AESK_GL.h"
#include "AESK_Electronic_Differential.h"

extern AESK_GL aesk_gl;


extern CAN_HandleTypeDef hcan1;
uint8_t reset_button_flag = 0;
uint8_t vel_resetted_flag = 0;
uint8_t torque_resetted_flag = 0;

uint8_t decrease_btn_pressed = 0;
uint8_t increase_btn_pressed = 0;
uint8_t res_btn_pressed = 0;
uint8_t fake_brake_pressed = 0;
uint8_t trq_btn_pressed = 0;
uint8_t ign_btn_pressed = 0;
uint8_t brake_switch_pressed = 0;

uint64_t l_ctr = 0;
uint64_t r_ctr = 0;

uint8_t state_changed = 0;
uint8_t state_changed_2 = 0;
void AESK_Read_All_Signals(void)
{
	Sys_On_Off_Handler();
	Brake_Switch_Handler();
	Fake_Brk_Handler();
	Ign_Handler();
	Reverse_Handler();
	Inc_Vel_Handler();
	Dec_Vel_Handler();
	TV_On_Off_Handler();
	Torque_Speed_Handler();
}

inline void Torque_Speed_Handler(void)
{
#if STEERING_DEBUG != ON
	if((Trq_Spd_Btn_GPIO_Port->IDR & Trq_Spd_Btn_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
	if(trq_btn_pressed == 1)
#endif
	{
		lyra_can_data.tx.mcu.states_union.states_t.mode = Speed_Control;

	}

	else if(state_changed != ON)
	{
		lyra_can_data.tx.mcu.states_union.states_t.mode = Torque_Control;
	}
}

inline void Sys_On_Off_Handler(void)
{
	if((System_On_Off_GPIO_Port->IDR & System_On_Off_Pin) != (uint32_t)GPIO_PIN_RESET)
	{
		static uint8_t ctr = 0;
		ctr++;
		if(ctr == 5)
		{
			lyra_can_data.tx.mcu.states_union.states_t.bms_wake = SLEEP;
			ctr = 0;
		}
		lyra_can_data.tx.mcu.states_union.states_t.mcu_wake = FALSE;
		vcu_data.wake_up_union.wake_up.system_on_off = PULL_UP_OPEN;
		lyra_can_data.tx.mcu.states_union.states_t.ignition = OFF;
	}

	else
	{
		if((lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8 & 0x3F) == 0)
		{
			lyra_can_data.tx.mcu.states_union.states_t.bms_wake = WAKE;
			if(lyra_can_data.rx.bms_data.bms_dc_bus_state.Bms_State_Data_Dc_Bus_State.Dc_Bus_Ready == 1)
			{
				lyra_can_data.tx.mcu.states_union.states_t.mcu_wake = WAKE;
			}
		}
		vcu_data.wake_up_union.wake_up.system_on_off = PULL_UP_CLOSE;
	}
}


inline void Ign_Handler(void)
{
#if STEERING_DEBUG != ON
	if((Ignition_Button_2_GPIO_Port->IDR & Ignition_Button_2_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
	if(ign_btn_pressed == 1)
#endif
	{
		if(lyra_can_data.rx.bms_data.bms_dc_bus_state.Bms_State_Data_Dc_Bus_State.Dc_Bus_Ready == TRUE && lyra_can_data.tx.mcu.states_union.states_t.bms_wake == WAKE)
		{
			if(lyra_can_data.rx.mcu.state.condition_t.input_scaling_calib == OFF)
			{
				lyra_can_data.tx.mcu.states_union.states_t.ignition = ON;
			}
		}

		else
		{
			lyra_can_data.tx.mcu.states_union.states_t.ignition = OFF;
		}
	}

	else
	{
		lyra_can_data.tx.mcu.states_union.states_t.ignition = OFF;
	}
}


inline void Reverse_Handler(void)
{
	if((Reverse_Switch_GPIO_Port->IDR & Reverse_Switch_Pin) != (uint32_t)GPIO_PIN_RESET)
	{
		lyra_can_data.tx.mcu.states_union.states_t.reverse = 1;
		vcu_data.wake_up_union.wake_up.direction = 1;
	}

	else
	{
		lyra_can_data.tx.mcu.states_union.states_t.reverse = 0;
		vcu_data.wake_up_union.wake_up.direction = 0;
	}
}

inline void Inc_Vel_Handler(void)
{
	static uint8_t increase_torque_ctr = 0;
#if STEERING_DEBUG != ON
	if((Increase_Vel_Btn_GPIO_Port->IDR & Increase_Vel_Btn_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
	if(increase_btn_pressed == 1)
#endif
	{
		vcu_data.ignition_closed = 0;

		if(lyra_can_data.tx.mcu.states_union.states_t.mode == Speed_Control)
		{
			if(vel_resetted_flag == 1)
			{
				vel_resetted_flag = 0;
#if SINGLE_MOTOR == 1
				vcu_data.set_velocity_u8 = abs(vcu_data.actual_velocity_i8);
#else
				vcu_data.set_velocity_u8 = abs(vcu_data.m2_actual_velocity_i8 + vcu_data.m1_actual_velocity_i8) / 2;
#endif
			}

			if(vcu_data.set_velocity_u8 < MAX_VELOCITY)
			{
				vcu_data.set_velocity_u8++;
			}
		}

		else
		{
			if(torque_resetted_flag == 1)
			{
				torque_resetted_flag = 0;
				vcu_data.set_torque_u8 = vcu_data.stored_torque_u8;
			}

			if(vcu_data.set_torque_u8 < MAX_TORQUE)
			{
				increase_torque_ctr++;
				vcu_data.set_torque_u8++;
				if(increase_torque_ctr == 10)
				{
					if(vcu_data.set_torque_u8 < MAX_TORQUE - 2)
					{
						vcu_data.set_torque_u8 += 2;
					}

					else
					{
						vcu_data.set_torque_u8 = MAX_TORQUE;
					}
					increase_torque_ctr = 9;
				}
			}
		}
		increase_btn_pressed = 0;
	}

	else
	{
		increase_torque_ctr = 0;
	}
}

inline void Dec_Vel_Handler(void)
{
	static uint8_t decrease_torque_ctr = 0;
#if STEERING_DEBUG != ON
	if((Decrease_Vel_Btn_GPIO_Port->IDR & Decrease_Vel_Btn_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
	if(decrease_btn_pressed == 1)
#endif
	{
		vcu_data.ignition_closed = 0;

		if(lyra_can_data.tx.mcu.states_union.states_t.mode == Speed_Control)
		{
			if(vel_resetted_flag == 1)
			{
				vel_resetted_flag = 0;
#if SINGLE_MOTOR == 1
				vcu_data.set_velocity_u8 = abs(vcu_data.actual_velocity_i8);
#else
				vcu_data.set_velocity_u8 = abs(vcu_data.m2_actual_velocity_i8 + vcu_data.m1_actual_velocity_i8) / 2;
#endif
			}

			if(vcu_data.set_velocity_u8 > MIN_VELOCITY)
			{
				vcu_data.set_velocity_u8--;
			}
		}

		else
		{
			if(torque_resetted_flag == 1)
			{
				torque_resetted_flag = 0;
				vcu_data.set_torque_u8 = vcu_data.stored_torque_u8;
			}

			if(vcu_data.set_torque_u8 > MIN_TORQUE)
			{
				decrease_torque_ctr++;
				vcu_data.set_torque_u8--;
				if(decrease_torque_ctr == 10)
				{
					if(vcu_data.set_torque_u8 > MIN_TORQUE + 2)
					{
						vcu_data.set_torque_u8 -= 2;
					}

					else
					{
						vcu_data.set_torque_u8 = MIN_TORQUE;
					}

					decrease_torque_ctr = 9;
				}
			}
		}
		decrease_btn_pressed = 0;
	}

	else
	{
		decrease_torque_ctr = 0;
	}
}

inline void TV_On_Off_Handler(void)
{
	static uint8_t state_changed = 0;
#if STEERING_DEBUG != ON
	if((Reset_Vel_Btn_GPIO_Port->IDR & Reset_Vel_Btn_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
	if(res_btn_pressed == 1)
#endif
	{
		if(state_changed != 1)
		{
			if(vcu_data.tv_active_u8 == 1)
			{
				vcu_data.tv_active_u8 = 0;
			}
			else if(vcu_data.tv_active_u8 == 0)
			{
				vcu_data.tv_active_u8 = 1;
			}
		}
		state_changed = 1;
	}

	else
	{
		state_changed = 0;
	}
}

inline void Fake_Brk_Handler(void)
{
	if(lyra_can_data.tx.mcu.states_union.states_t.brake == 0)
	{
#if STEERING_DEBUG != ON
		if((Fake_Brake_GPIO_Port->IDR & Fake_Brake_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
	if(res_btn_pressed == 1)
#endif
		{
			lyra_can_data.tx.mcu.states_union.states_t.regen = 1;
			if(lyra_can_data.tx.mcu.states_union.states_t.mode == Speed_Control)
			{
				vcu_data.stored_vel_u8 = vcu_data.set_velocity_u8;
				vcu_data.set_velocity_u8 = 0;
				vcu_data.fake_brake = 1;
			}
		}

		else
		{
			//lyra_can_data.tx.mcu.states_union.states_t.regen = 0;
			vcu_data.fake_brake = 0;
		}
	}
}
inline void Brake_Switch_Handler(void)
{
#if STEERING_DEBUG != ON
		if((Brake_Switch_GPIO_Port->IDR & Brake_Switch_Pin) != (uint32_t)GPIO_PIN_RESET)
#else
		if(brake_switch_pressed == 1)
#endif
		{
			if(abs(lyra_can_data.rx.mcu.motor_rpm_f32) < 100.0f)
			{
				lyra_can_data.tx.mcu.torque_set_i16 = 0;
			}

			else
			{
				lyra_can_data.tx.mcu.torque_set_i16 = SPEED_PID_NEG_LIMIT;
			}
			lyra_can_data.tx.mcu.states_union.states_t.brake = 1;
		}

		else
		{
			lyra_can_data.tx.mcu.states_union.states_t.brake = 0;
		}
}

void vars_to_str(char *buffer, const char *format, ...)
{
	va_list args;
	va_start (args, format);
	vsprintf (buffer, format, args);
	va_end (args);
}


void HAL_SYSTICK_Callback()
{
	static uint32_t ctr = 0;
	ctr++;

	if(ctr % 1 == 0)
	{
		sys_timer._1ms_flag = 1;
	}
	if(ctr % 10 == 0)
	{
		sys_timer._10ms_flag = 1;
	}

	if(ctr % 20 == 0)
	{
		sys_timer._20ms_flag = 1;
	}

	if(ctr % 50 == 0)
	{
		sys_timer._50ms_flag = 1;
	}
	if(ctr % 100 == 0)
	{
		sys_timer._100ms_flag = 1;
	}
	if(ctr % 120 == 0)
	{
		sys_timer._120ms_flag = 1;
	}

	if(ctr % 200 == 0)
	{
		sys_timer._200ms_flag = 1;
	}

	if(ctr % 1000 == 0)
	{
		sys_timer._1000ms_flag = 1;
	}
}

void VCU_Can_Rx_Handler(uint32_t ExtId)
{
	switch(ExtId)
	{
		case MCU_PACK_ID_3 : //1. Motor Sürücü
		{
			uint16_t mcu_idx = 0;
			uint8_t torque;
			int16_t motor_rpm, set_torque;
			AESK_UINT8toINT16_LE(&set_torque, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			lyra_can_data.rx.mcu.set_torque_i16 = set_torque / 100.0f;
			AESK_UINT8toINT16_LE(&motor_rpm, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			lyra_can_data.rx.mcu.motor_rpm_f32 = motor_rpm / 10.0f;
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.mcu.motor_temp_u8, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			AESK_UINT8toUINT16_LE(&lyra_can_data.rx.mcu.state.condition_u16, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			AESK_UINT8toUINT8ENCODE(&torque, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			lyra_can_data.rx.mcu.torque_i8 = torque - 100;

			break;
		}

		case MCU_PACK_ID_6 : //2. Motor Sürücü
		{
			uint16_t mcu_idx = 0;
			uint8_t torque;
			int16_t motor_rpm, set_torque;
			AESK_UINT8toINT16_LE(&set_torque, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			lyra_can_data.rx.mcu_2.set_torque_i16 = set_torque / 100.0f;
			AESK_UINT8toINT16_LE(&motor_rpm, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			lyra_can_data.rx.mcu_2.motor_rpm_f32 = motor_rpm / 10.0f;
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.mcu_2.motor_temp_u8, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			AESK_UINT8toUINT16_LE(&lyra_can_data.rx.mcu_2.state.condition_u16, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			AESK_UINT8toUINT8ENCODE(&torque, aesk_gl.aesk_can_1.receivedData, &mcu_idx);
			lyra_can_data.rx.mcu_2.torque_i8 = torque - 100;

			break;
		}

		case BMS_MEASUREMENTS :
		{
			uint16_t bms_measurement_index = 0;

			float cur;

			AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Bat_Voltage_f32, aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_2, &bms_measurement_index);
			AESK_INT16toFLOAT_LE(&cur, aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_2, &bms_measurement_index);
			AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Bat_Cons_f32, aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_1, &bms_measurement_index);
			AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Soc_f32, aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_2, &bms_measurement_index);

			lyra_can_data.rx.bms_data.Bat_Current_f32 = cur + 0.5f;

			break;
		}

		case BMS_STATE_DATA :
		{
			uint16_t bms_state_data_index = 0;
		 	AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8, aesk_gl.aesk_can_1.receivedData, &bms_state_data_index);
		 	AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_dc_bus_state.bms_state_data_dc_bus_state_u8, aesk_gl.aesk_can_1.receivedData, &bms_state_data_index);
		 	AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32, aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_1, &bms_state_data_index);
		 	AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.Worst_Cell_Address_u8, aesk_gl.aesk_can_1.receivedData, &bms_state_data_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.Temperature_u8, aesk_gl.aesk_can_1.receivedData, &bms_state_data_index);

			uint8_t iso_pos_u8;
			uint8_t iso_neg_u8;

			AESK_UINT8toUINT8ENCODE(&iso_pos_u8, aesk_gl.aesk_can_1.receivedData, &bms_state_data_index);
			AESK_UINT8toUINT8ENCODE(&iso_neg_u8, aesk_gl.aesk_can_1.receivedData, &bms_state_data_index);

			lyra_can_data.rx.bms_data.iso_res_pos = iso_pos_u8 * 10;
			lyra_can_data.rx.bms_data.iso_res_neg = iso_neg_u8 * 10;
			vcu_data.bms_on_bus = 1;
			break;
		}

		case CHARGER_TX_ID :
		{
			vcu_data.wake_up_union.wake_up.charger_wake_up = 1;
			uint16_t charger_index = 0;
			AESK_INT16toFLOAT_BE(&(charger.actual_volt_f32), aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_1, &charger_index);
			AESK_INT16toFLOAT_BE(&(charger.actual_current_f32), aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_1, &charger_index);
			AESK_UINT8toUINT16_BE(&(charger.chargerFlags.ChargerFlags_u16), aesk_gl.aesk_can_1.receivedData, &charger_index);
			charger.charger_counter++;
			break;
		}

		case BMS_CELLS_1 :
		{
			uint16_t bms_cells_1_index = 0;
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_1_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_2_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_3_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_4_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_5_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_6_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_7_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_8_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_1_index);
			break;
		}

		case BMS_CELLS_2 :
		{
			uint16_t bms_cells_2_index = 0;
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_9_u8,  aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_10_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_11_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_12_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_13_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_14_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_15_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_16_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_2_index);
			break;
		}

		case BMS_CELLS_3 :
		{
			uint16_t bms_cells_3_index = 0;
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_17_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_18_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_19_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_20_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_21_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_22_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_23_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_24_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_3_index);
			break;
		}

		case BMS_CELLS_4 :
		{
			uint16_t bms_cells_4_index = 0;
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_25_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_26_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_27_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_28_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_1_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_2_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_3_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_4_u8, aesk_gl.aesk_can_1.receivedData, &bms_cells_4_index);
			break;
		}

		case BMS_SOC_1 :
		{
			uint16_t bms_soc_1_index = 0;
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_1_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_2_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_3_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_4_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_5_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_6_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_7_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_8_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_1_index);
			break;
		}

		case BMS_SOC_2 :
		{
			uint16_t bms_soc_2_index = 0;
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_9_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_10_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_11_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_12_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_13_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_14_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_15_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_16_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_2_index);
			break;
		}

		case BMS_SOC_3 :
		{
			uint16_t bms_soc_3_index = 0;
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_17_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_18_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_19_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_20_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_21_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_22_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_23_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_24_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_3_index);
			break;
		}

		case BMS_SOC_4 :
		{
			uint16_t bms_soc_4_index = 0;
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_25_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_26_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_27_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_28_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_5_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_6_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_7_u8, aesk_gl.aesk_can_1.receivedData, &bms_soc_4_index);
			break;
		}

		case BMS_SOH :
		{
			uint16_t bms_soh_index = 0;
			AESK_UINT8toUINT16_LE(&lyra_can_data.rx.bms_data.power_limited_u16, aesk_gl.aesk_can_1.receivedData, &bms_soh_index);
			AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.soh_u8, aesk_gl.aesk_can_1.receivedData, &bms_soh_index);
			break;
		}

		case KP_KI_SET:
		{
			uint16_t telemetry_index = 0;
			electronic_dif_param.integral_sum_f32 = 0;
			AESK_UINT8toFLOAT32_LE(&electronic_dif_param.kp_f32, aesk_gl.aesk_can_1.receivedData, &telemetry_index);
			AESK_UINT8toFLOAT32_LE(&electronic_dif_param.ki_f32, aesk_gl.aesk_can_1.receivedData, &telemetry_index);
			break;
		}

		case KD_SET:
		{
			uint16_t telemetry_index = 0;
			electronic_dif_param.integral_sum_f32 = 0;
			AESK_UINT8toFLOAT32_LE(&electronic_dif_param.kd_f32, aesk_gl.aesk_can_1.receivedData, &telemetry_index);
			//AESK_UINT16toFLOAT_LE(&electronic_dif_param.control_input_coef_f32, aesk_gl.aesk_can_1.receivedData, FLOAT_CONVERTER_2, &telemetry_index);
			break;
		}

		case TELEMETRY_QUERY:
		{
			tel_data.query_answer = 1;
			break;
		}

		default :
		{
			break;
		}
	}
}

void Nextion_Init()
{
	screen_data.page = MAIN_PAGE;
	//HAL_Delay(2200);
	AESK_Nextion_Change_Pic(IGN_PIC, PASIVE_PIC_ID);
	AESK_Nextion_Change_Pic(MCU_WAKE_PIC, PASIVE_PIC_ID);
	AESK_Nextion_Change_Pic(BMS_WAKE_PIC, PASIVE_PIC_ID);
}

void Send_Buf_2_Nextion()
{
	static uint8_t tv_active_flag = 0, reverse_flag = 0, bms_wake_flag = 0, mcu_wake_flag = 0, ignition_flag = 0, error_flag = 0, vehicle_mode = 0;
	if(screen_data.page == MAIN_PAGE)
	{
#if SINGLE_MOTOR == 1
		AESK_Nextion_Write_Num(VELOCITY, abs(vcu_data.actual_velocity_i8));
#else
		AESK_Nextion_Write_Num(VELOCITY, abs(vcu_data.m1_actual_velocity_i8 + vcu_data.m2_actual_velocity_i8) / 2);
#endif
		 AESK_Nextion_Write_Num(T1, lyra_can_data.tx.mcu.torque_set_i16);
		 AESK_Nextion_Write_Num(T2, -lyra_can_data.tx.mcu.torque_set_2_i16);
		 AESK_Nextion_Write_Num(SOC, lyra_can_data.rx.bms_data.Soc_f32);
		 AESK_Nextion_Write_Num(POWER, lyra_can_data.rx.bms_data.Bat_Current_f32 * lyra_can_data.rx.bms_data.Bat_Voltage_f32);
		 AESK_Nextion_Write_Num(CONSUMPTION, lyra_can_data.rx.bms_data.Bat_Cons_f32);
		 AESK_Nextion_Write_Num(ODOMETER, vcu_data.odometer_u32);
		 AESK_Nextion_Write_Num(WCV, lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		 AESK_Nextion_Write_Num(BATTERY_TEMP, lyra_can_data.rx.bms_data.Temperature_u8);
		 AESK_Nextion_Write_Num(BATTERY_VOLT, lyra_can_data.rx.bms_data.Bat_Voltage_f32);
		 AESK_Nextion_Write_Num(BATTERY_CUR, lyra_can_data.rx.bms_data.Bat_Current_f32 * 10.0f);
		 AESK_Nextion_Write_Num(ISO_POS, lyra_can_data.rx.bms_data.iso_res_pos);
		 AESK_Nextion_Write_Num(ISO_NEG, lyra_can_data.rx.bms_data.iso_res_neg);
		 AESK_Nextion_Write_Num(RIGH_MOTOR_TEMP, lyra_can_data.rx.mcu.motor_temp_u8);
		 AESK_Nextion_Write_Num(LEFT_MOTOR_TEMP, lyra_can_data.rx.mcu.motor_temp_u8 + 1);

		 if(lyra_can_data.tx.mcu.states_union.states_t.mode == Speed_Control)
		 {
			 AESK_Nextion_Write_Num(SET_VEL, vcu_data.set_velocity_u8);
		 }

		 else
		 {
			 AESK_Nextion_Write_Num(SET_VEL, vcu_data.set_torque_u8);
		 }

		//Bms Wake Up Control
		  if(lyra_can_data.tx.mcu.states_union.states_t.bms_wake == WAKE && bms_wake_flag != 1)
		  {
			  bms_wake_flag = 1;
			  AESK_Nextion_Change_Pic(BMS_WAKE_PIC, ACTIVE_PIC_ID);
		  }

		  else if(lyra_can_data.tx.mcu.states_union.states_t.bms_wake == SLEEP && bms_wake_flag != 2)
		  {
			bms_wake_flag = 2;
			AESK_Nextion_Change_Pic(BMS_WAKE_PIC, PASIVE_PIC_ID);
		  }
		//

		//Mcu Wake Up Control
		  if(lyra_can_data.tx.mcu.states_union.states_t.mcu_wake == TRUE && mcu_wake_flag != 1)
		  {
			 mcu_wake_flag = 1;
			 AESK_Nextion_Change_Pic(MCU_WAKE_PIC, ACTIVE_PIC_ID);
		  }

		  else if (lyra_can_data.tx.mcu.states_union.states_t.mcu_wake != TRUE && mcu_wake_flag != 2)
		  {
			 mcu_wake_flag = 2;
			 AESK_Nextion_Change_Pic(MCU_WAKE_PIC, PASIVE_PIC_ID);
		  }
		//

		//Ignition Control
		  if((lyra_can_data.tx.mcu.states_union.states_t.ignition == OFF || lyra_can_data.tx.mcu.states_union.states_t.brake == 1) &&  ignition_flag != 1)
		  {
			  ignition_flag = 1;
			 AESK_Nextion_Change_Pic(IGN_PIC, PASIVE_PIC_ID);
		  }

		  else if(lyra_can_data.tx.mcu.states_union.states_t.ignition == ON && lyra_can_data.tx.mcu.states_union.states_t.brake == 0 && ignition_flag != 2)
		  {
			  ignition_flag = 2;
			 AESK_Nextion_Change_Pic(IGN_PIC, ACTIVE_PIC_ID);
		  }

		//
		//Direction Control
		 if(lyra_can_data.tx.mcu.states_union.states_t.reverse == 1 && reverse_flag != 1)
		 {
			 AESK_Nextion_Write_Text(DIRECTION, "R");
			 reverse_flag = 1;

		 }

		 else if (lyra_can_data.tx.mcu.states_union.states_t.reverse == 0 && reverse_flag != 2)
		 {
			 AESK_Nextion_Write_Text(DIRECTION, "D");
			 reverse_flag = 2;
		 }
		//

		//Brake Control
		 if(lyra_can_data.tx.mcu.states_union.states_t.brake == ON && vehicle_mode != 1)
		 {
			 vehicle_mode = 1;
			 AESK_Nextion_Write_Text(VEHICLE_STATE_BOX, "Braking");
		 }

		 else if(lyra_can_data.tx.mcu.states_union.states_t.brake == OFF)
		 {
			 if(lyra_can_data.tx.mcu.states_union.states_t.mode == Torque_Control && vehicle_mode != 2)
			 {
				 vehicle_mode = 2;
				 AESK_Nextion_Write_Text(VEHICLE_STATE_BOX, "Torque Mode");
			 }

			 else if (lyra_can_data.tx.mcu.states_union.states_t.mode == Speed_Control && vehicle_mode != 3)
			 {
				 vehicle_mode = 3;
				 AESK_Nextion_Write_Text(VEHICLE_STATE_BOX, "Speed Mode");
			 }
		 }

		 //TV Screen Print
		  if(vcu_data.tv_active_u8 == TRUE && tv_active_flag != 1)
		  {
			 tv_active_flag = 1;
			 AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "TV On");
		  }

		  else if (vcu_data.tv_active_u8 != TRUE && tv_active_flag != 2)
		  {
			  tv_active_flag = 2;
			  AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "TV Off");
		  }
		//

		//Error Detection
		 uint16_t mcu_errors = lyra_can_data.rx.mcu.state.condition_u16 & 0x0fff;
		 uint16_t mcu_errors_2 = lyra_can_data.rx.mcu_2.state.condition_u16 & 0x0fff;

		 if(lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8 != 0 && error_flag != 1)
		 {
			error_flag = 1;
			for(uint8_t i = 1; i < 64; i <<= 1)
			{
				uint8_t error = lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8 & i;

				if(error != 0)
				{
					switch(error)
					{
						case  High_Voltage:
						{
							AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "High Voltage");
							break;
						}

						case Low_Voltage:
						{
							AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Low Voltage");
							break;
						}

						case Temp_Error:
						{
							AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Temp");
							break;
						}

						case Com_Error:
						{
							AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Comm Error");
							break;
						}

						case Over_Cur:
						{
							AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Current");
							break;
						}

						default: break;
					}
				}
			}
		 }

		 //MCU 1
		 else if(mcu_errors != 0 && error_flag != 2 && lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8 == 0)
		 {
			 error_flag = 2;
			 for(uint16_t i = 1; i < 1025; i <<= 1)
			 {
				uint16_t mcu_error = mcu_errors & i;
				switch(mcu_error)
				{
					case  Over_Current_IA:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Current IA");
						break;
					}

					case Over_Current_IB:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Current IB");
						break;
					}

					case Over_Current_IC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Current IC");
						break;
					}

					case Over_Current_IDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Current IDC");
						break;
					}

					case Under_Current_IDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Under Current IDC");
						break;
					}

					case Under_Voltage_VDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Under Volt VDC");
						break;
					}

					case Over_Voltage_VDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Volt VDC");
						break;
					}

					case Under_Speed:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Under Speed");
						break;
					}

					case Over_Speed:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "Over Speed");
						break;
					}

					case Over_Temperature:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "M. Over Temp");
						break;
					}

					case ZPC_Calibration:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "ZPC Error");
						break;
					}

					default:	break;
				}
			 }
		 }


		 //MCU 2
		 else if(mcu_errors_2 != 0 && error_flag != 3 && lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8 == 0)
		 {
			 error_flag = 3;
			 for(uint16_t i = 1; i < 1025; i <<= 1)
			 {
				uint16_t mcu_error = mcu_errors_2 & i;
				switch(mcu_error)
				{
					case  Over_Current_IA:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Over Current IA");
						break;
					}

					case Over_Current_IB:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Over Current IB");
						break;
					}

					case Over_Current_IC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Over Current IC");
						break;
					}

					case Over_Current_IDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Over Current IDC");
						break;
					}

					case Under_Current_IDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Under Current IDC");
						break;
					}

					case Under_Voltage_VDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Under Volt VDC");
						break;
					}

					case Over_Voltage_VDC:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Over Volt VDC");
						break;
					}

					case Under_Speed:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Under Speed");
						break;
					}

					case Over_Speed:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 Over Speed");
						break;
					}

					case Over_Temperature:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 M. Over Temp");
						break;
					}

					case ZPC_Calibration:
					{
						AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, "2 ZPC Error");
						break;
					}

					default:	break;
				}
			 }
		 }

		else if (error_flag != 4 && mcu_errors == 0 && lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8 == 0)
		{
			error_flag = 4;
			AESK_Nextion_Write_Text(BMS_MCU_ERROR_BOX, " ");
		}
	}

	else if(screen_data.page == CELLS_PAGE)
	{
		reverse_flag = 0; bms_wake_flag = 0; mcu_wake_flag = 0; ignition_flag = 0; error_flag = 0; vehicle_mode = 0;
		AESK_Nextion_Write_Num(CELL_1, lyra_can_data.rx.bms_data.bms_cells.Cell_1_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_2, lyra_can_data.rx.bms_data.bms_cells.Cell_2_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_3, lyra_can_data.rx.bms_data.bms_cells.Cell_3_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_4, lyra_can_data.rx.bms_data.bms_cells.Cell_4_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_5, lyra_can_data.rx.bms_data.bms_cells.Cell_5_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_6, lyra_can_data.rx.bms_data.bms_cells.Cell_6_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_7, lyra_can_data.rx.bms_data.bms_cells.Cell_7_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_8, lyra_can_data.rx.bms_data.bms_cells.Cell_8_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_9, lyra_can_data.rx.bms_data.bms_cells.Cell_9_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_10, lyra_can_data.rx.bms_data.bms_cells.Cell_10_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_11, lyra_can_data.rx.bms_data.bms_cells.Cell_11_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_12, lyra_can_data.rx.bms_data.bms_cells.Cell_12_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_13, lyra_can_data.rx.bms_data.bms_cells.Cell_13_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_14, lyra_can_data.rx.bms_data.bms_cells.Cell_14_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_15, lyra_can_data.rx.bms_data.bms_cells.Cell_15_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_16, lyra_can_data.rx.bms_data.bms_cells.Cell_16_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_17, lyra_can_data.rx.bms_data.bms_cells.Cell_17_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_18, lyra_can_data.rx.bms_data.bms_cells.Cell_18_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_19, lyra_can_data.rx.bms_data.bms_cells.Cell_19_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_20, lyra_can_data.rx.bms_data.bms_cells.Cell_20_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_21, lyra_can_data.rx.bms_data.bms_cells.Cell_21_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_22, lyra_can_data.rx.bms_data.bms_cells.Cell_22_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_23, lyra_can_data.rx.bms_data.bms_cells.Cell_23_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_24, lyra_can_data.rx.bms_data.bms_cells.Cell_24_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_25, lyra_can_data.rx.bms_data.bms_cells.Cell_25_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_26, lyra_can_data.rx.bms_data.bms_cells.Cell_26_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_27, lyra_can_data.rx.bms_data.bms_cells.Cell_27_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);
		AESK_Nextion_Write_Num(CELL_28, lyra_can_data.rx.bms_data.bms_cells.Cell_28_u8 + lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32);

		AESK_Nextion_Write_Num(TEMP_1, lyra_can_data.rx.bms_data.bms_temps.temp_1_u8);
		AESK_Nextion_Write_Num(TEMP_2, lyra_can_data.rx.bms_data.bms_temps.temp_2_u8);
		AESK_Nextion_Write_Num(TEMP_3, lyra_can_data.rx.bms_data.bms_temps.temp_3_u8);
		AESK_Nextion_Write_Num(TEMP_4, lyra_can_data.rx.bms_data.bms_temps.temp_4_u8);
		AESK_Nextion_Write_Num(TEMP_5, lyra_can_data.rx.bms_data.bms_temps.temp_5_u8);
		AESK_Nextion_Write_Num(TEMP_6, lyra_can_data.rx.bms_data.bms_temps.temp_6_u8);
		AESK_Nextion_Write_Num(TEMP_7, lyra_can_data.rx.bms_data.bms_temps.temp_7_u8);
	}


	 HAL_UART_Transmit_DMA(&SCREEN_Serial, screen_data.nextion_tx_buf, screen_data.screen_tx_buf_index);

	 screen_data.screen_tx_buf_index = 0;
	 screen_data.nextion_tx_buf[0] = '\0';
}
void Beacon_Control()
{
		if(lyra_can_data.rx.bms_data.Temperature_u8 > MAX_TEMPERATURE || (lyra_can_data.rx.bms_data.iso_res_neg <= 150 || lyra_can_data.rx.bms_data.iso_res_pos <= 150))
		{
			HAL_GPIO_TogglePin(Beacon_Trigger_GPIO_Port, Beacon_Trigger_Pin);
		}

		else
		{
			Beacon_Trigger_GPIO_Port->BSRR = (uint32_t)Beacon_Trigger_Pin << 16U;
			HAL_GPIO_WritePin(Beacon_Trigger_GPIO_Port, Beacon_Trigger_Pin, 0);
		}
}



void Telemetry_Query_Answer()
{
	 if(tel_data.query_answer == 1)
	 {
		 static uint8_t ctr = 0;
		 ctr++;
		 if(ctr == 1)
		 {
			 uint8_t kp_ki_data[8];
			 uint16_t kp_ki_idx = 0;
			 AESK_FLOAT32toUINT8_LE(&electronic_dif_param.kp_f32, kp_ki_data, &kp_ki_idx);
			 AESK_FLOAT32toUINT8_LE(&electronic_dif_param.ki_f32, kp_ki_data, &kp_ki_idx);
			 AESK_CAN_SendExtIDMessage(&aesk_gl.aesk_can_1, KP_KI_SEND, kp_ki_data, 8);
		 }

		 if(ctr == 2)
		 {
			 uint8_t kd_regen_data[6];
			 uint16_t kd_regen_idx = 0;
			 uint16_t regen_coef = 100 * electronic_dif_param.control_input_coef_f32;
			 AESK_FLOAT32toUINT8_LE(&electronic_dif_param.kd_f32, kd_regen_data, &kd_regen_idx);
			 AESK_UINT16toUINT8_LE(&regen_coef, kd_regen_data, &kd_regen_idx);
			 AESK_CAN_SendExtIDMessage(&aesk_gl.aesk_can_1, KD_REGEN_SEND, kd_regen_data, sizeof(kd_regen_data));
			 tel_data.query_answer = 0;
			 ctr = 0;
		 }
	 }
}


#if COM_MODE == COMPRO || COM_MODE == MCU_OLD
void Solver(const Aesk_Compro_Pack *pack)
{
	if(pack->source_id & BMS)
	{
		switch(pack->source_msg_id)
		{
			case S_BMS_MEASUREMENTS:
			{
				uint16_t bms_measurement_index = 0;
				AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Bat_Voltage_f32, pack->msg, FLOAT_CONVERTER_2, &bms_measurement_index);
				AESK_INT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Bat_Current_f32, pack->msg, FLOAT_CONVERTER_2, &bms_measurement_index);
				AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Bat_Cons_f32, pack->msg, FLOAT_CONVERTER_1, &bms_measurement_index);
				AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Soc_f32, pack->msg, FLOAT_CONVERTER_2, &bms_measurement_index);
				/************************************************************************************************************************/
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8, pack->msg, &bms_measurement_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_dc_bus_state.bms_state_data_dc_bus_state_u8, pack->msg, &bms_measurement_index);
				AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32, pack->msg, FLOAT_CONVERTER_1, &bms_measurement_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.Worst_Cell_Address_u8, pack->msg, &bms_measurement_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.Temperature_u8, pack->msg, &bms_measurement_index);
				/************************************************************************************************************************/

				uint8_t iso_pos_u8;
				uint8_t iso_neg_u8;

				AESK_UINT8toUINT8ENCODE(&iso_pos_u8, pack->msg, &bms_measurement_index);
				AESK_UINT8toUINT8ENCODE(&iso_neg_u8, pack->msg, &bms_measurement_index);

				bms_data.iso_res_pos = iso_pos_u8 * 10000;
				bms_data.iso_res_neg = iso_neg_u8 * 10000;

				if(bms_data.Temperature_u8 > MAX_TEMPERATURE || (bms_data.iso_res_neg <= 150000 || bms_data.iso_res_pos <= 150000))
				{

					Beacon_Trigger_GPIO_Port->BSRR = (uint32_t)Beacon_Trigger_Pin;
				}

				else
				{
					Beacon_Trigger_GPIO_Port->BSRR = (uint32_t)Beacon_Trigger_Pin << 16U;
				}
				break;
			}

			/*case S_BMS_STATE_DATA:
			{
				uint16_t bms_state_data_index = 0;
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_error.bms_state_data_error_u8, pack->msg, &bms_state_data_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_dc_bus_state.bms_state_data_dc_bus_state_u8, pack->msg, &bms_state_data_index);
				AESK_UINT16toFLOAT_LE(&lyra_can_data.rx.bms_data.Worst_Cell_Voltage_f32, pack->msg, FLOAT_CONVERTER_1, &bms_state_data_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.Worst_Cell_Address_u8, pack->msg, &bms_state_data_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.Temperature_u8, pack->msg, &bms_state_data_index);

				uint8_t iso_pos_u8;
				uint8_t iso_neg_u8;

				AESK_UINT8toUINT8ENCODE(&iso_pos_u8, pack->msg, &bms_state_data_index);
				AESK_UINT8toUINT8ENCODE(&iso_neg_u8, pack->msg, &bms_state_data_index);

				bms_data.iso_res_pos = iso_pos_u8 * 10000;
				bms_data.iso_res_neg = iso_neg_u8 * 10000;

				if(bms_data.Temperature_u8 > MAX_TEMPERATURE || (bms_data.iso_res_neg <= 150000 || bms_data.iso_res_pos <= 150000))
				{

					Beacon_Trigger_GPIO_Port->BSRR = (uint32_t)Beacon_Trigger_Pin;
				}

				else
				{
					Beacon_Trigger_GPIO_Port->BSRR = (uint32_t)Beacon_Trigger_Pin << 16U;
				}
				break;
			}*/

			case S_BMS_CELLS_1 :
			{
				uint16_t bms_cells_1_index = 0;
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_1_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_2_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_3_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_4_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_5_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_6_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_7_u8, pack->msg, &bms_cells_1_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_8_u8, pack->msg, &bms_cells_1_index);
				break;
			}

			case S_BMS_CELLS_2 :
			{
				uint16_t bms_cells_2_index = 0;
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_9_u8,  pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_10_u8, pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_11_u8, pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_12_u8, pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_13_u8, pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_14_u8, pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_15_u8, pack->msg, &bms_cells_2_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_16_u8, pack->msg, &bms_cells_2_index);
				break;
			}

			case S_BMS_CELLS_3 :
			{
				uint16_t bms_cells_3_index = 0;
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_17_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_18_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_19_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_20_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_21_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_22_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_23_u8, pack->msg, &bms_cells_3_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_24_u8, pack->msg, &bms_cells_3_index);
				break;
			}

			case S_BMS_CELLS_4 :
			{
				uint16_t bms_cells_4_index = 0;
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_25_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_26_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_27_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_cells.Cell_28_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_1_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_2_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_3_u8, pack->msg, &bms_cells_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_4_u8, pack->msg, &bms_cells_4_index);
				break;
			}

			case S_BMS_SOC_1 :
			{
				uint16_t bms_soc_1_index = 0;
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_1_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_2_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_3_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_4_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_5_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_6_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_7_u8, pack->msg, &bms_soc_1_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_8_u8, pack->msg, &bms_soc_1_index);
				break;
			}

			case S_BMS_SOC_2:
			{
				uint16_t bms_soc_2_index = 0;
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_9_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_10_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_11_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_12_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_13_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_14_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_15_u8, pack->msg, &bms_soc_2_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_16_u8, pack->msg, &bms_soc_2_index);
				break;
			}

			case S_BMS_SOC_3 :
			{
				uint16_t bms_soc_3_index = 0;
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_17_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_18_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_19_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_20_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_21_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_22_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_23_u8, pack->msg, &bms_soc_3_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_24_u8, pack->msg, &bms_soc_3_index);
				break;
			}

			case S_BMS_SOC_4 :
			{
				uint16_t bms_soc_4_index = 0;
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_25_u8, pack->msg, &bms_soc_4_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_26_u8, pack->msg, &bms_soc_4_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_27_u8, pack->msg, &bms_soc_4_index);
				AESK_UINT8toINT8ENCODE(&lyra_can_data.rx.bms_data.bms_soc.offset_SoC_28_u8, pack->msg, &bms_soc_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_5_u8, pack->msg, &bms_soc_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_6_u8, pack->msg, &bms_soc_4_index);
				AESK_UINT8toUINT8ENCODE(&lyra_can_data.rx.bms_data.bms_temps.temp_7_u8, pack->msg, &bms_soc_4_index);
				break;
			}

			case S_BMS_SOH :
			{
				uint16_t bms_soh_index = 0;
				AESK_UINT8toUINT16_LE(&lyra_can_data.rx.bms_data.power_limited_u16, pack->msg, &bms_soh_index);
				AESK_UINT8toUINT8ENCODE(&bms_data.soh_u8, pack->msg, &bms_soh_index);
				break;
			}

			default :
			{
				break;
			}

		}
	}

	else if(pack->source_id & TELEMETRY)
	{
		switch(pack->source_msg_id)
		{

			case S_BMS_CELLS_1:
			{
				sayac_Telemetry++;
			}
			case S_PID_TUNNING:
			{
				uint16_t pid_tune_index = 0;
				AESK_UINT8toFLOAT32_LE(&electronic_dif_param.kp_f32, pack->msg, &pid_tune_index);
				AESK_UINT8toFLOAT32_LE(&electronic_dif_param.ki_f32, pack->msg, &pid_tune_index);
				AESK_UINT8toFLOAT32_LE(&electronic_dif_param.kd_f32, pack->msg, &pid_tune_index);
				break;
			}

			case S_PID_QUERY:
			{
				Aesk_Compro_Pack compro;
				uint16_t pid_query_idx = 0;
				uint8_t data[12];
				AESK_FLOAT32toUINT8_LE(&electronic_dif_param.kp_f32, data, &pid_query_idx);
				AESK_FLOAT32toUINT8_LE(&electronic_dif_param.ki_f32, data, &pid_query_idx);
				AESK_FLOAT32toUINT8_LE(&electronic_dif_param.kd_f32, data, &pid_query_idx);
				AESK_Compro_Create(&compro, Electromobile, TELEMETRY, VCU, (uint8_t *)data, sizeof(data), S_PID_QUERY_ANSWER);
				AESK_Compro_Send_Ring_Buf(&compro, &CAN1_Rng_Buf_Tx);
				AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_1, &CAN1_Rng_Buf_Tx, 0x15550000);
				break;
			}
		}
	}

	else if (pack->source_id & MCU)
	{
		/*for(uint16_t i = 0; i < pack->msg_size; i ++)
		{
			mcu_pack[i] = pack->msg[i];
		}*/
		sayac_MCU++;
	}
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(sensor_data.r_rpm_pin_triggered == 1)
	{
		sensor_data.r_rim_hole_ctr++;
		sensor_data.r_rpm_pin_triggered = 0;
	}

	else
	{
		sensor_data.l_rim_hole_ctr++;
		sensor_data.l_rpm_pin_triggered = 0;
	}
	HAL_TIM_Base_Stop_IT(&htim6);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RPM_Sensor_L_Pin)
	{
		sensor_data.l_rpm_pin_triggered = 1;
		l_ctr++;
	}

	else
	{
		r_ctr++;
		sensor_data.r_rpm_pin_triggered = 1;
	}
	HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch(screen_data.Screen_States_u8)
	{
		case FIRST_BYTE_CONTROL:
		{
			if(screen_data.rx_u8 == FIRST_CONTROL_BYTE)
			{
				screen_data.Screen_States_u8 = SECOND_BYTE_CONTROL;
			}
			break;
		}

		case SECOND_BYTE_CONTROL:
		{
			if(screen_data.rx_u8 == SECOND_CONTROL_BYTE)
			{
				screen_data.Screen_States_u8 = PAGE_CONTROL;
			}
			break;
		}

		case PAGE_CONTROL:
		{
			if(screen_data.rx_u8 == PAGE_1)
			{
				screen_data.page = MAIN_PAGE;
			}

			else if(screen_data.rx_u8 == PAGE_2)
			{
				screen_data.page = CELLS_PAGE;
			}
			screen_data.Screen_States_u8 = FIRST_BYTE_CONTROL;
			break;
		}
	}
	 HAL_UART_Receive_DMA(&huart1, &screen_data.rx_u8, 1);
}




