/*
 * AESK_General.h
 *
 *  Created on: Mar 8, 2021
 *      Author: Ahmet
 */

#ifndef INC_AESK_VCU_GENERAL_H_
#define INC_AESK_VCU_GENERAL_H_

#include "main.h"
#include "Can_Lyra_Header.h"
#include "stdarg.h"
#include "AESK_CAN_Library.h"
#include "AESK_VCU_Nextion.h"
#include "AESK_Data_Pack_lib.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;


#define PULL_UP_OPEN					0
#define PULL_UP_CLOSE					1
#define PULL_DOWN_OPEN					1
#define PULL_DOWN_CLOSE					0
#define WAKE							1
#define SLEEP							0
#define SCREEN_Serial					huart1
#define RS485_Serial					huart6
#define TRUE							1
#define FALSE 							0
#define	ON								1
#define OFF								0
#define LOCK							0
#define UNLOCK							1
#define MAX_SPEED_LIMIT     			100
#define MIN_SPEED_LIMIT     			0
#define MAX_TEMPERATURE     			55
#define SOC_CONVERTER 					50
#define SPEED_LIMIT						512
#define TORQUE_LIMIT					40
#define CONTROL_INPUT					0.35f
#define REGEN_RAMP_COEF					0.6f
#define MAX_STEERING_ADC_VAL			1791.0f
#define MIDDLE_POINT_ADC_VAL			1330.0f
#define MIN_STEERING_ADC_VAL			909.0f
#define MIDDLE_POINT_ADC_VAL_CALCULATED	((MAX_STEERING_ADC_VAL - MIN_STEERING_ADC_VAL) / 2.0f)
#define MAX_STEERING_TURN				54.0f

#define LNG_PRESS_CTR_OFFSET	4
#define CHANGE_VEL_OFFSET		4
#define MIN_VELOCITY			0
#define MAX_VELOCITY			65
#define MAX_TORQUE				255
#define MIN_TORQUE				0

/****************VCU Configurations*****************/
#define MCU_OLD							0			//MCU için 2020 Haberleşme Yapısı
#define COMPRO							1			//2021 Haberleşme Yapısı
#define OLD_COM							2			//2020 Haberleşme Yapısı
#define COM_MODE						OLD_COM
#define SINGLE_MOTOR					ON		//Çift Motor kullanıldığı senaryoda 0 yapılır.

#define Nextion_UI_2021					1			//Eski arayüzü kullanmak için 0 yapılır.

#define STEERING_DEBUG					OFF			//Masa üstü testleri yapılırken direksiyon olmadığı durumda 1 yapılır.
/****************************************************/

/********************CAN ID's********************/
#define VCU_CAN_ID_1					0x15550000
#define VCU_CAN_ID_2					0x15550001
#define MCU_PACK_ID_1					0x1555DDD0
#define MCU_PACK_ID_2					0x1555DDD1
#define	MCU_PACK_ID_3					0x1555DDD2
#define	MCU_PACK_ID_4					0x1555DDD3
#define	MCU_PACK_ID_5					0x1555DDD4
#define	MCU_PACK_ID_6					0x1555DDD5
#define WAKE_UP_COMMANDS				0x15550000
#define EMS_CURRENT            			0x1555EEE0
#define EMS_VOLTAGE						0x1555EEE1
#define EMS_CONSUMPTION					0x1555EEE2
#define EMS_STATE_DATA					0x1555EEE3
#define BMS_MEASUREMENTS				0x1555BBB0
#define BMS_STATE_DATA					0x1555BBB1
#define CHARGER_TX_ID					0x18FF50E5
#define BMS_CELLS_1						0x1555BBB2
#define BMS_CELLS_2						0x1555BBB3
#define BMS_CELLS_3						0x1555BBB4
#define BMS_CELLS_4						0x1555BBB5
#define BMS_SOC_1						0x1555BBB6
#define BMS_SOC_2						0x1555BBB7
#define BMS_SOC_3						0x1555BBB8
#define BMS_SOC_4						0x1555BBB9
#define BMS_SOH							0x1555BBBA
#define KP_KI_SET						0x15550011
#define KD_SET							0x15550012
#define TELEMETRY_QUERY					0x15550013
#define KP_KI_SEND						0x15550014
#define KD_REGEN_SEND					0x15550015
/*************************************************/

//BMS Error enum
enum
{
	High_Voltage  	= 1,
	Low_Voltage   	= 2,
	Temp_Error      = 4,
	Com_Error 		= 8,
	Over_Cur   		= 16,
	Fatal_Error		= 32,
	Reserved
};


//MCU Error enum
enum
{
	Over_Current_IA = 1,
	Over_Current_IB = 2,
	Over_Current_IC = 4,
	Over_Current_IDC = 8,
	Under_Current_IDC = 16,
	Under_Voltage_VDC = 32,
	Over_Voltage_VDC = 64,
	Under_Speed	= 128,
	Over_Speed = 256,
	Over_Temperature = 512,
	ZPC_Calibration = 1024,
};


typedef struct
{
	uint8_t _1ms_flag 		: 1;
	uint8_t _10ms_flag 		: 1;
	uint8_t _20ms_flag		: 1;
	uint8_t _50ms_flag 		: 1;
	uint8_t _100ms_flag 	: 1;
	uint8_t _120ms_flag 	: 1;
	uint8_t _200ms_flag 	: 1;
	uint8_t _1000ms_flag 	: 1;
}Sys_Timer_s;
Sys_Timer_s sys_timer;

enum
{
	Torque_Control = 0,
	Speed_Control = 1,
};


/*	Bu structure Lyra uzerindeki sensor bilgilerini ve durumlarini icerir.
 *
 */
typedef struct
{
	uint8_t r_rpm_pin_triggered : 1;
	uint8_t l_rpm_pin_triggered : 1;
	uint8_t reserved 			: 6;
	uint16_t triggered_pin;
	float l_rim_hole_ctr;
	float r_rim_hole_ctr;
	uint16_t adc_val_u16;
	uint16_t filtered_adc_val_u16;
	uint8_t steering_angle_u8;
}Sensor_Data_s;
Sensor_Data_s sensor_data;

typedef struct
{
	uint8_t query_answer : 1;
	uint8_t reserved	 : 7;
}Telemetry_t;
Telemetry_t tel_data;

typedef union
{
	struct
	{
		uint8_t ZPC_Completed 		  		: 1;
		uint8_t PWM_Enabled	  		  		: 1;
		uint8_t Dc_Bara_Error 		  		: 1;
		uint8_t Temp_Error	  		 		: 1;
		uint8_t Dc_Bara_Current_Error 		: 1;
		uint8_t tork_limit					: 1;
		uint8_t Reserved			  		: 2;
	}Driver_error;
	uint8_t driver_error_u8;
}Driver_error_union;
Driver_error_union driver_states;



typedef union
{
	struct
	{
		uint8_t free_wheeling	: 1;
		uint8_t mode			: 1;
		uint8_t brake	 		: 1;
		uint8_t reverse			: 1;
		uint8_t reserved		: 4;
	}drive_status;
	uint8_t drive_commands_u8;
}Driver_Commands;


typedef struct
{
	float Phase_A_Current_f32;
	float Phase_B_Current_f32;
	float Dc_Bus_Current_f32;
	float Dc_Bus_voltage_f32;
	float Id_f32;
	float Iq_f32;
	float IArms_f32;
	float IBrms_f32;
	uint32_t Odometer_u32;
	uint8_t Motor_Temperature_u8;
	uint8_t actual_velocity_u8;
	Driver_Commands drive_command_data;
	Driver_error_union driver_error_union;
}Driver_Data;


typedef union
{
	struct
	{
		uint8_t Bat_Current_Error : 1;
		uint8_t Fc_Current_Error  : 1;
		uint8_t Out_Current_Error : 1;
		uint8_t Bat_Voltage_Error : 1;
		uint8_t Fc_Voltage_Error  : 1;
		uint8_t Out_Voltage_Error : 1;
		uint8_t Wake_Up_Control   : 1;
		uint8_t Reserved          : 1;
	}Ems_Error;
	uint8_t ems_error_union_u8;
}Ems_Error_Union;


typedef union
{
	struct
	{
		uint8_t bms_wake_up				: 1;
		uint8_t direction				: 1;   //1 => reverse, 0 => forward
		uint8_t bms_screen_control 		: 1;
		uint8_t charger_wake_up 		: 1;
		uint8_t manual_balance			: 1;
		uint8_t system_on_off			: 1;
		uint8_t ignition				: 1;
		uint8_t torque_mod				: 1;
	}wake_up;
	uint8_t wake_up_u8;
}Wake_Up_Union;


typedef struct
{
	Wake_Up_Union wake_up_union;
	uint8_t set_torque_u8;
	uint8_t m1_set_torque_u8;
	uint8_t m2_set_torque_u8;
	uint8_t set_velocity_u8;
	uint8_t m1_set_velocity_u8;
	uint8_t m2_set_velocity_u8;
	uint8_t stored_vel_u8;
	uint8_t stored_torque_u8;
	uint8_t ignition_closed;
	uint8_t mod_selected;
	uint8_t fake_brake;
	int8_t actual_velocity_i8;
	int8_t m1_actual_velocity_i8;
	int8_t m2_actual_velocity_i8;
	int16_t r_rpm_i16;
	int16_t l_rpm_i16;
	uint32_t odometer_u32;
	float set_r_torque_f32;
	float set_l_torque_f32;
	float set_rpm_f32;
	uint8_t tv_active_u8;
	uint8_t bms_on_bus;
	uint8_t sp_on;

}VCU_Data;
VCU_Data vcu_data;

typedef union
{
	struct
	{
		uint8_t High_Voltage_Error   	:1;
		uint8_t Low_Voltage_Error    	:1;
		uint8_t Temp_Error           	:1;
		uint8_t Communication_Errror 	:1;
		uint8_t Over_Current_Error   	:1;
		uint8_t Fatal_Error			 	:1;
		uint8_t Reserved             	:2;
	}State_Data_Error;
	uint8_t bms_state_data_error_u8;
}Bms_State_Data_Error_Union;
Bms_State_Data_Error_Union bms_errors;

typedef union
{
	struct
	{
		uint8_t Precharge_Flag 		: 1;
		uint8_t Discharge_Flag 		: 1;
		uint8_t Dc_Bus_Ready   		: 1;
		uint8_t Charging 			: 1;
		uint8_t Reserved       		: 4;
	}Bms_State_Data_Dc_Bus_State;
	uint8_t bms_state_data_dc_bus_state_u8;
}Bms_State_Data_Dc_Bus_State_Error_union;
Bms_State_Data_Dc_Bus_State_Error_union bms_states;

typedef struct
{
	uint8_t Cell_1_u8;
	uint8_t Cell_2_u8;
	uint8_t Cell_3_u8;
	uint8_t Cell_4_u8;
	uint8_t Cell_5_u8;
	uint8_t Cell_6_u8;
	uint8_t Cell_7_u8;
	uint8_t Cell_8_u8;
	uint8_t Cell_9_u8;
	uint8_t Cell_10_u8;
	uint8_t Cell_11_u8;
	uint8_t Cell_12_u8;
	uint8_t Cell_13_u8;
	uint8_t Cell_14_u8;
	uint8_t Cell_15_u8;
	uint8_t Cell_16_u8;
	uint8_t Cell_17_u8;
	uint8_t Cell_18_u8;
	uint8_t Cell_19_u8;
	uint8_t Cell_20_u8;
	uint8_t Cell_21_u8;
	uint8_t Cell_22_u8;
	uint8_t Cell_23_u8;
	uint8_t Cell_24_u8;
	uint8_t Cell_25_u8;
	uint8_t Cell_26_u8;
	uint8_t Cell_27_u8;
	uint8_t Cell_28_u8;
}Bms_Cells;

typedef struct
{
	int8_t offset_SoC_1_u8;
	int8_t offset_SoC_2_u8;
	int8_t offset_SoC_3_u8;
	int8_t offset_SoC_4_u8;
	int8_t offset_SoC_5_u8;
	int8_t offset_SoC_6_u8;
	int8_t offset_SoC_7_u8;
	int8_t offset_SoC_8_u8;
	int8_t offset_SoC_9_u8;
	int8_t offset_SoC_10_u8;
	int8_t offset_SoC_11_u8;
	int8_t offset_SoC_12_u8;
	int8_t offset_SoC_13_u8;
	int8_t offset_SoC_14_u8;
	int8_t offset_SoC_15_u8;
	int8_t offset_SoC_16_u8;
	int8_t offset_SoC_17_u8;
	int8_t offset_SoC_18_u8;
	int8_t offset_SoC_19_u8;
	int8_t offset_SoC_20_u8;
	int8_t offset_SoC_21_u8;
	int8_t offset_SoC_22_u8;
	int8_t offset_SoC_23_u8;
	int8_t offset_SoC_24_u8;
	int8_t offset_SoC_25_u8;
	int8_t offset_SoC_26_u8;
	int8_t offset_SoC_27_u8;
	int8_t offset_SoC_28_u8;
}Bms_SocS;


typedef struct
{
	uint8_t temp_1_u8;
	uint8_t temp_2_u8;
	uint8_t temp_3_u8;
	uint8_t temp_4_u8;
	uint8_t temp_5_u8;
	uint8_t temp_6_u8;
	uint8_t temp_7_u8;
}Bms_Temps;

typedef struct
{
	float    Bat_Voltage_f32;
	float    Bat_Current_f32;
	float    Bat_Cons_f32;
	float    Soc_f32;
	float    Worst_Cell_Voltage_f32;
	uint16_t power_limited_u16;
	uint8_t  soh_u8;
	uint8_t  Worst_Cell_Address_u8;
	uint8_t  Temperature_u8;
	uint32_t iso_res_pos;
	uint32_t iso_res_neg;
	Bms_Temps bms_temps;
	Bms_Cells bms_cells;
	Bms_State_Data_Error_Union bms_error;
	Bms_State_Data_Dc_Bus_State_Error_union bms_dc_bus_state;
	Bms_SocS bms_soc;
}Bms_Datas;

Bms_Datas bms_data;

typedef union
{
	struct
	{
		uint16_t relay_on_off 				: 1;
		uint16_t start_delay_error 			: 1;
		uint16_t over_current_error 		: 1;
		uint16_t over_voltage_error 		: 1;
		uint16_t temp_error					: 1;
		uint16_t comm_error					: 1;
		uint16_t charge_mode 				: 1; // cc = 1, cv = 0
		uint16_t bms_on_off					: 1;
		uint16_t zpc_completed 				: 1;
		uint16_t battery_connection 		: 1;
		uint16_t reserved					: 6;
	}ChargerFlags_struct;
		uint16_t ChargerFlags_u16;
}ChargerFlags_union;

typedef struct
{
	float actual_volt_f32;
	float actual_current_f32;
	ChargerFlags_union chargerFlags;
	uint32_t charger_counter;
}ChargerDatas;

ChargerDatas charger;


typedef union
{
	struct
	{
		uint8_t bms_wake		: 1;
		uint8_t mcu_wake		: 1;
		uint8_t ignition		: 1;
		uint8_t mode 			: 1;
		uint8_t brake 			: 1;
		uint8_t reverse			: 1;
		uint8_t regen			: 1;
		uint8_t reserved 		: 1;
	}states_t;
	uint8_t states_u8;
}mcu_states_union;


typedef union
{
	struct
	{
		uint16_t overcur_IA					: 1;
		uint16_t overcur_IB 				: 1;
		uint16_t overcur_IC 				: 1;
		uint16_t overcur_IDC				: 1;
		uint16_t undercur_IDC 				: 1;
		uint16_t undervol_VDC 				: 1;
		uint16_t overvol_VDC 				: 1;
		uint16_t under_speed 				: 1;
		uint16_t over_speed 				: 1;
		uint16_t over_temp 					: 1;
		uint16_t input_scaling_calib 		: 1;		//ZPC Calibration Error when 0
		uint16_t pwm_enable 				: 1;
		uint16_t free_wheeling 				: 1;
		uint16_t mode	 					: 1;
		uint16_t reserved 					: 2;
	}condition_t;
	uint16_t condition_u16;
}mcu_condition_union;


typedef struct
{
	struct
	{
		struct
		{
			mcu_states_union states_union;
			uint8_t torque_limit_u8;
			int16_t torque_set_i16;
			int16_t torque_set_2_i16;
			int16_t speed_set_rpm_i16;
			uint16_t speed_limit_u16;
		}mcu;

		struct
		{

		}bms;

		struct
		{
			uint8_t steering_angle_u8;
		}telemetry;
	}tx;

	struct
	{
		//can rx paketleri buraya alinacak
		Bms_Datas bms_data;
		struct
		{
			int16_t set_torque_i16;
			float motor_rpm_f32;
			uint8_t motor_temp_u8;
			mcu_condition_union state;
			int8_t torque_i8;
		}mcu;

		struct
		{
			int16_t set_torque_i16;
			float motor_rpm_f32;
			uint8_t motor_temp_u8;
			mcu_condition_union state;
			int8_t torque_i8;
		}mcu_2;
	}rx;
}CAN_Buffers;
CAN_Buffers lyra_can_data;




void AESK_Read_All_Signals(void);
void Sys_On_Off_Handler(void);
void Ign_Handler(void);
void Reverse_Handler(void);
void Inc_Vel_Handler(void);
void Dec_Vel_Handler(void);
void TV_On_Off_Handler(void);
void Fake_Brk_Handler(void);
void Brake_Switch_Handler(void);
void Torque_Speed_Handler(void);
void vars_to_str(char *buffer, const char *format, ...);
int vsprintf (char * s, const char * format, va_list arg );
void VCU_Can_Config(void);
void Vcu_Data_Init(void);
void Nextion_Init(void);
void Send_Buf_2_Nextion(void);
void VCU_Can_Rx_Handler(uint32_t ExtId);
void Telemetry_Query_Answer(void);
void Beacon_Control(void);


#endif /* INC_AESK_VCU_GENERAL_H_ */
