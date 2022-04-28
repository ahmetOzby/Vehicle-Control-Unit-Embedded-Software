/*
 * AESK_VCU_Nextion.c
 *
 *  Created on: 11 Nis 2021
 *      Author: Ahmet
 */

#ifndef STM32_NEXTION_LIB_H_
#define STM32_NEXTION_LIB_H_

#include "main.h"
#include "string.h"
#include "stdlib.h"


#define SCREEN_Serial				huart1
#define RED_COLOR					"63488"
#define GREEN_COLOR					"2016"

#define EMS_SCREEN_ID				"EMS"


#define BMS_WAKE_PIC				"bms_wake"
#define MCU_WAKE_PIC				"mcu_wake"
#define IGN_PIC						"ign_wake"
#define VELOCITY					"velocity"
#define SET_VEL						"cruise"
#define SOC							"soc_label"
#define T1							"torque_set_1"
#define T2							"torque_set_2"
#define POWER						"power_data"
#define ODOMETER					"odometer_data"
#define CONSUMPTION					"cons_data"
#define WCV							"wcv"
#define BATTERY_TEMP				"b_temp"
#define BATTERY_VOLT				"b_volt"
#define BATTERY_CUR					"b_current"
#define ISO_POS						"iso_pos"
#define ISO_NEG						"iso_neg"
#define RIGH_MOTOR_TEMP				"rm_temp"
#define LEFT_MOTOR_TEMP				"lm_temp"
#define DIRECTION					"direction"
#define BMS_MCU_ERROR_BOX			"bms_errors"
#define VEHICLE_STATE_BOX			"vehicle_con"


#define FULL_BATTERY_ID				"1"
#define ZERO_VEL_ID					"2"
#define ZERO_CRUISE_ID				"3"
#define LOW_VEL_ID					"5"
#define HALF_VEL_ID					"6"
#define HIGH_VEL_ID					"7"
#define PASIVE_PIC_ID				"9"
#define ACTIVE_PIC_ID				"10"
#define LOW_CRUISE_ID				"11"
#define HALF_CRUISE_ID				"12"
#define HIGH_CRUISE_ID				"13"
#define HALF_BATTERY_ID				"14"
#define LOW_BATTERY_ID				"15"


	
#define CELL_1					"cell_1"
#define CELL_2					"cell_2"
#define CELL_3					"cell_3"
#define CELL_4					"cell_4"
#define CELL_5					"cell_5"
#define CELL_6					"cell_6"
#define CELL_7					"cell_7"
#define CELL_8					"cell_8"
#define CELL_9					"cell_9"
#define CELL_10					"cell_10"
#define CELL_11					"cell_11"
#define CELL_12					"cell_12"
#define CELL_13					"cell_13"
#define CELL_14					"cell_14"
#define CELL_15					"cell_15"
#define CELL_16					"cell_16"
#define CELL_17					"cell_17"
#define CELL_18					"cell_18"
#define CELL_19					"cell_19"
#define CELL_20					"cell_20"
#define CELL_21					"cell_21"
#define CELL_22					"cell_22"
#define CELL_23					"cell_23"
#define CELL_24					"cell_24"
#define CELL_25					"cell_25"
#define CELL_26					"cell_26"
#define CELL_27					"cell_27"
#define CELL_28					"cell_28"

#define SOC_1_ID					"x0"
#define SOC_2_ID					"x1"
#define SOC_3_ID					"x2"
#define SOC_4_ID					"x3"
#define SOC_5_ID					"x4"
#define SOC_6_ID					"x5"
#define SOC_7_ID					"x6"
#define SOC_8_ID					"x7"
#define SOC_9_ID					"x8"
#define SOC_10_ID					"x9"
#define SOC_11_ID					"x10"
#define SOC_12_ID					"x11"
#define SOC_13_ID					"x12"
#define SOC_14_ID					"x13"
#define SOC_15_ID					"x14"
#define SOC_16_ID					"x15"
#define SOC_17_ID					"x16"
#define SOC_18_ID					"x17"
#define SOC_19_ID					"x18"
#define SOC_20_ID					"x19"
#define SOC_21_ID					"x20"
#define SOC_22_ID					"x21"
#define SOC_23_ID					"x22"
#define SOC_24_ID					"x23"
#define SOC_25_ID					"x24"
#define SOC_26_ID					"x25"
#define SOC_27_ID					"x26"
#define SOC_28_ID					"x27"

#define TEMP_1						"temp_1"
#define TEMP_2						"temp_2"
#define TEMP_3						"temp_3"
#define TEMP_4						"temp_4"
#define TEMP_5						"temp_5"
#define TEMP_6						"temp_6"
#define TEMP_7						"temp_7"


#define FIRST_CONTROL_BYTE			0x07
#define SECOND_CONTROL_BYTE			0x29

#define PAGE_1						1
#define PAGE_2						2

#define MAIN_PAGE					1
#define CELLS_PAGE					2
typedef enum
{
	FIRST_BYTE_CONTROL = 0,
	SECOND_BYTE_CONTROL,
	PAGE_CONTROL,
	LAST_BYTE_CONTROL

}Nextion_Control_States;

typedef struct
{
	uint8_t rx_u8;
	uint8_t nextion_tx_buf[2000];
	uint8_t Screen_States_u8;
	uint8_t page;
	uint16_t screen_tx_buf_index;
}Screen_Data_Struct;
Screen_Data_Struct screen_data;


char * ftoa(double f, char * buf, int precision);
char* itoa(int value, char* buffer, int base);
void AESK_Nextion_Change_Pic(char* picID, char* newPicID);
void AESK_Nextion_Write_Text(char* textID, char* text);
void AESK_Nextion_Change_Bckgrnd_Color(char* textID, char* colorCode);
void AESK_Nextion_Change_Pco_Color(char* textID, char* colorCode);
void AESK_Nextion_Write_Num(char* numberID, int32_t number);
void AESK_Nextion_Write_Text(char* textID, char* text);

HAL_StatusTypeDef HAL_UART_DMA_Tx_Stop(UART_HandleTypeDef *huart);



#endif /* STM32_NEXTION_LIB_H_ */
