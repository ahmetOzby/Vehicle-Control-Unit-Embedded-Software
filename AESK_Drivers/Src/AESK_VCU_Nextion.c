/*
 * AESK_VCU_Nextion.c
 *
 *  Created on: 11 Nis 2021
 *      Author: Ahmet
 */

#include "AESK_VCU_Nextion.h"

extern UART_HandleTypeDef huart1;

#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

void AESK_Nextion_Change_Pic(char* picID, char* newPicID)
{
	uint8_t temp_buf[50];
	uint8_t enderArray[4] = {0xff, 0xff, 0xff, '\0'};
	strcpy((char *)temp_buf, picID);
	strcat((char *)temp_buf, ".pic=");
	strcat((char *)temp_buf, newPicID);
	strcat((char*)temp_buf, (char*)enderArray);
	strcat((char*)screen_data.nextion_tx_buf, (char *)temp_buf);
	screen_data.screen_tx_buf_index += strlen((char*)temp_buf);
}

void AESK_Nextion_Change_Bckgrnd_Color(char* textID, char* colorCode)
{
	uint8_t temp_buf[50];
	uint8_t enderArray[4] = {0xff, 0xff, 0xff, '\0'};
	strcpy((char *)temp_buf, textID);
	strcat((char *)temp_buf, ".bco=");
	strcat((char *)temp_buf, colorCode);
	strcat((char*)temp_buf, (char*)enderArray);
	strcat((char*)screen_data.nextion_tx_buf, (char *)temp_buf);
	screen_data.screen_tx_buf_index += strlen((char*)temp_buf);
}

void AESK_Nextion_Change_Pco_Color(char* textID, char* colorCode)
{
	uint8_t temp_buf[50];
	uint8_t enderArray[4] = {0xff, 0xff, 0xff, '\0'};
	strcpy((char *)temp_buf, textID);
	strcat((char *)temp_buf, ".bco=");
	strcat((char *)temp_buf, colorCode);
	strcat((char*)temp_buf, (char*)enderArray);
	strcat((char*)screen_data.nextion_tx_buf, (char *)temp_buf);
	screen_data.screen_tx_buf_index += strlen((char*)temp_buf);
}

void AESK_Nextion_Write_Num(char* numberID, int32_t number)
{
	uint8_t temp_buf[50];
	uint8_t enderArray[4] = {0xff, 0xff, 0xff, '\0'};
	char ntos[10];
	strcpy((char *)temp_buf, numberID);
	strcat((char *)temp_buf, ".val=");
	itoa(number, ntos, 10);
	strcat((char *)temp_buf, ntos);
	strcat((char*)temp_buf, (char*)enderArray);
	strcat((char*)screen_data.nextion_tx_buf, (char *)temp_buf);
	screen_data.screen_tx_buf_index += strlen((char*)temp_buf);
};

void AESK_Nextion_Write_Text(char* textID, char* text)
{
	uint8_t temp_buf[50];
	uint8_t enderArray[4] = {0xff, 0xff, 0xff, '\0'};
	strcpy((char *)temp_buf, textID);
	strcat((char *)temp_buf, ".txt=\"");
	strcat((char *)temp_buf, text);
	strcat((char *)temp_buf, "\"");
	strcat((char*)temp_buf, (char*)enderArray);
	strcat((char*)screen_data.nextion_tx_buf, (char *)temp_buf);
	screen_data.screen_tx_buf_index += strlen((char*)temp_buf);
}

char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

//inline void, void olarak değiştirildi
void swap(char *x, char *y) {
	char t = *x; *x = *y; *y = t;
}

// function to reverse buffer[i..j]
char* reverse(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]);

	return buffer;
}

// Iterative function to implement itoa() function in C
char* itoa(int value, char* buffer, int base)
{
	// invalid input
	if (base < 2 || base > 32)
		return buffer;

	// consider absolute value of number
	int n = abs(value);

	int i = 0;
	while (n)
	{
		int r = n % base;

		if (r >= 10) 
			buffer[i++] = 65 + (r - 10);
		else
			buffer[i++] = 48 + r;

		n = n / base;
	}

	// if number is 0
	if (i == 0)
		buffer[i++] = '0';

	// If base is 10 and value is negative, the resulting string 
	// is preceded with a minus sign (-)
	// With any other base, value is always considered unsigned
	if (value < 0 && base == 10)
		buffer[i++] = '-';

	buffer[i] = '\0'; // null terminate string

	// reverse the string and return it
	return reverse(buffer, 0, i - 1);
}









