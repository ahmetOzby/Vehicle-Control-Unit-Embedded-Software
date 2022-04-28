/*
 * AESK_Data_Pack_lib.h
 *
 *  Created on: 17 Eki 2019
 *      Author: yemrelaydin
 */

#ifndef AESK_DATA_PACK_LIB_H_
#define AESK_DATA_PACK_LIB_H_
#include "stdint.h"

#define FLOAT_CONVERTER_1 			10
#define FLOAT_CONVERTER_2 			100
#define FLOAT_CONVERTER_3 			1000

/* PACKAGE FUNCTIONS */
void AESK_UINT16toUINT8_LE(uint16_t *packData, uint8_t *packBuf, uint16_t *index);
void AESK_UINT16toUINT8_BE(uint16_t *packData, uint8_t *packBuf, uint16_t *index);

void AESK_INT16toUINT8_LE(int16_t *packData, uint8_t *packBuf, uint16_t *index);
void AESK_INT16toUINT8_BE(int16_t *packData, uint8_t *packBuf, uint16_t *index);

void AESK_UINT32toUINT8_LE(uint32_t *packData, uint8_t *packBuf, uint16_t *index);
void AESK_UINT32toUINT8_BE(uint32_t *packData, uint8_t* packBuf, uint16_t *index);

void AESK_INT32toUINT8_LE(int32_t *packData, uint8_t *packBuf, uint16_t *index);
void AESK_INT32toUINT8_BE(int32_t *packData, uint8_t* packBuf, uint16_t *index);

void AESK_FLOAT32toUINT8_LE(float *packData, uint8_t *packBuf, uint16_t *index);
void AESK_FLOAT32toUINT8_BE(float *packData, uint8_t *packBuf, uint16_t *index);

void AESK_FLOAT64toUINT8_LE(double *packData, uint8_t *packBuf, uint16_t *index);
void AESK_FLOAT64toUINT8_BE(double *packData, uint8_t *packBuf, uint16_t *index);

void AESK_UINT8toUINT8CODE(uint8_t *packData, uint8_t *packBuf, uint16_t *index);
void AESK_INT8toUINT8CODE(int8_t *packData, uint8_t *packBuf, uint16_t *index);


/*ENCODE FUNCTIONS */
void AESK_UINT8toINT16_LE(int16_t *packData, uint8_t* packBuf, uint16_t *index);
void AESK_UINT8toINT16_BE(int16_t *packData, uint8_t* packBuf, uint16_t *index);

void AESK_UINT8toUINT16_LE(uint16_t *packData, const uint8_t* packBuf, uint16_t *index);
void AESK_UINT8toUINT16_BE(uint16_t *packData, const uint8_t* packBuf, uint16_t *index);

void AESK_UINT8toINT32_LE(int32_t *packData, uint8_t* packBuf, uint16_t *index);
void AESK_UINT8toINT32_BE(int32_t *packData, uint8_t* packBuf, uint16_t *index);

void AESK_UINT8toUINT32_BE(uint32_t *packData, uint8_t *packBuf, uint16_t *index);
void AESK_UINT8toUINT32_LE(uint32_t *packData, uint8_t *packBuf, uint16_t *index);

void AESK_UINT8toUINT8ENCODE(uint8_t *packData, const uint8_t *packBuf, uint16_t *index);
void AESK_UINT8toINT8ENCODE(int8_t *packData, const uint8_t *packBuf, uint16_t *index);

void AESK_UINT8toFLOAT32_LE(float *packData, const uint8_t *packBuf, uint16_t *index);
void AESK_UINT8toFLOAT32_BE(float *packData, uint8_t *packBuf, uint16_t *index);

void AESK_UINT8toFLOAT64_LE(double *packData, uint8_t *packBuf, uint16_t *index);
void AESK_UINT8toFLOAT64_BE(double *packData, uint8_t *packBuf, uint16_t *index);

void AESK_UINT16toFLOAT_BE(float *packData, uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index);
void AESK_UINT16toFLOAT_LE(float *packData, uint8_t const *packBuf, uint16_t CONVERTER, uint16_t *index);

void AESK_INT16toFLOAT_BE(float *packData, uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index);
void AESK_INT16toFLOAT_LE(float *packData, const uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index);
#endif /* AESK_DATA_PACK_LIB_H_ */
