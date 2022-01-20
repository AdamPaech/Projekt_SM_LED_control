/*
 * bh_1750.c
 *
 *  Created on: Nov 13, 2021
 *      Author: adamp
 */

#include "bh_1750.h"

void BH1750_Init(BH1750_HandleTypeDef* hbh1750)
{
	uint8_t a = 0;

	a = BH1750_POWER_ON;
	HAL_StatusTypeDef try = HAL_I2C_Master_Transmit(hbh1750->I2C, hbh1750->Address , &a, BH1750_COMMAND_SIZE, hbh1750->Timeout);

	a = BH1750_CONTINOUS_H_RES_MODE;
	HAL_I2C_Master_Transmit(hbh1750 -> I2C, hbh1750->Address , &a, BH1750_COMMAND_SIZE, hbh1750->Timeout);
}

float BH1750_ReadLux(BH1750_HandleTypeDef* hbh1750)
{
	uint8_t rxarray[BH1750_DATA_SIZE];

	HAL_StatusTypeDef dupa = HAL_I2C_Master_Receive(hbh1750->I2C, hbh1750->Address, rxarray, BH1750_DATA_SIZE, hbh1750->Timeout);

	return ((rxarray[BH1750_DATA_MSB]<<8) | rxarray[BH1750_DATA_LSB]) / 1.2; // '|' sumuje oba wyrazenia binarne

}
