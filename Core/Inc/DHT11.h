/*
 * DHT11.h
 *
 *  Created on: Nov 9, 2021
 *      Author: oxford
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "main.h"
#include "dwt_stm32_delay.h"

#define DHT11_PORT GPIOD
#define DHT11_PIN GPIO_PIN_0

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

/*float Temperature = 0;
float Humidity = 0;*/
uint8_t isItHere;

//void delayinginus (uint16_t time);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);
void DHT11_TempAndHumidity(uint32_t *Temperature, uint32_t *Humidity);

#endif /* INC_DHT11_H_ */
