/*
 * gps.h
 *
 *  Created on: Jan 31, 2023
 *      Author: ethan
 */

#include "stm32h7xx_hal.h"
#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "gps.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;

#define Neo6M_UART &huart2;


uint8_t Rxdata[750];
char Txdata[750];
char GPS_Payyload[100];
uint8_t Flag = 0;
static int Msgindex;
char *ptr;
float time, Latitude, Longitude;
int Hours, Min, Sec;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	//abstract function from HAL library
	Flag = 1;	//When UART RX is complete, triggers this and sets flag to one
}

void get_location(void){

	//HAL_UART_Transmit(&huart2,(uint8_t*)Txdata,strlen(Txdata),HAL_MAX_DELAY);

	if(Flag == 1){
		Msgindex=0;
		strcpy(Txdata, (char*)(Rxdata));
		ptr=strstr(Txdata, "GPRMC");
		if(*ptr == 'G'){
			while(1){
				GPS_Payyload[Msgindex]= *ptr;
				Msgindex++;
				*ptr=*(ptr+Msgindex);
				if(*ptr == '\n'){
					GPS_Payyload[Msgindex]='\0';
					break;
				}
			}
			sscanf(GPS_Payyload, "GPRMC, %f,A,%f,N,%f", &time, &Latitude, &Longitude);
			Format_data(time, Latitude, Longitude);
			HAL_Delay(1);
			Flag=0;
		}
	}
}

void Format_data(float Time, float Lat, float Long){
	char Data[100];
	Hours=(int)Time/1000;
	Min=(int)(Time - (Hours*10000))/100;
	Sec=(int)(Time-((Hours*10000)+(Min*100)));
	sprintf(Data, "\r\n Lat=%f, Long=%f",Latitude,Longitude);
	HAL_UART_Transmit(&huart2,(uint8_t*)Data,strlen(Data),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n\n",3,HAL_MAX_DELAY);
}



