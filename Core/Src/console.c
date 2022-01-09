/*
 * console.c
 *
 *  Created on: Oct 22, 2021
 * @Author: User
 * @file           : console.c
 * @brief          : fichier servant à la création de la console
 */
/*
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "console.h"

char uart_rx_buffer[UART_RX_BUFFER_SIZE];
char uart_tx_buffer[UART_TX_BUFFER_SIZE];
int it_uart_rx_ready=0;
int idxCmd = 0;
int valCmd[4];

const uint8_t starting[32] = {"\r\n\r\n---Jour---\r\n"};
const uint8_t prompt[32] = {"<user>@Nucleo-G431 >> "};
char cmd[CMD_BUFFER_SIZE];

const uint8_t help[128] = {"\r\n pinout:contient la liste des pin utilisées\n on: contient le message d'allumage du moteur \n off: contient le message d'extinction du moteur\r\n"};
const uint8_t pinout[];
const uint8_t power_on[CMD_BUFFER_SIZE]={"\r\nLancement du Moteur\r\n"};
const uint8_t power_off[CMD_BUFFER_SIZE]={"\r\nArret du Moteur\r\n"};
const uint8_t not_found[CMD_BUFFER_SIZE]={"\r\nCommande inconnue\r\n"};

char backspace[] = " \b";


void Uartprint(char * s){
	sprintf((char *)uart_tx_buffer,"%s",s);
	HAL_UART_Transmit(&huart2, uart_tx_buffer, strlen(s), HAL_MAX_DELAY);
}

void Uartreceive(){
	HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  it_uart_rx_ready=1;
}

void echo(){
	if(it_uart_rx_ready==1){
		uart_tx_buffer[0]=uart_rx_buffer[0];
		HAL_UART_Transmit(&huart2, uart_tx_buffer, 1, HAL_MAX_DELAY);
		it_uart_rx_ready=0;
	}
}

void Console(){
	Uartreceive();
	if (it_uart_rx_ready==1) {
		echo();
		cmd[idxCmd++]=uart_rx_buffer[0];
		if (uart_rx_buffer[0]=='\r') {
			cmd[idxCmd-1]='\0';
			if(CprCommande(cmd)){
				idxCmd=0;
			}
		}
		else if (uart_rx_buffer[0]=='\b') {
			if (idxCmd>0) {      //is there a char to delete?
				idxCmd=idxCmd-2;          //remove it in buffer
				Uartprint(backspace);
			}
		}
	}
}

int CprCommande(char * c){
	if(strcmp(c,"help")==0) {
		Uartprint((char *) help);
		Uartprint((char *) prompt);
		return 1;
	}
	else if(strcmp(c,"on")==0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,0);
		for (int i=0;i<25;i++);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
		Uartprint((char *) power_on);
		Uartprint((char *) prompt);
		return 1;
	}
	else if(strcmp(c,"off")==0) {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,1);
		Uartprint((char *) power_off);
		Uartprint((char *) prompt);
		return 1;
	}
	else if(strcmp(c,"pinout")==0) {
		Uartprint((char *) pinout);
		Uartprint((char *) prompt);
		return 1;
	}
	else if(strcmp(c,"state")==0) {
		sprintf((char *)uart_tx_buffer,"\r\nARR = %d ",htim1.Instance->ARR);
		HAL_UART_Transmit(&huart2, uart_tx_buffer, 14, HAL_MAX_DELAY);
		sprintf((char *)uart_tx_buffer,"\r\nCCR1 = %d ",htim1.Instance->CCR1);
		HAL_UART_Transmit(&huart2, uart_tx_buffer, 14, HAL_MAX_DELAY);
		sprintf((char *)uart_tx_buffer,"\r\nCCR2 = %d \r\n",htim1.Instance->CCR2);
		HAL_UART_Transmit(&huart2, uart_tx_buffer, 14, HAL_MAX_DELAY);
		Uartprint((char *) prompt);
		return 1;
	}
	else if(strncmp(c,"speedLisse",10)==0) {
		int al=(atoi(c+10));
		if (al<=0) al=1;
		if (al>=100) al=99;
		int oldA=100*htim1.Instance->CCR1/htim1.Instance->ARR;
		int alT=oldA;
		if (al>oldA) {
			while (al-alT>0 & alT!=1 & alT!=99) {
				alT++;
				if (alT<=0) al=1;
				if (alT>=100) al=99;
				htim1.Instance->CCR1=alT*htim1.Instance->ARR/100;
				htim1.Instance->CCR2=(100-alT)*htim1.Instance->ARR/100;
				HAL_Delay(100);
			}
		}
		else if (al<oldA) {
			while (alT-al>0 & alT!=1 & alT!=99) {
				alT--;
				if (alT<=0) al=1;
				if (alT>=100) al=99;
				htim1.Instance->CCR1=alT*htim1.Instance->ARR/100;
				htim1.Instance->CCR2=(100-alT)*htim1.Instance->ARR/100;
				HAL_Delay(100);
			}
		}
		Uartprint("\r\n");
		Uartprint((char *) prompt);
		return 1;
	}
	else if(strncmp(c,"speed",5)==0) {
		int al=(atoi(c+5));
		if (al<=0) al=1;
		if (al>=100) al=99;
		htim1.Instance->CCR1=al*htim1.Instance->ARR/100;
		htim1.Instance->CCR2=(100-al)*htim1.Instance->ARR/100;
		Uartprint("\r\nAjustement du pulse\r\n");
		Uartprint((char *) prompt);
		return 1;
	}
	else{
		Uartprint((char *) not_found);
		Uartprint((char *) prompt);
		return 1;
	}

}

*/



