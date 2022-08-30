/*
 * periodic_services.c
 *
 *  Created on: Apr 12, 2022
 *      Author: Rok Plesko
 */

//Periodic services omogoča periodično izvajanje s pomočjo prekinitev ki jih prozi timer6

// ----------- Include other modules (for private) -------------


#include "periodic_services.h"
#include "timing_utils.h"
#include "stm32g474xx.h"
#include "stm32g4xx_ll_tim.h"
#include "kbd.h"
#include "joystick.h"

//	Handle struktura
typedef struct
{
	TIM_TypeDef *Timer;
} periodic_services_handle_t;


//	Privatna globalna instanca strukture
periodic_services_handle_t PService;



//	Funkcije
void PSERV_init(void) //i
{
	PService.Timer = TIM6;
	LL_TIM_EnableCounter (PService.Timer);
}



void PSERV_enable(void)  //ii
{
	LL_TIM_EnableIT_UPDATE (PService.Timer);
}


void PSERV_disable(void)  //iii
{
	LL_TIM_DisableIT_UPDATE (PService.Timer);
}

void PSERV_run_services_Callback(void) //iv
{
	KBD_scan();
	//KBD_demo_toggle_LEDs_if_buttons_pressed();
	JOY_scan_button();
}





