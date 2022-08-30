/*
 * periodic_services.h
 *
 *  Created on: Apr 12, 2022
 *      Author: Rok Plesko
 */

#ifndef PERIODIC_SERVICES_H_
#define PERIODIC_SERVICES_H_


// Pri implementaciji modula z orodji za merjenje časa bomo potrebovali sledeče HAL funkcije:
//
//    HAL_GetTick()
//    LL_TIM_EnableCounter (TIM_TypeDef * TIMx)
//    LL_TIM_EnableIT_UPDATE (TIM_TypeDef * TIMx)
//    LL_TIM_DisableIT_UPDATE (TIM_TypeDef * TIMx)
//    LL_TIM_IsEnabledIT_UPDATE (TIM_TypeDef * TIMx)
//    LL_TIM_IsActiveFlag_UPDATE (TIM_TypeDef * TIMx)
//    LL_TIM_ClearFlag_UPDATE (TIM_TypeDef * TIMx)
//


// ----------- Include other modules (for public) -------------



#endif /* PERIODIC_SERVICES_H_ */


void PSERV_init(void);

void PSERV_enable(void);
void PSERV_disable(void);

void PSERV_run_services_Callback(void);
