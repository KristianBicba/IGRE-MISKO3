/*
 * SCI.h
 *
 *  Created on:29.3.2022
 *      Author: Rok Plesko
 */

#ifndef INCLUDE_SCI_H_
#define INCLUDE_SCI_H_



// ----------- Include other modules (for public) -------------

// Vključimo nizko-nivojsko LL knjižnjico, da dobimo podporo za delo z USART vmesnikom.

	// DOPOLNI done
#include "stm32g4xx_ll_usart.h"
#include <stdio.h>			// dodamo printf() funkcionalnost

// Pri implementaciji sistemskih funkcij serijskega vmesnika SCI bomo potrebovali sledeče nizko-nivojske funkcije:
//
//	// DOPOLNI
//	LL_USART_IsActiveFlag_RXNE  LL_USART_IsActiveFlag_RXNE_RXFNE
//	uint32_t LL_USART_IsActiveFlag_TXE_TXFNF(USART_TypeDef *USARTx)
//	__STATIC_INLINE void LL_USART_TransmitData8 (USART_TypeDef * USARTx, uint8_t Value)
//	uint8_t LL_USART_ReceiveData8(USART_TypeDef *USARTx)

// Iz zgornjega seznama funkcij lahko vidimo, da bo potrebno v "handle" strukturi SCI vmesnika hraniti
// samo en parameter, ki določa, katero USARTx periferno enoto bomo uporabili.

//			DOPOLNI seznam z novimi LL funkcijami,
//				s pomočjo katerih uporabljamo USART prekinitve
//
//
//
//
//


// -------------------- Public definitions --------------------


// "Javno" definirajmo še pomožne "vrnjene kode" (angl. return codes), s katerimi bodo
// SCI funkcije sporočale, ali je bila operacija uspešna ali ne.
// Uporabimo naštevni tip.
typedef enum { SCI_ERROR=-1, SCI_NO_ERROR=0} SCI_rtrn_codes_t;



// ---------  Dodati je potrebno PROTOTIPE za nove funkcije za delo s pomočjo prekinitev:

// SCI modulu bomo namreč dodali novo funkcionalnost, ki temelji na uporabi prekinitev.
// Dodali bomo funkciji za pošiljanje niza znakov oziroma skupine binarnih podatkov:
//
//		- SCI_send_string_IT
//		- SCI_send_bytes_IT
//
// Mimogrede, končnica "_IT" v imenu funkcij namiguje, da sta ti dve funkciji implementirani
// s pomočjo prekinitev.

	// DOPOLNI s prototipi funkcij



// SCI modulu pa moramo tudi dodati t.i. "callback" funkciji:
//
//		- SCI_receive_char_Callback
//		- SCI_transmit_char_Callback
//
// "Callback" funkcije so funkcije, ki jih polličemo, ko se zgodi ustrezna prekinitev,
// in pravzaprav realizirajo vsebino prekinitvene rutine.
//
// Prvo funkcijo bomo s pomočjo prekinitev poklicali vsakič, ko bo USART sprejemnik
// sprejel nov podatek in bo poskrbela, da se ta podatek shrani v
// sprejemni RX sistemski medpomnilnik.
//
// Drugo funkcijo pa bomo s pomočjo prekinitev poklicali vsakič, ko bo USART oddajnik
// pripravljen sprejeti nov podatek za pošiljanje. Ta funkcija bo poskrbela, da se
// pošljejo vsi podatki, ki čakajo v oddajnem TX sistemskem medpomnilniku.


	// DOPOLNI s prototipi funkcij

//Prototipi funkcij za operiranje s pomočjo interuptov:
 //VERJETNO KAKŠNI VHODNI ALI VRAČANI PARAMETRI NISO PRAVILNI
void SCI_send_string_IT (char *str);
void SCI_send_bytes_IT (uint8_t *data, uint32_t size);
void SCI_receive_char_Callback (void);
void SCI_transmit_char_Callback (void);

void SCI_demo_receive_with_interrupts(void);
void SCI_demo_transmit_with_interrupts(void);
void SCI_demo_echo_with_interrupts(void);

// ---------------- Public function prototypes ----------------

void SCI_init(void);

void SCI_send_char(char c);
void SCI_send_byte(uint8_t data);

uint8_t SCI_is_data_waiting(void);

SCI_rtrn_codes_t SCI_read_char(char *c);
SCI_rtrn_codes_t SCI_read_byte(uint8_t *data);


void SCI_send_string(char *str);
void SCI_send_bytes(uint8_t *data, uint32_t size);


void SCI_demo_Hello_world(void);
void SCI_demo_echo_with_polling(void);
void SCI_demo_Hello_world_printf(void);


#endif /* INCLUDE_SCI_H_ */
