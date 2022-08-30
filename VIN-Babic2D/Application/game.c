/*
 * game.c
 *
 *  Created on: 18.5.2022
 *      Author: Rok Plesko
 */


/* **************** MODULE DESCRIPTION *************************

Ta modul implementira algoritem igrice s pomoÄjo avtomatov stanj.
Algoritem igre za svoje delovanje potrebuje podporo sistemskih
modulov za delo z vhodno-izhodnimi napravami sistema ter podporo
aplikacijskih modulov za delo z objekti aplikacije ter podporo
za izris objekotv na zaslon.

************************************************************* */



// ----------- Include other modules (for private) -------------



// ------ Standard C libraries -----
#include <stdio.h>
#include <stdlib.h>		// support for random number generation


// POGLEJTE: igra (tj. aplikacija) je odvisna le od modulov s
// sistemskega nivoja, ne pa tudi od modulov s strojnega nivoja!


// --- Moduli sistemskega nivoja ---


#include "LED.h"				// podpora za delo z LEDicami
#include "kbd.h"				// podpora za delo s tipkovnico
#include "SCI.h"				// podpora za delo s serijskim vmesnikom
#include "joystick.h"			// podpora za delo z joystickom
#include "lcd.h"				// podpora za delo z LCDjem
#include "timing_utils.h"		// podpora za delo z orodji za merjenje Äasa
#include "ugui.h"


// --- Moduli applikacijskega nivoja ----

#include "game.h"			// lastne definicije game.c modula

#include "objects.h"			// v igro vkljuÄimo modul za delo z objekti igre
#include "graphics.h"			// v igro vkljuÄimo modul za prikaz objektov na zaslon



// ---------------------- Private definitions ------------------

void GamePlay_Update_Aiming_LED_Indicator(void);

// ----- Definicija stanj avtomatov  --------


// stanja avtomata Game()
typedef enum GAME_states {GAME_INTRO_STATE, GAME_PLAY_STATE, GAME_OVER_STATE} GAME_states_t;

// stanja avtomata Intro()
typedef enum INTRO_states {INTRO_INIT, INTRO_WAIT_BEFORE_KBD_ACTIVE, INTRO_PRESS_ANY_KEY, INTRO_WAIT_FOR_ANY_KEY, INTRO_LEDS_OFF} INTRO_states_t;

// stanja avtomata GamePlay()
typedef enum GAMEPLAY_states {	GAMEPLAY_INIT,
								GAMEPLAY_COUNTDOWN_3,
								GAMEPLAY_COUNTDOWN_2,
								GAMEPLAY_COUNTDOWN_1,
								GAMEPLAY_COUNTDOWN_CLEAR,
								GAMEPLAY_START_MEASURING_GAMEPLAY_TIME,
								GAMEPLAY_SPAWN_TARGET,
								GAMEPLAY_SHOOT_TARGET,
								GAMEPLAY_BEFORE_GAME_OVER
} GAMEPLAY_states_t;

// stanja avtomata GameOver()
typedef enum GAMEOVER_states {GAMEOVER_SCREEN, GAMEOVER_WAIT_BEFORE_LEDS_OFF, GAMEOVER_LEDS_OFF, GAMEOVER_WAIT_FOR_ANY_KEY} GAMEOVER_states_t;




// ------------- Public function implementations --------------


// Funkcija, ki implementira nad-avtomat Game().
void Game()
{

	static GAME_states_t state = GAME_INTRO_STATE;



	uint8_t exit_value = 0;



	switch (state)
	{

		case GAME_INTRO_STATE:


			exit_value = Intro();


			if (exit_value != 0)
				state = GAME_PLAY_STATE;

		break;



		case GAME_PLAY_STATE:

			// Znotraj tega stanja izvajamo pod-avtomat GamePlay().
			exit_value = GamePlay();


			if (exit_value != 0)
				state = GAME_OVER_STATE;

		break;





		case GAME_OVER_STATE:

			// Znotraj tega stanja izvajamo pod-avtomat GameOver().
			exit_value = GameOver();


			if (exit_value != 0)
				state = GAME_INTRO_STATE;

		break;





		default:


			printf("Game(): Error - undefined state (%d)", state);


			HAL_Delay(5000);


			state = GAME_INTRO_STATE;
			exit_value = 0;

		break;
	}
}




// Funkcija, ki implementira pod-avtomat Intro().
uint8_t Intro()
{

	// Definicija makro parametrov za pod-avtomat Intro().
	#define INTRO_DELAY_BEFORE_KBD_ACTIVE	0



	static INTRO_states_t state = INTRO_INIT;



	uint8_t exit_value = 0;


	buttons_enum_t	key = BTN_NONE;



	static stopwatch_handle_t    stopwatch;





	switch (state)
	{

		case INTRO_INIT:

			// Opravila stanja.
			OBJ_init();

			LEDs_write(255);
			//OBJ_init_splash_screen();




			TIMUT_stopwatch_set_time_mark(&stopwatch);


			state = INTRO_WAIT_BEFORE_KBD_ACTIVE;
			exit_value = 0;

		break;





		case INTRO_WAIT_BEFORE_KBD_ACTIVE:


			if ( TIMUT_stopwatch_has_X_ms_passed(&stopwatch, INTRO_DELAY_BEFORE_KBD_ACTIVE) )
			{



				KBD_flush();


				state = INTRO_PRESS_ANY_KEY;
				exit_value = 0;

			}

		break;



		// Izpis "press any key".
		case INTRO_PRESS_ANY_KEY:

			// Opravila stanja.

			GFX_draw_gfx_object(&splash_screen);
			GFX_draw_one_gfx_object_on_background(&press_any_key_sprite, &background);

			state = INTRO_WAIT_FOR_ANY_KEY;
			exit_value = 0;

		break;




		case INTRO_WAIT_FOR_ANY_KEY:


			key = KBD_get_pressed_key();


			if ( key != BTN_NONE )
			{


				state = INTRO_LEDS_OFF;
				exit_value = 0;

			}

		break;




		// Ugasnitev vseh LEDic
		case INTRO_LEDS_OFF:

			// Opravila stanja.
			LEDs_write(0);


			state = INTRO_INIT;
			exit_value = 1;

		break;







		default:


			printf("Intro(): Error - unknown state (%d)", state);


			HAL_Delay(5000);

			// In na koncu avomat restiramo tako, da ga postavimo v zaÄetno stanje.
			state = INTRO_INIT;
			exit_value = 0;

		break;


	}



	return exit_value;

}









uint8_t GamePlay()
{


	static GAMEPLAY_states_t state = GAMEPLAY_INIT;

	uint8_t exit_value = 0;



	buttons_enum_t	key = BTN_NONE;

	static stopwatch_handle_t   stopwatch;





	void GamePlay_UpdateChanges(void)
	{

		// Spremenljivka za implementacijo ure štoparice.
		static stopwatch_handle_t   update_stopwatch;



		if ( TIMUT_stopwatch_has_another_X_ms_passed( &update_stopwatch, settings.game_play_update_period) )
		{

			GFX_update_moving_gfx_object_location(&target.gfx_object);


			OBJ_crosshair_set_center_location_with_joystick();

			OBJ_crosshair_update_distance_to_target();

			 OBJ_set_timeout_bar_value(TIMUT_get_stopwatch_elapsed_time(&stopwatch));

			 OBJ_set_score_text_value(game_status.score);





			 GFX_draw_two_gfx_objects_on_background(&crosshair.gfx_object, &target.gfx_object, &background);


			 GFX_display_text_object(&score_text);

			 GFX_display_progress_bar(&timeout_bar);


			 GamePlay_Update_Aiming_LED_Indicator();
		}

	}





	switch (state)
	{


		case GAMEPLAY_INIT:

			MATH_init_random_generator();
			GFX_draw_gfx_object(&background);
			GFX_display_progress_bar(&timeout_bar);
			GFX_display_text_object(&score_box_title); //maybe
			GFX_display_text_object(&score_text);

			state = GAMEPLAY_COUNTDOWN_3;
			exit_value = 0;

		break;

		case GAMEPLAY_COUNTDOWN_3:


					TIMUT_stopwatch_set_time_mark(&stopwatch);
					GFX_draw_one_gfx_object_on_background(&countdown_digit_3_sprite, &background);


					state = GAMEPLAY_COUNTDOWN_2;
					exit_value = 0;

				break;

		case GAMEPLAY_COUNTDOWN_2:


					if(TIMUT_stopwatch_has_another_X_ms_passed(&stopwatch, 1000))
					{
						GFX_clear_gfx_object_on_background(&countdown_digit_3_sprite, &background);
						GFX_draw_one_gfx_object_on_background(&countdown_digit_2_sprite, &background);

						state = GAMEPLAY_COUNTDOWN_1;
					}
						exit_value = 0;

				break;

		case GAMEPLAY_COUNTDOWN_1:

					if(TIMUT_stopwatch_has_another_X_ms_passed(&stopwatch, 1000))
					{
						GFX_clear_gfx_object_on_background(&countdown_digit_2_sprite, &background);
						GFX_draw_one_gfx_object_on_background(&countdown_digit_1_sprite, &background);

						state = GAMEPLAY_COUNTDOWN_CLEAR;
						exit_value = 0;
					}

				break;

		case GAMEPLAY_COUNTDOWN_CLEAR:


					if(TIMUT_stopwatch_has_another_X_ms_passed(&stopwatch, 1000))
					{
						GFX_clear_gfx_object_on_background(&countdown_digit_1_sprite, &background);



						state = GAMEPLAY_START_MEASURING_GAMEPLAY_TIME;
						exit_value = 0;
					}
				break;

		case GAMEPLAY_START_MEASURING_GAMEPLAY_TIME:


			TIMUT_stopwatch_set_time_mark(&stopwatch);


					state = GAMEPLAY_SPAWN_TARGET;
					exit_value = 0;

				break;

		case GAMEPLAY_SPAWN_TARGET:


				OBJ_spawn_target();
				KBD_flush();
				GFX_draw_one_gfx_object_on_background(&target.gfx_object, &background);

				state = GAMEPLAY_SHOOT_TARGET;
				exit_value = 0;

				break;


		case GAMEPLAY_SHOOT_TARGET:


				if(TIMUT_stopwatch_has_another_X_ms_passed(&stopwatch, settings.game_play_update_period))
					{
					GamePlay_UpdateChanges();
					}

				key = KBD_get_pressed_key();

				if( key != BTN_NONE )
				{
					if(OBJ_is_crosshair_on_target())
					{
						game_status.score += target.points;
						state = GAMEPLAY_SPAWN_TARGET;
						GFX_clear_gfx_object_on_background(&target.gfx_object, &background);

					}
					else
					{
						game_status.score += settings.missed_shot_penalty;
					}

				}



				if(TIMUT_stopwatch_has_X_ms_passed(&stopwatch, settings.game_play_time))
				{
					// Določimo naslednje stanje ter "exit" vrednost.
					state = GAMEPLAY_BEFORE_GAME_OVER;
					exit_value = 0;
				}
				break;







		// Varovalka za nedefinirana stanja.
		default:

			// Javimo vir napake in vrednost nedefiniranega stanja, v katerem se je našel avtomat.
			printf("GamePlay(): Error - unknown state (%d)", state);

			// Ustavimo izvajanje z namenom, da razvojnik lahko opazi neobičajno obnašanje aplikacije.
			HAL_Delay(5000);

			// In na koncu avomat restiramo tako, da ga postavimo v začetno stanje.
			state = GAMEPLAY_INIT;
			exit_value = 0;

		break;

	}


	// Vrnemo "exit value" pod-avtomata.
	return exit_value;


}



// Funkcija, ki implementira pod-avtomat GameOver().
uint8_t GameOver()
{

	static GAMEOVER_states_t state = GAMEOVER_SCREEN;

	uint8_t exit_value = 0;
	stopwatch_handle_t    stopwatch;


	switch (state)
		{

			case GAMEOVER_SCREEN:
				//opravila stanja
				GFX_draw_one_gfx_object_on_background(&game_over_sprite, &background);

				LEDs_on(0xFF);


				state = GAMEOVER_WAIT_BEFORE_LEDS_OFF;
				exit_value = 0;

			break;

			case GAMEOVER_WAIT_BEFORE_LEDS_OFF:
				// Opravila stanja.
				KBD_flush();

				LEDs_on(0xFF);
				TIMUT_stopwatch_set_time_mark(&stopwatch);

				if(TIMUT_stopwatch_has_X_ms_passed(&stopwatch, 3000))
				{
				  state = GAMEOVER_LEDS_OFF;
				  exit_value = 0;
				}

			break;


			case GAMEOVER_LEDS_OFF:
				LEDs_off(0xFF);
				  state = GAMEOVER_WAIT_FOR_ANY_KEY;
				  exit_value = 0;
				  TIMUT_stopwatch_set_time_mark(&stopwatch);
				  KBD_flush();
			break;


			case GAMEOVER_WAIT_FOR_ANY_KEY:
				if(TIMUT_stopwatch_has_X_ms_passed(&stopwatch, 10000) || KBD_get_pressed_key() == BTN_NONE)
				{
					//cas 10 s je pretekel
					exit_value = 1;
					state = GAMEOVER_SCREEN;
				}
				else
				{
					 state = GAMEOVER_WAIT_FOR_ANY_KEY;
					 exit_value = 0;
				}


			break;


			default:

				printf("Intro(): Error - unknown state (%d)", state);

				HAL_Delay(5000);

				state = GAMEOVER_SCREEN;
				exit_value = 0;

			break;


		}


	return exit_value;

}




void GamePlay_Update_Aiming_LED_Indicator(void)
{

	// Definicija makro parametrov.
	#define NOMINAL_DISTANCE_BETWEEN_TARGET_AND_CROSSHAIR	150

	// Pomožne spremenljivke.
	int16_t percent_distance;
	int16_t percent_vicinity;

	uint8_t number_of_LEDs_on;

	uint8_t bitmask_right = 0;
	uint8_t bitmask_left = 0;
	uint8_t bitmask;


	// Izračunamo relativno oddaljenost tarče
	percent_distance = (crosshair.distance_to_target_center * 100) / NOMINAL_DISTANCE_BETWEEN_TARGET_AND_CROSSHAIR;


	// Porežemo prevelike vrednosti.
	if (percent_distance > 100)
		percent_distance = 100;

	percent_vicinity = 100 - percent_distance;



	number_of_LEDs_on = ( (1 + NUM_OF_LEDS/2) * percent_vicinity ) / 100;

	if (number_of_LEDs_on  > (NUM_OF_LEDS/2) )
		number_of_LEDs_on = (NUM_OF_LEDS/2);


	for(uint8_t i = 1; i <= number_of_LEDs_on ; i++ )
	{
		bitmask_right |= 1;
		bitmask_right = bitmask_right << 1;
	}
	bitmask_right = bitmask_right >> 1;


	bitmask_left = bitmask_right << (8 - number_of_LEDs_on);

	bitmask = bitmask_right | bitmask_left;


	LEDs_write(bitmask);

}



