/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "kbd.h"
#include "SCI.h"
#include "timing_utils.h"
#include "periodic_services.h"
#include "lcd.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "ugui.h"
#include "podatki.h"
#include "pomozne_funkcije.h"
#include "ray_funkcije.h"
#include "joystick.h"

#include "t_brickWall.ppm"
#include "t_wall.ppm"
#include "t_wood.ppm"
#include "t_floor.ppm"
#include "hitler_sprite.ppm"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc4;

CORDIC_HandleTypeDef hcordic;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

DMA_HandleTypeDef hdma_memtomem_dma1_channel2;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int f32_to_q31(double input){
	const float Q31_MAX_F = 0x0.FFFFFFp0F;
	const float Q31_MIX_F = -1.0F;
	return (int)roundf(scalbnf(fmaxf(fminf(input, Q31_MAX_F), Q31_MIX_F), 31));
}

float q31_to_f32(int x)
{
	float out = ldexp((int32_t) x, -31);
	return out;
}

inline float sin_cordic(float a){

	int32_t inp = f32_to_q31(fmod(a, 2.0f*PI) / (2.0f * PI)) << 1;
	int32_t out;
	CORDIC_ConfigTypeDef sConfig;
	sConfig.Function = CORDIC_FUNCTION_SINE;
	sConfig.Precision = CORDIC_PRECISION_6CYCLES;
	sConfig.Scale = CORDIC_SCALE_0;
	sConfig.NbWrite = CORDIC_NBWRITE_1;
	sConfig.NbRead = CORDIC_NBREAD_1;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
	HAL_CORDIC_Configure(&hcordic, &sConfig);

	HAL_CORDIC_CalculateZO(&hcordic, &inp, &out, 1, 0);
	return q31_to_f32(out);
}

inline float cos_cordic(float a){

	int32_t inp = f32_to_q31(fmod(a, 2.0f*PI) / (2.0f * PI)) << 1;
	int32_t out;
	CORDIC_ConfigTypeDef sConfig;
	sConfig.Function = CORDIC_FUNCTION_COSINE;
	sConfig.Precision = CORDIC_PRECISION_6CYCLES;
	sConfig.Scale = CORDIC_SCALE_0;
	sConfig.NbWrite = CORDIC_NBWRITE_1;
	sConfig.NbRead = CORDIC_NBREAD_1;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
	HAL_CORDIC_Configure(&hcordic, &sConfig);

	HAL_CORDIC_CalculateZO(&hcordic, &inp, &out, 1, 0);
	return q31_to_f32(out);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_TIM6_Init();
	MX_TIM4_Init();
	MX_FMC_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_CORDIC_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	MX_ADC4_Init();
	LED_init();
	KBD_init();
	SCI_init();
	PSERV_init();
	PSERV_enable();
	JOY_init(&hadc4, &htim1);
	LCD_Init();
	LCD_uGUI_init();
	//LED_on(LED1);
	//DEFINIRAJ SPRITE
	sp[0].tip = 0; sp[0].map = 0; sp[0].x = 1.5 * Blocksize; sp[0].y = 5 * Blocksize; sp[0].sizeX = 128; sp[0].sizeY = 128; sp[0].life = 3;
	sp[0].damadgedTime = 0; sp[0].damadgedTimeMax = 300000;
	sp[1].tip = 0; sp[1].map = 0; sp[1].x = 2.5 * Blocksize; sp[1].y = 7.5 * Blocksize; sp[1].sizeX = 128; sp[1].sizeY = 128; sp[1].life = 3;
	sp[1].damadgedTime = 0; sp[1].damadgedTimeMax = 300000;


	UG_FillFrame(0, 0, 40, s_height, C_BLACK);
	UG_FillFrame(280, 0, 40, s_height+50, C_BLACK);
	for(int c = 0;c < 20;c++)
	{
		UG_PutChar('F', 10, c*15, C_RED, C_BLACK);
		UG_PutChar('R', 20, c*15, C_RED, C_BLACK);
		UG_PutChar('I', 30, c*15, C_RED, C_BLACK);

		UG_PutChar('F', 280+10, c*15, C_RED, C_BLACK);
		UG_PutChar('E', 280+20, c*15, C_RED, C_BLACK);
	}

	//test_sprite();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//ILI9341_SetDisplayWindow(0, 0, 240, 240);
	stopwatch_handle_t sw;
	TIMUT_stopwatch_set_time_mark(&sw);
	while (1)
	{
		//JOY_SCI_send_status();
		//printf("test");
		if(TIMUT_stopwatch_has_X_ms_passed(&sw, 50))
		{
			//drawMap();

			float dx = 0, dy = 0, d1 = 10000, d2 = 10000, rvx = 0, rvy = 0, dof = 0,  newAngle = 0, rhx = 0, rhy = 0,d = 0, rx= 0, ry=0;
			int wx = 0, wy = 0;
			KBD_scan();

			//MOVE PLAYER
			if(JOY_get_axis_position(Y) < 40)
			{
				if(JOY_get_axis_position(X) < 40) angle += 0.1;
				if(JOY_get_axis_position(X) > 60) angle -= 0.1;
			}else
			{
				if(JOY_get_axis_position(X) < 40) angle -= 0.1;
				if(JOY_get_axis_position(X) > 60) angle += 0.1;
			}
			if (angle < 0) angle = 2 * PI;
			else if (angle > 2 * PI) angle = 0;


			//POGLEJ A SE PLAYER LAHKO PREMAKNE
			int fCheck = 5;
			int fCheckX = 0, fCheckY = 0;
			if (cos_cordic(angle) < 0) fCheckX = -fCheck;
			else fCheckX = fCheck;
			if (sin_cordic(angle) < 0) fCheckY = -fCheck;
			else fCheckY = fCheck;

			if (JOY_get_axis_position(Y) > 60) {
				if(map[(int)py / 10][((int)px + fCheckX) / 10] == 0) px += 3 * cos_cordic(angle);
				if (map[((int)py + fCheckY) / 10][(int)px/ 10] == 0) py += 3 * sin_cordic(angle);
				//LED_on(LED0);
			}else LED_off(LED0);
			if (JOY_get_axis_position(Y) < 40) {
				if (map[(int)py / 10][((int)px - fCheckX) / 10] == 0) px += -3 * cos_cordic(angle);
				if (map[((int)py - fCheckY) / 10][(int)px / 10] == 0) py += -3 * sin_cordic(angle);
				//LED_on(LED0);
			}else LED_off(LED0);

			//DA LAHKO IGRALEC ODPRE VRATA
			float roka = 6;
			float rokaPosX = px + roka * cos_cordic(angle);
			float rokaPosY = py + roka * sin_cordic(angle);
			if (!KBD_get_button_state(BTN_OK) && map[(int)rokaPosY / Blocksize][(int)rokaPosX / Blocksize] == 3) map[(int)rokaPosY / Blocksize][(int)rokaPosX / Blocksize] = 0;

			for (float z = -30;z < 30;z+=1) { //LOOPEJ OD - 30STOPINJ DO 30 STOPINJ
				float vmt = 0, hmt = 0; //HORIZONTALNA IN VERTIKALNA ŠTEVILKA STENE DA VEM KATERO TEKSTURO NARISAT
				float newAngle = z * DEGREE + angle;
				if (newAngle < 0) newAngle += 2 * PI;
				else if (newAngle > 2 * PI) newAngle -= 2 * PI;


				//GLEJ GOR PA DOL
				float aTan = 1 / tanf(newAngle);
				dof = 0;
				if (newAngle >= PI) { //GLEDA GOR
					rvy = (int)py - ((int)py % Blocksize) - 0.0001;
					rvx = (py - rvy) * -aTan + px;
					dy = -Blocksize;
					dx = dy * aTan;
				}
				else if (newAngle < PI) {  //GLEDA DOL
					rvy = (int)py - ((int)py % Blocksize) + Blocksize;
					rvx = (py - rvy) * -aTan + px;
					dy = Blocksize;
					dx = dy * aTan;
				}
				else if (newAngle == 0 || newAngle == PI) { // GLEDA  CIST LECVO ALPA CIST DESNO
					rvx = px;
					rvy = py;
					dof = 15;
				}

				while (dof < 15) { //PREVER ALI SE KAM ZALETI V 15 KORAKIH
					wx = rvx / Blocksize;
					wy = rvy / Blocksize;
					if (wx > -1 && wx < sizeof(map)/sizeof(map[0]) && wy > -1 && wy < sizeof(map)/sizeof(map[0]) && map[(int)wy][(int)wx] != 0) {
						dof = 15;
						vmt = map[(int)wy][(int)wx];
					}
					else {
						rvx += dx;
						rvy += dy;
						dof++;
					}
				}
				d1 = sqrt((px - rvx) * (px - rvx) + (py - rvy) * (py - rvy));


				//GLEJ LEVO PA DESNO
				dof = 0;
				float nTan = tanf(newAngle);
				if (newAngle >= ((3 * PI) / 2) || newAngle <= PI / 2) { //GLEDA DESNO
					rhx = (int)px - ((int)px % Blocksize) + Blocksize;
					rhy = (px - rhx) * -nTan + py;
					dx = Blocksize;
					dy = dx * nTan;
				}
				else if (newAngle > PI / 2 && newAngle < (3 * PI) / 2) {  //GLEDA LEVO
					rhx = (int)px - ((int)px % Blocksize) - 0.0001;
					rhy = (px - rhx) * -nTan + py;
					dx = -Blocksize;
					dy = dx * nTan;
				}
				else if (newAngle == PI / 2 || newAngle == (3 * PI) / 2) {
					rhx = px;
					rhy = py;
					dof = 15;
				}

				while (dof < 15) { //PREVERI ALI SI SE ZALETU V 15 KORAKIH
					wx = rhx / Blocksize;
					wy = rhy / Blocksize;
					if (wx > -1 && wx < sizeof(map)/sizeof(map[0]) && wy > -1 && wy < sizeof(map)/sizeof(map[0]) && map[wy][wx] != 0) {
						dof = 15;
						hmt = map[wy][wx];
					}
					else {
						rhx += dx;
						rhy += dy;
						dof++;
					}
				}
				d2 = sqrt((px - rhx) * (px - rhx) + (py - rhy) * (py - rhy));

				//NARIŠI ZARKE IN 3D MAPO
				float shade = 1;
				if (d1 > d2) {
					d = d2;
					shade = 0.5;
					rx = rhx;
					ry = rhy;
				}
				else {
					d = d1;
					rx = rvx;
					ry = rvy;
					hmt = vmt; //DA SE OBARVAJO VSE STRANI
				}
				//NARIŠI STENE
				float lineH = (Blocksize * 100) / (d * cos_cordic(z * DEGREE)); //POPRAVI FISHEYE
				float ty_step = 32.0f / (float)lineH; //ZATO DA SE TEKSTURA ENAKOMERNO PORAZDELI
				float ty_offset = 0; //ZATO DA SE TEKSTURE LEPO PORAZDELIJO
				if (lineH > height) {
					ty_offset = (lineH - height) / 2.0f;
					lineH = height;
				}
				float lineOff = 50 - lineH / 2;
				float ty = ty_offset * ty_step; //+ (hmt - 1) * 32;
				float scale = (float)Blocksize / 32.0f;
				float tx = 0;
				if (shade == 1) {
					tx = (int)(rx / scale) % 32;
					if (newAngle < 2 * PI) tx = 31 - tx;
				}
				else {
					tx = (int)(ry / scale) % 32;
					if (newAngle > PI / 2 && newAngle < (3 * PI) / 2) tx = 31 - tx;
				}

				for (int j = 0;j < lineH;j++) {

					int pixel = ((int)ty * 32 + (int)tx) * 3;
					int red = 0, blue = 0, green = 0;
					switch ((int)hmt)
					{
					case 1:
						red = t_brickWall[pixel + 0] * shade;
						green = t_brickWall[pixel + 1] * shade;
						blue = t_brickWall[pixel + 2] * shade;
						break;
					case 2:
						red = t_wall[pixel + 0] * shade;
						green = t_wall[pixel + 1] * shade;
						blue = t_wall[pixel + 2] * shade;
						break;
					case 3:
						red = t_wood[pixel + 0] * shade;
						green = t_wood[pixel + 1] * shade;
						blue = t_wood[pixel + 2] * shade;
						break;

					default:
						break;
					}
					int wallx = (int)((float)(z+30)*width/60);
					int wally = lineOff + j;

					wallx *= s_scale;
					wallx += s_offset;

					wally *= s_scale;

					narisi_velik_kvadrat_stene(wallx, wally, create_rgb(red, green, blue));
					ty += ty_step;
				}

				//NARIŠI TLA IN STROP
				for (int y = lineOff + lineH;y < height;y++) {
					float dy = y - (height / 2.0f);
					float floorAngle = (angle - newAngle);
					if (floorAngle < 0) floorAngle += 2 * PI;
					else if (floorAngle > 2 * PI) floorAngle -= 2 * PI;
					tx = px / 0.3 + cos_cordic(newAngle) * 57.735 * 32 / dy / cos_cordic(floorAngle);
					ty = py / 0.3 + sin_cordic(newAngle) * 57.735 * 32 / dy / cos_cordic(floorAngle);



					int pixel = (((int)(ty) & 31) * 32 + ((int)(tx)&31)) * 3;
					int red = t_floor[pixel + 0] * 0.7;
					int green = t_floor[pixel + 1] * 0.7;
					int blue = t_floor[pixel + 2] * 0.7;
					int tlax = (int)((float)(z+30)*width/60);
					int tlay = y;

					tlax *= s_scale;
					tlax += s_offset;

					tlay *= s_scale;

					narisi_velik_kvadrat_stene(tlax, tlay, create_rgb(red, green, blue));

					//NARISI STROP
					pixel = (((int)(ty) & 31) * 32 + ((int)(tx) & 31)) * 3;
					red = t_wall[pixel + 0] * 0.7;
					green = t_wall[pixel + 1] * 0.7;
					blue = t_wall[pixel + 2] * 0.7;
					int stropx = (int)((float)((z+30)*width)/60);
					int stropy = height - y;

					stropx *= s_scale;
					stropx += s_offset;

					stropy *= s_scale;

					narisi_velik_kvadrat_stene(stropx, stropy, create_rgb(red, green, blue));
				}




			}

			drawGun();
			ILI9341_SetDisplayWindow(40, 0, 240, 240);
			if(ILI9341_SendData(pixels, 57600)) LED_on(LED0);
			else LED_off(LED0);
			TIMUT_stopwatch_set_time_mark(&sw);
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC4_Init(void)
{

	/* USER CODE BEGIN ADC4_Init 0 */

	/* USER CODE END ADC4_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC4_Init 1 */

	/* USER CODE END ADC4_Init 1 */
	/** Common config
	 */
	hadc4.Instance = ADC4;
	hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc4.Init.Resolution = ADC_RESOLUTION_12B;
	hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc4.Init.GainCompensation = 0;
	hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc4.Init.LowPowerAutoWait = DISABLE;
	hadc4.Init.ContinuousConvMode = DISABLE;
	hadc4.Init.NbrOfConversion = 2;
	hadc4.Init.DiscontinuousConvMode = DISABLE;
	hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc4.Init.DMAContinuousRequests = ENABLE;
	hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc4.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc4) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC4_Init 2 */

	/* USER CODE END ADC4_Init 2 */

}

/**
 * @brief CORDIC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CORDIC_Init(void)
{

	/* USER CODE BEGIN CORDIC_Init 0 */

	/* USER CODE END CORDIC_Init 0 */

	/* USER CODE BEGIN CORDIC_Init 1 */

	/* USER CODE END CORDIC_Init 1 */
	hcordic.Instance = CORDIC;
	if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CORDIC_Init 2 */

	/* USER CODE END CORDIC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	TIM_InitStruct.Prescaler = 1440;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 99;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM4, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM4);
	LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM4);
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	/**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

	/* TIM6 interrupt Init */
	NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	TIM_InitStruct.Prescaler = 144;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 49999;
	LL_TIM_Init(TIM6, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM6);
	LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM6);
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	/**USART3 GPIO Configuration
  PB8-BOOT0   ------> USART3_RX
  PB9   ------> USART3_TX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USART3 interrupt Init */
	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(USART3_IRQn);

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART3, &USART_InitStruct);
	LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_DisableFIFO(USART3);
	LL_USART_DisableOverrunDetect(USART3);
	LL_USART_ConfigAsyncMode(USART3);

	/* USER CODE BEGIN WKUPType USART3 */

	/* USER CODE END WKUPType USART3 */

	LL_USART_Enable(USART3);

	/* Polling USART3 initialisation */
	while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3))))
	{
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 * Configure DMA for memory to memory transfers
 *   hdma_memtomem_dma1_channel2
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* Configure DMA request hdma_memtomem_dma1_channel2 on DMA1_Channel2 */
	hdma_memtomem_dma1_channel2.Instance = DMA1_Channel2;
	hdma_memtomem_dma1_channel2.Init.Request = DMA_REQUEST_MEM2MEM;
	hdma_memtomem_dma1_channel2.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma1_channel2.Init.PeriphInc = DMA_PINC_ENABLE;
	hdma_memtomem_dma1_channel2.Init.MemInc = DMA_MINC_DISABLE;
	hdma_memtomem_dma1_channel2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_memtomem_dma1_channel2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_memtomem_dma1_channel2.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma1_channel2.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_memtomem_dma1_channel2) != HAL_OK)
	{
		Error_Handler( );
	}

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_NORSRAM_TimingTypeDef Timing = {0};

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FMC_NORSRAM_DEVICE;
	hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
	hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
	hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
	hsram1.Init.NBLSetupTime = 0;
	hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
	hsram1.Init.MaxChipSelectPulse = DISABLE;
	/* Timing */
	Timing.AddressSetupTime = 1;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 1;
	Timing.DataHoldTime = 1;
	Timing.BusTurnAroundDuration = 1;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
	{
		Error_Handler( );
	}

	/* USER CODE BEGIN FMC_Init 2 */

	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);

	/**/
	LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_3);

	/**/
	LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_4);

	/**/
	LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_5);

	/**/
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);

	/**/
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1);

	/**/
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);

	/**/
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);

	/**/
	LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_2);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);

	/**/
	LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_1);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

