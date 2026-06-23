/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>   // memcpy() for packing the float current into the CAN frame
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- CAN protocol (raw classic-CAN frames bridged to ROS 2 on the PC) ----
 * /brake_command (std_msgs/Bool)  : PC -> STM32, ID 0x130, data[0] = 1 ON / 0 OFF
 * /brake_status  (BrakeStatus.msg): STM32 -> PC, ID 0x131, 8-byte heartbeat:
 *     [0..3] float32 current_ma (little-endian)
 *     [4]    uint8  relay_active   (1 = Relay G2R-24 ON, 0 = OFF)
 *     [5]    uint8  watchdog_status(0 = Normal, 1 = Triggered/fault)
 *     [6..7] uint16 heartbeat sequence counter (little-endian)
 * /servo_command (std_msgs/Float32): PC -> STM32, ID 0x132, 4-byte:
 *     [0..3] float32 angle_deg (little-endian, clamped to 0–180°). This is the
 *            "brake engaged" servo position: stored in flash and recalled on
 *            boot. The servo only drives to it while the relay is ON.
 * /servo_status  (std_msgs/Float32): STM32 -> PC, ID 0x133, 4-byte, sent on the
 *            same 20 ms tick as 0x131:
 *     [0..3] float32 brake_angle_deg (little-endian). The angle the STM32 is
 *            actually holding: the flash-recalled value at boot, then the live
 *            (clamped) /servo_command value after each overwrite. */
#define CAN_ID_BRAKE_CMD      0x130u
#define CAN_ID_BRAKE_STATUS   0x131u
#define CAN_ID_SERVO_CMD      0x132u
#define CAN_ID_SERVO_STATUS   0x133u

/* ---- INA240A2D current sensor (gain 50 V/V) with a 2 mOhm shunt ----------
 * Unidirectional wiring (REF tied to GND) so 0 A -> ~0 V and only the
 * positive direction is measured.
 *   Vout      = I * Rshunt * Gain  ->  0.002 * 50 = 0.1 V/A
 *   I [A]     = (Vadc - offset) / 0.1
 * Change CURRENT_ADC_INDEX if the INA240 output is wired to a different
 * ADC1 rank (adc_buffer[0]=PA0/IN1, [1]=PA1, [2]=PA2, [3]=PA3).            */
#define ADC_VREF              3.3f
#define ADC_FULL_SCALE        4095.0f
#define INA240_GAIN           50.0f
#define SHUNT_OHMS            0.002f
#define CURRENT_V_PER_A       (INA240_GAIN * SHUNT_OHMS)   /* 0.1 V/A */
#define CURRENT_OFFSET_V      0.0f                         /* unidirectional */
#define CURRENT_ADC_INDEX     0u

/* Open-load / motor-fault check: Relay commanded ON but ~no current flowing.
 * Temporarily disabled (set to 1 to re-enable once a real load is connected). */
#define ENABLE_OPENLOAD_CHECK        0
#define CURRENT_FAULT_THRESHOLD_MA   50.0f

/* Overcurrent trip: cut the relay if the current stays >= threshold continuously
 * for OVERCURRENT_TRIP_MS. The delay rejects the brief inrush/noise spike the
 * servo motor produces when it starts moving (engage or release), while still
 * catching a real sustained fault (stall / short). */
#define OVERCURRENT_THRESHOLD_A      14.0f
#define OVERCURRENT_TRIP_MS          150u

/* Moving-average window over the raw ADC current samples. Smooths single-sample
 * spikes (relay switching noise) that were tripping the overcurrent latch. */
#define CURRENT_AVG_WINDOW           16u

/* Heartbeat: TIM6 elapses every 10 ms, so 2 ticks -> 20 ms status rate. */
#define HEARTBEAT_TICKS       2u

/* On a normal release (relay commanded OFF) the servo is driven back to 0° and
 * we keep the relay energised for this long so it can physically get there
 * before power is cut. Faults (E-Stop/overcurrent) skip this and cut at once. */
#define SERVO_SETTLE_MS       3000u
#define SERVO_HOME_DEG        0.0f

/* Persisted "brake engaged" servo angle, stored in the last 2 KB page of bank 2.
 * NOTE: the G474 has no read-while-write, so a page erase (~22 ms) stalls the
 * whole CPU — interrupts included — for its duration. To avoid stalling during a
 * burst of /servo_command frames (which would make CAN look unresponsive), the
 * write is debounced: we only commit once the angle has been stable for
 * FLASH_WRITE_QUIET_MS. One 64-bit record: [31:0] magic, [63:32] float. */
#define FLASH_USER_ADDR       0x0807F800UL
#define FLASH_USER_BANK       FLASH_BANK_2
#define FLASH_USER_PAGE       127u
#define FLASH_USER_MAGIC      0xB7A4E001UL
#define BRAKE_ANGLE_DEFAULT   45.0f
#define FLASH_WRITE_QUIET_MS  800u

/* On a verified flash write, blink LED2 rapidly for a moment as a visual ACK.
 * Driven from the 10 ms TIM6 tick: toggle every LED2_BLINK_PERIOD ticks for
 * LED2_BLINK_TICKS ticks total (100 ticks = ~1 s, toggling every 50 ms ≈ 10 Hz). */
#define LED2_BLINK_TICKS      100u
#define LED2_BLINK_PERIOD     5u

/* Local push-button on PC1 (USER_SW) that toggles the relay. Wired active-high:
 * press connects the pin to 3.3V, internal pull-down holds it low when released. */
#define BTN_TOGGLE_Pin        GPIO_PIN_1
#define BTN_TOGGLE_GPIO_Port  GPIOC

/* E-Stop on PD2: active-low with pull-up (idle HIGH, triggered LOW -> falling). */
#define ESTOP_Pin             GPIO_PIN_2
#define ESTOP_GPIO_Port       GPIOD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t relay_cmd = 0;       // 0 = OFF, 1 = ON. Set by /brake_command over CAN (or Live Expression)
volatile float   brake_angle_deg = BRAKE_ANGLE_DEFAULT; // engaged servo position, persisted in flash, set by /servo_command
volatile uint8_t  flash_save_req = 0;  // set by RX ISR when brake_angle_deg changes; flash write runs in main loop
volatile uint32_t flash_req_tick = 0;  // HAL tick of the last brake_angle_deg change (debounce reference)
volatile uint8_t flash_write_ok = 0;  // 1 = last flash write read back OK (visible in Live Expression)
volatile uint16_t led2_blink_ticks = 0; // >0 = LED2 rapid-blink countdown (10 ms ticks), set on a verified write
uint16_t adc_buffer[4];     // สร้าง Buffer รอรับค่าจาก ADC DMA
uint16_t led_counter = 0;

/* Relay/servo release sequencer state (driven in the main loop). */
typedef enum {
	BRAKE_OFF = 0,    // relay open, servo parked at 0°
	BRAKE_ON,         // relay closed, servo driven to brake_angle_deg
	BRAKE_RELEASING   // servo driving back to 0°, relay still on until settle elapses
} brake_state_t;

/* --- Brake status / safety state shared with the ISRs --- */
volatile uint8_t  watchdog_status = 0;   // 0 = Normal, 1 = Triggered (E-Stop or open-load fault). Latched.
volatile uint8_t  can_tx_flag = 0;       // set by TIM6 every HEARTBEAT_TICKS, consumed in main loop
float             current_ma = 0.0f;     // latest INA240 current reading, milliamps
uint16_t          heartbeat_seq = 0;     // rolling counter so the PC can detect dropped frames

FDCAN_TxHeaderTypeDef CanTxHeader;        // configured once in CAN_App_Init() — used for 0x131
FDCAN_TxHeaderTypeDef CanServoTxHeader;   // configured once in CAN_App_Init() — used for 0x133
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void  CAN_App_Init(void);          // FDCAN filter + start + RX notification
static void  BrakeStatus_Send(void);      // pack and transmit the /brake_status heartbeat frame
static void  ServoStatus_Send(void);      // transmit the held brake angle (0x133)
static float   Flash_LoadBrakeAngle(float fallback); // recall the persisted brake angle on boot
static uint8_t Flash_SaveBrakeAngle(float angle);    // persist the brake angle; returns 1 if readback verifies
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t deg_to_pulse(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    return (uint32_t)(1000.0f + deg * (1000.0f / 180.0f));
}

/* Read the persisted brake angle. Returns `fallback` if the page was never
 * written (magic mismatch) or holds an out-of-range value. */
static float Flash_LoadBrakeAngle(float fallback) {
    uint64_t rec = *(__IO uint64_t *) FLASH_USER_ADDR;
    if ((uint32_t) (rec & 0xFFFFFFFFUL) != FLASH_USER_MAGIC) {
        return fallback;
    }
    uint32_t bits = (uint32_t) (rec >> 32);
    float angle;
    memcpy(&angle, &bits, sizeof(float));
    if (angle < 0.0f || angle > 180.0f) {
        return fallback;
    }
    return angle;
}

/* Erase the user page and store the brake angle as one 64-bit record. The ~22 ms
 * page erase stalls the whole CPU (no RWW on the G474), so callers must debounce
 * this — see FLASH_WRITE_QUIET_MS. Reads the record back and returns 1 only if it
 * matches what we intended to write. */
static uint8_t Flash_SaveBrakeAngle(float angle) {
    uint32_t bits;
    memcpy(&bits, &angle, sizeof(float));
    uint64_t rec = ((uint64_t) bits << 32) | (uint64_t) FLASH_USER_MAGIC;

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_err = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_USER_BANK;
    erase.Page      = FLASH_USER_PAGE;
    erase.NbPages   = 1;

    HAL_StatusTypeDef st = HAL_ERROR;
    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR, rec);
    }
    HAL_FLASH_Lock();

    /* Verify: the programmed doubleword must read back exactly. */
    return (st == HAL_OK && *(__IO uint64_t *) FLASH_USER_ADDR == rec) ? 1u : 0u;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
	/* PC1 = local toggle button (active-high, internal pull-down). Configured
	 * here in USER CODE so it survives CubeMX regeneration of gpio.c (CubeMX
	 * generates this pin as USER_SW with GPIO_NOPULL). */
	GPIO_InitTypeDef btn_init = {0};
	__HAL_RCC_GPIOC_CLK_ENABLE();
	btn_init.Pin = BTN_TOGGLE_Pin;
	btn_init.Mode = GPIO_MODE_INPUT;
	btn_init.Pull = GPIO_PULLDOWN;
	btn_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BTN_TOGGLE_GPIO_Port, &btn_init);

	/* E-Stop (PD2): CubeMX generates this EXTI pin as GPIO_NOPULL, which leaves
	 * the active-low line floating and fires spurious falling edges -> false
	 * watchdog_status=1. Re-init with internal pull-up (USER CODE = regen-safe)
	 * and drop any edge latched during boot/re-config before we start. */
	GPIO_InitTypeDef estop_init = {0};
	__HAL_RCC_GPIOD_CLK_ENABLE();
	estop_init.Pin = ESTOP_Pin;
	estop_init.Mode = GPIO_MODE_IT_FALLING;
	estop_init.Pull = GPIO_PULLUP;
	estop_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ESTOP_GPIO_Port, &estop_init);
	__HAL_GPIO_EXTI_CLEAR_IT(ESTOP_Pin);
	HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);
	watchdog_status = 0;            // clear false latch from a floating boot edge

	/* Recall the brake angle saved by the last /servo_command (default 0° if the
	 * flash page was never written). */
	brake_angle_deg = Flash_LoadBrakeAngle(BRAKE_ANGLE_DEFAULT);

	CAN_App_Init();                 // ตั้งค่า Filter + Start FDCAN + เปิด RX Interrupt
	HAL_TIM_Base_Start_IT(&htim6);  // เริ่ม Timer ของ WDI
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // เริ่ม PWM ของ Servo
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Relay/servo sequencer:
		 *   OFF  -> ON         : commanded ON (no fault) -> relay closes, servo to brake_angle
		 *   ON                 : servo follows brake_angle live (re-sent /servo_command)
		 *   ON   -> RELEASING  : commanded OFF -> servo to 0°, relay stays on
		 *   RELEASING -> OFF   : after SERVO_SETTLE_MS the relay opens
		 * A fault (watchdog_status=1) cuts the relay immediately from any state. */
		static brake_state_t brake_state = BRAKE_OFF;
		static float    servo_target  = SERVO_HOME_DEG;
		static uint32_t release_tick  = 0;

		if (watchdog_status == 1) {
			/* E-Stop / latched fault: open relay now, no graceful return. */
			HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
			servo_target = SERVO_HOME_DEG;
			brake_state  = BRAKE_OFF;
		} else {
			switch (brake_state) {
			case BRAKE_OFF:
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
				servo_target = SERVO_HOME_DEG;
				if (relay_cmd == 1) {
					HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
					servo_target = brake_angle_deg;
					brake_state  = BRAKE_ON;
				}
				break;

			case BRAKE_ON:
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
				servo_target = brake_angle_deg;   /* follow live angle updates */
				if (relay_cmd == 0) {
					servo_target = SERVO_HOME_DEG; /* drive home before powering off */
					release_tick = HAL_GetTick();
					brake_state  = BRAKE_RELEASING;
				}
				break;

			case BRAKE_RELEASING:
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
				servo_target = SERVO_HOME_DEG;
				if (relay_cmd == 1) {              /* re-engaged mid-release */
					servo_target = brake_angle_deg;
					brake_state  = BRAKE_ON;
				} else if ((HAL_GetTick() - release_tick) >= SERVO_SETTLE_MS) {
					HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
					brake_state = BRAKE_OFF;
				}
				break;
			}
		}

		/* Moving average over the last CURRENT_AVG_WINDOW raw ADC samples to reject
		 * single-sample spikes before converting to current. Ring buffer keeps a
		 * running sum so each iteration is O(1). */
		static uint16_t adc_window[CURRENT_AVG_WINDOW] = {0};
		static uint32_t adc_sum = 0;
		static uint8_t  adc_idx = 0;
		uint16_t adc_raw = adc_buffer[CURRENT_ADC_INDEX];
		adc_sum -= adc_window[adc_idx];
		adc_window[adc_idx] = adc_raw;
		adc_sum += adc_raw;
		adc_idx = (uint8_t) ((adc_idx + 1u) % CURRENT_AVG_WINDOW);
		float adc_avg = (float) adc_sum / (float) CURRENT_AVG_WINDOW;

		/* Convert the averaged INA240 ADC reading into milliamps (unidirectional). */
		float vout = (adc_avg / ADC_FULL_SCALE) * ADC_VREF;
		float amps = (vout - CURRENT_OFFSET_V) / CURRENT_V_PER_A;
		if (amps < 0.0f) {
			amps = 0.0f;
		}
		current_ma = amps * 1000.0f;

		/* Overcurrent protection with a persistence filter: only trip when the
		 * current has been over the threshold continuously for OVERCURRENT_TRIP_MS,
		 * so the servo's brief start-up inrush/noise spike does not cut the relay. */
		static uint32_t oc_over_since = 0;   /* tick the current first crossed over; 0 = below */
		if (amps >= OVERCURRENT_THRESHOLD_A) {
			uint32_t now = HAL_GetTick();
			if (oc_over_since == 0) {
				oc_over_since = (now != 0u) ? now : 1u;  /* 0 is the "below" sentinel */
			}
			if ((now - oc_over_since) >= OVERCURRENT_TRIP_MS) {
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
				relay_cmd    = 0;
				servo_target = SERVO_HOME_DEG;
				brake_state  = BRAKE_OFF;
			}
		} else {
			oc_over_since = 0;   /* dropped back below threshold -> restart the timer */
		}

		/* Drive the servo PWM to the sequencer's current target. */
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, deg_to_pulse(servo_target));

		/* Persist a freshly received brake angle outside the RX ISR. Erasing
		 * bank 2 is RWW-safe, so this does not stall the heartbeat below. */
		/* Debounced flash persist: commit only after the angle has been stable for
		 * FLASH_WRITE_QUIET_MS, so a burst of /servo_command frames does not trigger
		 * back-to-back ~22 ms erase stalls (which would drop/delay CAN frames). */
		if (flash_save_req && (HAL_GetTick() - flash_req_tick) >= FLASH_WRITE_QUIET_MS) {
			flash_save_req = 0;
			flash_write_ok = Flash_SaveBrakeAngle(brake_angle_deg);
			if (flash_write_ok) {
				led2_blink_ticks = LED2_BLINK_TICKS;  // rapid-blink LED2 as a write ACK
			}
		}

		/* Transmit the held servo angle (0x133) + /brake_status heartbeat on the
		 * 20 ms tick. 0x133 goes FIRST so the bridge has the current angle in
		 * hand before it publishes /brake_status on receiving 0x131 — otherwise
		 * the first heartbeat after a fresh MCU start carries a stale 0°. */
		if (can_tx_flag) {
			can_tx_flag = 0;
			ServoStatus_Send();
			BrakeStatus_Send();
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV8;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

/* USER CODE BEGIN 4 */

/**
  * @brief Configure the FDCAN acceptance filter, start the peripheral and
  *        enable the RX FIFO0 "new message" interrupt. Also prepares the
  *        static TX header used for every /brake_status frame.
  */
static void CAN_App_Init(void)
{
	/* Re-initialise FDCAN1 here (in USER CODE) so the settings survive CubeMX
	 * regeneration of fdcan.c. MX_FDCAN1_Init() already ran, so the peripheral
	 * is in configuration mode; we override the values that CubeMX regenerates
	 * from the .ioc back to placeholders.
	 *   1 Mbps @ 170 MHz kernel clock: 170e6 / (10 * (1+13+3)) = 1 000 000 bps
	 *   Sample point = (1+13)/17 = 82.4 %                                      */
	hfdcan1.Init.AutoRetransmission = ENABLE;
	hfdcan1.Init.NominalPrescaler = 10;
	hfdcan1.Init.NominalSyncJumpWidth = 3;
	hfdcan1.Init.NominalTimeSeg1 = 13;
	hfdcan1.Init.NominalTimeSeg2 = 3;
	hfdcan1.Init.StdFiltersNbr = 2;   /* slot 0: /brake_command, slot 1: /servo_command */
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID2 = 0x7FF; /* full mask -> exact ID match */

	/* Accept /brake_command (0x130) */
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterID1 = CAN_ID_BRAKE_CMD;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Accept /servo_command (0x132) */
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterID1 = CAN_ID_SERVO_CMD;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	/* TX header for /brake_status (0x131, classic CAN, 8 bytes). */
	CanTxHeader.Identifier = CAN_ID_BRAKE_STATUS;
	CanTxHeader.IdType = FDCAN_STANDARD_ID;
	CanTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CanTxHeader.DataLength = FDCAN_DLC_BYTES_8;
	CanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CanTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	CanTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	CanTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CanTxHeader.MessageMarker = 0;

	/* Servo-status frame (0x133): 4-byte float32 of the held brake angle. */
	CanServoTxHeader = CanTxHeader;
	CanServoTxHeader.Identifier = CAN_ID_SERVO_STATUS;
	CanServoTxHeader.DataLength = FDCAN_DLC_BYTES_4;

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
}

/**
  * @brief Pack the brake status into an 8-byte frame and queue it for TX.
  *        Also runs the "Relay ON but no current" open-load validation.
  */
static void BrakeStatus_Send(void)
{
	uint8_t relay_active =
			(HAL_GPIO_ReadPin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin) == GPIO_PIN_SET) ? 1u : 0u;

	/* Current Validation: Relay is ON but almost no current is flowing ->
	 * broken motor wire or failed motor. Latch the fault so the PC sees it.
	 * Disabled while bench-testing without a load (see ENABLE_OPENLOAD_CHECK). */
#if ENABLE_OPENLOAD_CHECK
	if (relay_active && (current_ma < CURRENT_FAULT_THRESHOLD_MA)) {
		watchdog_status = 1;
	}
#endif

	uint8_t TxData[8];
	memcpy(&TxData[0], &current_ma, sizeof(float)); /* [0..3] float32, little-endian */
	TxData[4] = relay_active;                        /* [4]    relay_active */
	TxData[5] = watchdog_status;                     /* [5]    watchdog_status */
	TxData[6] = (uint8_t) (heartbeat_seq & 0xFFu);   /* [6..7] sequence counter */
	TxData[7] = (uint8_t) (heartbeat_seq >> 8);
	heartbeat_seq++;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanTxHeader, TxData);
}

/**
  * @brief Report the held brake angle (0x133): flash value at boot, then the
  *        live /servo_command value after each overwrite.
  */
static void ServoStatus_Send(void)
{
	float angle = brake_angle_deg;          /* volatile -> local snapshot */
	uint8_t TxData[4];
	memcpy(&TxData[0], &angle, sizeof(float)); /* [0..3] float32, little-endian */

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanServoTxHeader, TxData);
}

/**
  * @brief FDCAN RX FIFO0 callback: decode /brake_command and /servo_command frames.
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
		return;
	}

	/* Drain every frame queued in FIFO0, not just one: a flash-erase stall can
	 * let several pile up, and reading only one would leave the rest unprocessed. */
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		if (RxHeader.Identifier == CAN_ID_BRAKE_CMD) {
			/* std_msgs/Bool: data:true -> Relay ON (engage), data:false -> Relay OFF. */
			relay_cmd = (RxData[0] != 0u) ? 1u : 0u;
		} else if (RxHeader.Identifier == CAN_ID_SERVO_CMD) {
			/* std_msgs/Float32: new "brake engaged" angle. Takes effect immediately;
			 * the flash persist is debounced in the main loop (FLASH_WRITE_QUIET_MS). */
			float angle;
			memcpy(&angle, &RxData[0], sizeof(float));
			if (angle < 0.0f)   angle = 0.0f;
			if (angle > 180.0f) angle = 180.0f;
			if (angle != brake_angle_deg) {
				brake_angle_deg = angle;
				flash_save_req  = 1;
				flash_req_tick  = HAL_GetTick();  /* restart the debounce window */
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) // ทุกๆ 10ms
	{
		// 1. ทริกเกอร์ Watchdog
		HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);

		// 2a. LED2 rapid-blink (~10 Hz) as a flash-write ACK, for ~1 s after a
		//     verified write. Overrides LED2's normal heartbeat while active.
		if (led2_blink_ticks > 0) {
			led2_blink_ticks--;
			static uint8_t b = 0;
			if (++b >= LED2_BLINK_PERIOD) {
				b = 0;
				HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			}
			if (led2_blink_ticks == 0) {
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // leave LED2 in a known state
			}
		}

		// 2b. นับเวลาให้ครบ 1 วินาที (10ms * 100 = 1000ms) สำหรับ LED heartbeat
		led_counter++;
		if (led_counter >= 100) {
			HAL_GPIO_TogglePin(GPIOB, LED_1_Pin);
			if (led2_blink_ticks == 0) {
				HAL_GPIO_TogglePin(GPIOB, LED2_Pin); // normal LED2 heartbeat when not ACK-blinking
			}
			led_counter = 0;
		}

		// 3. ตั้ง Flag ส่ง Heartbeat /brake_status ทุกๆ HEARTBEAT_TICKS (20ms)
		static uint16_t hb_counter = 0;
		if (++hb_counter >= HEARTBEAT_TICKS) {
			hb_counter = 0;
			can_tx_flag = 1;
		}

		// 4. ปุ่ม Toggle relay ที่ขา PC1 (active-high). สุ่มอ่านทุก 10ms = debounce
		//    สลับสถานะเฉพาะตอน "กดลง" (ขอบ released->pressed) เท่านั้น
		static GPIO_PinState btn_prev = GPIO_PIN_RESET; // released = low (pull-down)
		GPIO_PinState btn_now = HAL_GPIO_ReadPin(BTN_TOGGLE_GPIO_Port, BTN_TOGGLE_Pin);
		if (btn_prev == GPIO_PIN_RESET && btn_now == GPIO_PIN_SET) {
			relay_cmd = (relay_cmd == 1) ? 0u : 1u; // toggle
		}
		btn_prev = btn_now;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2) // ตรวจสอบว่าเป็น Interrupt จากขา PD2
  {
    // 0. กรอง glitch: noise ตอนคอยล์ relay สวิตช์ทำให้ PD2 เกิด falling edge ปลอม.
    //    E-Stop จริงจะกดค้างให้สาย LOW ต่อเนื่อง ส่วน glitch จะเด้งกลับ HIGH ทันที.
    //    spin สั้นๆ แล้วอ่านซ้ำ ถ้ากลับเป็น HIGH = glitch -> ไม่ latch.
    for (volatile uint32_t i = 0; i < 4000u; i++) { __NOP(); }
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) != GPIO_PIN_RESET) {
      __HAL_GPIO_EXTI_CLEAR_IT(ESTOP_Pin);  // ทิ้ง edge ปลอม
      return;
    }

    // 1. สั่งปิด Relay ทันทีที่ระดับฮาร์ดแวร์
    HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);

    // 2. บังคับเคลียร์ตัวแปรคำสั่งใน Live Expression
    // เพื่อป้องกันไม่ให้โค้ดใน while(1) กลับมาสั่งเปิด Relay อีก
    relay_cmd = 0;

    // 3. ตั้งสถานะ watchdog ให้เป็น Triggered เพื่อให้ PC รู้ว่าเกิด E-Stop
    watchdog_status = 1;
  }
}
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
