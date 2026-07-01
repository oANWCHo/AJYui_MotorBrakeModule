/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : curtis_io.c
 * @brief          : Read and drive the interface signals to/from the Curtis 1510.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "curtis_io.h"
#include "main.h"     /* pin defines + HAL_GPIO_ReadPin / HAL_GetTick */
#include "i2c.h"      /* hi2c2 for the MCP4725 MCOR DAC */

/* Private define ------------------------------------------------------------*/
#define ADC_VREF              3.3f
#define ADC_FULL_SCALE        4095.0f

/* MCP4725 MCOR DAC on I2C2 (PA8=SDA, PA9=SCL). Default 7-bit address 0x60
 * (A0 = GND); change to 0x61 if the board straps A0 high, or per the variant. */
#define MCP4725_I2C_ADDR      0x60u
/* Supply/reference the MCP4725 runs from; sets the full-scale output voltage.
 * Confirm against the board — a Curtis MCOR throttle is typically 0..5 V. */
#define MCP4725_VREF          5.0f
#define MCP4725_MAX_CODE      4095u
#define MCP4725_I2C_TIMEOUT   10u   /* ms */

/* TIM2 counts at the APB1 timer clock. SYSCLK = 170 MHz and APB1 prescaler = 1,
 * so the TIM2 kernel clock is the full 170 MHz; MX_TIM2_Init() runs it with
 * prescaler 0, i.e. one tick = 1/170 MHz. */
#define CURTIS_SPEED_TIMER_HZ   170000000UL
/* Treat the wheel as stopped if no capture edge arrives within this window, so a
 * stale frequency does not linger after the pulses stop. */
#define CURTIS_SPEED_TIMEOUT_MS 500u

/* Private variables ---------------------------------------------------------*/
/* Speed-capture state, written by the ISR (via CurtisIO_SpeedOnCapture) and read
 * by CurtisIO_SpeedHz in the main loop. */
static volatile uint32_t s_last_capture = 0;
static volatile uint8_t  s_have_prev    = 0;
static volatile uint32_t s_last_edge_ms = 0;
static volatile float    s_freq_hz      = 0.0f;

/* Exported functions --------------------------------------------------------*/
uint8_t CurtisIO_Forward(void) {
    return (HAL_GPIO_ReadPin(Forward_IN_to_MCU_GPIO_Port, Forward_IN_to_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_Backward(void) {
    return (HAL_GPIO_ReadPin(Backward_IN_to_MCU_GPIO_Port, Backward_IN_to_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_Pedal(void) {
    return (HAL_GPIO_ReadPin(Pedal_IN_to_MCU_GPIO_Port, Pedal_IN_to_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

void CurtisIO_ReadDigital(CurtisDigitalInputs_t *out) {
    out->forward  = CurtisIO_Forward();
    out->backward = CurtisIO_Backward();
    out->pedal    = CurtisIO_Pedal();
}

float CurtisIO_McorVolts(uint16_t mcor_adc_raw) {
    return ((float) mcor_adc_raw / ADC_FULL_SCALE) * ADC_VREF;
}

void CurtisIO_SpeedOnCapture(uint32_t capture) {
    if (s_have_prev) {
        /* 32-bit unsigned subtraction wraps correctly across the timer's rollover. */
        uint32_t delta = capture - s_last_capture;
        if (delta != 0u) {
            s_freq_hz = (float) CURTIS_SPEED_TIMER_HZ / (float) delta;
        }
    }
    s_last_capture = capture;
    s_have_prev    = 1u;
    s_last_edge_ms = HAL_GetTick();
}

float CurtisIO_SpeedHz(void) {
    if (!s_have_prev ||
        (HAL_GetTick() - s_last_edge_ms) > CURTIS_SPEED_TIMEOUT_MS) {
        return 0.0f;
    }
    return s_freq_hz;
}

/* ------------------------------- Output side ------------------------------ */
void CurtisIO_OutputEnable(uint8_t on) {
    HAL_GPIO_WritePin(Relay_Mode_GPIO_Port, Relay_Mode_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_SetForward(uint8_t on) {
    HAL_GPIO_WritePin(Forward_IN_from_MCU_GPIO_Port, Forward_IN_from_MCU_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_SetBackward(uint8_t on) {
    HAL_GPIO_WritePin(Backward_IN_from_MCU_GPIO_Port, Backward_IN_from_MCU_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_SetPedal(uint8_t on) {
    HAL_GPIO_WritePin(Pedal_IN_from_MCU_GPIO_Port, Pedal_IN_from_MCU_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_WriteDigitalOut(uint8_t forward, uint8_t backward, uint8_t pedal) {
    CurtisIO_SetForward(forward);
    CurtisIO_SetBackward(backward);
    CurtisIO_SetPedal(pedal);
}

uint8_t CurtisIO_McorWriteRaw(uint16_t code12) {
    if (code12 > MCP4725_MAX_CODE) {
        code12 = MCP4725_MAX_CODE;
    }
    /* MCP4725 fast-write: 2 bytes.
     *   byte0 = [C2 C1 PD1 PD0 D11 D10 D9 D8] with C=00 (fast write), PD=00 (on)
     *   byte1 = [D7 .. D0] */
    uint8_t buf[2];
    buf[0] = (uint8_t) ((code12 >> 8) & 0x0Fu);
    buf[1] = (uint8_t) (code12 & 0xFFu);
    return (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) (MCP4725_I2C_ADDR << 1),
            buf, sizeof(buf), MCP4725_I2C_TIMEOUT) == HAL_OK) ? 1u : 0u;
}

uint8_t CurtisIO_McorWriteVolts(float volts) {
    if (volts < 0.0f)         volts = 0.0f;
    if (volts > MCP4725_VREF) volts = MCP4725_VREF;
    uint16_t code = (uint16_t) ((volts / MCP4725_VREF) * (float) MCP4725_MAX_CODE + 0.5f);
    return CurtisIO_McorWriteRaw(code);
}
