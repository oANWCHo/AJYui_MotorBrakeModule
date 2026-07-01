/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : curtis_io.h
 * @brief          : Read and drive the interface signals to/from the Curtis 1510.
 ******************************************************************************
 * @attention
 *
 * Groups the three ways this shield senses the Curtis 1510:
 *   - Digital direction/pedal lines (GPIO input)      : Forward/Backward/Pedal
 *   - MCOR throttle wiper (analog, ADC1_IN2 on PA1)   : CurtisIO_McorVolts()
 *   - Motor speed sensor pulse train (TIM2 CH1 input  : CurtisIO_SpeedHz()
 *     capture on PA15)
 *
 * Pure sensing only: the reads talk to the STM32 GPIO/ADC/timer and return
 * their result. Pin identities come from main.h. The speed path keeps its own
 * capture state, fed from the TIM2 input-capture ISR.
 *
 * NOTE on polarity: the digital lines are wired GPIO_NOPULL (driven by the
 * Curtis), so the reads return the raw pin level (1 = high). Confirm the active
 * sense against the 1510 wiring at the call site.
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef INC_CURTIS_IO_H_
#define INC_CURTIS_IO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* Snapshot of the three digital lines from the Curtis (raw levels, 1 = high). */
typedef struct {
    uint8_t forward;    /* Forward_IN_to_MCU  (PC5) */
    uint8_t backward;   /* Backward_IN_to_MCU (PC4) */
    uint8_t pedal;      /* Pedal_IN_to_MCU    (PB0) */
} CurtisDigitalInputs_t;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Read all three digital direction/pedal lines at once.
  * @param  out : receives the raw pin levels (1 = high).
  */
void CurtisIO_ReadDigital(CurtisDigitalInputs_t *out);

/* Individual raw reads (1 = pin high, 0 = pin low). */
uint8_t CurtisIO_Forward(void);
uint8_t CurtisIO_Backward(void);
uint8_t CurtisIO_Pedal(void);

/**
  * @brief  Convert an MCOR throttle ADC sample (ADC1_IN2 / PA1) to volts.
  * @param  mcor_adc_raw : raw 12-bit DMA sample for the MCOR rank.
  * @retval Wiper voltage, 0..VREF.
  */
float CurtisIO_McorVolts(uint16_t mcor_adc_raw);

/**
  * @brief  Feed one TIM2 CH1 capture value. Call from the input-capture ISR.
  * @param  capture : the 32-bit counter value latched on the edge.
  */
void CurtisIO_SpeedOnCapture(uint32_t capture);

/**
  * @brief  Latest measured speed-sensor pulse frequency.
  * @retval Frequency in Hz, or 0 if no edge has arrived recently (stopped).
  */
float CurtisIO_SpeedHz(void);

/* ============================= Output side ================================ *
 * Drive the Curtis 1510 instead of sensing it. The mode relay (Relay_Mode /
 * PA10) selects which side is wired through, so it MUST be energised before the
 * MCU's outputs reach the Curtis: call CurtisIO_OutputEnable(1) first, then use
 * the setters / DAC below; CurtisIO_OutputEnable(0) hands control back to the
 * pass-through (input) side. The setters do not toggle the relay themselves —
 * the mode relay needs settling time, so switching sides is left explicit.
 * ------------------------------------------------------------------------- */

/**
  * @brief  Energise/de-energise the mode relay (Relay_Mode, PA10).
  * @param  on : 1 = route the MCU outputs to the Curtis (output side active),
  *              0 = release back to the pass-through / input side.
  */
void CurtisIO_OutputEnable(uint8_t on);

/* Drive the direction/pedal output lines to the Curtis (Forward/Backward/Pedal
 * _IN_from_MCU, PC6/PC8/PC7). 1 = high, 0 = low. Requires OutputEnable(1). */
void CurtisIO_SetForward(uint8_t on);
void CurtisIO_SetBackward(uint8_t on);
void CurtisIO_SetPedal(uint8_t on);
void CurtisIO_WriteDigitalOut(uint8_t forward, uint8_t backward, uint8_t pedal);

/**
  * @brief  Write the MCOR throttle output via the MCP4725 DAC (I2C2, PA8/PA9).
  * @param  code12 : 12-bit DAC code, 0..4095 (clamped). Uses MCP4725 fast-write
  *                  mode (RAM only — no EEPROM wear on frequent updates).
  * @retval 1 if the I2C transfer succeeded, 0 otherwise.
  * @note   Requires OutputEnable(1) for the Curtis to see the level.
  */
uint8_t CurtisIO_McorWriteRaw(uint16_t code12);

/**
  * @brief  Write the MCOR output as a voltage (0..MCP4725 VREF).
  * @param  volts : target wiper voltage; clamped to [0, VREF].
  * @retval 1 if the I2C transfer succeeded, 0 otherwise.
  */
uint8_t CurtisIO_McorWriteVolts(float volts);

#ifdef __cplusplus
}
#endif

#endif /* INC_CURTIS_IO_H_ */
