/*
 * max30100.h
 *
 *  Created on: Apr 4, 2023
 *      Author: Dmitry
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

#include "main.h"
#include <stdbool.h>

#define ALPHA 0.95
#define MEAN_FILTER_SIZE	15

#define MAX30100_I2C_ADDR 0x57 // 7-bit I2C adress , for 8-bit i2c have another adress
#define MAX30100_I2C_TIMEOUT 1000

#define MAX30100_BYTES_PER_SAMPLE 4
#define MAX30100_SAMPLE_LEN_MAX 16

// STATUS
#define MAX30100_INTERRUPT_STATUS 0x00
#define MAX30100_INTERRUPT_ENABLE 0x01
#define MAX30100_INTERRUPT_A_FULL 7
#define MAX30100_INTERRUPT_TEMP_RDY 6
#define MAX30100_INTERRUPT_HR_RDY 5
#define MAX30100_INTERRUPT_SPO2_RDY 4
#define MAX30100_INTERRUPT_PWR_RDY 0

// FIFO
#define MAX30100_FIFO_WR_PTR 0x02
#define MAX30100_OVF_COUNTER 0x03
#define MAX30100_FIFO_RD_PTR 0x04

#define MAX30100_FIFO_DATA 0x05

//CONFIGURATION
#define MAX30100_MODE_CONFIG 0x06
#define MAX30100_MODE_SHDN 7
#define MAX30100_MODE_RESET 6
#define MAX30100_MODE_TEMP_EN 3
#define MAX30100_MODE_MODE 0

#define MAX30100_SPO2_CONFIG 0x07
#define MAX30100_SPO2_HI_RES_EN 6
#define MAX30100_SPO2_SR 2
#define MAX30100_SPO2_LED_PW 0

#define MAX30100_LED_CONFIGURATION 0x09
#define MAX30100_LED_IR_PA 0
#define MAX30100_LED_RED_PA 4

// TEMPERATURE
#define MAX30100_TEMP_INTEGER 0x16
#define MAX30100_TEMP_FRACTION 0x17

//PART ID
#define MAX30100_REVISION_ID 0xFE
#define MAX30100_PART_ID 0xFF
#define EXPECTED_PART_ID 0x11

#define PULSE_BPM_SAMPLE_SIZE 10

typedef enum max30100_mode_t
{
    max30100_heart_rate = 0x02,
    max30100_spo2 = 0x03
} max30100_mode_t;

#define DEFAULT_MODE 0x02 // heart_rate_mode

typedef enum max30100_smp_ave_t
{
	max30100_smp_ave_1,
	max30100_smp_ave_2,
	max30100_smp_ave_4,
	max30100_smp_ave_8,
	max30100_smp_ave_16,
	max30100_smp_ave_32,
} max30100_smp_ave_t;

typedef enum max30100_sr_t
{
    max30100_sr_50 = 0x00,
    max30100_sr_100 = 0x01,
    max30100_sr_167 = 0x02,
    max30100_sr_200 = 0x03,
    max30100_sr_400 = 0x04,
    max30100_sr_600 = 0x05,
    max30100_sr_800 = 0x06,
    max30100_sr_1000 = 0x07
} max30100_sr_t;

#define DEFAULT_SAMPLING_RATE 0x01

typedef enum max30100_adc_t
{
    max30100_adc_13_bit,
    max30100_adc_14_bit,
    max30100_adc_15_bit,
    max30100_adc_16_bit
} max30100_adc_t;

/*typedef enum max30100_led_pw_t{
	max30100_pw_200,
	max30100_pw_400,
	max30100_pw_800,
	max30100_pw_1600
}max30100_led_pw_t;*/

typedef enum max30100_pa_t{
	max30100_red_pa_0 = 0x00,
	max30100_red_pa_4_5 = 0x01,
	max30100_red_pa_7_6 = 0x02,
	max30100_red_pa_11 = 0x03,
	max30100_red_pa_14_2 = 0x04,
	max30100_red_pa_17_4 = 0x05,
	max30100_red_pa_20_8 = 0x06,
	max30100_red_pa_24 = 0x07,
	max30100_red_pa_27_1 = 0x08,
	max30100_red_pa_30_6 = 0x09,
	max30100_red_pa_33_8 = 0x0A,
	max30100_red_pa_37 = 0x0B,
	max30100_red_pa_40_2 = 0x0C,
	max30100_red_pa_43_6 = 0x0D,
	max30100_red_pa_46_8 = 0x0E,
	max30100_red_pa_50 = 0x0F,
} max30100_pa_t;

typedef enum max30100_hi_res_en_t{
	max_30100_disable = 0x00,
	max_30100_enable = (1 << 6)
}max30100_hi_res_en_t;

typedef struct dc_filter_t {
  float w;
  float result;
} dc_filter_t;

typedef enum max30100_leds_t{
	red_led,
	ir_led
} max30100_leds_t;

typedef enum{
	so2, // so2 interrupt
	hr, // heart-rate interrupt
	temp,	// temperature interrupt
	full, // fifo full interrupt
} max30100_interrpt_sourse_t;

typedef struct max30100_t
{
    I2C_HandleTypeDef *_ui2c;
    uint32_t _ir_samples[16];
    uint32_t _red_samples[16];
    uint8_t _interrupt_flag;

    float heart_BPM;
    float SPO2;
    float _dc_filter_ir;
    float _dc_filter_red;
    float ir_AC_value_Sq_Sum;
    float red_AC_value_Sq_Sum;
    uint16_t samples_recorded;
    uint16_t pulses_detected;
    float current_SaO2_value;
    uint8_t bpmIndex;

    uint8_t RED_led_current;
    float last_RED_led_current_check;
    uint8_t current_pulse_detector_state;
    float current_bmp;
    float values_bmp[PULSE_BPM_SAMPLE_SIZE];
    float values_bmp_sum;
    uint8_t values_bmp_count;
    uint32_t last_beat_threshold;
    uint8_t bmp_index;
} max30100_t;

void max30100_init(max30100_t *obj, I2C_HandleTypeDef *hi2c);
void max30100_write(max30100_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);
void max30100_read(max30100_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);
void max30100_reset(max30100_t *obj);
bool max30100_begin(max30100_t *obj);

void max30100_set_a_full(max30100_t *obj, uint8_t enable);


void max30100_shutdown(max30100_t *obj, uint8_t shdn);

void max30100_set_mode(max30100_t *obj, max30100_mode_t mode);

void max30100_led_pa_config(max30100_t *obj, max30100_pa_t pa, max30100_leds_t led);


// configuration
void max30100_set_sampling_rate(max30100_t *obj, max30100_sr_t sr);

void max30100_set_adc_resolution(max30100_t *obj, max30100_adc_t adc);

//void max30100_set_led_pulse_width(max30100_t *obj, max30100_led_pw_t pw);

void max30100_spo2_hi_res_en(max30100_t *obj, max30100_hi_res_en_t hi_res);

// fifo buffer
void max30100_clear_fifo(max30100_t *obj);

void max30100_read_fifo(max30100_t *obj);

// temperature
float max30100_read_temp(max30100_t *obj);

uint8_t get_part_id(max30100_t *obj);

//interrupt
//uint8_t max30100_has_interrupt(max30100_t *obj);
//void max30100_interrupt_handler(max30100_t *obj);
//void max30100_on_interrupt(max30100_t *obj);


// filter of ir and red leds
#define PULSE_MAX_THRESHOLD	2000
#define PULSE_MIN_THRESHOLD	100
#define PULSE_GO_DOWN_THRESHOLD 1

/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF 65000
#define RED_LED_CURRENT_ADJUSTMENT_MS 500

typedef struct meanDiffFilter_t
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
} meanDiffFilter_t;

typedef struct butterworthFilter_t
{
  float value[2];
  float result;
} butterworthFilter_t;

typedef enum pulse_state_machine {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} pulse_state_machine;

float dc_remove(float x, float prev_w, float alpha);

float mean_diff(float present_value, meanDiffFilter_t* filterValues);

void balanceIntesities(max30100_t *obj, float redLedDC, float IRLedDC );

void butterworth_filter(float x, butterworthFilter_t * filterResult);

bool max30100_detect_pulse(max30100_t *obj ,float sensor_value);

#endif /* INC_MAX30100_H_ */
