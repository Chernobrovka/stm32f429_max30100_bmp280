/*
 * max30100.c
 *
 *  Created on: Apr 4, 2023
 *      Author: Dmitry
 */
#include "max30100.h"


void max30100_init(max30100_t *obj, I2C_HandleTypeDef *hi2c)
{
    obj->_ui2c = hi2c;
    obj->_interrupt_flag = 0;

    obj->current_pulse_detector_state;
    obj->last_RED_led_current_check = 0;

    memset(obj->_ir_samples, 0, MAX30100_SAMPLE_LEN_MAX * sizeof(uint16_t));
    memset(obj->_red_samples, 0, MAX30100_SAMPLE_LEN_MAX * sizeof(uint16_t));
}

//void max30100_update()

void max30100_set_a_full(max30100_t *obj, uint8_t enable)
{
    uint8_t reg = 0;
    max30100_read(obj, MAX30100_INTERRUPT_ENABLE, &reg, 1);
    reg &= ~(0x01 << MAX30100_INTERRUPT_A_FULL);
    reg |= ((enable & 0x01) << MAX30100_INTERRUPT_A_FULL);
    max30100_write(obj, MAX30100_INTERRUPT_ENABLE, &reg, 1);
}

void max30100_shutdown(max30100_t *obj, uint8_t shdn)
{
    uint8_t config;
    max30100_read(obj, MAX30100_MODE_CONFIG, &config, 1);
    config = (config & 0x7f) | (shdn << MAX30100_MODE_SHDN);
    max30100_write(obj, MAX30100_MODE_CONFIG, &config, 1);
}

void max30100_set_mode(max30100_t *obj, max30100_mode_t mode)
{
    uint8_t config;
    max30100_read(obj, MAX30100_MODE_CONFIG, &config, 1);
    config = (config & 0xF8) | (uint8_t)mode;
    max30100_write(obj, MAX30100_MODE_CONFIG, &config, 1);
    max30100_clear_fifo(obj);
}

// configuration
void max30100_set_sampling_rate(max30100_t *obj, max30100_sr_t sr)
{
    uint8_t config;
    max30100_read(obj, MAX30100_SPO2_CONFIG, &config, 1);
    config = config | ((uint8_t) (sr << 2));
    max30100_write(obj, MAX30100_SPO2_CONFIG, &config, 1);
}

/*void max30100_set_led_pulse_width(max30100_t *obj, max30100_led_pw_t pw)
{
    uint8_t config;
    max30100_read(obj, MAX30100_SPO2_CONFIG, &config, 1);
    config = config | pw;
    max30100_write(obj, MAX30100_SPO2_CONFIG, &config, 1);
}*/

void max30100_set_adc_resolution(max30100_t *obj, max30100_adc_t adc)
{
    uint8_t config;
    max30100_read(obj, MAX30100_SPO2_CONFIG, &config, 1);
    config = config | (uint8_t) adc;
    max30100_write(obj, MAX30100_SPO2_CONFIG, &config, 1);
}

void max30100_spo2_hi_res_en(max30100_t *obj, max30100_hi_res_en_t hi_res)
{
	uint8_t config;
	max30100_read(obj, MAX30100_SPO2_CONFIG, &config, 1);
	config = config | ((uint8_t) (hi_res << 6));
	max30100_write(obj, MAX30100_SPO2_CONFIG, &config, 1);
}

void max30100_led_pa_config(max30100_t *obj, max30100_pa_t pa, max30100_leds_t led){
	uint8_t config;

	if (led == 0){
		max30100_read(obj, MAX30100_LED_CONFIGURATION, &config, 1);
		config = config | pa;	// set OR for: 0000xxxx
		max30100_write(obj, MAX30100_LED_CONFIGURATION, &config, 1);
	}
	else if (led == 1){
		max30100_read(obj, MAX30100_LED_CONFIGURATION, &config, 1);
		config = config | (pa << 4); // set OR for: xxxx0000
		max30100_write(obj, MAX30100_LED_CONFIGURATION, &config, 1);
	}
}

// fifo buffer
void max30100_clear_fifo(max30100_t *obj)
{
    uint8_t val = 0x00;
    max30100_write(obj, MAX30100_FIFO_WR_PTR, &val, 4);
    max30100_write(obj, MAX30100_OVF_COUNTER, &val, 4);
    max30100_write(obj, MAX30100_FIFO_RD_PTR, &val, 4);
}

void max30100_read_fifo(max30100_t *obj)
{
    // First transaction: Get the FIFO_WR_PTR
    uint8_t wr_ptr = 0, rd_ptr = 0;
    max30100_read(obj, MAX30100_FIFO_WR_PTR, &wr_ptr, 1);
    max30100_read(obj, MAX30100_FIFO_RD_PTR, &rd_ptr, 1);

    int8_t num_samples;

    num_samples = ((int8_t)wr_ptr - (int8_t)rd_ptr) & (0x10 - 1); // 0x10 -- fifo buffer depth

    num_samples = num_samples * 4;
//    if (num_samples < 1)
//    {
//        num_samples += 16;
//    }

    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
    for (int8_t i = 0; i < num_samples; i++)
    {
        uint8_t sample[4];
        max30100_read(obj, MAX30100_FIFO_DATA, sample, 4);
        uint32_t ir_sample = ((uint32_t)(sample[0] << 8) | (uint32_t)(sample[1])) ;
        uint32_t red_sample = ((uint32_t)(sample[2] << 8) | (uint32_t)(sample[3]));
        obj->_ir_samples[i] = ir_sample;
        obj->_red_samples[i] = red_sample;
        //max30100_plot(ir_sample, red_sample);
    }
}

// Temperature
float max30100_read_temp(max30100_t *obj)
{
	uint8_t temp_int;
	uint8_t temp_frac;
	float temp;
    max30100_read(obj, MAX30100_TEMP_INTEGER, &temp_int, 1);
    max30100_read(obj, MAX30100_TEMP_FRACTION, &temp_frac, 1);
    temp = (float)temp_int + ( (float)temp_frac / 100.0 );
    return temp;
}



