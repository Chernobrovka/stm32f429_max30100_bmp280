#include "max30100.h"
#include <stdlib.h>
#include "stm32f4xx_hal_i2c.h"

void max30100_write(max30100_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen){
	uint8_t res_reg[2] = { reg, buf };
	HAL_I2C_Master_Transmit(obj->_ui2c, MAX30100_I2C_ADDR << 1, res_reg, 2, MAX30100_I2C_TIMEOUT);
}

void max30100_read(max30100_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen){
	uint8_t reg_addr = reg;
	HAL_I2C_Master_Transmit(obj->_ui2c, MAX30100_I2C_ADDR << 1, &reg_addr, 1, MAX30100_I2C_TIMEOUT);
	HAL_I2C_Master_Receive(obj->_ui2c, MAX30100_I2C_ADDR << 1, buf, buflen, MAX30100_I2C_TIMEOUT);
}

void max30100_reset(max30100_t *obj)
{
    uint8_t val = 0x40;
    max30100_write(obj, MAX30100_MODE_CONFIG, &val, 1);
}

/*uint8_t max30100_has_interrupt(max30100_t *obj)
{
    return obj->_interrupt_flag;
}*/

/*void max30100_interrupt_handler(max30100_t *obj)
{
    uint8_t reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    max30100_read(obj, MAX30100_INTERRUPT_STATUS, reg, 2);

    if ((reg[0] >> MAX30100_INTERRUPT_A_FULL) & 0x01)
    {
        // FIFO almost full
        max30100_read_fifo(obj);
    }

    if ((reg[0] >> MAX30100_INTERRUPT_HR_RDY) & 0x01)
    {
        // New FIFO data ready
    	max30100_read_fifo(obj);
    }

    if ((reg[0] >> MAX30100_INTERRUPT_SPO2_RDY) & 0x01)
    {
        // Ambient light overflow
    	max30100_read_fifo(obj);
    }

    if ((reg[1] >> MAX30100_INTERRUPT_TEMP_RDY) & 0x01)
    {
        // Temperature data ready
        int8_t temp_int;
        uint8_t temp_frac;
        max30100_read_temp(obj, &temp_int, &temp_frac);
        // float temp = temp_int + 0.0625f * temp_frac;
    }

    // Reset interrupt flag
    obj->_interrupt_flag = 0;
}*/

/*void max30100_on_interrupt(max30100_t *obj)
{
    obj->_interrupt_flag = 1;
}*/




