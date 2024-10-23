/*
 * DAC.h
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */

#ifndef SRC_DAC_H_
#define SRC_DAC_H_

void DAC_init(void);
void DAC_write(uint16_t transmission_data);
uint16_t DAC_volt_conv (uint16_t voltage_value);

#endif /* SRC_DAC_H_ */
