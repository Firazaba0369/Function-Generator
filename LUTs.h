/*
 * LULtables.h
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */

#ifndef INC_LUTS_H_
#define INC_LUTS_H_

extern const uint16_t sine[588];
//extern uint16_t* square;
extern const uint16_t ramp[588];
extern const uint16_t triangle[588];
void gen_square_wave(uint16_t *square_array, uint8_t duty);

#endif /* INC_LUTS_H_ */
