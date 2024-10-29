/*
 * LULtables.h
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */

#ifndef INC_LUTS_H_
#define INC_LUTS_H_

extern const uint16_t sine[1720]; //
extern const uint16_t ramp[1720];
extern const uint16_t triangle[1720];
void gen_square_wave(uint16_t *square_array, uint8_t dc);

#endif /* INC_LUTS_H_ */
