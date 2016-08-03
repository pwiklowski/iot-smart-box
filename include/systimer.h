/*
 * systimer.h
 *
 *  Created on: 27 maj 2016
 *      Author: pawwik
 */

#ifndef SYSTIMER_H_
#define SYSTIMER_H_


void delay_ms(uint32_t ms);

void mstimer_init(void);


uint32_t mstimer_get(void);




#endif /* SYSTIMER_H_ */
