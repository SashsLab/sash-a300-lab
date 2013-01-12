/*
 * vars.h
 *
 *  Created on: May 2, 2012
 *      Author: sash
 */

#ifndef VARS_H_
#define VARS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** State enum */
typedef enum
{
	RESET,
	INIT,
	IDLE,
	RX_STATE,
	TX_STATE,
	ERROR,
} MCU_STATES;


#ifdef __cplusplus
}
#endif

/** @} (end group SegmentLcd) */
/** @} (end group Drivers) */


#endif /* VARS_H_ */
