/*
 * io.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_DRIVERS_IO_H_
#define INC_DRIVERS_IO_H_

#include <stdbool.h>

bool IO_IsHomeSensorTriggered(void);

bool IO_IsEmergencyPressed(void);

bool IO_IsResetPressed(void);

void IO_SetSSR(bool state);

#endif /* INC_DRIVERS_IO_H_ */
