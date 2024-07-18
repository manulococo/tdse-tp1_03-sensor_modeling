/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_c_fsm.h
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

#ifndef TASK_INC_TASK_SENSOR_ATTRIBUTE_H_
#define TASK_INC_TASK_SENSOR_ATTRIBUTE_H_

/********************** CPP guard ********************************************/
#ifdef __cplusplus
extern "C" {
#endif

/********************** inclusions *******************************************/

/********************** macros ***********************************************/

/********************** typedef **********************************************/
/* Finite State Machine Task Sensor Table */
/* 	+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 * 	| Current               |                       | Event                 | Next                  |                       |
 * 	| State                 | [Guard]               | (Parameters)          | State                 | Actions               |
 * 	|=======================+=======================+=======================+=======================+=======================|
 * 	| ST_BTN_XX_UP          |                       | EV_BTN_XX_DOWN        | ST_BTN_XX_FALLING     | tick = tick_max       |
 * 	|-----------------------+-----------------------+-----------------------+-----------------------+-----------------------|
 * 	| ST_BTN_XX_FALLING     | [tick >  0]           |                       |                       | tick--                |
 * 	|                       +-----------------------+-----------------------+-----------------------+-----------------------|
 * 	|                       | [tick == 0]           | EV_BTN_XX_DOWN        | ST_BTN_XX_DOWN        | put_event_task_system |
 * 	|                       |                       |                       |                       |  (signal_down)        |
 * 	|                       |                       +-----------------------+-----------------------+-----------------------|
 * 	|                       |                       | EV_BTN_XX_UP          | ST_BTN_XX_UP          |                       |
 * 	|-----------------------+-----------------------+-----------------------+-----------------------+-----------------------|
 *	| ST_BTN_XX_DOWN        |                       | EV_BTN_XX_UP          | ST_BTN_XX_RISING      | tick = tick_max       |
 * 	|-----------------------+-----------------------+-----------------------+-----------------------+-----------------------|
 * 	| ST_BTN_XX_RISING      | [tick >  0]           |                       |                       | tick--                |
 * 	|                       +-----------------------+-----------------------+-----------------------+-----------------------|
 * 	|                       | [tick == 0]           | EV_BTN_XX_UP          | ST_BTN_XX_UP          | put_event_task_system |
 * 	|                       |                       |                       |                       |  (signal_up)          |
 * 	|                       |						+-----------------------+-----------------------+-----------------------|
 * 	|                       |                       | EV_BTN_XX_DOWN        | ST_BTN_XX_DOWN        |                       |
 * 	+-----------------------+-----------------------+-----------------------+-----------------------+-----------------------+
 */
/* Events to excite Task Sensor */
typedef enum task_sensor_ev {EV_BTN_XX_UP,
							 EV_BTN_XX_DOWN} task_sensor_ev_t;

/* States of Task Sensor */
typedef enum task_sensor_st {ST_BTN_XX_UP,
							 ST_BTN_XX_FALLING,
							 ST_BTN_XX_DOWN,
						     ST_BTN_XX_RISING} task_sensor_st_t;

/* Identifier of Task Sensor */
typedef enum task_sensor_id {ID_BTN_S1, ID_BTN_S2, ID_BTN_S3, ID_BTN_S4,
							 ID_BTN_K1, ID_BTN_K2, ID_BTN_A} task_sensor_id_t;

/* Signals of Task Sensor */
typedef enum task_sensor_sig {SIG_BTN_S1_UP, SIG_BTN_S1_DOWN,
						 	  SIG_BTN_S2_UP, SIG_BTN_S2_DOWN,
							  SIG_BTN_S3_UP, SIG_BTN_S3_DOWN,
							  SIG_BTN_S4_UP, SIG_BTN_S4_DOWN,
							  SIG_BTN_K1_UP, SIG_BTN_K1_DOWN,
							  SIG_BTN_K2_UP, SIG_BTN_K2_DOWN,
							  SIG_BTN_A_UP,  SIG_BTN_A_DOWN} task_sensor_sig_t;

typedef struct
{
	task_sensor_id_t	identifier;
	GPIO_TypeDef *		gpio_port;
	uint16_t			pin;
	GPIO_PinState		pressed;
	uint32_t			tick_max;
	task_sensor_sig_t	signal_up;
	task_sensor_sig_t	signal_down;
} task_sensor_cfg_t;

typedef struct
{
	uint32_t			tick;
	task_sensor_st_t	state;
	task_sensor_ev_t	event;
} task_sensor_dta_t;

/********************** external data declaration ****************************/

/********************** external functions declaration ***********************/

/********************** End of CPP guard *************************************/
#ifdef __cplusplus
}
#endif

#endif /* TASK_INC_TASK_SENSOR_ATTRIBUTE_H_ */

/********************** end of file ******************************************/
