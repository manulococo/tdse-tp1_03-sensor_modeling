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
 *
 * @file   : app.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes. */
#include <task_sensor.h>
#include "main.h"

/* Demo includes. */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes. */
#include "board.h"
#include "task_sensor.h"

/********************** macros and definitions *******************************/
#define G_APP_CNT_INI	0u

typedef struct {
	void (*p_task_x)(void *);	// Pointer to task
								// (must be a 'void (void *)' function)
	void *parameters;			// Pointer to parameters
} task_x_t;

/********************** internal data declaration ****************************/
task_x_t task_x_init_list[]		= {{task_sensor_init, NULL}};

#define TASK_X_INIT_QTY	(sizeof(task_x_init_list)/sizeof(task_x_t))

task_x_t task_x_update_list[]	= {{task_sensor_update, NULL}};

#define TASK_X_UPDATE_QTY	(sizeof(task_x_update_list)/sizeof(task_x_t))
/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
const char *p_sys	= " Bare Metal - Event-Triggered Systems (ETS)\r\n";
const char *p_app	= " App - Sensor Modeling\r\n";

/********************** external data declaration *****************************/
uint32_t g_app_cnt;
/********************** external functions definition ************************/
void app_init(void)
{
	uint32_t index;

	/* Print out: Application Initialized */
	LOGGER_LOG("\r\n");
	LOGGER_LOG("%s is running - Tick [mS] = %d\r\n", GET_NAME(app_init), (int)HAL_GetTick());

	LOGGER_LOG(p_sys);
	LOGGER_LOG(p_app);

	g_app_cnt = G_APP_CNT_INI;

	/* Print out: Application execution counter */
	LOGGER_LOG(" %s = %d\r\n", GET_NAME(g_app_cnt), (int)g_app_cnt);

	for (index = 0; TASK_X_INIT_QTY > index; index++)
	{
		/* Run task_x_init */
		(*task_x_init_list[index].p_task_x)(task_x_init_list[index].parameters);
	}
}

void app_update(void)
{
	uint32_t index;

	/* Update App Counter */
	g_app_cnt++;

	for (index = 0; TASK_X_UPDATE_QTY > index; index++)
	{
		/* Run task_x_update */
		(*task_x_update_list[index].p_task_x)(task_x_update_list[index].parameters);
	}
}

void HAL_SYSTICK_Callback(void)
{
	g_task_sensor_tick++;
}

/********************** end of file ******************************************/
