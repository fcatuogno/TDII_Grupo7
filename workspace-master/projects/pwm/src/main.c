/* Copyright 2015, Pablo Ridolfi
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief PWM example with Timer.
 *
 * PWM signal on Board LED 0, period 1ms, duty between 10% and 90%.
 */

 /** \addtogroup pwm PWM example
 ** @{ */
#include "board.h"
#include "main.h"
/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);
/** @brief delay function
* @param t desired milliseconds to wait
*/

#define FREQ_50HZ	475600 // cantidad de ticks para llegar a los 20mseg.
/*
#define ANGLE_N90	475600/40	// a los 0.5mseg
#define ANGLE_N45	475600/20	// a los 1mseg
#define ANGLE_0		475600*0.075	// a los 1.5mseg
#define ANGLE_P45	475600/10	// a los 2mseg
#define ANGLE_P90	475600*0.11	// a los 2.2mseg //
#define ANGLE_PX 	475600*0.12 // a los 2.4mseg
*/

#define ANGLE_N95	475600/40		// a los 0.5mseg
#define ANGLE_N90	475600*0.03		// a los 0.6mseg
#define ANGLE_N45	475600*0.05125	// a los 1.025mseg
#define ANGLE_DELTA1 475600*0.0094*0.5  // salto de 1 grado
#define ANGLE_0 	475600*0.0725	// a los 1.45mseg
#define ANGLE_P45	475600*0.09375  // a los 1.875mseg
#define ANGLE_P90 	475600*0.115	// a los 2.3mseg
#define ANGLE_P95 	475600*0.12 	// a los 2.4mseg

static void pausems(uint32_t t);

/*==================[internal data definition]===============================*/
/** @brief used for delay counter */
static uint32_t pausems_count;
static void initHardware(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);
    Board_Init();
    Board_LED_Set(0, false);
   /******************* PWM *******************/
    /* Configuramos los pines con la funcion PWM */
	PWM1_SelectChannel(LPC_PWM1,CH1);
	PWM1_SelectChannel(LPC_PWM1,CH2);
	PWM1_SelectChannel(LPC_PWM1,CH3);
	PWM1_SelectChannel(LPC_PWM1,CH4);
	/* Habilitamos el clock y su prescale para el modulo */
	PWM1_enableCLK(LPC_PWM1);
	PWM1_preescaleCLK(LPC_PWM1,0);

	/* Match 0 (period) */
	PWM1_ValueMatch(LPC_PWM1,CH0,FREQ_50HZ);//
	/* Match 1 (duty) */
	PWM1_ValueMatch(LPC_PWM1,CH1,ANGLE_0);
	/* Match 2 (duty) */
	PWM1_ValueMatch(LPC_PWM1,CH2,ANGLE_0);
	/* Match 3 (duty) */
	PWM1_ValueMatch(LPC_PWM1,CH3,ANGLE_0);
	/* Match 4 (duty) */
	PWM1_ValueMatch(LPC_PWM1,CH4,ANGLE_0);

	/* Definimos que el MR0 resetee el TC */
	PWM1_ConfigMatch(LPC_PWM1,CH0,0,0,0);

	/* Activamos el modo PWM y habilitamos el conteo de TC y PC */
	PWM1_EnableCounters(LPC_PWM1);

	/* Habilito los cambios en los MATCH's */
	PWM1_EnableMatchValue(LPC_PWM1,CH0);
	PWM1_EnableMatchValue(LPC_PWM1,CH1);
	PWM1_EnableMatchValue(LPC_PWM1,CH2);
	PWM1_EnableMatchValue(LPC_PWM1,CH3);
	PWM1_EnableMatchValue(LPC_PWM1,CH4);

	/* Habilito las salidas del canal */
	PWM1_ControlChannel(LPC_PWM1,CH1,0,1);
	PWM1_ControlChannel(LPC_PWM1,CH2,0,1);
	PWM1_ControlChannel(LPC_PWM1,CH3,0,1);
	PWM1_ControlChannel(LPC_PWM1,CH4,0,1);

	PWM1_ResetCounters(LPC_PWM1);
}
static void pausems(uint32_t t)
{
   pausems_count = t;
   while(pausems_count != 0) {
      __WFI();
   }
}
void SysTick_Handler(void)
{
   if (pausems_count != 0) {
      pausems_count--;
   }
}
int main(void)
{
	initHardware();
	uint8_t i=1;
   while(1) {

      pausems(1000);

      if(i==1)
     // Cambio los valores de los match's
      {PWM1_ValueMatch(LPC_PWM1,CH1,ANGLE_0 + ANGLE_DELTA1);
      PWM1_ValueMatch(LPC_PWM1,CH2,ANGLE_0  + ANGLE_DELTA1);
      PWM1_ValueMatch(LPC_PWM1,CH3,ANGLE_0 + ANGLE_DELTA1);
      PWM1_ValueMatch(LPC_PWM1,CH4,ANGLE_0 + ANGLE_DELTA1);
  	 // Habilito los cambios en los MATCH's
      PWM1_EnableMatchValue(LPC_PWM1,CH0);
      PWM1_EnableMatchValue(LPC_PWM1,CH1);
      PWM1_EnableMatchValue(LPC_PWM1,CH2);
      PWM1_EnableMatchValue(LPC_PWM1,CH3);
      PWM1_EnableMatchValue(LPC_PWM1,CH4);
      i++;
      }
      if(i==4)
	   // Cambio los valores de los match's
		{PWM1_ValueMatch(LPC_PWM1,CH1,ANGLE_0);
		PWM1_ValueMatch(LPC_PWM1,CH2,ANGLE_0);
		PWM1_ValueMatch(LPC_PWM1,CH3,ANGLE_0);
		PWM1_ValueMatch(LPC_PWM1,CH4,ANGLE_0);
		 // Habilito los cambios en los MATCH's
		PWM1_EnableMatchValue(LPC_PWM1,CH0);
		PWM1_EnableMatchValue(LPC_PWM1,CH1);
		PWM1_EnableMatchValue(LPC_PWM1,CH2);
		PWM1_EnableMatchValue(LPC_PWM1,CH3);
		PWM1_EnableMatchValue(LPC_PWM1,CH4);
		i=1;
		}
      if(i==2)
	   // Cambio los valores de los match's
		{PWM1_ValueMatch(LPC_PWM1,CH1,ANGLE_0);
		PWM1_ValueMatch(LPC_PWM1,CH2,ANGLE_0);
		PWM1_ValueMatch(LPC_PWM1,CH3,ANGLE_0);
		PWM1_ValueMatch(LPC_PWM1,CH4,ANGLE_0);
		 // Habilito los cambios en los MATCH's
		PWM1_EnableMatchValue(LPC_PWM1,CH0);
		PWM1_EnableMatchValue(LPC_PWM1,CH1);
		PWM1_EnableMatchValue(LPC_PWM1,CH2);
		PWM1_EnableMatchValue(LPC_PWM1,CH3);
		PWM1_EnableMatchValue(LPC_PWM1,CH4);
		i++;
		}
      if(i==3)
	   // Cambio los valores de los match's
		{PWM1_ValueMatch(LPC_PWM1,CH1,ANGLE_0 - ANGLE_DELTA1);
		PWM1_ValueMatch(LPC_PWM1,CH2,ANGLE_0 - ANGLE_DELTA1);
		PWM1_ValueMatch(LPC_PWM1,CH3,ANGLE_0 - ANGLE_DELTA1);
		PWM1_ValueMatch(LPC_PWM1,CH4,ANGLE_0 - ANGLE_DELTA1);
		 // Habilito los cambios en los MATCH's
		PWM1_EnableMatchValue(LPC_PWM1,CH0);
		PWM1_EnableMatchValue(LPC_PWM1,CH1);
		PWM1_EnableMatchValue(LPC_PWM1,CH2);
		PWM1_EnableMatchValue(LPC_PWM1,CH3);
		PWM1_EnableMatchValue(LPC_PWM1,CH4);
		i++;
		}

   }/* End while */
} /* End Main*/

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
