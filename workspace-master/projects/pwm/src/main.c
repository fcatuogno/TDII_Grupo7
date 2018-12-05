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
#include "chip.h"
#include "main.h"
#include "motor.h"


/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);
/** @brief delay function
* @param t desired milliseconds to wait
*/

static void pausems(uint32_t t);

/*==================[internal data definition]===============================*/
/** @brief used for delay counter */
static uint32_t pausems_count;
static void initHardware(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);
	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);
	/*Inicializamos el pwm*/
    Motor_Init(CHANNELs,FREQ_50HZ);
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

     pausems(100);

/*      switch(i)
      {
      case 0:

    	  Motor_Set(CH1,ANGLE_P45);
    	  i++;
    	  break;
      case 1:
    	  Motor_Set(CH1,ANGLE_N45);
    	  i++;
    	  break;
      case 2:
    	  Motor_Set(CH1,ANGLE_0);
    	  i=0;
    	  break;
      } */
   }/* End while */
} /* End Main*/

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
