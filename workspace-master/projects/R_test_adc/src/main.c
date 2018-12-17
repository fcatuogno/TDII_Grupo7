#include "../../R_test_adc/inc/main.h"
#include "../../R_test_adc/inc/FreeRTOSConfig.h"
#include "../../R_test_adc/inc/motor.h"
#include "../../R_test_adc/inc/joystick.h"
#include "FreeRTOS.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
/* ADC
 * aplicacion : Lectura del Joystick
 * frecuencia sampling: 100 Hz, 1ms  */
#define FsADC		10
/* Colas para los datos del ADC*/
/* Ejes */
xQueueHandle 		colaDatos;


/* Inicializacion del ADC
 * canales a utilizar: CH1 , CH2
 * frecuencia de muestreo : fs
 * modo de lectura : BURST
 * prioridad de interrupcion: mayor
 *  */
void InitADC(int fs)
{
	ADC_CLOCK_SETUP_T adc;
	Chip_ADC_Init(LPC_ADC, &adc);
	Chip_ADC_SetSampleRate(LPC_ADC, &adc, fs);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH2, ENABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC, ADC_CH2, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, ENABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC, ADC_CH1, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
	NVIC_SetPriority(ADC_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);
}

void ADC_IRQHandler(void)
{
	portBASE_TYPE contexto;
	static JOYSTICK_AXIS datos={0,0};
	/* Deshabilito la interrupcion del ADC para que no me interrumpa en el procesamiento */
	NVIC_DisableIRQ(ADC_IRQn);
	/* Pregunto y guardo los datos del adc */
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH2, &( datos.datos_y) ); // CANAL_2
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH1, &(datos.datos_x) ); // CANAL_1
	/* Los mando por una cola */
	xQueueSendFromISR(colaDatos,&(datos),&contexto);
	/* Vuelvo habilitar la interrupcion del ADC */
	NVIC_EnableIRQ(ADC_IRQn);
	/* Cambio de contexto */
	portEND_SWITCHING_ISR(contexto);
}

static void initHardware(void)
{
    SystemCoreClockUpdate();
	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);
	/* Inicializacion de los pines del ADC: canales y funcion */
    Chip_IOCON_PinMux(LPC_IOCON,CANAL_2,IOCON_MODE_INACT,IOCON_FUNC1);
    Chip_IOCON_PinMux(LPC_IOCON,CANAL_1,IOCON_MODE_INACT,IOCON_FUNC1);
    Motor_Init(2,FREQ_50HZ);
    Motor_Init(3,FREQ_50HZ);
    /* Inicializo el ADC : frecuencia de muestreo */
    InitADC(FsADC);

}
/* Verifica que el angulo este dentro de los permitidos */

/* Tarea parametrizada : Lectura de datos*/
void LecturaDeDatos(void * param)
{
	static int32_t ang_act_X=0,ang_ant_X=0;
	static int32_t ang_act_Y=0,ang_ant_Y=0;
	static int16_t debounce_x=0,debounce_y=0;
	static uint8_t time=0;
	static JOYSTICK_AXIS datos;
	while (1)
	{
		xQueueReceiveFromISR(colaDatos,&(datos),portMAX_DELAY);
			debounce_x=debounce_x + datos.datos_x;
			debounce_y=debounce_y + datos.datos_y;
			time++;

			if( time == DEBOUNCE)
			{	datos.datos_x=debounce_x/time;
				datos.datos_y=debounce_y/time;
				/**********		EJE X	*************************************************/
					/*	JOYSTICK :		SIN PULSAR		*/
				if ( ( DATO_MIN_I < datos.datos_x ) &&  ( datos.datos_x < DATO_MAX_I) )
				{
					Chip_GPIO_WritePortBit(LPC_GPIO, LED, TRUE);
				}
				/*	JOYSTICK : PULSO DECREMENTAR	*/
				else if ( datos.datos_x < DATO_MIN_I)
				{
					Chip_GPIO_WritePortBit(LPC_GPIO, LED, FALSE);
					/* Cambio el angulo */
					ang_act_X=ang_ant_X - DELTA_ANG;
					/* Muevo con el servomotor a una posicion */
					Motor_Set(2,ang_act_X);
					JOYSTICK_AngleLimit(&ang_act_X);
					ang_ant_X=ang_act_X;
				}
				/*	JOYSTICK : PULSO INCREMENTAR	*/
				else if( datos.datos_x > DATO_MAX_I)
				{
					Chip_GPIO_WritePortBit(LPC_GPIO, LED, FALSE);
					/* Cambio el angulo */
					ang_act_X=ang_ant_X + DELTA_ANG;
					/* Muevo con el servomotor a una posicion */
					Motor_Set(2, ang_act_X);
					JOYSTICK_AngleLimit(&ang_act_X);
					ang_ant_X=ang_act_X;
				}
				/**********		EJE Y	*************************************************/
				/*	JOYSTICK :		SIN PULSAR		*/
				if ( (DATO_MIN_I < datos.datos_y) && ( datos.datos_y < DATO_MAX_I) )
				{
					Chip_GPIO_WritePortBit(LPC_GPIO, LED, TRUE);
				}
				/*	JOYSTICK : PULSO DECREMENTAR	*/
				else if ( datos.datos_y < DATO_MIN_I)
				{
					Chip_GPIO_WritePortBit(LPC_GPIO, LED, FALSE);
					/* Cambio el angulo */
					ang_act_Y=ang_ant_Y - DELTA_ANG;
					/* Muevo con el servomotor a una posicion */
					Motor_Set(3,ang_act_Y);
					JOYSTICK_AngleLimit(&ang_act_Y);
					ang_ant_Y=ang_act_Y;
				}
				/*	JOYSTICK : PULSO INCREMENTAR	*/
				else if( datos.datos_y > DATO_MAX_I)
				{
					Chip_GPIO_WritePortBit(LPC_GPIO, LED, FALSE);
					/* Cambio el angulo */
					ang_act_Y=ang_ant_Y + DELTA_ANG;
					/* Muevo con el servomotor a una posicion */
					Motor_Set(3,ang_act_Y);
					JOYSTICK_AngleLimit(&ang_act_Y);
					ang_ant_Y=ang_act_Y;
				}

				debounce_x=0;
				debounce_y=0;
				time=0;
			}
	}
}
int main(void)
{

	initHardware();
	colaDatos = xQueueCreate(10, sizeof(JOYSTICK_AXIS));
	xTaskCreate(LecturaDeDatos, (const char *)"Datos", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+1, 0);
	NVIC_EnableIRQ(ADC_IRQn);
	vTaskStartScheduler();
	for(;;);
}

