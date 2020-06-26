/*==================[inlcusiones]============================================*/

#include "FreeRTOSConfig.h"								// Includes de FreeRTOS
#include "tipos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sapi.h"										// sAPI header
/*===============================[definiciones de datos internos]==============================*/

gpioMap_t teclas[] = {TEC3,TEC4};
gpioMap_t leds  [] = {LED2, LED3};

#define N_TECLAS sizeof(teclas)/sizeof(gpioMap_t) // 4*

tecla_led_t tecla_led_config[N_TECLAS];

/*===============================[definiciones de datos externos]==============================*/

DEBUG_PRINT_ENABLE;

/*============================[declaraciones de funciones internas]=========================*/

bool_t tecla_led_tarea_init(void);
bool_t tecla_led_tareas_crear(void);
extern bool_t tension = 0;

/*============================[declaraciones de funciones externas]=========================*/

/*==============================[Prototipo de funcion de la tarea]==========================*/

void tarea_tecla( void* taskParmPtr );
void tarea_led( void* taskParmPtr );
void supervisor( void* taskParmPtr );
void potenciometro( void* taskParmPtr );
void informar( void* taskParmPtr );

/*======================================[funcion principal]=================================*/

bool_t tecla_led_tarea_init(void)						//Creacion de tareas_LED
{
	uint8_t i;
	for(i=0; i < N_TECLAS; i++)
	{
		tecla_led_config[i].tecla = teclas[i];
		tecla_led_config[i].led = leds[i];
		tecla_led_config[i].sem_tec_pulsada = xSemaphoreCreateBinary();
		tecla_led_config[i].mutex		   = xSemaphoreCreateMutex();
	}
	return TRUE;
}

bool_t tecla_led_tareas_crear(void)
{
	uint8_t i;

	BaseType_t res_task_tecla[N_TECLAS];
	BaseType_t res_task_led[N_TECLAS];

	for( i=0; i < N_TECLAS; i++ )
	{
		res_task_tecla[i] =
				xTaskCreate(
						tarea_tecla,                     	// Funcion de la tarea a ejecutar
						(const char *)"tarea_tecla_1",      // Nombre de la tarea como String amigable para el usuario
						configMINIMAL_STACK_SIZE*2, 		// Cantidad de stack de la tarea
						&tecla_led_config[i],                          // Parametros de tarea
						tskIDLE_PRIORITY+1,         		// Prioridad de la tarea
						0                           		// Puntero a la tarea creada en el sistema
				);
		res_task_led[i] =
				xTaskCreate(
						tarea_led,                     		// Funcion de la tarea a ejecutar
						(const char *)"tarea_led_B",     	// Nombre de la tarea como String amigable para el usuario
						configMINIMAL_STACK_SIZE*2, 		// Cantidad de stack de la tarea
						&tecla_led_config[i], 	            // Parametros de tarea
						tskIDLE_PRIORITY+1,      		    // Prioridad de la tarea
						0                           	    // Puntero a la tarea creada en el sistema
				);
	}
	return TRUE;
}

int main(void)
{
   // ---------------------------------- CONFIGURACIONES ---------------------------------------
   boardConfig();										// Inicializar y configurar la plataforma

   debugPrintConfigUart( UART_USB, 115200 );			// UART for debug messages
   debugPrintlnString( "TP_Alejandro_Gorrin" );

   if (tecla_led_tarea_init() == FALSE )				// Inicializa arreglo tecla_led, crea su semaforo y mutex
	   return ERROR;

   if ( tecla_led_tareas_crear() == FALSE )				// creamos todas las tareas en freeRTOS
	   return ERROR;

   uint16_t muestra;
   bool_t tension;

   BaseType_t res =
      xTaskCreate(
         supervisor,                 					// Tarea supervisor, se encargara de enviar informacion de forma periodica
         (const char *)"myTask",     					// Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, 					// Cantidad de stack de la tarea
         0,                          					// Parametros de tarea
         tskIDLE_PRIORITY+1,         					// Prioridad de la tarea
         0                           					// Puntero a la tarea creada en el sistema
      );

      xTaskCreate(
         potenciometro,              					// Funcion de tarea que monitorea el potenciometro
         (const char *)"myTask",     					// Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, 					// Cantidad de stack de la tarea
         0,                          					// Parametros de tarea
         tskIDLE_PRIORITY+1,         					// Prioridad de la tarea
         0                           					// Puntero a la tarea creada en el sistema
      );

      xTaskCreate(
            informar,              	    				// Tarea OneShot que solo llamo para imprimir los valores
            (const char *)"myTask",     				// Nombre de la tarea como String amigable para el usuario
            configMINIMAL_STACK_SIZE*2, 				// Cantidad de stack de la tarea
            0,                          				// Parametros de tarea
            tskIDLE_PRIORITY+1,         				// Prioridad de la tarea
            0                           				// Puntero a la tarea creada en el sistema
         );

         if (res == pdFAIL){							//Gestionar el error
      	   debugPrintlnString( "No se lograron crear las tareas" );
         }

   vTaskStartScheduler();						// Iniciar scheduler

   // ------------------------------------ REPETIR POR SIEMPRE -----------------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==============================[definiciones de funciones internas]=============================*/

/*==============================[definiciones de funciones externas]=============================*/

// Implementacion de funcion de la tarea
void supervisor( void* taskParmPtr )									// Implementacion de funcion del Supervisor
{
   // ------------------------------------ CONFIGURACIONES ----------------------------------------
   gpioWrite( LED1, ON );
   vTaskDelay( 1000 / portTICK_RATE_MS );
   gpioWrite( LED1, OFF );

   portTickType xPeriodicity =  5000 / portTICK_RATE_MS;				// Tarea periodica cada 5000 ms
   portTickType xLastWakeTime = xTaskGetTickCount();

   // ---------------------------------- REPETIR POR SIEMPRE --------------------------------------
   while(TRUE) {

	   vTaskDelay( 50 / portTICK_RATE_MS );
	   gpioWrite(LEDB, ON);
	   debugPrintlnString( "Supervisor Acivo" );
	   vTaskDelay( 2450 / portTICK_RATE_MS );
	   gpioWrite(LEDB, OFF);
	   vTaskDelay( 2500 / portTICK_RATE_MS );
   }
   vTaskDelete(NULL);
}

void potenciometro( void* taskParmPtr )
{
	// ---------------------------------- CONFIGURACIONES -----------------------------------------

	pwmConfig( 0, PWM_ENABLE );												// Inicializar UART_USB a 115200 baudios
	pwmConfig( PWM7, PWM_ENABLE_OUTPUT );

	adcConfig( ADC_ENABLE );												// Inicializar AnalogIO
	dacConfig( DAC_ENABLE );

	static char uartBuff[10];												// Buffer

	uint16_t muestra = 0;													// Variable para almacenar el valor leido del ADC CH1

	portTickType xPeriodicity =  3000 / portTICK_RATE_MS;

	// --------------------------------- REPETIR POR SIEMPRE -----------------------------------------
	while(TRUE) {

		uint16_t b = 300;
		vTaskDelay( 2000 / portTICK_RATE_MS );

		muestra = adcRead( CH1 );											// Leo la Entrada Analogica AI0 - ADC0 CH1
		vTaskDelay( 50 / portTICK_RATE_MS );
		itoa( muestra, uartBuff, 10 ); 										// Conversión de muestra entera a ascii con base decimal
		vTaskDelay( 50 / portTICK_RATE_MS );
		if (muestra < 300){
			vTaskDelay( 2000 / portTICK_RATE_MS );
			uartWriteString( UART_USB, "Valor del potenciometro bajo: " );	// Envío la primer parte del mnesaje a la Uart
			vTaskDelay( 50 / portTICK_RATE_MS );
			uartWriteString( UART_USB, uartBuff );							// Enviar muestra y Enter
			vTaskDelay( 50 / portTICK_RATE_MS );
			uartWriteString( UART_USB, ";\r\n" );
		}
	}
}

void tarea_led( void* taskParmPtr )
{
	tecla_led_t* config = (tecla_led_t*) taskParmPtr;
	TickType_t diferencia;
	bool_t tension = 0;
   // ----------------------------------- REPETIR POR SIEMPRE ---------------------------------------
   while(TRUE) {

	   xSemaphoreTake(config->sem_tec_pulsada, portMAX_DELAY );

	   if (config->tecla == TEC3)
		   tension = !tension;

	   xSemaphoreTake(config->mutex, portMAX_DELAY);				// Se establece como seccion critica

	   if (config->tecla == TEC3)
	   {
		   debugPrintlnString( "Se pidio peticion" );
	   }
	   else
	   {
		   debugPrintlnString( "se activo cambio de tension" );
		   //tension =  !tension;
	   }

	   xSemaphoreGive(config->mutex);								// Se cierra la seccion critica

	   gpioWrite(config->led, ON);
	   vTaskDelay( diferencia );
	   gpioWrite(config->led, OFF);
   }
   vTaskDelete(NULL);
}

void tarea_tecla( void* taskParmPtr )
{
   // ------------------------------------- CONFIGURACIONES -----------------------------------------

	tecla_led_t* config = (tecla_led_t*) taskParmPtr;

	fsmButtonInit(config);

   // ----------------------------------- REPETIR POR SIEMPRE ---------------------------------------
   while(TRUE) {
	   fsmButtonUpdate( config );
	   vTaskDelay( 1 / portTICK_RATE_MS );
   }
   vTaskDelete(NULL);
}

void informar( void* taskParmPtr )									// Implementacion de funcion del Supervisor
{
   // ------------------------------------- CONFIGURACIONES -----------------------------------------
	TickType_t diferencia = *((TickType_t*)taskParmPtr);

	bool_t tension;
	uint16_t muestra;

	pwmConfig( 0, PWM_ENABLE );										// Inicializar UART_USB a 115200 baudios
	pwmConfig( PWM7, PWM_ENABLE_OUTPUT );

	adcConfig( ADC_ENABLE );										// Inicializar AnalogIO
	dacConfig( DAC_ENABLE );

	static char uartBuff[10];
	muestra = adcRead( CH1 );
	muestra = adcRead( CH1 );										// Leo la Entrada Analogica AI0 - ADC0 CH1
	vTaskDelay( 50 / portTICK_RATE_MS );
	itoa( muestra, uartBuff, 10 ); 									// Conversión de muestra entera a ascii con base decimal

	if ((tension == 0) && (muestra > 300) ){
		debugPrintlnString( "Todos los estados se encuentran estables" );
	}
	else if ((tension == 1) && (muestra > 300)){
		debugPrintlnString( "Se encontro problemas de tension" );
	}
	else if ((tension == 0) && (muestra < 300)){
		debugPrintlnString( "Se encontro problemas de bateria" );
	}
	else if ((tension == 1) && (muestra < 300)){
		debugPrintlnString( "Estado de alerta maxima" );
	}
	vTaskDelete(NULL);
}

/*===========================================[fin del archivo]=============================================*/
