#include "FreeRTOSConfig.h"
#include "tipos.h"
#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"

void fsmButtonError( tecla_led_t* config );
void fsmButtonInit( tecla_led_t* config );
void fsmButtonUpdate( tecla_led_t* config );
void buttonPressed( tecla_led_t* config );
void buttonReleased( tecla_led_t* config );
void informar( void* taskParmPtr );

void tarea_led( void* taskParmPtr );

void buttonPressed( tecla_led_t* config )
{
	config->tiempo_down = xTaskGetTickCount();
}

void buttonReleased( tecla_led_t* config )
{
	config->tiempo_up = xTaskGetTickCount();

	xSemaphoreTake(config->mutex, portMAX_DELAY);		// Se abre seccion critica


	BaseType_t res =
			xTaskCreate(
				  informar,              	    			// Tarea OneShot que solo llamo para imprimir los valores
				  (const char *)"myTask",     				// Nombre de la tarea como String amigable para el usuario
				  configMINIMAL_STACK_SIZE*2, 				// Cantidad de stack de la tarea
				  0,                          				// Parametros de tarea
				  tskIDLE_PRIORITY+1,         				// Prioridad de la tarea
				  0                           				// Puntero a la tarea creada en el sistema
			   );

	xSemaphoreGive(config->mutex);						// Se cierra seccion critica

	xSemaphoreGive( config->sem_tec_pulsada);
}

void fsmButtonError( tecla_led_t* config )
{
   config->fsmButtonState = STATE_BUTTON_UP;
}

void fsmButtonInit( tecla_led_t* config )
{
   config->fsmButtonState = STATE_BUTTON_UP;
   config->contRising = 0;
   config->contFalling = 0;
}

void fsmButtonUpdate( tecla_led_t* config )
{
   switch( config->fsmButtonState )
   {
      case STATE_BUTTON_UP: 
         if( !gpioRead(config->tecla) ){
            config->fsmButtonState = STATE_BUTTON_FALLING;
         }
      break;

      case STATE_BUTTON_DOWN:
         if( gpioRead(config->tecla) ){
        	 config->fsmButtonState = STATE_BUTTON_RISING;
         }
      break;

      case STATE_BUTTON_FALLING:      
         if( config->contFalling >= DEBOUNCE_TIME ){
            if( !gpioRead(config->tecla) ){
            	config->fsmButtonState = STATE_BUTTON_DOWN;
               buttonPressed(config);
            } else{
            	config->fsmButtonState = STATE_BUTTON_UP;
            }
            config->contFalling = 0;
         }
         config->contFalling++;
         break;

      case STATE_BUTTON_RISING:      
         if( config->contRising >= DEBOUNCE_TIME ){
            if( gpioRead(config->tecla) )
            {
            	config->fsmButtonState = STATE_BUTTON_UP;
                buttonReleased(config);
            }
            else
            {
            	config->fsmButtonState = STATE_BUTTON_DOWN;
            }
            config->contRising = 0;
         }
         config->contRising++;
         break;

      default:
         fsmButtonError(config);
         break;
   }
}
