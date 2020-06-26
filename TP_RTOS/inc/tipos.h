#ifndef TP_ALEJANDRO_GORRIN_H_
#define TP_ALEJANDRO_GORRIN_H_

#include "../../TP_RTOS/inc/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sapi.h"
#include "semphr.h"

#define DEBOUNCE_TIME 40

typedef enum{
   STATE_BUTTON_UP,
   STATE_BUTTON_DOWN,
   STATE_BUTTON_FALLING,
   STATE_BUTTON_RISING
} fsmButtonState_t;

typedef struct{
	gpioMap_t tecla;
	gpioMap_t led;
	fsmButtonState_t fsmButtonState;

	TickType_t tiempo_medido;
	TickType_t tiempo_down;
	TickType_t tiempo_up;

	uint8_t contFalling;
	uint8_t contRising;

	SemaphoreHandle_t sem_tec_pulsada;
	SemaphoreHandle_t mutex;
} tecla_led_t;




#endif
