#ifndef INC_ACTIO_H_
#define INC_ACTIO_H_

#include "stm32g4xx_hal.h"
#include <stdbool.h>

#define NOMBRE_SERVOS				16
#define NOMBRE_POSITIONS_SERVOS		8
#define NOMBRE_POSITIONS_ASC		8
#define NOMBRE_RELAIS				3

#define RELAY_1_PORT				GPIOB
#define RELAY_1_PIN					GPIO_PIN_6

#define RELAY_2_PORT				GPIOB
#define RELAY_2_PIN					GPIO_PIN_8

#define RELAY_3_PORT				GPIOA
#define RELAY_3_PIN					GPIO_PIN_1

typedef struct Actionneurs {
	bool etatLedArmement;
	bool enJeu;
	uint16_t armementTirette;
	uint16_t angleServo[NOMBRE_SERVOS];
	// uint16_t valeursPositionsServos[NOMBRE_SERVOS][NOMBRE_POSITIONS_SERVOS];
	uint16_t positionAscenseur;
	uint16_t valeursPositionsAscenseur[NOMBRE_POSITIONS_ASC];
	GPIO_TypeDef * relayPorts[NOMBRE_RELAIS];
	uint16_t relayPortsNumbers[NOMBRE_RELAIS];
} Actionneurs;

void initActionneurs();
void setAscPosValue(uint16_t posNum, uint16_t val);
void PCA9685_ContinuousServoRun(uint8_t Channel);
void PCA9685_ContinuousServoStop(uint8_t Channel);
void PCA9685_LEDOn(uint8_t Channel);
void PCA9685_LEDOff(uint8_t Channel);
float getServoAngle(uint16_t servoNum);
uint16_t getRelayState(uint16_t relayNum);
uint16_t getAscPos();
void moveServo(uint16_t servoNum, float angle);
void moveRelay(uint16_t relayNum, uint16_t state);
void moveAsc(uint16_t posNum);
void armTirette();
bool tiretteStatus();
void disarmTirette();


#endif /* INC_ACTIO_H_ */
