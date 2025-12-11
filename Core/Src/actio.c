#ifndef SRC_ACTIO_C_
#define SRC_ACTIO_C_

#include "actio.h"


/* ######################## PARTIE PCA9685 SERVO ########################### */

#define PCA9685_ADDRESS 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x0         // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x6         // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52


void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value);
void PCA9685_SetPWMFrequency(uint16_t frequency);
void PCA9685_Init(uint16_t frequency);
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
void PCA9685_SetServoAngle(uint8_t Channel, float Angle);
void PCA9685_ContinuousServoRun(uint8_t Channel);
void PCA9685_ContinuousServoStop(uint8_t Channel);

extern I2C_HandleTypeDef hi2c1;

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  uint8_t registerAddress;
  uint8_t pwm[4];
  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;
  pwm[1] = OnTime>>8;
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime>>8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // Centieme de radians
  // 0 rad --> 0.5 ms(102.4 Value) and 2 PI radians --> 2.5 ms(511.9 Value)
  Value = (Angle * (511.9 - 102.4) / (100 * 3.141592)) + 102.4;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

void PCA9685_ContinuousServoRun(uint8_t Channel)
{
  PCA9685_SetPWM(Channel, 0, 511);
}

void PCA9685_ContinuousServoStop(uint8_t Channel)
{
  PCA9685_SetPWM(Channel, 0, 307);
}

void PCA9685_LEDOff(uint8_t Channel)
{
  PCA9685_SetPWM(Channel, 0, 0);
}

void PCA9685_LEDOn(uint8_t Channel)
{
  PCA9685_SetPWM(Channel, 0, 1000);
}

/* ######################## PARTIE TMC2209 STEPPER ########################### */

#define STEP_PIN GPIO_PIN_5
#define STEP_PORT GPIOB

#define DIR_PIN GPIO_PIN_4
#define DIR_PORT GPIOB

void TMC2209_writeRegister(uint8_t addr, uint8_t reg, uint32_t data);
uint8_t tmc_crc(uint8_t *data, size_t len);
void TMC2209_init();
void step(uint32_t steps, uint32_t delay_us);

extern UART_HandleTypeDef huart1;

void TMC2209_writeRegister(uint8_t addr, uint8_t reg, uint32_t data)
{
    uint8_t buf[8];
    buf[0] = 0x05;         // Sync
    buf[1] = addr | 0x80;  // Write command (addr | 0x80)
    buf[2] = reg;          // Register address
    buf[3] = (data >> 24) & 0xFF;
    buf[4] = (data >> 16) & 0xFF;
    buf[5] = (data >> 8) & 0xFF;
    buf[6] = data & 0xFF;
    buf[7] = tmc_crc(buf, 7); // CRC8

    HAL_UART_Transmit(&huart1, buf, 8, HAL_MAX_DELAY);
}

uint8_t tmc_crc(uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0x8C & -(crc & 1));
    }
    return crc;
}

void TMC2209_init()
{
    // Adresse = 0x00 si MS1/MS2 à GND
    uint8_t driver_addr = 0x00;

    // Exemple : courant RMS = 600mA
    uint32_t ihold_irun = (0x10 << 0) |  // IHOLD = 16
                          (0x10 << 8) |  // IRUN = 16
                          (0x0A << 16);  // IHOLDDELAY = 10
    TMC2209_writeRegister(driver_addr, 0x10, ihold_irun);

    // CHOPCONF : microstepping = 16, enable stealthChop
    uint32_t chopconf = (4 << 24); // MRES = 4 => 1/16 microsteps
    TMC2209_writeRegister(driver_addr, 0x6C, chopconf);
}

void step(uint32_t steps, uint32_t delay_us)
{
    for (uint32_t i = 0; i < steps; i++)
    {
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(1); // ou delay_us avec timer si plus fin
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}


/* #################################### INTERFACE PUBLIQUE ############################################" */

extern Actionneurs act;

void initActionneurs(){
	PCA9685_Init(50);
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
	TMC2209_init();
	act.etatLedArmement = 0;
	act.armementTirette = 0;
	act.enJeu = 0;
	for(uint16_t servoId = 0; servoId < NOMBRE_SERVOS; servoId++){
		act.angleServo[servoId] = 0;
	}
	act.positionAscenseur = 0;
	for(uint16_t posId = 0; posId < NOMBRE_POSITIONS_ASC; posId++){
		act.valeursPositionsAscenseur[posId] = 0;
	}
	act.relayPorts[0] = RELAY_1_PORT;
	act.relayPorts[1] = RELAY_2_PORT;
	act.relayPorts[2] = RELAY_3_PORT;
	act.relayPortsNumbers[0] = RELAY_1_PIN;
	act.relayPortsNumbers[1] = RELAY_2_PIN;
	act.relayPortsNumbers[2] = RELAY_3_PIN;
}

/*void setServoPosValue(uint16_t servoNum, uint16_t posNum, uint16_t val){
	act.valeursPositionsServos[servoNum][posNum] = val;
}*/

void setAscPosValue(uint16_t posNum, uint16_t val){
	act.valeursPositionsAscenseur[posNum] = val;
}

float getServoAngle(uint16_t servoNum){
	return act.angleServo[servoNum];
}

uint16_t getRelayState(uint16_t relayNum)
{
	return HAL_GPIO_ReadPin(act.relayPorts[relayNum-1], act.relayPortsNumbers[relayNum-1]);
}

uint16_t getAscPos(){
	return act.positionAscenseur;
}

void moveServo(uint16_t servoNum, float angle){
	PCA9685_SetServoAngle(servoNum, angle);
	act.angleServo[servoNum] = angle;
}

void moveRelay(uint16_t relayNum, uint16_t state){
	if(state == 1){
		HAL_GPIO_WritePin(act.relayPorts[relayNum-1], act.relayPortsNumbers[relayNum-1], GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(act.relayPorts[relayNum-1], act.relayPortsNumbers[relayNum-1], GPIO_PIN_RESET);
	}
}

void moveAsc(uint16_t posNum){
	act.positionAscenseur = posNum;
}

void armTirette(){
	act.armementTirette = 1;
}

bool tiretteStatus(){
	return act.armementTirette;
}

void disarmTirette(){
	act.armementTirette = 0;
}

#endif /* SRC_ACTIO_C_ */
