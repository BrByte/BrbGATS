/*
 * main.h
 *
 *  Created on: 2018-02-01
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
 *
 * Copyright (c) 2018 BrByte Software (Oliveira Alves & Amorim LTDA)
 * Todos os direitos reservados. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAIN_H_
#define MAIN_H_
/**********************************************************************************************************************/
/* Import needed libraries */
#include "Arduino.h"

#include "BrbMCUServoBase.h"

#include "BrbBase.h"
#include "BrbBtnBase.h"
#include "BrbToneBase.h"


#include "BrbDisplayBase.h"
#include "BrbRS485Session.h"

#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <avr/wdt.h>
/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
// #define RESERVED    0 /* RX0 */
// #define RESERVED    1 /* TX0 */
#define GATS_ZEROCROSS_PIN 2 /* INT4 - PWM */
// #define RESERVED    3 /* INT5 - PWM */
#define GATS_STARTER_PIN 4 /* PWM */
#define GATS_PARADA_PIN 5  /* PWM */
#define GATS_SOLENOID_PIN 6   /* PWM */
#define BUZZER_PIN 7          /* PWM */
#define DHT_SENSOR_PIN 8      /* PWM */
#define DHT_SENSOR_TYPE DHT11
#define GATS_SERVO_PIN 9   /* PWM */
// #define RESERVED    10 /* PCINT 4 */
// #define RESERVED    11 /* PCINT 5 */
// #define RESERVED    12 /* PCINT 6 */
// #define RESERVED    13 /* PCINT 7 */
#define RS485_DI_PIN 14 /* PCINT10 - TX3 */
#define RS485_RO_PIN 15 /* PCINT9 - RX3 */
// #define RESERVED    16 /* TX2 */
// #define RESERVED    17 /* RX2 */
// #define RESERVED    18 /* INT3 - TX1 */
// #define RESERVED    19 /* INT2 - RX1 */
// #define RESERVED    20 /* INT0 - SCL */
// #define RESERVED    21 /* INT1 - SDA */
#define RS485_REDE_PIN 22 /* TOGGLE PIN (RE + DE) */
// #define RESERVED    23 /* */
// #define RESERVED    24 /* */
// #define RESERVED    25 /* */
#define BTN_PIN_SELECT 26 /* */
#define BTN_PIN_NEXT 27   /* */
#define BTN_PIN_PREV 28   /* */
// #define RESERVED    29 /* */
// #define RESERVED    30 /* */
// #define RESERVED    31 /* */
// #define RESERVED    32 /* */
// #define RESERVED    33 /* */
// #define RESERVED    34 /* */
// #define RESERVED    35 /* */
// #define RESERVED    36 /* */
// #define RESERVED    37 /* */
// #define RESERVED    38 /* */
// #define RESERVED    39 /* */
// #define RESERVED    40 /* */
// #define RESERVED    41 /* */
// #define RESERVED    42
// #define RESERVED    43 /*  */
// #define RESERVED    44 /*  */
// #define RESERVED    45 /* PWM */
#define TFT_LED 46  /* PWM */
#define TFT_CS 47   /*  */
#define TFT_DC 48   /*  */
#define TFT_RST 49  /* */
#define TFT_MISO 50 /* PCINT3 - MISO */
#define TFT_MOSI 51 /* PCINT2 - MOSI */
#define TFT_CLK 52  /* PCINT1 - SCK */
// #define RESERVED     53 /* PCINT0 - SS */

#define SENSOR_DC_SUPPLY_01_IN_PIN A0
// #define SENSOR_DC_SUPPLY_01_OUT_PIN A1
// #define SENSOR_DC_SUPPLY_02_IN_PIN A2
// #define SENSOR_DC_SUPPLY_02_OUT_PIN A3

#define SENSOR_AC_POWER_PIN A5
// #define SENSOR_AC_AUX_PIN A6
// #define SENSOR_AC_BAT_PIN A7
/**********************************************************/
#define GATS_EEPROM_OFFSET (BRB_RS485_EEPROM_OFFSET + 64)

// #define GATS_POWER_REVERSE 1

#ifdef GATS_POWER_REVERSE
#define GATS_POWER_ON LOW
#define GATS_POWER_OFF HIGH
#else
#define GATS_POWER_ON HIGH
#define GATS_POWER_OFF LOW
#endif

#define GATS_HOURMETER_MAX 20

#define GATS_SERVO_BB_POS_OPEN 		180
#define GATS_SERVO_BB_POS_CLOSE 	120

#define GATS_POWER_MIN_MS 3000

#define GATS_POWER_MIN_VALUE 30
#define GATS_POWER_MIN_HZ 15

// #define GATS_AUX_MIN_VALUE 10
// #define GATS_AUX_MIN_HZ 10
/**********************************************************/
#define GATS_TIMER_FAIL_ALARM_MS 5000

#define GATS_TIMER_ZERO_WAIT_MS 2000

#define GATS_TIMER_START_WAIT_MS 15000

#define GATS_TIMER_START_DELAY_MS 5000
#define GATS_TIMER_START_CHECK_MS 30000
#define GATS_TIMER_START_RETRY_MAX 3

#define GATS_TIMER_STOP_DELAY_MS 15000
#define GATS_TIMER_STOP_CHECK_MS 20000
#define GATS_TIMER_STOP_RETRY_MAX 3

#define GATS_TIMER_CHECK_MS 5000

#define GATS_TIMER_SENSOR_WAIT_MS 2000
#define GATS_TIMER_SENSOR_SAMPLES 5

#define GATS_TIMER_DHT_MS 1000
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/
typedef enum
{
	GATS_ACTION_NONE,
	GATS_ACTION_START,
	GATS_ACTION_STOP,
	GATS_ACTION_CUT_FUEL
	
} BrbGATSActionCode;

typedef enum
{
	GATS_FAILURE_NONE,
	GATS_FAILURE_RUNNING_WITHOUT_START,
	GATS_FAILURE_DOWN_WITHOUT_STOP,

	GATS_FAILURE_START_RETRY_LIMIT,

	GATS_FAILURE_STOP_RETRY_LIMIT,

} BrbGATSFailureCode;

typedef enum
{
	GATS_STATE_NONE,
	GATS_STATE_FAILURE,
	
	GATS_STATE_START_INIT,
	GATS_STATE_START_DELAY,
	GATS_STATE_START_CHECK,

	GATS_STATE_RUNNING,

	GATS_STATE_STOP_INIT,
	GATS_STATE_STOP_DELAY,
	GATS_STATE_STOP_CHECK,

} BrbGATSStateCode;
/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef struct _BrbGATSBase
{
	BrbBase *brb_base;
	BrbToneBase *tone_base;
	DHT *dht_sensor;

	long delay;

	// int pin_servo;
	// int pin_partida;
	// int pin_parada;

	BrbZeroCross zero_power;
	BrbSensorVoltage sensor_power;

	BrbSensorVoltage sensor_sp01_in;

	struct
	{
		long cur;
		long last;
		long delay;

		long power_time;
		long power_delay;

		long off_time;

		// long aux_time;
		// long aux_delay;

	} ms;

	struct
	{
		long ms_delta;
		long ms_last;

	} sensor;

	struct
	{
		long ms_delta;
		long ms_last;

	} zerocross;
	
	struct
	{
		float dht_temp;
		float dht_humi;
		float dht_hidx;
		
		long ms_delta;
		long ms_last;

		uint8_t pin;
		uint8_t type;
	} dht_data;

	struct
	{
		BrbGATSStateCode code;		
		BrbGATSFailureCode fail;

		long ms_last;
		long ms_change;
		long ms_delta;
		
		int retry;

	} state;

	struct
	{
		double gas;
		double load;

		long hourmeter_ms;
		long hourmeter_sec;

	} info;

	/* data is persistent */
	struct
	{
		long hourmeter_total;
		long hourmeter_time;
		long hourmeter_reset;

		long reserved1;
		long reserved2;
		long reserved3;

	} data;

	struct
	{
		unsigned int foo : 1;
	} flags;

} BrbGATSBase;
/**********************************************************************************************************************/
int BrbGATSBase_Init(BrbGATSBase *gats_base);
int BrbGATSBase_Loop(BrbGATSBase *gats_base);
int BrbGATSBase_Save(BrbGATSBase *gats_base);
int BrbGATSBase_HourmeterReset(BrbGATSBase *gats_base);

int BrbGATSBase_ActionCmd(BrbGATSBase *gats_base, int cmd_code);

int BrbGATSBase_FailureConfirm(BrbGATSBase *gats_base);

const char *BrbGATSBase_GetState(BrbGATSBase *gats_base);
const char *BrbGATSBase_GetStateAction(BrbGATSBase *gats_base);
const char *BrbGATSBase_GetStateButton(BrbGATSBase *gats_base);
const char *BrbGATSBase_GetFailure(BrbGATSBase *gats_base);
/**********************************************************************************************************************/
/* Display */
/**********************************************************/
int BrbCtlDisplay_Setup(BrbBase *brb_base);
/**********************************************************************************************************************/
/* RS485 */
/**********************************************************/
int BrbCtlRS485_Setup(BrbBase *brb_base);
/**********************************************************************************************************************/
/* Global control structures */
extern BrbLogBase *glob_log_base;
extern BrbBase glob_brb_base;
extern BrbRS485Session glob_rs485_sess;

extern BrbBtnBase glob_btn_base;
extern BrbDisplayBase glob_display_base;
extern BrbToneBase glob_tone_base;
extern BrbMCUServoBase glob_servo_base;
extern BrbGATSBase glob_gats_base;
/**********************************************************************************************************************/
#endif /* MAIN_H_ */
