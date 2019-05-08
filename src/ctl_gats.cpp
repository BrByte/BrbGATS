/*
 * gats_main.cpp
 *
 *  Created on: 2019-02-18
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

#include "main.h"

static int BrbGATSBase_PowerStart(BrbGATSBase *gats_base);
static int BrbGATSBase_PowerStop(BrbGATSBase *gats_base);
static int BrbGATSBase_PowerSolenoidCut(BrbGATSBase *gats_base);

static int BrbGATSBase_PowerOff(BrbGATSBase *gats_base);
static int BrbGATSBase_PowerSetState(BrbGATSBase *gats_base, BrbGATSStateCode code, BrbGATSFailureCode fail);

/**********************************************************************************************************************/
int BrbGATSBase_Init(BrbGATSBase *gats_base)
{
	/* Sanitize */
	if (!gats_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMRead(gats_base->brb_base, (uint8_t *)&gats_base->data, sizeof(gats_base->data), GATS_EEPROM_OFFSET);

	if (gats_base->dht_data.pin > 0)
		gats_base->dht_sensor = new DHT(gats_base->dht_data.pin, gats_base->dht_data.type);

	pinMode(GATS_STARTER_PIN, OUTPUT);
	digitalWrite(GATS_STARTER_PIN, GATS_POWER_OFF);

	pinMode(GATS_PARADA_PIN, OUTPUT);
	digitalWrite(GATS_PARADA_PIN, GATS_POWER_OFF);

	pinMode(GATS_SOLENOID_PIN, OUTPUT);
	digitalWrite(GATS_SOLENOID_PIN, GATS_POWER_OFF);

	if (gats_base->sensor_power.pin > 0)
		pinMode(gats_base->sensor_power.pin, INPUT);

	if (gats_base->sensor_sp01_in.pin > 0)
		pinMode(gats_base->sensor_sp01_in.pin, INPUT);

	BrbMCUServoSetPosByPin(&glob_servo_base, GATS_SERVO_PIN, GATS_SERVO_BB_POS_OPEN);

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_DHTCheck(BrbGATSBase *gats_base)
{
	gats_base->dht_data.ms_delta = (gats_base->ms.cur - gats_base->dht_data.ms_last);

	if ((gats_base->dht_sensor) && ((gats_base->dht_data.ms_last <= 0) || (gats_base->dht_data.ms_delta >= GATS_TIMER_DHT_MS)))
	{
		gats_base->dht_data.ms_last = gats_base->ms.cur;

		gats_base->dht_data.dht_temp = gats_base->dht_sensor->readTemperature();
		gats_base->dht_data.dht_humi = gats_base->dht_sensor->readHumidity();

		/* Compute heat index in Celsius (isFahreheit = false) */
		gats_base->dht_data.dht_hidx = gats_base->dht_sensor->computeHeatIndex(gats_base->dht_data.dht_temp, gats_base->dht_data.dht_humi, false);

		gats_base->dht_data.dht_temp = isnan(gats_base->dht_data.dht_temp) ? 0.0 : gats_base->dht_data.dht_temp;
		gats_base->dht_data.dht_humi = isnan(gats_base->dht_data.dht_humi) ? 0.0 : gats_base->dht_data.dht_humi;
		gats_base->dht_data.dht_hidx = isnan(gats_base->dht_data.dht_hidx) ? 0.0 : gats_base->dht_data.dht_hidx;
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_SupplyCheck(BrbGATSBase *gats_base)
{
#define RDC1 30000.0
#define RDC2 7500.0
#define RDCR (RDC2 / (RDC2 + RDC1))

	gats_base->sensor_sp01_in.value = ((analogRead(gats_base->sensor_sp01_in.pin) * 5.0) / 1024.0) / RDCR;
	// gats_base->sensor_sp01_out.value = ((analogRead(gats_base->sensor_sp01_out.pin) * 5.0) / 1024.0) / RDCR;

	// gats_base->sensor_sp02_in.value = ((analogRead(gats_base->sensor_sp02_in.pin) * 5.0) / 1024.0) / RDCR;
	// gats_base->sensor_sp02_out.value = ((analogRead(gats_base->sensor_sp02_out.pin) * 5.0) / 1024.0) / RDCR;

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_PowerCheck(BrbGATSBase *gats_base)
{
	gats_base->zerocross.ms_delta = (gats_base->ms.cur - gats_base->zerocross.ms_last);

	/* We are waiting delay */
	if ((gats_base->zerocross.ms_last <= 0) || (gats_base->zerocross.ms_delta >= GATS_TIMER_ZERO_WAIT_MS))
	{
		gats_base->zerocross.ms_last = gats_base->ms.cur;

		noInterrupts();
		gats_base->zero_power.value = (gats_base->zero_power.counter / (gats_base->zerocross.ms_delta / 1000.0)) / 2.0;
		gats_base->zero_power.counter = 0;

		// gats_base->zero_aux.value = (gats_base->zero_aux.counter / (gats_base->zerocross.ms_delta / 1000.0)) / 2.0;
		// gats_base->zero_aux.counter = 0;
		interrupts();
	}

#define RAC1 220000.0
#define RAC2 10000.0
// #define RACR (RAC2 / (RAC2 + RAC1))
#define RACR 0.0165

	// gats_base->sensor_power.value = ((analogRead(gats_base->sensor_power.pin) * 5.0) / 1024.0) / 0.013;
	gats_base->sensor.ms_delta = (gats_base->ms.cur - gats_base->sensor.ms_last);

	/* We are waiting delay */
	if ((gats_base->sensor.ms_last <= 0) || (gats_base->sensor.ms_delta >= GATS_TIMER_SENSOR_WAIT_MS))
	{
		gats_base->sensor.ms_last = gats_base->ms.cur;
		long samples_value;
		int i;

		/* Read sensor data */
		if (gats_base->sensor_power.pin > 0)
		{
			for (i = 0, samples_value = 0; i < GATS_TIMER_SENSOR_SAMPLES; i++)
			{
				samples_value += analogRead(gats_base->sensor_power.pin);
			}

			samples_value /= GATS_TIMER_SENSOR_SAMPLES;
			gats_base->sensor_power.value = ((samples_value * 5.0) / 1023.0) / RACR;
			// gats_base->sensor_power.value = ((analogRead(gats_base->sensor_power.pin) * 5.0) / 1023.0) / RACR;
		}

		// if (gats_base->sensor_aux.pin > 0)
		// {
		// 	for (i = 0, samples_value = 0; i < PDU_TIMER_SENSOR_SAMPLES; i++)
		// 	{
		// 		samples_value += analogRead(gats_base->sensor_aux.pin);
		// 	}

		// 	samples_value /= PDU_TIMER_SENSOR_SAMPLES;
		// 	gats_base->sensor_aux.value = ((samples_value * 5.0) / 1023.0) / RACR;
		// 	// gats_base->sensor_aux.value = ((analogRead(gats_base->sensor_aux.pin) * 5.0) / 1023.0) / RACR;
		// }

		if (gats_base->sensor_power.value > GATS_POWER_MIN_VALUE || gats_base->zero_power.value > GATS_POWER_MIN_HZ)
		{
			if (gats_base->ms.power_time <= 0)
				gats_base->ms.power_time = gats_base->ms.cur;

			gats_base->ms.power_delay = (gats_base->ms.cur - gats_base->ms.power_time);
		}
		else if (gats_base->ms.power_time > 0)
		{
			gats_base->ms.power_delay = -1;
			gats_base->ms.power_time = 0;
			gats_base->ms.off_time = gats_base->ms.cur;
		}

		// if (gats_base->sensor_aux.value > GATS_AUX_MIN_VALUE && gats_base->zero_aux.value > GATS_AUX_MIN_HZ)
		// {
		// 	if (gats_base->ms.aux_time <= 0)
		// 		gats_base->ms.aux_time = gats_base->ms.cur;

		// 	gats_base->ms.aux_delay = (gats_base->ms.cur - gats_base->ms.aux_time);
		// }
		// else
		// {
		// 	gats_base->ms.aux_delay = -1;
		// 	gats_base->ms.aux_time = -1;
		// }
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_Loop(BrbGATSBase *gats_base)
{
	gats_base->ms.cur = millis();
	gats_base->ms.delay = gats_base->ms.cur - gats_base->ms.last;

	/* Loop Delay */
	if (gats_base->ms.delay < 50)
		return -1;

	gats_base->ms.last = gats_base->ms.cur;

	gats_base->state.ms_delta = (gats_base->ms.cur - gats_base->state.ms_last);

	gats_base->info.gas = random(750, 1000) / 10.0;
	gats_base->info.load = random(250, 300) / 10.0;

	BrbGATSBase_DHTCheck(gats_base);

	BrbGATSBase_SupplyCheck(gats_base);

	BrbGATSBase_PowerCheck(gats_base);

	switch (gats_base->state.code)
	{
	case GATS_STATE_NONE:
	{
		/* This can't happen here, do something */
		if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
		{
		  /* Power off pins */
		  BrbGATSBase_PowerOff(gats_base);

			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_RUNNING, GATS_FAILURE_RUNNING_WITHOUT_START);

			BrbToneBase_PlayArrive(gats_base->tone_base);

			break;
		}

		break;
	}
	case GATS_STATE_START_INIT:
	{
		/* Power off pins */
		BrbGATSBase_PowerOff(gats_base);

		if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
		{
		  /* Power off pins */
		  BrbGATSBase_PowerOff(gats_base);

			BrbToneBase_PlayArrive(gats_base->tone_base);

			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_RUNNING, GATS_FAILURE_RUNNING_WITHOUT_START);

			break;
		}

		/* We are waiting delay */
		if ((gats_base->state.ms_last > 0) && (gats_base->state.ms_delta < GATS_TIMER_START_WAIT_MS))
			break;

		/* Reset info */
		gats_base->state.retry = 0;

		BrbGATSBase_PowerStart(gats_base);

		break;
	}
	case GATS_STATE_START_DELAY:
	{
		if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
		{
			/* Power off pins */
			BrbGATSBase_PowerOff(gats_base);

			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_RUNNING, GATS_FAILURE_NONE);

			break;
		}

		/* We are waiting delay */
		if ((gats_base->state.ms_last > 0) && (gats_base->state.ms_delta < GATS_TIMER_START_DELAY_MS))
			break;

		/* Power off pins */
		BrbGATSBase_PowerOff(gats_base);

		BrbMCUServoSetPosByPin(&glob_servo_base, GATS_SERVO_PIN, GATS_SERVO_BB_POS_OPEN);

		BrbGATSBase_PowerSetState(gats_base, GATS_STATE_START_CHECK, GATS_FAILURE_NONE);

		break;
	}
	case GATS_STATE_START_CHECK:
	{
		/* Power off pins */
		BrbGATSBase_PowerOff(gats_base);

		if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
		{
			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_RUNNING, GATS_FAILURE_NONE);

			BrbToneBase_PlayArrive(gats_base->tone_base);

			break;
		}

		/* We are waiting delay */
		if ((gats_base->state.ms_last > 0) && (gats_base->state.ms_delta < GATS_TIMER_START_CHECK_MS))
			break;

		/* We are waiting delay */
		if (gats_base->ms.power_delay > 1000)
			break;

		if (gats_base->state.retry >= GATS_TIMER_START_RETRY_MAX)
		{
			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_FAILURE, GATS_FAILURE_START_RETRY_LIMIT);

			break;
		}

		BrbGATSBase_PowerStart(gats_base);

		break;
	}
	case GATS_STATE_STOP_INIT:
	{
		/* Power off pins */
		BrbGATSBase_PowerOff(gats_base);

		/* Reset info */
		gats_base->state.retry = 0;

		BrbGATSBase_PowerSolenoidCut(gats_base);

		break;
	}
	case GATS_STATE_STOP_DELAY:
	{
		/* We are waiting delay */
		if ((gats_base->state.ms_last > 0) && (gats_base->state.ms_delta < GATS_TIMER_STOP_DELAY_MS))
			break;

		// if (gats_base->ms.power_delay > 0)
		// {
		// 	/* Power off pins */
		// 	BrbGATSBase_PowerOff(gats_base);

		// 	BrbGATSBase_PowerSolenoidCut(gats_base);

		// 	break;
		// }

		// /* Power off pins */
		// BrbGATSBase_PowerOff(gats_base);

		// /* Maintain fuel off */
		// BrbGATSBase_PowerSolenoidCut(gats_base);

		BrbGATSBase_PowerSetState(gats_base, GATS_STATE_STOP_CHECK, GATS_FAILURE_NONE);

		break;
	}
	case GATS_STATE_STOP_CHECK:
	{
		// /* Power off pins */
		// BrbGATSBase_PowerOff(gats_base);

		/* We are waiting delay */
		if ((gats_base->state.ms_last > 0) && (gats_base->state.ms_delta < GATS_TIMER_STOP_CHECK_MS))
			break;

		if (gats_base->ms.power_delay < 1000 && ((gats_base->ms.cur - gats_base->ms.off_time) > 5000))
    {
		  /* Power off pins */
		  BrbGATSBase_PowerOff(gats_base);
      
			return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);
    }
		
		if (gats_base->state.retry >= (GATS_TIMER_START_RETRY_MAX * 2))
			return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_FAILURE, GATS_FAILURE_STOP_RETRY_LIMIT);

		if (gats_base->state.retry >= GATS_TIMER_START_RETRY_MAX)
		{
			BrbGATSBase_PowerStop(gats_base);

			break;
		}

		BrbGATSBase_PowerSolenoidCut(gats_base);

		break;
	}
	case GATS_STATE_RUNNING:
	{
		/* Check Power */
		if (gats_base->ms.power_delay < 1000 && ((gats_base->ms.cur - gats_base->ms.off_time) > 5000))
		{
			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_FAILURE, GATS_FAILURE_DOWN_WITHOUT_STOP);

			BrbToneBase_PlayAlarm3(gats_base->tone_base);

			break;
		}

		break;
	}
	case GATS_STATE_FAILURE:
	{
		/* We are waiting delay */
		if ((gats_base->state.ms_last > 0) && (gats_base->state.ms_delta < GATS_TIMER_FAIL_ALARM_MS))
			break;

		/* Power off pins */
		BrbGATSBase_PowerOff(gats_base);

		BrbToneBase_PlayAlarm3(gats_base->tone_base);

		switch (gats_base->state.fail)
		{
		case GATS_FAILURE_RUNNING_WITHOUT_START:
		{
			if (gats_base->ms.power_delay < 1000 && ((gats_base->ms.cur - gats_base->ms.off_time) > 5000))
				return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);

			break;
		}
		case GATS_FAILURE_DOWN_WITHOUT_STOP:
		{
			if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
				return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);

			break;
		}
		case GATS_FAILURE_START_RETRY_LIMIT:
		{
			if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
				return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);

			break;
		}
		case GATS_FAILURE_STOP_RETRY_LIMIT:
		{
			if (gats_base->ms.power_delay < 1000 && ((gats_base->ms.cur - gats_base->ms.off_time) > 5000))
				return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);

			break;
		}
		default:
		{
			return BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);
			break;
		}
		}
		
		/* Reset failure */
		BrbGATSBase_PowerSetState(gats_base, GATS_STATE_FAILURE, gats_base->state.fail);

		break;
	}
	}

	if (gats_base->ms.power_delay > GATS_POWER_MIN_MS)
	{
		gats_base->info.hourmeter_ms = gats_base->info.hourmeter_ms + gats_base->ms.delay;

		/* 5 seconds delay */
		if (gats_base->info.hourmeter_ms > 5000)
		{
			gats_base->info.hourmeter_ms = (gats_base->info.hourmeter_ms - 5000);

			gats_base->info.hourmeter_sec = gats_base->info.hourmeter_sec + 5;

			/* 60 seconds delay */
			if (gats_base->info.hourmeter_sec > 60)
			{
				gats_base->data.hourmeter_total++;
				gats_base->data.hourmeter_time++;
				gats_base->info.hourmeter_sec = (gats_base->info.hourmeter_sec - 60);

				BrbGATSBase_Save(gats_base);
			}
		}
	}

	return 0;
}
/**********************************************************************************************************************/
static int BrbGATSBase_PowerStart(BrbGATSBase *gats_base)
{
	// BrbBase *brb_base = gats_base->brb_base;

	BrbToneBase_PlayAlarm2(gats_base->tone_base);

	BrbGATSBase_PowerSetState(gats_base, GATS_STATE_START_DELAY, GATS_FAILURE_NONE);

	BrbMCUServoSetPosByPin(&glob_servo_base, GATS_SERVO_PIN, GATS_SERVO_BB_POS_CLOSE);

	/* Set pin data */
	digitalWrite(GATS_STARTER_PIN, GATS_POWER_ON);

	gats_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbGATSBase_PowerStop(BrbGATSBase *gats_base)
{
	// BrbBase *brb_base = gats_base->brb_base;

	BrbToneBase_PlayAlarm3(gats_base->tone_base);

	BrbGATSBase_PowerSetState(gats_base, GATS_STATE_STOP_DELAY, GATS_FAILURE_NONE);

	/* Set pin data */
	digitalWrite(GATS_PARADA_PIN, GATS_POWER_ON);

	gats_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbGATSBase_PowerSolenoidCut(BrbGATSBase *gats_base)
{
	// BrbBase *brb_base = gats_base->brb_base;

	BrbToneBase_PlayAlarm3(gats_base->tone_base);

	BrbGATSBase_PowerSetState(gats_base, GATS_STATE_STOP_DELAY, GATS_FAILURE_NONE);

	/* Set pin data */
	digitalWrite(GATS_SOLENOID_PIN, GATS_POWER_ON);

	gats_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbGATSBase_PowerOff(BrbGATSBase *gats_base)
{
	// BrbBase *brb_base = gats_base->brb_base;

	/* Set pin data */
	digitalWrite(GATS_STARTER_PIN, GATS_POWER_OFF);
	digitalWrite(GATS_PARADA_PIN, GATS_POWER_OFF);
	digitalWrite(GATS_SOLENOID_PIN, GATS_POWER_OFF);

	return 0;
}
/**********************************************************************************************************************/
static int BrbGATSBase_PowerSetState(BrbGATSBase *gats_base, BrbGATSStateCode code, BrbGATSFailureCode fail)
{
	gats_base->state.ms_delta = 0;
	gats_base->state.ms_last = gats_base->ms.cur;

	if (gats_base->state.code != code || gats_base->state.fail != fail)
	{
		gats_base->state.code = code;
		gats_base->state.fail = fail;
		gats_base->state.ms_change = gats_base->ms.cur;
	}
	
	BrbRS485PacketVal rs485_pkt_val = {0};

	if (code == GATS_STATE_FAILURE)
	{
		rs485_pkt_val.type = RS485_PKT_DATA_TYPE_NOTIFY;
		rs485_pkt_val.code = fail;
	}
	else
	{
		rs485_pkt_val.type = RS485_PKT_DATA_TYPE_INFORM;
		rs485_pkt_val.code = code;
	}	

	/* Send RS485 to notify the network */
	BrbRS485Session_SendPacketData(&glob_rs485_sess, 0xFF, &rs485_pkt_val);

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_ActionCmd(BrbGATSBase *gats_base, int cmd_code)
{
	switch (cmd_code)
	{
		case GATS_ACTION_START:
		{
			switch (gats_base->state.code)
			{
			case GATS_STATE_START_INIT:
			case GATS_STATE_START_DELAY:
			case GATS_STATE_START_CHECK:
			case GATS_STATE_RUNNING:
			{
				return 1;
			}
			case GATS_STATE_FAILURE:
			case GATS_STATE_STOP_INIT:
			case GATS_STATE_STOP_DELAY:
			case GATS_STATE_STOP_CHECK:
			case GATS_STATE_NONE:
			default:
				BrbGATSBase_PowerSetState(gats_base, GATS_STATE_START_INIT, GATS_FAILURE_NONE);

				BrbToneBase_PlayAction(gats_base->tone_base);
				break;
			}

			break;
		}
		case GATS_ACTION_STOP:
		{
			switch (gats_base->state.code)
			{
			case GATS_STATE_STOP_INIT:
			case GATS_STATE_STOP_DELAY:
			case GATS_STATE_STOP_CHECK:
			{
				return 1;
			}
			case GATS_STATE_START_INIT:
			case GATS_STATE_START_DELAY:
			case GATS_STATE_START_CHECK:
			case GATS_STATE_RUNNING:
			case GATS_STATE_FAILURE:
			case GATS_STATE_NONE:
			default:
				BrbGATSBase_PowerSetState(gats_base, GATS_STATE_STOP_INIT, GATS_FAILURE_NONE);

				BrbToneBase_PlayAction(gats_base->tone_base);
				break;
			}

			break;
		}
		case GATS_ACTION_CUT_FUEL:
		{
			// BrbGATSBase_PowerStop(gats_base);
			break;
		}
		case GATS_ACTION_NONE:
		{
			BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);

			BrbToneBase_PlayAction(gats_base->tone_base);
			break;
		}
		default:
			return -1;
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_FailureConfirm(BrbGATSBase *gats_base)
{
	// BrbBase *brb_base = gats_base->brb_base;

	BrbGATSBase_PowerSetState(gats_base, GATS_STATE_NONE, GATS_FAILURE_NONE);

	BrbToneBase_PlayAction(gats_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_HourmeterReset(BrbGATSBase *gats_base)
{
	if (!gats_base)
		return -1;

	gats_base->data.hourmeter_time = 0;
	gats_base->data.hourmeter_reset++;

	BrbGATSBase_Save(gats_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGATSBase_Save(BrbGATSBase *gats_base)
{
	if (!gats_base || !gats_base->brb_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMWrite(gats_base->brb_base, (uint8_t *)&gats_base->data, sizeof(gats_base->data), GATS_EEPROM_OFFSET);

	return 0;
}
/**********************************************************************************************************************/
const char *BrbGATSBase_GetState(BrbGATSBase *gats_base)
{
	const char *ret_ptr = PSTR("Parado");

	switch (gats_base->state.code)
	{
	case GATS_STATE_START_INIT:
	{
		ret_ptr = PSTR("Iniciando");
		break;
	}
	case GATS_STATE_START_DELAY:
	{
		ret_ptr = PSTR("Ligando");
		break;
	}
	case GATS_STATE_START_CHECK:
	{
		ret_ptr = PSTR("Verificando");
		break;
	}
	case GATS_STATE_RUNNING:
	{
		ret_ptr = PSTR("Funcionando");
		break;
	}
	case GATS_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha");
		break;
	}
	case GATS_STATE_STOP_INIT:
	{
		ret_ptr = PSTR("Finalizando");
		break;
	}
	case GATS_STATE_STOP_DELAY:
	{
		ret_ptr = PSTR("Desligando");
		break;
	}
	case GATS_STATE_STOP_CHECK:
	{
		ret_ptr = PSTR("Encerrando");
		break;
	}
	case GATS_STATE_NONE:
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbGATSBase_GetStateAction(BrbGATSBase *gats_base)
{
	const char *ret_ptr = PSTR("None");

	switch (gats_base->state.code)
	{
	case GATS_STATE_START_INIT:
	case GATS_STATE_START_DELAY:
	case GATS_STATE_START_CHECK:
	case GATS_STATE_RUNNING:
	{
		ret_ptr = PSTR("Desligar Sistema?");
		break;
	}
	case GATS_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha no Sistema!");
		break;
	}
	case GATS_STATE_STOP_INIT:
	case GATS_STATE_STOP_DELAY:
	case GATS_STATE_STOP_CHECK:
	case GATS_STATE_NONE:
	{
		ret_ptr = PSTR("Iniciar Sistema?");
		break;
	}
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbGATSBase_GetStateButton(BrbGATSBase *gats_base)
{
	const char *ret_ptr = PSTR("None");

	switch (gats_base->state.code)
	{
	case GATS_STATE_START_INIT:
	case GATS_STATE_START_DELAY:
	case GATS_STATE_START_CHECK:
	case GATS_STATE_RUNNING:
	{
		ret_ptr = PSTR("DESLIGAR");
		break;
	}
	case GATS_STATE_FAILURE:
	{
		ret_ptr = PSTR("IGNORAR");
		break;
	}
	case GATS_STATE_STOP_INIT:
	case GATS_STATE_STOP_DELAY:
	case GATS_STATE_STOP_CHECK:
	case GATS_STATE_NONE:
	default:
	{
		ret_ptr = PSTR("LIGAR");
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbGATSBase_GetFailure(BrbGATSBase *gats_base)
{
	const char *ret_ptr = PSTR("- - - - -");

	switch (gats_base->state.fail)
	{
	case GATS_FAILURE_RUNNING_WITHOUT_START:
	{
		ret_ptr = PSTR("Energia Detectada");
		break;
	}
	case GATS_FAILURE_DOWN_WITHOUT_STOP:
	{
		ret_ptr = PSTR("Sem Energia");
		break;
	}
	case GATS_FAILURE_START_RETRY_LIMIT:
	{
		ret_ptr = PSTR("Limite Partida");
		break;
	}
	case GATS_FAILURE_STOP_RETRY_LIMIT:
	{
		ret_ptr = PSTR("Limite Parada");
		break;
	}
	default:
	{
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/