/*
 * main.cpp
 *
 *  Created on: 2019-02-09
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

/* Global control structures */
BrbLogBase *glob_log_base;
BrbBase glob_brb_base;
BrbRS485Session glob_rs485_sess;

BrbBtnBase glob_btn_base;
BrbDisplayBase glob_display_base;
BrbToneBase glob_tone_base;

BrbMCUServoBase glob_servo_base;

BrbGATSBase glob_gats_base;
/**********************************************************************************************************************/
/* RUN ONE TIME ON START */
/**********************************************************************************************************************/
void BrbSetup(void)
{
    /* Clean up base */
    memset(&glob_brb_base, 0, sizeof(BrbBase));

    glob_log_base = BrbLogBase_New();
    BrbLogBase_Init(glob_log_base, &Serial);

    glob_brb_base.log_base = glob_log_base;

    BrbBaseInit(&glob_brb_base);

    return;
}
/**********************************************************************************************************************/
void BrbBtnSetup(void)
{
    /* Clean up base */
    memset(&glob_btn_base, 0, sizeof(BrbBtnBase));

    glob_btn_base.brb_base = &glob_brb_base;
    glob_btn_base.buttons[BRB_BTN_SELECT].pin = BTN_PIN_SELECT;
    glob_btn_base.buttons[BRB_BTN_NEXT].pin = BTN_PIN_NEXT;
    glob_btn_base.buttons[BRB_BTN_PREV].pin = BTN_PIN_PREV;

    BrbBtnBase_Init(&glob_btn_base);

    attachInterrupt(digitalPinToInterrupt(14), BrbAppBase_BtnNext, FALLING);
    attachInterrupt(digitalPinToInterrupt(13), BrbAppBase_BtnPrev, FALLING);
    attachInterrupt(digitalPinToInterrupt(12), BrbAppBase_BtnSelect, FALLING);

    return;
}
/**********************************************************************************************************************/
void BrbToneSetup(void)
{
    /* Clean up base */
    memset(&glob_tone_base, 0, sizeof(BrbToneBase));

    glob_tone_base.pin = BUZZER_PIN;
    BrbToneBase_Init(&glob_tone_base);

    return;
}
/**********************************************************************************************************************/
static void BrbGATSBase_ZeroCrossPower()
{
    glob_gats_base.zero_power.counter++;
}
/**********************************************************************************************************************/
void BrbGATSSetup(void)
{
    /* Clean up base */
    memset(&glob_servo_base, 0, sizeof(BrbMCUServoBase));
    
    BrbMCUServoBase_Init(&glob_servo_base);

    /* Clean up base */
    memset(&glob_gats_base, 0, sizeof(BrbGATSBase));

    glob_gats_base.brb_base = (BrbBase *)&glob_brb_base;
    glob_gats_base.tone_base = (BrbToneBase *)&glob_tone_base;
    // glob_gats_base.pin_partida = GATS_PARTIDA_PIN;
    // glob_gats_base.pin_parada = GATS_PARADA_PIN;
    // glob_gats_base.pin_servo = GATS_SERVO_PIN;
    
    glob_gats_base.zero_power.pin = GATS_ZEROCROSS_PIN;

    glob_gats_base.sensor_sp01_in.pin = SENSOR_DC_SUPPLY_01_IN_PIN;
    glob_gats_base.sensor_power.pin = SENSOR_AC_POWER_PIN;

    glob_gats_base.dht_data.pin = DHT_SENSOR_PIN;
    glob_gats_base.dht_data.type = DHT_SENSOR_TYPE;

    BrbGATSBase_Init(&glob_gats_base);

    attachInterrupt(digitalPinToInterrupt(glob_gats_base.zero_power.pin), BrbGATSBase_ZeroCrossPower, RISING);

    return;
}
/**********************************************************************************************************************/
void setup()
{
    randomSeed(((analogRead(0) + analogRead(1)) / 2));

    /* Initialize Brb internal data */
    BrbSetup();

    /* Setup Display before anything, because it can display some info, eg logs */
    BrbCtlDisplay_Setup(&glob_brb_base);

    /* Setup Tone  */
    BrbToneSetup();

    /* Setup Buttons  */
    BrbBtnSetup();

    BrbGATSSetup();

    /* Setup RS485 Serial */
    BrbCtlRS485_Setup(&glob_brb_base);

    LOG_NOTICE(glob_log_base, "BrbBox Panel Control - START [%u] - 0.1.2\r\n", micros());
    LOG_NOTICE(glob_log_base, "BRB [%p], RS485 [%p]\r\n", &glob_brb_base, &glob_rs485_sess);
    LOG_NOTICE(glob_log_base, "RS485 - ADDR 0x%02x UUID [%02x-%02x-%02x-%02x]\r\n",
               glob_rs485_sess.data.address, glob_rs485_sess.data.uuid[0], glob_rs485_sess.data.uuid[1], glob_rs485_sess.data.uuid[2], glob_rs485_sess.data.uuid[3]);
    LOG_HEAP(glob_log_base);

    return;
}
/**********************************************************************************************************************/
/* RUN FOREVER */
/**********************************************************************************************************************/
void loop()
{
    /* Dispatch */
    BrbBaseLoop(&glob_brb_base);
    
    // /* Check for Buttons */
    // BrbBtnBase_Loop((BrbBtnBase *)&glob_btn_base);

    if (glob_btn_base.buttons[BRB_BTN_SELECT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_SELECT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_SELECT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_NEXT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_NEXT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_NEXT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_PREV].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_PREV].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_PREV);
    }
    
    /* Do RS485 loop */
    BrbRS485Session_Loop(&glob_rs485_sess);
    
    /* Do GATS loop */
    BrbGATSBase_Loop(&glob_gats_base);
    
    /* Do TONE loop */
    BrbToneBase_Loop(&glob_tone_base);

    return;
}
/**********************************************************************************************************************/