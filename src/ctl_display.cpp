#include "main.h"

typedef enum
{
    DISPLAY_SCREEN_CORE,
    DISPLAY_SCREEN_RS485,
    DISPLAY_SCREEN_INFO,
    DISPLAY_SCREEN_CONTROL,
    DISPLAY_SCREEN_TEMP,
    DISPLAY_SCREEN_CONSUME,
    DISPLAY_SCREEN_LASTITEM
} BrbDisplayScreen;

static BrbGenericCBH BrbCtlDisplay_Timer;

static BrbGenericCBH BrbCtlDisplay_ScreenCore;
static BrbGenericCBH BrbCtlDisplay_ScreenRS485;

static BrbGenericCBH BrbCtlDisplay_ScreenInfo;
static BrbGenericCBH BrbCtlDisplay_ScreenControl;
static BrbGenericCBH BrbCtlDisplay_ScreenConsume;
static BrbGenericCBH BrbCtlDisplay_ScreenTemp;

static const BrbDisplayScreenPrototype glob_display_screen_prototype[] =
    {
        DISPLAY_SCREEN_CORE,
        "CORE",
        BrbCtlDisplay_ScreenCore,

        DISPLAY_SCREEN_RS485,
        "RS485",
        BrbCtlDisplay_ScreenRS485,

        DISPLAY_SCREEN_INFO,
        "INFO",
        BrbCtlDisplay_ScreenInfo,

        DISPLAY_SCREEN_CONTROL,
        "CONTROL",
        BrbCtlDisplay_ScreenControl,

        DISPLAY_SCREEN_TEMP,
        "TEMP",
        BrbCtlDisplay_ScreenConsume,

        DISPLAY_SCREEN_CONSUME,
        "CONSUME",
        BrbCtlDisplay_ScreenTemp,

        DISPLAY_SCREEN_LASTITEM,
        NULL,
        NULL,
};
/**********************************************************************************************************************/
int BrbCtlDisplay_Setup(BrbBase *brb_base)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)&glob_display_base;

    /* Clean up base */
    memset(&glob_display_base, 0, sizeof(BrbDisplayBase));

    display_base->brb_base = brb_base;
    // display_base->screen_cur = DISPLAY_SCREEN_INFO;
    display_base->screen_cur = DISPLAY_SCREEN_CONTROL;
    // display_base->screen_cur = DISPLAY_SCREEN_TEMP;
    // display_base->screen_cur = DISPLAY_SCREEN_CONSUME;

    display_base->pin_led = TFT_LED;
    display_base->pin_dc = TFT_DC;
    display_base->pin_rst = TFT_RST;
    display_base->pin_cs = TFT_CS;
    display_base->pin_miso = TFT_MISO;
    display_base->pin_mosi = TFT_MOSI;
    display_base->pin_clk = TFT_CLK;
    display_base->screen_arr_ptr = (BrbDisplayScreenPrototype *)&glob_display_screen_prototype;
    display_base->screen_arr_cnt = sizeof(glob_display_screen_prototype) / sizeof(BrbDisplayScreenPrototype);

    // display_base->tft = (ILI9341_due *)&tft;
    display_base->tft = new ILI9341_due(display_base->pin_cs, display_base->pin_dc, display_base->pin_rst);
    // display_base->tft = new TFT_eSPI();

    BrbDisplayBase_Init(display_base);
    BrbDisplayBase_ScreenAction(display_base, -1);

    BrbTimerAdd(display_base->brb_base, 5000, 0, BrbCtlDisplay_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
/* DISPLAY */
/**********************************************************************************************************************/
static int BrbCtlDisplay_Timer(void *base_ptr, void *cb_data_ptr)
{
    // BrbTimer *timer = (BrbTimer *)base_ptr;
    BrbDisplayBase *display_base = (BrbDisplayBase *)cb_data_ptr;
    BrbGATSBase *gats_base = (BrbGATSBase *)&glob_gats_base;

    BrbDisplayBase_ScreenAction(display_base, -1);

    // display_base->screen_cur++;

    int delay = 5000;

    switch (gats_base->state.code)
    {
    case GATS_STATE_START_INIT:
    case GATS_STATE_START_DELAY:
    case GATS_STATE_START_CHECK:
    case GATS_STATE_STOP_INIT:
    case GATS_STATE_STOP_DELAY:
    case GATS_STATE_STOP_CHECK:
    {
        delay = 1500;
        break;
    }
    case GATS_STATE_FAILURE:
    {
        delay = 2500;
        break;
    }
    case GATS_STATE_RUNNING:
    case GATS_STATE_NONE:
    default:
    {
        delay = 5000;
        break;
    }
    }

    BrbTimerAdd(&glob_brb_base, delay, 0, BrbCtlDisplay_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenInfo(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGATSBase *gats_base = (BrbGATSBase *)&glob_gats_base;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Geral"));
    }

    pos_x = 10;
    pos_y = 50;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("Bateria"), gats_base->sensor_sp01_in.value, 1, PSTR("VDC"));

    display_base->tft->fillRect(pos_x + 160, pos_y + 15, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x + 160, pos_y, PSTR("Energia"), gats_base->sensor_power.value, 1, PSTR("VAC"));

    pos_x = 10;
    pos_y = pos_y + 65;

    BrbDisplayBase_DrawArc(display_base, gats_base->info.gas, 0, 100, pos_x, pos_y, 65, PSTR("Tanque"), DISPLAY_ARC_RED2GREEN);

    pos_x = pos_x + 160;

    BrbDisplayBase_DrawArc(display_base, gats_base->info.load, 0, 30, pos_x, pos_y, 65, PSTR("Amp"), DISPLAY_ARC_GREEN2RED);

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenTemp(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGATSBase *gats_base = (BrbGATSBase *)&glob_gats_base;

    int pos_x;
    int pos_y;

    int sz_w = 224;
    int sz_h = 40;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Temperatura"));
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = sz_h;

    BrbDisplayBase_DrawBarGraph(display_base, pos_x, pos_y, 130, gats_base->dht_data.dht_temp, -50, 150);

    pos_x = 90;
    pos_y = sz_h;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(gats_base->dht_data.dht_temp, -25, 100, 0, 127));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("TEMP"), gats_base->dht_data.dht_temp, 1, PSTR("C"));

    // BrbDisplayBase_DrawArcSeg(display_base, gats_base->dht_data.dht_temp, 0, 130, pos_x, pos_y, 100, PSTR("Celsius"), DISPLAY_ARC_GREEN2RED, 0, 3, 5);

    pos_x = pos_x;
    pos_y = pos_y + 60;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(gats_base->dht_data.dht_humi, -25, 100, 127, 0));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HUMIDADE"), gats_base->dht_data.dht_humi, 1, PSTR("%"));

    pos_x = pos_x;
    pos_y = pos_y + 60;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(gats_base->dht_data.dht_hidx, -25, 100, 0, 127));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HEAT INDEX"), gats_base->dht_data.dht_hidx, 1, PSTR("C"));

    return 0;
}
/****************************************************************************************************/
int BrbCtlDisplay_ScreenControl(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGATSBase *gats_base = (BrbGATSBase *)&glob_gats_base;

    char sub_str[16] = {0};

    int pos_x;
    int pos_y;

    const char *title_ptr = NULL;
    const char *text_ptr = NULL;

    int retry_max = GATS_TIMER_START_RETRY_MAX;
    int color = ILI9341_ORANGERED;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Controle"));
    }

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = -1;
    }

    switch (gats_base->state.code)
    {
    case GATS_STATE_START_INIT:
    {
        retry_max = GATS_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GATS_TIMER_START_WAIT_MS - gats_base->state.delta) / 1000));
        color = ILI9341_ORANGERED;
        break;
    }
    case GATS_STATE_START_DELAY:
    {
        retry_max = GATS_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GATS_TIMER_START_DELAY_MS - gats_base->state.delta) / 1000));
        color = ILI9341_ORANGERED;
        break;
    }
    case GATS_STATE_START_CHECK:
    {
        retry_max = GATS_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GATS_TIMER_START_CHECK_MS - gats_base->state.delta) / 1000));
        color = ILI9341_ORANGERED;
        break;
    }
    case GATS_STATE_RUNNING:
    {
        retry_max = GATS_TIMER_START_RETRY_MAX;
        long delta_minutes = (gats_base->state.delta / 1000) / 60;
        snprintf(sub_str, sizeof(sub_str) - 1, "%dh%02dm", (int)(delta_minutes / 60), (int)(delta_minutes % 60));
        color = ILI9341_ORANGERED;
        break;
    }
    case GATS_STATE_FAILURE:
    {
        retry_max = GATS_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((gats_base->state.delta) / 1000));
        break;
    }
    case GATS_STATE_STOP_INIT:
    {
        retry_max = GATS_TIMER_STOP_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GATS_TIMER_STOP_CHECK_MS - gats_base->state.delta) / 1000));
        color = ILI9341_SEAGREEN;
        break;
    }
    case GATS_STATE_STOP_DELAY:
    {
        retry_max = GATS_TIMER_STOP_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GATS_TIMER_STOP_DELAY_MS - gats_base->state.delta) / 1000));
        color = ILI9341_SEAGREEN;
        break;
    }
    case GATS_STATE_STOP_CHECK:
    {
        retry_max = GATS_TIMER_STOP_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GATS_TIMER_STOP_CHECK_MS - gats_base->state.delta) / 1000));
        color = ILI9341_SEAGREEN;
        break;
    }
    case GATS_STATE_NONE:
    {
        retry_max = 0;
        long delta_minutes = (gats_base->state.delta / 1000) / 60;
        snprintf(sub_str, sizeof(sub_str) - 1, "%dh%02dm", (int)(delta_minutes / 60), (int)(delta_minutes % 60));
        color = ILI9341_SEAGREEN;
        break;
    }
    default:
    {
        /**/
    }
    }

    display_base->tft->fillRect(10, 50, 300, 60, ILI9341_WHITE);

    if (display_base->flags.on_action)
    {
        title_ptr = BrbGATSBase_GetStateAction(gats_base);
        text_ptr = BrbGATSBase_GetFailure(gats_base);

        if (!text_ptr)
        {
            text_ptr = BrbGATSBase_GetState(gats_base);
        }
    }
    else
    {
        title_ptr = BrbGATSBase_GetState(gats_base);
        text_ptr = BrbGATSBase_GetFailure(gats_base);

        // /* First check for failures */
        // title_ptr = BrbGATSBase_GetFailure(gats_base);

        // if (!title_ptr)
        // {
        //     title_ptr = BrbGATSBase_GetStateAction(gats_base);
        // }
        // else
        // {
        //     text_ptr = BrbGATSBase_GetState(gats_base);
        // }

        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
        display_base->tft->setTextScale(2);
        display_base->tft->printAtPivoted(sub_str, 310, 50, gTextPivotTopRight);

        sprintf(sub_str, "%d/%d", gats_base->state.retry, retry_max);
        display_base->tft->printAtPivoted(sub_str, 310, 75, gTextPivotTopRight);
    }

    display_base->tft->setTextColor(color, ILI9341_WHITE);
    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(10, 50);
    display_base->tft->println((const __FlashStringHelper *)title_ptr);

    if (text_ptr)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(10, 80);
        display_base->tft->print((const __FlashStringHelper *)text_ptr);

        // if (gats_base->state.fail > 0)
        // {
        //     display_base->tft->print(":");
        //     display_base->tft->cursorToXY(display_base->tft->getCursorX() + 5, 80);
        //     display_base->tft->println(gats_base->state.fail);
        // }
    }

    pos_x = 10;
    pos_y = 110;

    BrbServo *servo_bb;

    servo_bb = BrbServoGrabByPin(gats_base->brb_base, gats_base->pin_servo);

    display_base->tft->fillRect(pos_x, pos_y + 15, 300, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("ENERGIA"), gats_base->sensor_power.value, 1, PSTR("VAC"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 110, pos_y, PSTR("FREQUENCIA"), gats_base->zero_power.value, 1, PSTR("Hz"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 210, pos_y, PSTR("SERVO"), servo_bb ? servo_bb->pos_cur : 0, 1, PSTR("o"));
    // BrbDisplayBase_BoxMax(display_base, pos_x, pos_y, PSTR("Tentativas"), gats_base->state.retry, retry_max);

    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == DISPLAY_ACTION_SELECT)
        {
            if (display_base->user_int == 1)
            {
                switch (gats_base->state.code)
                {
                case GATS_STATE_NONE:
                case GATS_STATE_STOP_INIT:
                case GATS_STATE_STOP_DELAY:
                case GATS_STATE_STOP_CHECK:
                {
                    BrbGATSBase_ActionCmd(gats_base, GATS_ACTION_START);
                    break;
                }
                case GATS_STATE_START_INIT:
                case GATS_STATE_START_DELAY:
                case GATS_STATE_START_CHECK:
                case GATS_STATE_RUNNING:
                {
                    BrbGATSBase_ActionCmd(gats_base, GATS_ACTION_STOP);
                    break;
                }
                case GATS_STATE_FAILURE:
                {
                    BrbGATSBase_FailureConfirm(gats_base);
                    break;
                }
                default:
                {
                    break;
                }
                }
            }

            display_base->flags.on_action = 0;
            display_base->user_int = 0;
            display_base->screen_last = -1;

            BrbDisplayBase_ScreenAction(display_base, -1);

            return 0;
        }
        else if ((display_base->action_code == DISPLAY_ACTION_NEXT) || (display_base->action_code == DISPLAY_ACTION_PREV))
        {
            display_base->user_int = !display_base->user_int;
        }

        BrbDisplayBase_DrawBtn(display_base, 20, 170, 120, 60, PSTR("SIM"), display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
        BrbDisplayBase_DrawBtn(display_base, 170, 170, 120, 60, PSTR("NAO"), !display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, !display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
    }
    else
    {

        const char *btn_text_ptr = BrbGATSBase_GetStateButton(gats_base);

        BrbDisplayBase_DrawBtn(display_base, 20, 170, 280, 60, btn_text_ptr, color, ILI9341_WHITE);
    }

    return 0;
}
/****************************************************************************************************/
int BrbCtlDisplay_ScreenConsume(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGATSBase *gats_base = (BrbGATSBase *)&glob_gats_base;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Consumo"));
    }

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = -1;
    }

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == DISPLAY_ACTION_SELECT)
        {
            if (display_base->user_int == 1)
            {
                BrbGATSBase_HourmeterReset(gats_base);
            }

            display_base->flags.on_action = 0;
            display_base->user_int = 0;
            display_base->screen_last = -1;

            BrbDisplayBase_ScreenAction(display_base, -1);

            return 0;
        }
        else if ((display_base->action_code == DISPLAY_ACTION_NEXT) || (display_base->action_code == DISPLAY_ACTION_PREV))
        {
            display_base->user_int = !display_base->user_int;
        }

        display_base->tft->setFont(DISPLAY_FONT_TITLE);
        display_base->tft->setTextScale(2);
        display_base->tft->printAtPivoted(PSTR("Zerar Horimetro?"), 160, 80, gTextPivotMiddleCenter);

        display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
        display_base->tft->setTextScale(1);

        BrbDisplayBase_DrawBtn(display_base, 20, 170, 120, 60, PSTR("SIM"), display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
        BrbDisplayBase_DrawBtn(display_base, 170, 170, 120, 60, PSTR("NAO"), !display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, !display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);

        return 0;
    }

    pos_x = 10;
    pos_y = 50;

    double value_dec = ((double)(gats_base->data.hourmeter_time) / 60.0);

    BrbDisplayBase_DrawArcSeg(display_base, value_dec, 0, GATS_HOURMETER_MAX, pos_x, pos_y, 100, PSTR("Horas"), DISPLAY_ARC_GREEN2RED, 0, 3, 5);

    pos_x = 220;
    pos_y = 50;

    // display_base->tft->fillRect(pos_x, pos_y + 20, 70, 30, ILI9341_PURPLE);
    // BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("SOBRA"), (GATS_HOURMETER_MAX - value_dec), 1, PSTR("Hrs"));

    // display_base->tft->fillRect(pos_x, pos_y + 80, 70, 30, ILI9341_PURPLE);
    // BrbDisplayBase_BoxSub(display_base, pos_x + 220, pos_y, PSTR("ZERAGEM"), gats_base->data.hourmeter_reset, 0, PSTR("x"));

    pos_x = 10;
    pos_y = 180;

    display_base->tft->fillRect(pos_x, pos_y + 20, 300, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("SOBRA"), (GATS_HOURMETER_MAX - value_dec), 1, PSTR("Hrs"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 110, pos_y, PSTR("TOTAL"), (gats_base->data.hourmeter_time / 60), 0, PSTR("Hrs"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 220, pos_y, PSTR("ZERAGEM"), gats_base->data.hourmeter_reset, 0, PSTR("x"));

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenCore(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbBase *brb_base = (BrbBase *)brb_base_ptr;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Core"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = ILI9341_MIDNIGHTBLUE;
    BrbDisplayBase_BoxUptime(display_base, pos_x, pos_y, PSTR("UpTime"), millis() / 1000);
    BrbDisplayBase_BoxUptime(display_base, pos_x + 140, pos_y, PSTR("LifeTime"), brb_base->data.lifetime_sec);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Memoria"), PSTR("%d"), BrbBase_FreeRAM());
    // BrbDisplayBase_BoxSub(display_base, pos_x + 160, pos_y, PSTR("Memoria"), BrbBase_FreeRAM(), 1, PSTR("%"));

    // pos_x = DISPLAY_SZ_MARGIN;
    // pos_y = pos_y + 55;

    // display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    // BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("RX/TX"), PSTR("%lu/%lu"), rs485_sess->stats.byte.rx, rs485_sess->stats.byte.tx);
    // BrbDisplayBase_BoxUnit(display_base, pos_x, pos_y + DISPLAY_SZ_BOX_H, PSTR("KB"));

    // BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("Error"), PSTR("%lu/%lu"),
    //                       (rs485_sess->stats.err.bad_char + rs485_sess->stats.err.crc + rs485_sess->stats.err.overflow + rs485_sess->stats.err.pkt),
    //                       (rs485_sess->stats.pkt.err.cmd_id + rs485_sess->stats.pkt.err.cmd_no_bcast + rs485_sess->stats.pkt.err.cmd_no_cb));

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenRS485(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("RS485"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    BrbBase *brb_base = (BrbBase *)brb_base_ptr;
    BrbRS485Session *rs485_sess = (BrbRS485Session *)&glob_rs485_sess;

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 110, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = ILI9341_MIDNIGHTBLUE;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("UUID"), PSTR("%02x-%02x-%02x-%02x"), rs485_sess->data.uuid[0], rs485_sess->data.uuid[1], rs485_sess->data.uuid[2], rs485_sess->data.uuid[2]);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("ADDR"), PSTR("%02x"), rs485_sess->data.address);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Packet RX/TX"), PSTR("%lu/%lu"), rs485_sess->stats.pkt.rx, rs485_sess->stats.pkt.tx);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 130, pos_y, PSTR("Me"), PSTR("%lu"), rs485_sess->stats.pkt.me);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Broadcast"), PSTR("%lu"), rs485_sess->stats.pkt.bcast);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    //
    // BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Pkt Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.pkt.err.cmd_id, rs485_sess->stats.pkt.err.cmd_no_bcast, rs485_sess->stats.pkt.err.cmd_no_cb);
    // BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Pkt Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.pkt.err.cmd_id, rs485_sess->stats.pkt.err.cmd_no_bcast, rs485_sess->stats.pkt.err.cmd_no_cb);

    char byte_rx[16];
    char byte_tx[16];

    dtostrf((double)(rs485_sess->stats.byte.rx / 1024.0), 5, 2, byte_rx);
    dtostrf((double)(rs485_sess->stats.byte.tx / 1024.0), 5, 2, byte_tx);

    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Bytes RX/TX"), PSTR("%s/%s"), byte_rx, byte_tx);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.err.bad_char, rs485_sess->stats.err.crc, rs485_sess->stats.err.overflow);

    return 0;
}
/**********************************************************************************************************************/
