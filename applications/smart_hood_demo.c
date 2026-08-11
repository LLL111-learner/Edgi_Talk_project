#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <lvgl.h>
#include <stdint.h>
#include <string.h>

#include "drv_ipc.h"
#include "hood_audio_ipc.h"

#define APP_TAG "smart_hood"
#define HISTORY_POINTS 60
#define EVENT_LOG_LINES 5
#define AUTO_OFF_DELAY_TICKS 8
#define GAS_CLEAR_DELAY_TICKS 5
#define LED_PIN_G GET_PIN(16, 6)
#define LED_PIN_B GET_PIN(16, 5)
#define BUTTON_PIN GET_PIN(8, 3)
#define LED_ON PIN_LOW
#define LED_OFF PIN_HIGH

typedef enum
{
    SCENE_IDLE = 0,
    SCENE_SOUP,
    SCENE_STIR_FRY,
    SCENE_DEEP_FRY,
    SCENE_GAS_LEAK,
    SCENE_DRY_BURN,
} hood_scene_t;

typedef enum
{
    FAN_OFF = 0,
    FAN_LOW,
    FAN_HIGH,
} fan_level_t;

typedef enum
{
    MODE_AUTO = 0,
    MODE_MANUAL,
} run_mode_t;

typedef enum
{
    STATE_STANDBY = 0,
    STATE_LOW_EXHAUST,
    STATE_HIGH_EXHAUST,
    STATE_PURGE_DELAY,
    STATE_GAS_WARN,
    STATE_GAS_DANGER,
    STATE_MANUAL_OVERRIDE,
} hood_state_t;


typedef enum
{
    UI_PAGE_HOME = 0,
    UI_PAGE_DASHBOARD,
    UI_PAGE_SCENE,
    UI_PAGE_SAFETY,
    UI_PAGE_REPORT,
} hood_ui_page_t;
typedef struct
{
    int smoke;
    int gas;
    int temperature;
} sensor_sample_t;

typedef struct
{
    hood_scene_t scene;
    run_mode_t mode;
    hood_state_t state;
    fan_level_t fan;
    sensor_sample_t sample;
    int risk_score;
    int scene_tick;
    int purge_countdown;
    int gas_clear_countdown;
    int exhaust_seconds;
    int alarm_count;
    int max_risk_score;
    rt_bool_t danger_latched;
    rt_bool_t alarm_muted;
    volatile rt_bool_t key_event_pending;
    int hardware_tick;
} hood_model_t;


    typedef struct
{
    lv_obj_t *screen;
    lv_obj_t *body;
    hood_ui_page_t page;
    lv_obj_t *title_label;
    lv_obj_t *state_label;
    lv_obj_t *mode_label;
    lv_obj_t *fan_label;
    lv_obj_t *risk_bar;
    lv_obj_t *risk_label;
    lv_obj_t *smoke_label;
    lv_obj_t *gas_label;
    lv_obj_t *temp_label;
    lv_obj_t *countdown_label;
    lv_obj_t *chart;
    lv_chart_series_t *smoke_series;
    lv_chart_series_t *gas_series;
    lv_obj_t *event_labels[EVENT_LOG_LINES];
    lv_obj_t *report_label;
    lv_obj_t *hardware_label;
} hood_ui_t;

static hood_model_t g_model;
static hood_ui_t g_ui;
static char g_events[EVENT_LOG_LINES][64];
static rt_device_t g_audio_ipc_dev = RT_NULL;
static rt_uint32_t g_audio_ipc_seq = 0;

static rt_err_t hood_audio_ipc_init(void)
{
    g_audio_ipc_dev = edge_ipc_device_find();
    if (g_audio_ipc_dev == RT_NULL)
    {
        if (edge_ipc_device_register() != RT_EOK)
        {
            rt_kprintf("[%s] audio IPC register failed\n", APP_TAG);
            return -RT_ERROR;
        }
        g_audio_ipc_dev = edge_ipc_device_find();
    }

    if (g_audio_ipc_dev == RT_NULL)
    {
        rt_kprintf("[%s] audio IPC device not found\n", APP_TAG);
        return -RT_ERROR;
    }

    if (rt_device_open(g_audio_ipc_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("[%s] audio IPC open failed\n", APP_TAG);
        g_audio_ipc_dev = RT_NULL;
        return -RT_ERROR;
    }

    rt_kprintf("[%s] audio IPC ready\n", APP_TAG);
    return RT_EOK;
}

static void hood_audio_send(hood_audio_cmd_t command, rt_uint16_t value)
{
    edge_rc_frame_t frame;

    if (g_audio_ipc_dev == RT_NULL)
    {
        rt_kprintf("[%s] audio IPC not ready: cmd=%d\n", APP_TAG, command);
        return;
    }

    hood_audio_frame_prepare(&frame, command, ++g_audio_ipc_seq, value);
    if (rt_device_write(g_audio_ipc_dev, 0, &frame, 1) != 1)
    {
        rt_kprintf("[%s] audio IPC send failed: cmd=%d\n", APP_TAG, command);
    }
    else
    {
        rt_kprintf("[%s] audio IPC sent: cmd=%d seq=%lu value=%u\n",
                   APP_TAG,
                   command,
                   g_audio_ipc_seq,
                   value);
    }
}

static hood_audio_cmd_t prompt_to_audio_cmd(const char *message)
{
    if (strstr(message, "Gas danger") != RT_NULL)
    {
        return HOOD_AUDIO_CMD_GAS_DANGER;
    }
    if (strstr(message, "safe range") != RT_NULL)
    {
        return HOOD_AUDIO_CMD_GAS_CLEAR;
    }
    if (strstr(message, "blocked") != RT_NULL)
    {
        return HOOD_AUDIO_CMD_MANUAL_BLOCKED;
    }
    if (strstr(message, "High exhaust") != RT_NULL)
    {
        return HOOD_AUDIO_CMD_HIGH_EXHAUST;
    }
    if (strstr(message, "low exhaust") != RT_NULL)
    {
        return HOOD_AUDIO_CMD_LOW_EXHAUST;
    }
    if (strstr(message, "stopped") != RT_NULL)
    {
        return HOOD_AUDIO_CMD_FAN_STOP;
    }

    return HOOD_AUDIO_CMD_NONE;
}
static const char *scene_name(hood_scene_t scene)
{
    switch (scene)
    {
    case SCENE_SOUP:
        return "Soup";
    case SCENE_STIR_FRY:
        return "Stir Fry";
    case SCENE_DEEP_FRY:
        return "Deep Fry";
    case SCENE_GAS_LEAK:
        return "Gas Leak";
    case SCENE_DRY_BURN:
        return "Dry Burn";
    default:
        return "Clean";
    }
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int approach_int(int current, int target, int step)
{
    if (current < target)
    {
        return clamp_int(current + step, current, target);
    }
    if (current > target)
    {
        return clamp_int(current - step, target, current);
    }
    return current;
}

static void voice_prompt(const char *message)
{
    hood_audio_cmd_t command = prompt_to_audio_cmd(message);

    if (g_model.alarm_muted)
    {
        rt_kprintf("[%s] VOICE muted: %s\n", APP_TAG, message);
        return;
    }

    if (command != HOOD_AUDIO_CMD_NONE)
    {
        hood_audio_send(command, (rt_uint16_t)g_model.risk_score);
    }

    rt_kprintf("[%s] VOICE: %s\n", APP_TAG, message);
}

static void event_log_push(const char *message)
{
    int i;

    for (i = EVENT_LOG_LINES - 1; i > 0; i--)
    {
        rt_strncpy(g_events[i], g_events[i - 1], sizeof(g_events[i]));
        g_events[i][sizeof(g_events[i]) - 1] = '\0';
    }

    rt_snprintf(g_events[0], sizeof(g_events[0]), "%02d:%02d  %s",
                (g_model.exhaust_seconds / 60) % 100,
                g_model.exhaust_seconds % 60,
                message);
}

static void hardware_key_isr(void *args)
{
    static rt_tick_t last_tick = 0;
    rt_tick_t now;

    LV_UNUSED(args);
    now = rt_tick_get();
    if (now - last_tick < RT_TICK_PER_SECOND / 5)
    {
        return;
    }

    last_tick = now;
    g_model.key_event_pending = RT_TRUE;
}

static void hardware_init(void)
{
    rt_pin_mode(LED_PIN_G, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_PIN_B, PIN_MODE_OUTPUT);
    rt_pin_write(LED_PIN_G, LED_OFF);
    rt_pin_write(LED_PIN_B, LED_OFF);

    rt_pin_mode(BUTTON_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(BUTTON_PIN, PIN_IRQ_MODE_FALLING, hardware_key_isr, RT_NULL);
    rt_pin_irq_enable(BUTTON_PIN, PIN_IRQ_ENABLE);
}

static void hardware_set_leds(void)
{
    g_model.hardware_tick++;

    if (g_model.state == STATE_GAS_DANGER)
    {
        rt_pin_write(LED_PIN_G, LED_OFF);
        rt_pin_write(LED_PIN_B, (g_model.hardware_tick % 2) ? LED_ON : LED_OFF);
    }
    else if (g_model.state == STATE_GAS_WARN || g_model.risk_score >= 60)
    {
        rt_pin_write(LED_PIN_G, LED_OFF);
        rt_pin_write(LED_PIN_B, LED_ON);
    }
    else
    {
        rt_pin_write(LED_PIN_G, LED_ON);
        rt_pin_write(LED_PIN_B, LED_OFF);
    }
}

static void handle_key_event(void)
{
    if (!g_model.key_event_pending)
    {
        return;
    }

    g_model.key_event_pending = RT_FALSE;
    g_model.alarm_muted = !g_model.alarm_muted;
    event_log_push(g_model.alarm_muted ? "Board key: alarm muted" : "Board key: alarm unmuted");
    hood_audio_send(g_model.alarm_muted ? HOOD_AUDIO_CMD_MUTE : HOOD_AUDIO_CMD_UNMUTE,
                    (rt_uint16_t)g_model.risk_score);
    rt_kprintf("[%s] board key toggled alarm mute: %s\n",
               APP_TAG,
               g_model.alarm_muted ? "on" : "off");
}

static void set_fan(fan_level_t fan, const char *reason)
{
    if (g_model.fan == fan)
    {
        return;
    }

    g_model.fan = fan;
    event_log_push(reason);

    if (fan == FAN_LOW)
    {
        voice_prompt("Smoke rising, low exhaust enabled.");
    }
    else if (fan == FAN_HIGH)
    {
        voice_prompt("High exhaust enabled.");
    }
    else
    {
        voice_prompt("Fan stopped, system standby.");
    }
}

static void set_state(hood_state_t state, const char *event)
{
    if (g_model.state == state)
    {
        return;
    }

    g_model.state = state;
    if (event != RT_NULL)
    {
        event_log_push(event);
    }
}

static int calc_risk_score(const sensor_sample_t *sample)
{
    int smoke_score = sample->smoke * 45 / 100;
    int gas_score = sample->gas * 60 / 100;
    int temp_score = 0;
    int duration_score = 0;
    int manual_penalty = 0;

    if (sample->temperature > 55)
    {
        temp_score = (sample->temperature - 55) * 2;
    }
    if (g_model.fan != FAN_OFF && g_model.exhaust_seconds > 60)
    {
        duration_score = 8;
    }
    if (g_model.mode == MODE_MANUAL)
    {
        manual_penalty = 5;
    }

    return clamp_int(smoke_score + gas_score + temp_score + duration_score + manual_penalty, 0, 100);
}

static void update_sensor_simulator(void)
{
    int target_smoke = 5;
    int target_gas = 3;
    int target_temp = 28;
    int smoke_step = 4;
    int gas_step = 3;
    int temp_step = 1;

    g_model.scene_tick++;

    switch (g_model.scene)
    {
    case SCENE_SOUP:
        target_smoke = 28 + ((g_model.scene_tick / 4) % 8);
        target_gas = 4;
        target_temp = 42;
        break;
    case SCENE_STIR_FRY:
        target_smoke = 54 + ((g_model.scene_tick / 3) % 18);
        target_gas = 6;
        target_temp = 50;
        break;
    case SCENE_DEEP_FRY:
        target_smoke = 82 + ((g_model.scene_tick / 2) % 12);
        target_gas = 8;
        target_temp = 62;
        smoke_step = 7;
        temp_step = 2;
        break;
    case SCENE_GAS_LEAK:
        target_smoke = 22;
        target_gas = 88 + ((g_model.scene_tick / 3) % 8);
        target_temp = 34;
        gas_step = 9;
        break;
    case SCENE_DRY_BURN:
        target_smoke = 46 + ((g_model.scene_tick / 5) % 12);
        target_gas = 10;
        target_temp = 84 + ((g_model.scene_tick / 4) % 8);
        smoke_step = 5;
        temp_step = 4;
        break;
    default:
        target_smoke = 5;
        target_gas = 3;
        target_temp = 28;
        break;
    }

    if (g_model.fan == FAN_LOW)
    {
        target_smoke -= 10;
    }
    else if (g_model.fan == FAN_HIGH)
    {
        target_smoke -= 24;
        target_gas -= 10;
    }

    g_model.sample.smoke = approach_int(g_model.sample.smoke, clamp_int(target_smoke, 0, 100), smoke_step);
    g_model.sample.gas = approach_int(g_model.sample.gas, clamp_int(target_gas, 0, 100), gas_step);
    g_model.sample.temperature = approach_int(g_model.sample.temperature, clamp_int(target_temp, 0, 100), temp_step);
}

static void update_control_fsm(void)
{
    g_model.risk_score = calc_risk_score(&g_model.sample);
    if (g_model.risk_score > g_model.max_risk_score)
    {
        g_model.max_risk_score = g_model.risk_score;
    }

    if (g_model.fan != FAN_OFF)
    {
        g_model.exhaust_seconds++;
    }

    if (g_model.sample.gas >= 75)
    {
        if (g_model.state != STATE_GAS_DANGER)
        {
            g_model.alarm_count++;
            g_model.danger_latched = RT_TRUE;
            g_model.gas_clear_countdown = GAS_CLEAR_DELAY_TICKS;
            set_state(STATE_GAS_DANGER, "Gas danger: forced high exhaust");
            voice_prompt("Gas danger detected. Forced exhaust is active.");
        }
        set_fan(FAN_HIGH, "Fan forced to HIGH");
        return;
    }

    if (g_model.danger_latched)
    {
        if (g_model.sample.gas <= 35)
        {
            if (g_model.gas_clear_countdown > 0)
            {
                g_model.gas_clear_countdown--;
                set_state(STATE_GAS_WARN, "Gas clearing confirmation");
                set_fan(FAN_HIGH, "Keep high exhaust while clearing");
                return;
            }

            g_model.danger_latched = RT_FALSE;
            event_log_push("Gas danger cleared");
            voice_prompt("Gas level returned to safe range.");
        }
        else
        {
            g_model.gas_clear_countdown = GAS_CLEAR_DELAY_TICKS;
            set_state(STATE_GAS_WARN, "Gas warning");
            set_fan(FAN_HIGH, "Keep high exhaust for gas warning");
            return;
        }
    }

    if (g_model.sample.gas >= 45)
    {
        set_state(STATE_GAS_WARN, "Gas warning");
        set_fan(FAN_HIGH, "Gas warning high exhaust");
        return;
    }

    if (g_model.mode == MODE_MANUAL)
    {
        set_state(STATE_MANUAL_OVERRIDE, RT_NULL);
        return;
    }

    if (g_model.sample.smoke >= 65 || g_model.sample.temperature >= 78)
    {
        g_model.purge_countdown = AUTO_OFF_DELAY_TICKS;
        set_state(STATE_HIGH_EXHAUST, "Auto high exhaust");
        set_fan(FAN_HIGH, "Smoke heavy: HIGH");
    }
    else if (g_model.sample.smoke >= 28)
    {
        g_model.purge_countdown = AUTO_OFF_DELAY_TICKS;
        set_state(STATE_LOW_EXHAUST, "Auto low exhaust");
        set_fan(FAN_LOW, "Smoke detected: LOW");
    }
    else if (g_model.fan != FAN_OFF)
    {
        if (g_model.purge_countdown > 0)
        {
            g_model.purge_countdown--;
            set_state(STATE_PURGE_DELAY, "Purge delay");
            set_fan(FAN_LOW, "Purge remaining smoke");
        }
        else
        {
            set_state(STATE_STANDBY, "Clean air standby");
            set_fan(FAN_OFF, "Auto fan off");
        }
    }
    else
    {
        set_state(STATE_STANDBY, RT_NULL);
        set_fan(FAN_OFF, "Standby");
    }
}

static const char *scene_name_cn(hood_scene_t scene)
{
    switch (scene)
    {
    case SCENE_SOUP:
        return "Soup";
    case SCENE_STIR_FRY:
        return "Stir";
    case SCENE_DEEP_FRY:
        return "Fry";
    case SCENE_GAS_LEAK:
        return "Gas Leak";
    case SCENE_DRY_BURN:
        return "Dry Burn";
    default:
        return "Clean";
    }
}

static const char *state_name_cn(hood_state_t state)
{
    switch (state)
    {
    case STATE_LOW_EXHAUST:
        return "Low Exhaust";
    case STATE_HIGH_EXHAUST:
        return "High Exhaust";
    case STATE_PURGE_DELAY:
        return "Purge";
    case STATE_GAS_WARN:
        return "Gas Warn";
    case STATE_GAS_DANGER:
        return "Gas Danger";
    case STATE_MANUAL_OVERRIDE:
        return "Manual";
    default:
        return "Standby";
    }
}

static const char *fan_name_cn(fan_level_t fan)
{
    switch (fan)
    {
    case FAN_LOW:
        return "LOW";
    case FAN_HIGH:
        return "HIGH";
    default:
        return "OFF";
    }
}

static const char *mode_name_cn(run_mode_t mode)
{
    return mode == MODE_AUTO ? "AUTO" : "MANUAL";
}

static const char *risk_level_cn(void)
{
    if (g_model.state == STATE_GAS_DANGER || g_model.risk_score >= 75)
    {
        return "Danger";
    }
    if (g_model.state == STATE_GAS_WARN || g_model.risk_score >= 45)
    {
        return "Warn";
    }
    return "Safe";
}

static const char *event_text_cn(const char *event)
{
    const char *body = event;
    static char text[64];

    if (event == RT_NULL || event[0] == '\0')
    {
        return "-";
    }

    if (rt_strlen(event) > 8 && event[2] == ':' && event[5] == ' ')
    {
        body = event + 8;
    }

    if (strstr(body, "Gas danger") != RT_NULL || strstr(body, "gas danger") != RT_NULL)
    {
        body = "Gas danger, forced HIGH";
    }
    else if (strstr(body, "Gas warning") != RT_NULL || strstr(body, "gas warning") != RT_NULL)
    {
        body = "Gas warning, high exhaust";
    }
    else if (strstr(body, "Gas clearing") != RT_NULL)
    {
        body = "Gas clearing confirm";
    }
    else if (strstr(body, "Gas danger cleared") != RT_NULL)
    {
        body = "Gas danger cleared";
    }
    else if (strstr(body, "Auto high") != RT_NULL || strstr(body, "HIGH") != RT_NULL)
    {
        body = "Auto HIGH exhaust";
    }
    else if (strstr(body, "Auto low") != RT_NULL || strstr(body, "LOW") != RT_NULL)
    {
        body = "AUTOOnLOW";
    }
    else if (strstr(body, "Purge") != RT_NULL)
    {
        body = "Purge remaining smoke";
    }
    else if (strstr(body, "standby") != RT_NULL || strstr(body, "Standby") != RT_NULL)
    {
        body = "Clean air standby";
    }
    else if (strstr(body, "Manual") != RT_NULL)
    {
        body = "Manual control";
    }
    else if (strstr(body, "Auto mode") != RT_NULL)
    {
        body = "Auto mode enabled";
    }
    else if (strstr(body, "System boot") != RT_NULL)
    {
        body = "System boot";
    }
    else if (strstr(body, "Soup") != RT_NULL)
    {
        body = "Scene: Soup";
    }
    else if (strstr(body, "Stir") != RT_NULL)
    {
        body = "Scene: Stir";
    }
    else if (strstr(body, "Deep") != RT_NULL)
    {
        body = "Scene: Fry";
    }
    else if (strstr(body, "Dry") != RT_NULL)
    {
        body = "Scene: Dry Burn";
    }
    else if (strstr(body, "Clean") != RT_NULL)
    {
        body = "Scene: Clean";
    }

    rt_snprintf(text, sizeof(text), "%.5s  %s", event, body);
    return text;
}

static lv_obj_t *make_panel(lv_obj_t *parent, int x, int y, int w, int h, lv_color_t color)
{
    lv_obj_t *panel = lv_obj_create(parent);

    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_style_radius(panel, 8, 0);
    lv_obj_set_style_bg_color(panel, color, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x26343a), 0);
    lv_obj_set_style_pad_all(panel, 8, 0);
    return panel;
}

static void style_label(lv_obj_t *label, lv_color_t color, lv_text_align_t align)
{
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_align(label, align, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
}

static lv_obj_t *make_label(lv_obj_t *parent, const char *text, int x, int y, int w, int h, int font_size_hint)
{
    lv_obj_t *label = lv_label_create(parent);
    LV_UNUSED(font_size_hint);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_size(label, w, h);
    style_label(label, lv_color_hex(0xe8f3f4), LV_TEXT_ALIGN_LEFT);
    return label;
}

static lv_obj_t *make_button(lv_obj_t *parent, const char *text, int x, int y, int w, int h, lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, h);
    lv_obj_set_style_radius(button, 7, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x176b87), 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x1f8fb3), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_border_color(button, lv_color_hex(0x4fb6cc), 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, user_data);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), 0);
    lv_obj_center(label);
    return button;
}

static void scene_button_cb(lv_event_t *event);
static void manual_button_cb(lv_event_t *event);
static void nav_button_cb(lv_event_t *event);
static void ui_rebuild_page(void);
static void ui_update(void);

static lv_color_t page_bg_color(void)
{
    if (g_model.state == STATE_GAS_DANGER)
    {
        return lv_color_hex(0x23090d);
    }
    if (g_model.state == STATE_GAS_WARN || g_model.risk_score >= 60)
    {
        return lv_color_hex(0x171307);
    }
    return lv_color_hex(0x071013);
}

static lv_color_t accent_color(void)
{
    if (g_model.state == STATE_GAS_DANGER)
    {
        return lv_color_hex(0xff4651);
    }
    if (g_model.state == STATE_GAS_WARN || g_model.risk_score >= 60)
    {
        return lv_color_hex(0xffb32c);
    }
    return lv_color_hex(0x23c483);
}

static void apply_theme(void)
{
    lv_color_t bg = page_bg_color();
    lv_color_t accent = accent_color();

    if (g_ui.screen != RT_NULL)
    {
        lv_obj_set_style_bg_color(g_ui.screen, bg, 0);
    }
    if (g_ui.risk_bar != RT_NULL)
    {
        lv_obj_set_style_bg_color(g_ui.risk_bar, lv_color_hex(0x223037), LV_PART_MAIN);
        lv_obj_set_style_bg_color(g_ui.risk_bar, accent, LV_PART_INDICATOR);
    }
    if (g_ui.chart != RT_NULL)
    {
        lv_obj_set_style_bg_color(g_ui.chart, lv_color_hex(0x101d22), 0);
        lv_obj_set_style_border_color(g_ui.chart, accent, 0);
    }
    if (g_ui.risk_label != RT_NULL)
    {
        lv_obj_set_style_text_color(g_ui.risk_label, accent, 0);
    }
}

static void ui_reset_page_refs(void)
{
    int i;

    g_ui.state_label = RT_NULL;
    g_ui.mode_label = RT_NULL;
    g_ui.fan_label = RT_NULL;
    g_ui.risk_bar = RT_NULL;
    g_ui.risk_label = RT_NULL;
    g_ui.smoke_label = RT_NULL;
    g_ui.gas_label = RT_NULL;
    g_ui.temp_label = RT_NULL;
    g_ui.countdown_label = RT_NULL;
    g_ui.chart = RT_NULL;
    g_ui.smoke_series = RT_NULL;
    g_ui.gas_series = RT_NULL;
    g_ui.report_label = RT_NULL;
    g_ui.hardware_label = RT_NULL;
    for (i = 0; i < EVENT_LOG_LINES; i++)
    {
        g_ui.event_labels[i] = RT_NULL;
    }
}

static void create_metric_card(lv_obj_t *parent, const char *title, lv_obj_t **value_label, int x, int y, lv_color_t color)
{
    lv_obj_t *panel = make_panel(parent, x, y, 140, 96, color);
    lv_obj_t *title_label = make_label(panel, title, 0, 0, 120, 24, 14);
    style_label(title_label, lv_color_hex(0x9fb5b9), LV_TEXT_ALIGN_CENTER);
    *value_label = make_label(panel, "--", 0, 30, 120, 42, 26);
    style_label(*value_label, lv_color_hex(0xffffff), LV_TEXT_ALIGN_CENTER);
}

static void create_top_summary(lv_obj_t *parent, int y)
{
    lv_obj_t *panel = make_panel(parent, 12, y, 456, 132, lv_color_hex(0x101d22));

    make_label(panel, "Kitchen Risk", 8, 0, 100, 24, 15);
    g_ui.risk_label = make_label(panel, "0", 0, 26, 126, 64, 42);
    style_label(g_ui.risk_label, lv_color_hex(0x23c483), LV_TEXT_ALIGN_CENTER);

    g_ui.risk_bar = lv_bar_create(panel);
    lv_obj_set_pos(g_ui.risk_bar, 132, 30);
    lv_obj_set_size(g_ui.risk_bar, 292, 18);
    lv_obj_set_style_radius(g_ui.risk_bar, 9, LV_PART_MAIN);
    lv_obj_set_style_radius(g_ui.risk_bar, 9, LV_PART_INDICATOR);
    lv_bar_set_range(g_ui.risk_bar, 0, 100);

    make_label(panel, "State", 132, 62, 64, 20, 13);
    make_label(panel, "Mode", 236, 62, 64, 20, 13);
    make_label(panel, "Fan", 340, 62, 64, 20, 13);
    g_ui.state_label = make_label(panel, "Standby", 132, 84, 88, 28, 17);
    g_ui.mode_label = make_label(panel, "AUTO", 236, 84, 76, 28, 17);
    g_ui.fan_label = make_label(panel, "OFF", 340, 84, 76, 28, 17);
    style_label(g_ui.state_label, lv_color_hex(0xffffff), LV_TEXT_ALIGN_LEFT);
    style_label(g_ui.mode_label, lv_color_hex(0x8ee8d1), LV_TEXT_ALIGN_CENTER);
    style_label(g_ui.fan_label, lv_color_hex(0xffcf6a), LV_TEXT_ALIGN_CENTER);
}

static void ui_show_home(void)
{
    lv_obj_t *hint;
    lv_obj_t *panel;

    create_top_summary(g_ui.body, 8);
    create_metric_card(g_ui.body, "Smoke", &g_ui.smoke_label, 12, 156, lv_color_hex(0x111d24));
    create_metric_card(g_ui.body, "Gas", &g_ui.gas_label, 170, 156, lv_color_hex(0x241719));
    create_metric_card(g_ui.body, "Temp", &g_ui.temp_label, 328, 156, lv_color_hex(0x121b28));

    panel = make_panel(g_ui.body, 12, 270, 456, 104, lv_color_hex(0x0d181d));
    make_label(panel, "Quick Control", 8, 0, 100, 22, 15);
    make_button(panel, "AUTO", 12, 34, 76, 38, manual_button_cb, "auto");
    make_button(panel, "ON", 100, 34, 76, 38, manual_button_cb, "on");
    make_button(panel, "OFF", 188, 34, 76, 38, manual_button_cb, "off");
    make_button(panel, "UP", 276, 34, 76, 38, manual_button_cb, "up");
    make_button(panel, "DOWN", 364, 34, 76, 38, manual_button_cb, "down");

    panel = make_panel(g_ui.body, 12, 390, 456, 116, lv_color_hex(0x101d22));
    make_label(panel, "Current Scene", 8, 0, 100, 22, 15);
    g_ui.report_label = make_label(panel, "Clean", 8, 32, 190, 34, 24);
    style_label(g_ui.report_label, lv_color_hex(0xffffff), LV_TEXT_ALIGN_LEFT);
    hint = make_label(panel, "Use bottom menu: Monitor / Scene / Safety / Report", 8, 74, 420, 24, 14);
    style_label(hint, lv_color_hex(0x9fb5b9), LV_TEXT_ALIGN_LEFT);

    g_ui.hardware_label = make_label(g_ui.body, "Board key: mute toggle", 12, 526, 456, 28, 14);
    style_label(g_ui.hardware_label, lv_color_hex(0x8ee8d1), LV_TEXT_ALIGN_CENTER);
}

static void ui_show_dashboard(void)
{
    lv_obj_t *chart_panel;

    create_top_summary(g_ui.body, 8);
    create_metric_card(g_ui.body, "Smoke", &g_ui.smoke_label, 12, 156, lv_color_hex(0x111d24));
    create_metric_card(g_ui.body, "Gas", &g_ui.gas_label, 170, 156, lv_color_hex(0x241719));
    create_metric_card(g_ui.body, "Temp", &g_ui.temp_label, 328, 156, lv_color_hex(0x121b28));

    chart_panel = make_panel(g_ui.body, 12, 272, 456, 214, lv_color_hex(0x101d22));
    make_label(chart_panel, "Trend", 8, 0, 120, 22, 15);
    make_label(chart_panel, "Yellow: Smoke  Red: Gas", 260, 0, 160, 22, 13);
    g_ui.chart = lv_chart_create(chart_panel);
    lv_obj_set_pos(g_ui.chart, 12, 34);
    lv_obj_set_size(g_ui.chart, 408, 126);
    lv_obj_set_style_radius(g_ui.chart, 6, 0);
    lv_obj_set_style_border_width(g_ui.chart, 1, 0);
    lv_chart_set_type(g_ui.chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(g_ui.chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_point_count(g_ui.chart, HISTORY_POINTS);
    g_ui.smoke_series = lv_chart_add_series(g_ui.chart, lv_color_hex(0xffcf6a), LV_CHART_AXIS_PRIMARY_Y);
    g_ui.gas_series = lv_chart_add_series(g_ui.chart, lv_color_hex(0xff6b6b), LV_CHART_AXIS_PRIMARY_Y);

    g_ui.hardware_label = make_label(chart_panel, "Sim data, can map to UART/I2C/ADC sensors later", 12, 170, 408, 24, 13);
    style_label(g_ui.hardware_label, lv_color_hex(0x9fb5b9), LV_TEXT_ALIGN_CENTER);
}

static void ui_show_scene(void)
{
    lv_obj_t *panel;

    panel = make_panel(g_ui.body, 12, 10, 456, 86, lv_color_hex(0x101d22));
    make_label(panel, "Scene Simulator", 8, 4, 120, 24, 18);
    g_ui.report_label = make_label(panel, "Choose a kitchen scene to generate smoke/gas/temp curves", 8, 36, 410, 34, 14);
    style_label(g_ui.report_label, lv_color_hex(0x9fb5b9), LV_TEXT_ALIGN_LEFT);

    panel = make_panel(g_ui.body, 12, 116, 456, 230, lv_color_hex(0x0d181d));
    make_button(panel, "Clean", 18, 24, 124, 50, scene_button_cb, (void *)(uintptr_t)SCENE_IDLE);
    make_button(panel, "Soup", 166, 24, 124, 50, scene_button_cb, (void *)(uintptr_t)SCENE_SOUP);
    make_button(panel, "Stir", 314, 24, 104, 50, scene_button_cb, (void *)(uintptr_t)SCENE_STIR_FRY);
    make_button(panel, "Fry", 18, 104, 124, 50, scene_button_cb, (void *)(uintptr_t)SCENE_DEEP_FRY);
    make_button(panel, "Gas", 166, 104, 124, 50, scene_button_cb, (void *)(uintptr_t)SCENE_GAS_LEAK);
    make_button(panel, "Dry", 314, 104, 104, 50, scene_button_cb, (void *)(uintptr_t)SCENE_DRY_BURN);
    g_ui.hardware_label = make_label(panel, "Current: Clean", 18, 178, 400, 26, 17);
    style_label(g_ui.hardware_label, lv_color_hex(0x8ee8d1), LV_TEXT_ALIGN_CENTER);

    create_metric_card(g_ui.body, "Smoke", &g_ui.smoke_label, 12, 370, lv_color_hex(0x111d24));
    create_metric_card(g_ui.body, "Gas", &g_ui.gas_label, 170, 370, lv_color_hex(0x241719));
    create_metric_card(g_ui.body, "Temp", &g_ui.temp_label, 328, 370, lv_color_hex(0x121b28));
}

static void ui_show_safety(void)
{
    lv_obj_t *panel;

    create_top_summary(g_ui.body, 8);
    panel = make_panel(g_ui.body, 12, 158, 456, 134, lv_color_hex(0x241719));
    make_label(panel, "GasSafe", 8, 0, 120, 24, 17);
    g_ui.gas_label = make_label(panel, "Gas 0%", 8, 34, 170, 36, 24);
    style_label(g_ui.gas_label, lv_color_hex(0xff6b6b), LV_TEXT_ALIGN_LEFT);
    g_ui.hardware_label = make_label(panel, "Danger forces HIGH; OFF/DOWN blocked", 8, 82, 408, 34, 14);
    style_label(g_ui.hardware_label, lv_color_hex(0xffcf6a), LV_TEXT_ALIGN_LEFT);

    panel = make_panel(g_ui.body, 12, 314, 456, 98, lv_color_hex(0x101d22));
    make_label(panel, "Clear Condition", 8, 0, 120, 24, 17);
    g_ui.countdown_label = make_label(panel, "Safe", 8, 34, 408, 34, 22);
    style_label(g_ui.countdown_label, lv_color_hex(0x8ee8d1), LV_TEXT_ALIGN_CENTER);

    panel = make_panel(g_ui.body, 12, 434, 456, 92, lv_color_hex(0x0d181d));
    make_label(panel, "Sound", 8, 0, 80, 24, 17);
    g_ui.report_label = make_label(panel, "Sound ON", 8, 34, 408, 28, 18);
    style_label(g_ui.report_label, lv_color_hex(0xffffff), LV_TEXT_ALIGN_CENTER);
}

static void ui_show_report(void)
{
    int i;
    lv_obj_t *panel;

    panel = make_panel(g_ui.body, 12, 10, 456, 128, lv_color_hex(0x101d22));
    make_label(panel, "Run Report", 8, 0, 120, 24, 18);
    g_ui.report_label = make_label(panel, "Exhaust 0s  Alarms 0  Peak 0", 8, 36, 408, 34, 20);
    style_label(g_ui.report_label, lv_color_hex(0xffffff), LV_TEXT_ALIGN_CENTER);
    g_ui.hardware_label = make_label(panel, "Shows closed-loop logic for demo", 8, 82, 408, 24, 14);
    style_label(g_ui.hardware_label, lv_color_hex(0x9fb5b9), LV_TEXT_ALIGN_CENTER);

    panel = make_panel(g_ui.body, 12, 160, 456, 260, lv_color_hex(0x0d181d));
    make_label(panel, "Recent Events", 8, 0, 120, 24, 17);
    for (i = 0; i < EVENT_LOG_LINES; i++)
    {
        g_ui.event_labels[i] = make_label(panel, "-", 10, 34 + i * 38, 412, 30, 14);
        style_label(g_ui.event_labels[i], lv_color_hex(0xd8e8ea), LV_TEXT_ALIGN_LEFT);
    }

    create_metric_card(g_ui.body, "Smoke", &g_ui.smoke_label, 12, 444, lv_color_hex(0x111d24));
    create_metric_card(g_ui.body, "Gas", &g_ui.gas_label, 170, 444, lv_color_hex(0x241719));
    create_metric_card(g_ui.body, "Temp", &g_ui.temp_label, 328, 444, lv_color_hex(0x121b28));
}

static void ui_rebuild_page(void)
{
    if (g_ui.body == RT_NULL)
    {
        return;
    }

    lv_obj_clean(g_ui.body);
    ui_reset_page_refs();

    switch (g_ui.page)
    {
    case UI_PAGE_DASHBOARD:
        ui_show_dashboard();
        break;
    case UI_PAGE_SCENE:
        ui_show_scene();
        break;
    case UI_PAGE_SAFETY:
        ui_show_safety();
        break;
    case UI_PAGE_REPORT:
        ui_show_report();
        break;
    default:
        ui_show_home();
        break;
    }
}

static void ui_update(void)
{
    int i;

    apply_theme();

    if (g_ui.title_label != RT_NULL)
    {
        lv_label_set_text_fmt(g_ui.title_label, "SMART HOOD  |  %s", scene_name_cn(g_model.scene));
    }
    if (g_ui.countdown_label != RT_NULL)
    {
        if (g_model.state == STATE_PURGE_DELAY)
        {
            lv_label_set_text_fmt(g_ui.countdown_label, "Purge %d s", g_model.purge_countdown);
        }
        else if (g_model.danger_latched)
        {
            lv_label_set_text_fmt(g_ui.countdown_label, "Gas clear %d s", g_model.gas_clear_countdown);
        }
        else
        {
            lv_label_set_text(g_ui.countdown_label, "SafeStandby");
        }
    }
    if (g_ui.state_label != RT_NULL)
    {
        lv_label_set_text(g_ui.state_label, state_name_cn(g_model.state));
    }
    if (g_ui.mode_label != RT_NULL)
    {
        lv_label_set_text(g_ui.mode_label, mode_name_cn(g_model.mode));
    }
    if (g_ui.fan_label != RT_NULL)
    {
        lv_label_set_text(g_ui.fan_label, fan_name_cn(g_model.fan));
    }
    if (g_ui.risk_label != RT_NULL)
    {
        lv_label_set_text_fmt(g_ui.risk_label, "%d", g_model.risk_score);
    }
    if (g_ui.risk_bar != RT_NULL)
    {
        lv_bar_set_value(g_ui.risk_bar, g_model.risk_score, LV_ANIM_ON);
    }
    if (g_ui.smoke_label != RT_NULL)
    {
        lv_label_set_text_fmt(g_ui.smoke_label, "%d%%", g_model.sample.smoke);
    }
    if (g_ui.gas_label != RT_NULL)
    {
        lv_label_set_text_fmt(g_ui.gas_label, "%d%%", g_model.sample.gas);
    }
    if (g_ui.temp_label != RT_NULL)
    {
        lv_label_set_text_fmt(g_ui.temp_label, "%d C", g_model.sample.temperature);
    }
    if (g_ui.chart != RT_NULL && g_ui.smoke_series != RT_NULL && g_ui.gas_series != RT_NULL)
    {
        lv_chart_set_next_value(g_ui.chart, g_ui.smoke_series, g_model.sample.smoke);
        lv_chart_set_next_value(g_ui.chart, g_ui.gas_series, g_model.sample.gas);
    }

    for (i = 0; i < EVENT_LOG_LINES; i++)
    {
        if (g_ui.event_labels[i] != RT_NULL)
        {
            lv_label_set_text(g_ui.event_labels[i], event_text_cn(g_events[i]));
        }
    }

    if (g_ui.report_label != RT_NULL)
    {
        if (g_ui.page == UI_PAGE_REPORT)
        {
            lv_label_set_text_fmt(g_ui.report_label,
                                  "Exhaust %d s  Alarms %d  Peak %d",
                                  g_model.exhaust_seconds,
                                  g_model.alarm_count,
                                  g_model.max_risk_score);
        }
        else if (g_ui.page == UI_PAGE_HOME || g_ui.page == UI_PAGE_SCENE)
        {
            lv_label_set_text_fmt(g_ui.report_label, "%s  Risk: %s", scene_name_cn(g_model.scene), risk_level_cn());
        }
        else if (g_ui.page == UI_PAGE_SAFETY)
        {
            lv_label_set_text(g_ui.report_label, g_model.alarm_muted ? "Muted" : "Sound ON");
        }
    }

    if (g_ui.hardware_label != RT_NULL)
    {
        if (g_ui.page == UI_PAGE_SAFETY)
        {
            lv_label_set_text(g_ui.hardware_label,
                              g_model.state == STATE_GAS_DANGER ? "Danger locked: forced HIGH, OFF blocked" : "Gas safe: manual control allowed");
        }
        else if (g_ui.page == UI_PAGE_SCENE)
        {
            lv_label_set_text_fmt(g_ui.hardware_label, "Current: %s", scene_name_cn(g_model.scene));
        }
        else
        {
            lv_label_set_text_fmt(g_ui.hardware_label,
                                  "Board LED: %s  Sound: %s",
                                  g_model.state == STATE_GAS_DANGER ? "Blue blink" :
                                  (g_model.state == STATE_GAS_WARN || g_model.risk_score >= 60) ? "Blue warn" : "Green safe",
                                  g_model.alarm_muted ? "Muted" : "On");
        }
    }
}

static void nav_button_cb(lv_event_t *event)
{
    g_ui.page = (hood_ui_page_t)(uintptr_t)lv_event_get_user_data(event);
    ui_rebuild_page();
    ui_update();
}

static void ui_create(void)
{
    lv_obj_t *nav;

    g_ui.screen = lv_obj_create(NULL);
    lv_obj_clear_flag(g_ui.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_ui.screen, lv_color_hex(0x071013), 0);
    lv_obj_set_style_pad_all(g_ui.screen, 0, 0);

    g_ui.title_label = make_label(g_ui.screen, "SMART HOOD", 12, 14, 330, 28, 22);
    lv_obj_set_style_text_color(g_ui.title_label, lv_color_hex(0xffffff), 0);

    g_ui.body = lv_obj_create(g_ui.screen);
    lv_obj_clear_flag(g_ui.body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(g_ui.body, 0, 52);
    lv_obj_set_size(g_ui.body, 480, 638);
    lv_obj_set_style_bg_opa(g_ui.body, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.body, 0, 0);
    lv_obj_set_style_pad_all(g_ui.body, 0, 0);

    nav = make_panel(g_ui.screen, 12, 704, 456, 72, lv_color_hex(0x0d181d));
    make_button(nav, "Home", 10, 18, 78, 36, nav_button_cb, (void *)(uintptr_t)UI_PAGE_HOME);
    make_button(nav, "Monitor", 96, 18, 78, 36, nav_button_cb, (void *)(uintptr_t)UI_PAGE_DASHBOARD);
    make_button(nav, "Scene", 182, 18, 78, 36, nav_button_cb, (void *)(uintptr_t)UI_PAGE_SCENE);
    make_button(nav, "Safe", 268, 18, 78, 36, nav_button_cb, (void *)(uintptr_t)UI_PAGE_SAFETY);
    make_button(nav, "Report", 354, 18, 78, 36, nav_button_cb, (void *)(uintptr_t)UI_PAGE_REPORT);

    g_ui.page = UI_PAGE_HOME;
    ui_rebuild_page();
    lv_screen_load(g_ui.screen);
    ui_update();
}

static void scene_button_cb(lv_event_t *event)
{
    hood_scene_t scene = (hood_scene_t)(uintptr_t)lv_event_get_user_data(event);

    g_model.scene = scene;
    g_model.scene_tick = 0;
    event_log_push(scene_name(scene));
    ui_update();
}

static void manual_button_cb(lv_event_t *event)
{
    const char *command = (const char *)lv_event_get_user_data(event);

    if (g_model.state == STATE_GAS_DANGER)
    {
        event_log_push("Manual command blocked by gas danger");
        voice_prompt("Manual command blocked during gas danger.");
        ui_update();
        return;
    }

    if (rt_strcmp(command, "auto") == 0)
    {
        g_model.mode = MODE_AUTO;
        event_log_push("Auto mode enabled");
        ui_update();
        return;
    }

    g_model.mode = MODE_MANUAL;

    if (rt_strcmp(command, "on") == 0)
    {
        set_fan(FAN_LOW, "Manual fan ON");
    }
    else if (rt_strcmp(command, "off") == 0)
    {
        set_fan(FAN_OFF, "Manual fan OFF");
    }
    else if (rt_strcmp(command, "up") == 0)
    {
        set_fan(g_model.fan == FAN_OFF ? FAN_LOW : FAN_HIGH, "Manual speed UP");
    }
    else if (rt_strcmp(command, "down") == 0)
    {
        set_fan(g_model.fan == FAN_HIGH ? FAN_LOW : FAN_OFF, "Manual speed DOWN");
    }

    ui_update();
}

static void hood_copy_command(char *dst, rt_size_t dst_size, const char *src)
{
    rt_size_t i;

    if (dst_size == 0U)
    {
        return;
    }

    for (i = 0; i < dst_size - 1U && src[i] != '\0'; i++)
    {
        char ch = src[i];
        if (ch >= 'A' && ch <= 'Z')
        {
            ch = (char)(ch - 'A' + 'a');
        }
        dst[i] = ch;
    }
    dst[i] = '\0';
}

static void hood_voice_set_scene(hood_scene_t scene)
{
    g_model.scene = scene;
    g_model.scene_tick = 0;
    g_model.mode = MODE_AUTO;
    event_log_push(scene_name(scene));
    voice_prompt("Smoke rising, low exhaust enabled.");
}

static void hood_voice_status(void)
{
    event_log_push("Voice status query");

    if (g_model.state == STATE_GAS_DANGER)
    {
        voice_prompt("Gas danger detected. Forced exhaust is active.");
    }
    else if (g_model.fan == FAN_HIGH)
    {
        voice_prompt("High exhaust enabled.");
    }
    else if (g_model.fan == FAN_LOW)
    {
        voice_prompt("Smoke rising, low exhaust enabled.");
    }
    else
    {
        voice_prompt("Fan stopped, system standby.");
    }
}

static rt_err_t hood_voice_command_apply(const char *raw_command)
{
    char command[24];

    if (raw_command == RT_NULL || raw_command[0] == '\0')
    {
        return -RT_EINVAL;
    }

    hood_copy_command(command, sizeof(command), raw_command);
    rt_kprintf("[%s] ASR command: %s\n", APP_TAG, command);

    if (rt_strcmp(command, "status") == 0)
    {
        hood_voice_status();
        return RT_EOK;
    }

    if (rt_strcmp(command, "clean") == 0 || rt_strcmp(command, "stopscene") == 0)
    {
        hood_voice_set_scene(SCENE_IDLE);
        return RT_EOK;
    }
    if (rt_strcmp(command, "soup") == 0)
    {
        hood_voice_set_scene(SCENE_SOUP);
        return RT_EOK;
    }
    if (rt_strcmp(command, "stir") == 0 || rt_strcmp(command, "stirfry") == 0)
    {
        hood_voice_set_scene(SCENE_STIR_FRY);
        return RT_EOK;
    }
    if (rt_strcmp(command, "fry") == 0 || rt_strcmp(command, "deepfry") == 0)
    {
        hood_voice_set_scene(SCENE_DEEP_FRY);
        return RT_EOK;
    }
    if (rt_strcmp(command, "gas") == 0)
    {
        hood_voice_set_scene(SCENE_GAS_LEAK);
        return RT_EOK;
    }
    if (rt_strcmp(command, "dry") == 0 || rt_strcmp(command, "dryburn") == 0)
    {
        hood_voice_set_scene(SCENE_DRY_BURN);
        return RT_EOK;
    }

    if (g_model.state == STATE_GAS_DANGER &&
        (rt_strcmp(command, "off") == 0 || rt_strcmp(command, "down") == 0 ||
         rt_strcmp(command, "low") == 0))
    {
        event_log_push("Voice command blocked by gas danger");
        voice_prompt("Manual command blocked during gas danger.");
        return -RT_EBUSY;
    }

    if (rt_strcmp(command, "auto") == 0)
    {
        g_model.mode = MODE_AUTO;
        event_log_push("Voice: auto mode");
        return RT_EOK;
    }

    g_model.mode = MODE_MANUAL;

    if (rt_strcmp(command, "on") == 0 || rt_strcmp(command, "low") == 0)
    {
        set_fan(FAN_LOW, "Voice fan LOW");
    }
    else if (rt_strcmp(command, "off") == 0)
    {
        set_fan(FAN_OFF, "Voice fan OFF");
    }
    else if (rt_strcmp(command, "up") == 0)
    {
        set_fan(g_model.fan == FAN_OFF ? FAN_LOW : FAN_HIGH, "Voice speed UP");
    }
    else if (rt_strcmp(command, "down") == 0)
    {
        set_fan(g_model.fan == FAN_HIGH ? FAN_LOW : FAN_OFF, "Voice speed DOWN");
    }
    else if (rt_strcmp(command, "high") == 0)
    {
        set_fan(FAN_HIGH, "Voice fan HIGH");
    }
    else
    {
        event_log_push("Unknown voice command");
        rt_kprintf("[%s] voice commands: auto on off up down high low clean soup stir fry gas dry status\n", APP_TAG);
        return -RT_ERROR;
    }

    return RT_EOK;
}

void hood_asr_command_detected(const char *command)
{
    if (hood_voice_command_apply(command) == RT_EOK)
    {
        ui_update();
    }
}

static int hood_voice(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: hood_voice <auto|on|off|up|down|high|low|clean|soup|stir|fry|gas|dry|status>\n");
        return -RT_EINVAL;
    }

    hood_asr_command_detected(argv[1]);
    return RT_EOK;
}
MSH_CMD_EXPORT(hood_voice, simulate smart hood ASR voice command);


static rt_bool_t hood_text_has(const char *text, const char *keyword)
{
    return (text != RT_NULL && keyword != RT_NULL && strstr(text, keyword) != RT_NULL) ? RT_TRUE : RT_FALSE;
}

static const char *hood_asr_text_to_command(const char *text)
{
    if (text == RT_NULL || text[0] == '\0')
    {
        return RT_NULL;
    }

    if (hood_text_has(text, "ȼ��") || hood_text_has(text, "ú��") || hood_text_has(text, "й©") ||
        hood_text_has(text, "gas leak") || hood_text_has(text, "gas"))
    {
        return "gas";
    }
    if (hood_text_has(text, "����") || hood_text_has(text, "dry burn") || hood_text_has(text, "dry"))
    {
        return "dry";
    }
    if (hood_text_has(text, "����") || hood_text_has(text, "���̴�") || hood_text_has(text, "deep fry") ||
        hood_text_has(text, "fry"))
    {
        return "fry";
    }
    if (hood_text_has(text, "����") || hood_text_has(text, "�峴") || hood_text_has(text, "stir"))
    {
        return "stir";
    }
    if (hood_text_has(text, "����") || hood_text_has(text, "��") || hood_text_has(text, "soup"))
    {
        return "soup";
    }
    if (hood_text_has(text, "�ָ�") || hood_text_has(text, "���") || hood_text_has(text, "����") ||
        hood_text_has(text, "clean"))
    {
        return "clean";
    }
    if (hood_text_has(text, "״̬") || hood_text_has(text, "��ǰ") || hood_text_has(text, "����") ||
        hood_text_has(text, "status"))
    {
        return "status";
    }
    if (hood_text_has(text, "�Զ�") || hood_text_has(text, "auto"))
    {
        return "auto";
    }
    if (hood_text_has(text, "�ߵ�") || hood_text_has(text, "ǿ��") || hood_text_has(text, "���") ||
        hood_text_has(text, "����") || hood_text_has(text, "high"))
    {
        return "high";
    }
    if (hood_text_has(text, "�͵�") || hood_text_has(text, "С��") || hood_text_has(text, "low"))
    {
        return "low";
    }
    if (hood_text_has(text, "����") || hood_text_has(text, "�Ӵ�") || hood_text_has(text, "��һ��") ||
        hood_text_has(text, "up"))
    {
        return "up";
    }
    if (hood_text_has(text, "����") || hood_text_has(text, "����") || hood_text_has(text, "Сһ��") ||
        hood_text_has(text, "down"))
    {
        return "down";
    }
    if (hood_text_has(text, "�ر�") || hood_text_has(text, "�ػ�") || hood_text_has(text, "ֹͣ") ||
        hood_text_has(text, "off"))
    {
        return "off";
    }
    if (hood_text_has(text, "��") || hood_text_has(text, "����") || hood_text_has(text, "���") ||
        hood_text_has(text, "����") || hood_text_has(text, "on"))
    {
        return "on";
    }

    return RT_NULL;
}

void hood_asr_text_received(const char *text)
{
    const char *command;

    if (text == RT_NULL)
    {
        return;
    }

    rt_kprintf("[%s] ASR text: %s\n", APP_TAG, text);
    command = hood_asr_text_to_command(text);
    if (command == RT_NULL)
    {
        event_log_push("ASR text not matched");
        rt_kprintf("[%s] ASR text not matched\n", APP_TAG);
        return;
    }

    hood_asr_command_detected(command);
}

static int hood_asr(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: hood_asr <recognized text>\n");
        return -RT_EINVAL;
    }

    hood_asr_text_received(argv[1]);
    return RT_EOK;
}
MSH_CMD_EXPORT(hood_asr, simulate cloud ASR recognized text);

static void model_tick_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    handle_key_event();
    update_sensor_simulator();
    update_control_fsm();
    hardware_set_leds();
    ui_update();
}

static void model_init(void)
{
    rt_memset(&g_model, 0, sizeof(g_model));
    rt_memset(g_events, 0, sizeof(g_events));

    g_model.scene = SCENE_IDLE;
    g_model.mode = MODE_AUTO;
    g_model.state = STATE_STANDBY;
    g_model.fan = FAN_OFF;
    g_model.sample.smoke = 5;
    g_model.sample.gas = 3;
    g_model.sample.temperature = 28;
    g_model.purge_countdown = AUTO_OFF_DELAY_TICKS;
    g_model.gas_clear_countdown = GAS_CLEAR_DELAY_TICKS;
    event_log_push("System boot");
}

void smart_hood_demo_start(void)
{
    static rt_bool_t started = RT_FALSE;

    if (started)
    {
        return;
    }

    started = RT_TRUE;
    hood_audio_ipc_init();
    model_init();
    hardware_init();
    ui_create();
    lv_timer_create(model_tick_cb, 1000, RT_NULL);
    rt_kprintf("[%s] demo started\n", APP_TAG);
}





