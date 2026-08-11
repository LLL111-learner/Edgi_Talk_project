#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <lv_rt_thread_conf.h>
#include "vg_lite.h"
#include "vg_lite_platform.h"
#include "lv_port_disp.h"

#define LED_PIN_G               GET_PIN(16, 6)

void lv_user_gui_init(void)
{
    extern void smart_hood_demo_start(void);
    smart_hood_demo_start();
}

int main(void)
{
    rt_kprintf("Hello RT-Thread\n");
    rt_kprintf("It's cortex-m55\n");
    lvgl_thread_init();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
