/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2021-10-17     Meco Man      First version
 */
#include <rtthread.h>
#include <lvgl.h>
//#include <lv_port_indev.h>
#define DBG_TAG    "LVGL.demo"
#define DBG_LVL    DBG_LOG
#include <rtdbg.h>
#include "lv_demo_benchmark.h"

#ifdef PKG_USING_LV_MUSIC_DEMO

#ifndef LV_THREAD_STACK_SIZE
    #define LV_THREAD_STACK_SIZE 4096
#endif

#ifndef LV_THREAD_PRIO
    #define LV_THREAD_PRIO (RT_THREAD_PRIORITY_MAX * 2 / 8)
#endif

//static struct rt_thread lvgl_thread;
//static rt_uint8_t lvgl_thread_stack[LV_THREAD_STACK_SIZE];

static void lvgl_entry(void *parameter)
{
    LOG_I("3333333\n");
    
    extern void lv_demo_benchmark(void);
    lv_demo_benchmark();

    while(1)
    {
        //LOG_I("111111111\n");
        lv_task_handler();
        rt_thread_mdelay(5);
    }
}

static int lvgl_demo_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("LVGL",
                    lvgl_entry,
                    RT_NULL,
                    LV_THREAD_STACK_SIZE,
                    LV_THREAD_PRIO,
                    10);
    rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(lvgl_demo_init);

#endif