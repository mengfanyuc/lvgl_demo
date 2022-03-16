/*
 * @Description: 
 * @Version: 1.0
 * @Autor: mengfanyu
 * @Date: 2022-02-7 20:55:18
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef PKG_USING_LVGL
#include "lvgl/lv_port_indev.h"
#endif

#ifdef RT_USING_TOUCH

#define DBG_TAG "gt9157"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "touch.h"

#define BSP_TOUCH_RST_PIN       GET_PIN(D, 13)
#define BSP_TOUCH_INT_PIN       GET_PIN(D, 12)

static struct rt_i2c_client *gt9157_client;

#define GTP_ADDR_LENGTH        (2)
#define GT9157_MAX_TOUCH       (5)
#define GT9157_POINT_INFO_NUM  (8)

#define GT9157_ADDRESS_HIGH    (0x14)
#define GT9157_ADDRESS_LOW     (0x5D)

#define GT9157_CONFIG          (0x8047)

#define GT9XX_PRODUCT_ID       (0x8140)
#define GT9157_READ_STATUS     (0x814E)

#define GT9157_POINT_REG       (0x814F)

#define GT9157_CHECK_SUM       (0X80FF)


static const rt_uint8_t GT9157_CFG_TBL[] =
{
    0x00,0x20,0x03,0xE0,0x01,0x05,0x3C,0x00,0x01,0x08,
	0x28,0x0C,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x17,0x19,0x1E,0x14,0x8B,0x2B,0x0D,
	0x33,0x35,0x0C,0x08,0x00,0x00,0x00,0x9A,0x03,0x11,
	0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,
	0x00,0x20,0x58,0x94,0xC5,0x02,0x00,0x00,0x00,0x04,
	0xB0,0x23,0x00,0x93,0x2B,0x00,0x7B,0x35,0x00,0x69,
	0x41,0x00,0x5B,0x4F,0x00,0x5B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,
	0x12,0x14,0x16,0x18,0x1A,0xFF,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0F,
	0x10,0x12,0x13,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,
	0x21,0x22,0x24,0x26,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,
};

static rt_err_t gt9157_write_reg(struct rt_i2c_client *dev, rt_uint8_t write_len, rt_uint8_t *write_data)
{
    struct rt_i2c_msg msgs;

    msgs.addr  = dev->client_addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf   = write_data;
    msgs.len   = write_len;

    if (rt_i2c_transfer(dev->bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static rt_err_t gt9157_read_regs(struct rt_i2c_client *dev, rt_uint8_t *cmd_buf, rt_uint8_t cmd_len, rt_uint8_t read_len, rt_uint8_t *read_buf)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = dev->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = cmd_buf;
    msgs[0].len   = cmd_len;

    msgs[1].addr  = dev->client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = read_buf;
    msgs[1].len   = read_len;

    if (rt_i2c_transfer(dev->bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static rt_err_t gt9157_get_product_id(struct rt_i2c_client *dev, rt_uint8_t read_len, rt_uint8_t *read_data)
{
    rt_uint8_t cmd_buf[2];

    cmd_buf[0] = (rt_uint8_t)(GT9XX_PRODUCT_ID >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT9XX_PRODUCT_ID & 0xFF);

    if (gt9157_read_regs(dev, cmd_buf, 2, read_len, read_data) != RT_EOK)
    {
        LOG_D("read id failed \n");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t gt9157_get_info(struct rt_i2c_client *dev, struct rt_touch_info *info)
{
    rt_uint8_t opr_buf[7];
    rt_uint8_t cmd_buf[2];

    cmd_buf[0] = (rt_uint8_t)(GT9157_CONFIG >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT9157_CONFIG & 0xFF);

    if (gt9157_read_regs(dev, cmd_buf, 2, 7, opr_buf) != RT_EOK)
    {
        LOG_D("read id failed \n");
        return -RT_ERROR;
    }

    info->range_x = (opr_buf[2] << 8) + opr_buf[1];
    info->range_y = (opr_buf[4] << 8) + opr_buf[3];
    info->point_num = opr_buf[5] & 0x0F;

    return RT_EOK;
}

static rt_err_t gt9157_control(struct rt_touch_device *device, int cmd, void *data)
{
    if (cmd == RT_TOUCH_CTRL_GET_ID)
    {
        return gt9157_get_product_id(gt9157_client, 6, data);
    }

    if (cmd == RT_TOUCH_CTRL_GET_INFO)
    {
        return gt9157_get_info(gt9157_client, data);
    }

    rt_uint8_t buf[4];
    rt_uint8_t i = 0;
    rt_uint8_t *cmd_buf;
    rt_uint8_t *opr_buf;

    cmd_buf = (rt_uint8_t *)rt_calloc(1, sizeof(GT9157_CFG_TBL) + GTP_ADDR_LENGTH);
    RT_ASSERT(cmd_buf != RT_NULL);
    
    opr_buf = (rt_uint8_t *)rt_calloc(1, sizeof(GT9157_CFG_TBL));
    RT_ASSERT(opr_buf != RT_NULL);

    cmd_buf[0] = (rt_uint8_t)(GT9157_CONFIG >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT9157_CONFIG & 0xFF);

    if(cmd != RT_TOUCH_CTRL_LOAD_CONFIG)
    {
       if (gt9157_read_regs(gt9157_client, cmd_buf, 2, sizeof(GT9157_CFG_TBL), opr_buf) != RT_EOK)
        {
            LOG_D("read regs failed \n");
            return -RT_ERROR;
        }
        rt_memcpy(&cmd_buf[2], opr_buf, sizeof(GT9157_CFG_TBL));
    }

    switch(cmd)
    {
        case RT_TOUCH_CTRL_SET_X_RANGE: 
        {
            rt_uint16_t x_ran;

            x_ran = *(rt_uint16_t *)data;
            cmd_buf[4] = (rt_uint8_t)(x_ran >> 8);
            cmd_buf[3] = (rt_uint8_t)(x_ran & 0xFF);

            break;
        }
        case RT_TOUCH_CTRL_SET_Y_RANGE: 
        {
            rt_uint16_t y_ran;

            y_ran = *(rt_uint16_t *)data;
            cmd_buf[6] = (rt_uint8_t)(y_ran >> 8);
            cmd_buf[5] = (rt_uint8_t)(y_ran & 0xFF);

            break;
        }
        case RT_TOUCH_CTRL_SET_X_TO_Y: 
        {
            cmd_buf[8] = cmd_buf[8] ^= (1 << 3);

            break;
        }
        case RT_TOUCH_CTRL_SET_MODE: 
        {
            rt_uint16_t trig_type;
            trig_type = *(rt_uint16_t *)data;

            switch (trig_type)
            {
                case RT_DEVICE_FLAG_INT_RX:
                    cmd_buf[8] &= 0xFC;
                    break;
                case RT_DEVICE_FLAG_RDONLY:
                    cmd_buf[8] &= 0xFC;
                    cmd_buf[8] |= 0x02;
                    break;
                default:
                    break;
            }
            break;
        }
        case RT_TOUCH_CTRL_LOAD_CONFIG:
        {
            rt_memcpy(&cmd_buf[2], GT9157_CFG_TBL, sizeof(GT9157_CFG_TBL));
            break;
        }
        default:
            break;
    }

    if (gt9157_write_reg(gt9157_client, sizeof(GT9157_CFG_TBL) + GTP_ADDR_LENGTH, cmd_buf) != RT_EOK)  
    {
        LOG_D("send config failed\n");
        return -RT_ERROR;
    }

    buf[0] = (rt_uint8_t)(GT9157_CHECK_SUM >> 8);
    buf[1] = (rt_uint8_t)(GT9157_CHECK_SUM & 0xFF);
    buf[2] = 0;

    for(i = GTP_ADDR_LENGTH; i < sizeof(GT9157_CFG_TBL) + GTP_ADDR_LENGTH; i++)
        buf[GTP_ADDR_LENGTH] += cmd_buf[i];

    buf[2] = (~buf[2]) + 1;
    buf[3] = 1;

    if (gt9157_write_reg(gt9157_client, 4, buf) != RT_EOK)
    {
        LOG_D("send check sum failed\n");
        return -RT_ERROR;
    }
    rt_free(cmd_buf);
    rt_free(opr_buf);

    return RT_EOK;
}

static rt_int16_t pre_x[GT9157_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static rt_int16_t pre_y[GT9157_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static rt_int16_t pre_w[GT9157_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static rt_uint8_t s_tp_dowm[GT9157_MAX_TOUCH];
static struct rt_touch_data *read_data;

static void gt9157_touch_up(void *buf, rt_int8_t id)
{
    read_data = (struct rt_touch_data *)buf;

    if(s_tp_dowm[id] == 1)
    {
        s_tp_dowm[id] = 0;
        read_data[id].event = RT_TOUCH_EVENT_UP;
    }
    else
    {
        read_data[id].event = RT_TOUCH_EVENT_NONE;
    }

    read_data[id].timestamp = rt_touch_get_ts();
    read_data[id].width = pre_w[id];
    read_data[id].x_coordinate = pre_x[id];
    read_data[id].y_coordinate = pre_y[id];
    read_data[id].track_id = id;
#ifdef PKG_USING_LVGL
    lv_port_indev_input(read_data[id].x_coordinate, read_data[id].y_coordinate, LV_INDEV_STATE_REL);
#endif
    pre_x[id] = -1;  
    pre_y[id] = -1;
    pre_w[id] = -1;
}

static void gt9157_touch_down(void *buf, rt_int8_t id, rt_int16_t x, rt_int16_t y, rt_int16_t w)
{
    read_data = (struct rt_touch_data *)buf;

    if (s_tp_dowm[id] == 1)
    {
        read_data[id].event = RT_TOUCH_EVENT_MOVE;

    }
    else
    {
        read_data[id].event = RT_TOUCH_EVENT_DOWN;
        s_tp_dowm[id] = 1;
    }

    read_data[id].timestamp = rt_touch_get_ts();
    read_data[id].width = w;
    read_data[id].x_coordinate = x;
    read_data[id].y_coordinate = y;
    read_data[id].track_id = id;
#ifdef PKG_USING_LVGL
    lv_port_indev_input(read_data[id].x_coordinate, read_data[id].y_coordinate, LV_INDEV_STATE_PR);
#endif
    pre_x[id] = x; 
    pre_y[id] = y;
    pre_w[id] = w;
}

static rt_size_t gt9157_read_point(struct rt_touch_device *touch, void *buf, rt_size_t read_num)
{
    rt_uint8_t point_status = 0;
    rt_uint8_t touch_num = 0;
    rt_uint8_t write_buf[3];
    rt_uint8_t cmd[2];
    rt_uint8_t read_buf[8 * GT9157_MAX_TOUCH] = {0};
    rt_uint8_t read_index;
    rt_int8_t read_id = 0;
    rt_int16_t input_x = 0;
    rt_int16_t input_y = 0;
    rt_int16_t input_w = 0;

    static rt_uint8_t pre_touch = 0;
    static rt_int8_t pre_id[GT9157_MAX_TOUCH] = {0};

    // 读取触摸状态
    cmd[0] = (rt_uint8_t)(GT9157_READ_STATUS >> 8);
    cmd[1] = (rt_uint8_t)(GT9157_READ_STATUS & 0xFF);

    if (gt9157_read_regs(gt9157_client, cmd, 2, 1, &point_status) != RT_EOK)
    {
        LOG_D("read point failed\n");
        read_num = 0;
        goto exit_;
    }

    // 无按键
    if (point_status == 0)             
    {
        read_num = 0;
        goto exit_;
    }

    // 未就绪
    if ((point_status & 0x80) == 0)    
    {
        read_num = 0;
        goto exit_;
    }

    // 触摸点数
    touch_num = point_status & 0x0F;  

    if (touch_num > GT9157_MAX_TOUCH) 
    {
        read_num = 0;
        goto exit_;
    }

    // 读取触摸点信息
    cmd[0] = (rt_uint8_t)(GT9157_POINT_REG >> 8);
    cmd[1] = (rt_uint8_t)(GT9157_POINT_REG & 0xFF);

    if (gt9157_read_regs(gt9157_client, cmd, 2, read_num * GT9157_POINT_INFO_NUM, read_buf) != RT_EOK)
    {
        LOG_D("read point failed\n");
        read_num = 0;
        goto exit_;
    }

    // 触摸点减少(有手指抬起)
    if (pre_touch > touch_num)                                      
    {
        for (read_index = 0; read_index < pre_touch; read_index++)
        {
            rt_uint8_t j;

            for (j = 0; j < touch_num; j++)                         
            {
                read_id = read_buf[j * GT9157_POINT_INFO_NUM] & 0x0F;

                if (pre_id[read_index] == read_id)                 
                    break;

                if (j >= touch_num - 1)
                {
                    rt_uint8_t up_id;
                    up_id = pre_id[read_index];
                    gt9157_touch_up(buf, up_id);
                }
            }
        }
    }

    if (touch_num)                                                 
    {
        rt_uint8_t off_set;

        for (read_index = 0; read_index < touch_num; read_index++)
        {
            off_set = read_index * GT9157_POINT_INFO_NUM;
            read_id = read_buf[off_set] & 0x0F;
            pre_id[read_index] = read_id;
            input_x = read_buf[off_set + 1] | (read_buf[off_set + 2] << 8);	
            input_y = read_buf[off_set + 3] | (read_buf[off_set + 4] << 8);	
            input_w = read_buf[off_set + 5] | (read_buf[off_set + 6] << 8);

            gt9157_touch_down(buf, read_id, input_x, input_y, input_w);
        }
    }
    else if (pre_touch)
    {
        for(read_index = 0; read_index < pre_touch; read_index++)
        {
            gt9157_touch_up(buf, pre_id[read_index]);
        }
    }

    pre_touch = touch_num;

exit_:
    // 读取后向触摸状态位写0
    write_buf[0] = (rt_uint8_t)(GT9157_READ_STATUS >> 8);
    write_buf[1] = (rt_uint8_t)(GT9157_READ_STATUS & 0xFF);
    write_buf[2] = 0x00;
    gt9157_write_reg(gt9157_client, 3, write_buf);
    return read_num;
}

static rt_sem_t     gt9157_sem = RT_NULL;

static rt_err_t rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(gt9157_sem);
    rt_device_control(dev, RT_TOUCH_CTRL_DISABLE_INT, RT_NULL);
    return 0;
}

void bsp_touch_entry(void *parameter)
{
    void *id;
    void *arg;
    rt_device_t  dev;
    struct rt_touch_data *read_data;
    struct rt_touch_info info;

    dev = rt_device_find("gt9157");
    RT_ASSERT(dev != RT_NULL);

    if (rt_device_open(dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open device failed!");
        return;
    }

    id = rt_malloc(sizeof(rt_uint8_t) * 8);
    rt_device_control(dev, RT_TOUCH_CTRL_GET_ID, id);
    rt_uint8_t *read_id = (rt_uint8_t *)id;

    LOG_I("id = %d %d %d %d \n", read_id[0] - '0', read_id[1] - '0', read_id[2] - '0', read_id[3] - '0');

    rt_device_control(dev, RT_TOUCH_CTRL_LOAD_CONFIG, arg);

    rt_device_control(dev, RT_TOUCH_CTRL_GET_INFO, id);
    LOG_I("range_x = %d \n", (*(struct rt_touch_info*)id).range_x);
    LOG_I("range_y = %d \n", (*(struct rt_touch_info*)id).range_y);
    LOG_I("point_num = %d \n", (*(struct rt_touch_info*)id).point_num);
    rt_free(id);

    rt_device_set_rx_indicate(dev, rx_callback);

    gt9157_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(gt9157_sem != RT_NULL);

    rt_device_control(dev, RT_TOUCH_CTRL_GET_INFO, &info);

    read_data = (struct rt_touch_data *)rt_malloc(sizeof(struct rt_touch_data) * info.point_num);

    while (1)
    {
        rt_sem_take(gt9157_sem, RT_WAITING_FOREVER);

        if (rt_device_read(dev, 0, read_data, info.point_num) == info.point_num)
        {
            /*for (rt_uint8_t i = 0; i < info.point_num; i++)
            {
                if (read_data[i].event == RT_TOUCH_EVENT_DOWN || read_data[i].event == RT_TOUCH_EVENT_MOVE)
                {
                    LOG_I("%d %d %d %d %d\n", read_data[i].track_id,
                            read_data[i].x_coordinate,
                            read_data[i].y_coordinate,
                            read_data[i].timestamp,
                            read_data[i].width);
               }
            }*/
        }
        rt_device_control(dev, RT_TOUCH_CTRL_ENABLE_INT, RT_NULL);
    }
}

static struct rt_touch_ops touch_ops =
{
    .touch_readpoint = gt9157_read_point,
    .touch_control = gt9157_control,
};

int rt_hw_gt9157_init(const char *name, struct rt_touch_config *cfg)
{
    rt_touch_t touch_device = RT_NULL;

    touch_device = (rt_touch_t)rt_calloc(1, sizeof(struct rt_touch_device));
    RT_ASSERT(touch_device != RT_NULL);

    // reset复位时int为低，配置7位地址0x5D
    rt_pin_mode(*(rt_uint8_t *)cfg->user_data, PIN_MODE_OUTPUT);
    rt_pin_mode(cfg->irq_pin.pin, PIN_MODE_OUTPUT);
    rt_pin_write(*(rt_uint8_t *)cfg->user_data, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(*(rt_uint8_t *)cfg->user_data, PIN_HIGH);
    rt_thread_mdelay(10);
    rt_pin_mode(cfg->irq_pin.pin, PIN_MODE_INPUT_PULLDOWN);
    rt_thread_mdelay(100);

    gt9157_client = (struct rt_i2c_client *)rt_calloc(1, sizeof(struct rt_i2c_client));
    RT_ASSERT(gt9157_client != RT_NULL);

    gt9157_client->bus = (struct rt_i2c_bus_device *)rt_device_find(cfg->dev_name);
    RT_ASSERT(gt9157_client->bus != RT_NULL);

    if (rt_device_open((rt_device_t)gt9157_client->bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open device failed\n");
        return -RT_ERROR;
    }

    gt9157_client->client_addr = GT9157_ADDRESS_LOW;

    touch_device->info.type = RT_TOUCH_TYPE_CAPACITANCE;
    touch_device->info.vendor = RT_TOUCH_VENDOR_GT;
    rt_memcpy(&touch_device->config, cfg, sizeof(struct rt_touch_config));
    touch_device->ops = &touch_ops;

    rt_hw_touch_register(touch_device, name, RT_DEVICE_FLAG_INT_RX, RT_NULL);

    return RT_EOK;
}

int rt_hw_gt9157_port(void)
{
    rt_uint8_t rst;
    struct rt_touch_config config;

    rst = BSP_TOUCH_RST_PIN;
    config.dev_name = "i2c2";
    config.irq_pin.pin  = BSP_TOUCH_INT_PIN;
    config.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;
    config.user_data = &rst;
    if(rt_hw_gt9157_init("gt9157", &config) != RT_EOK)
    {
        LOG_E("gt9157 int failed\n");
        return -RT_ERROR;
    }

    rt_thread_t tid = rt_thread_create("bsp_touch", bsp_touch_entry, RT_NULL,
                           1024, 10, 10);
    RT_ASSERT(tid != RT_NULL);
    
    rt_thread_startup(tid);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_gt9157_port);

#endif
