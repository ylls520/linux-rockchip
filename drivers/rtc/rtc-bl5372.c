/*
 * An I2C driver for Beilin BL5372 RTC
 */

//#define DEBUG           /* Enable dev_dbg */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>

#define DRV_VERSION "0.0.2"
#define DRIVER_NAME     "rtc-bl5372"

#define TIME24 1
#define RS5C_ADDR(R)        (((R) << 4) | 0)
#define RS5C372_REG_SECS    0
#define RS5C372_REG_MINS    1
#define RS5C372_REG_HOURS   2
#define RS5C372_REG_WDAY    3
#define RS5C372_REG_DAY     4
#define RS5C372_REG_MONTH   5
#define RS5C372_REG_YEAR    6
#define RS5C372_REG_TRIM    7
#define RS5C_REG_ALARM_A_MIN    8           /* or ALARM_W */
#define RS5C_REG_ALARM_A_HOURS  9
#define RS5C_REG_ALARM_A_WDAY   10

#define RS5C_REG_ALARM_B_MIN    11          /* or ALARM_D */
#define RS5C_REG_ALARM_B_HOURS  12
#define RS5C_REG_ALARM_B_WDAY   13          /* (ALARM_B only) */
#define RS5C_REG_CTRL1          14
#define RS5C_REG_CTRL2          15

struct bl5372 {
    struct rtc_device *rtc;
    struct i2c_client *client;
    bool irq_registered;
};

static int bl5372_clear_int_flags(struct bl5372 * bl5372);

static unsigned rs5c_reg2hr(unsigned reg)
{
#if TIME24
    return bcd2bin(reg & 0x3f);
#else
    unsigned    hour;
    hour = bcd2bin(reg & 0x1f);
    if (hour == 12)
        hour = 0;
    if (reg & 0x20)
        hour += 12;
    return hour;
#endif
}

static unsigned rs5c_hr2reg(unsigned hour)
{
#if TIME24
    return bin2bcd(hour);

#else
    if (hour > 12)
        return 0x20 | bin2bcd(hour - 12);
    if (hour == 12)
        return 0x20 | bin2bcd(12);
    if (hour == 0)
        return bin2bcd(12);
    return bin2bcd(hour);
#endif
}

/* caller holds rtc->ops_lock */
static int bl5372_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
    int err = 0;
    // struct bl5372 *bl5372 = i2c_get_clientdata(client);
    unsigned char buf[7] = { RS5C_ADDR(RS5C372_REG_SECS) };

    struct i2c_msg msgs[] = {
        {/* setup read ptr */
            .addr = client->addr,
            .flags = 0,/* write */
            .len = 1,
            .buf = buf
        },
        {/* read the sec,min,hour,week,day,month,year */
            .addr = client->addr,
            .flags = I2C_M_RD,/* read */
            .len = 7,
            .buf = buf
        },
    };

    /* read registers */
    err = i2c_transfer(client->adapter, msgs, 2);
    if (err != 2) {
        dev_err(&client->dev, "[%s:%d]: read error\n", __func__, __LINE__);
        err = -EIO;
        goto EXIT_PROC;
    }

    tm->tm_sec = bcd2bin(buf[0] & 0x7f);
    tm->tm_min = bcd2bin(buf[1] & 0x7f);
    tm->tm_hour = rs5c_reg2hr(buf[2]);
    tm->tm_mday = bcd2bin(buf[4] & 0x7f);;
    tm->tm_wday = bcd2bin(buf[3] & 0x7f);
    tm->tm_mon = rs5c_reg2hr(buf[5])-1;
    tm->tm_year = bcd2bin(buf[6] & 0x7f)+100;

    /* the clock can give out invalid datetime, but we cannot return
     * -EINVAL otherwise hwclock will refuse to set the time on bootup.
     */
    if (rtc_valid_tm(tm) < 0)
        dev_err(&client->dev, "retrieved date/time is not valid.\n");

EXIT_PROC:

    return err;
}

/* caller holds rtc->ops_lock */
static int bl5372_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
    // struct bl5372 *bl5372 = i2c_get_clientdata(client);
    int err = 0;
    unsigned char buf[7];
    struct i2c_msg msgs2[] = {
        {/* setup read  */
            .addr = client->addr,
            .len = 1,
            .buf = buf
        },
        {/* read is_24hour */
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = buf
        },
    };

    dev_dbg(&(client->dev), "%s secs=%d, mins=%d, "
        "hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
        "write", tm->tm_sec, tm->tm_min,
        tm->tm_hour, tm->tm_mday,
        tm->tm_mon, tm->tm_year, tm->tm_wday);

    buf[0] = RS5C_ADDR(RS5C_REG_CTRL2);

    /* read registers */
    err = i2c_transfer(client->adapter, msgs2, 2);
    if (err != 2) {
        dev_err(&client->dev, "[%s:%d]: read error\n", __func__, __LINE__);
        err = -EIO;
        goto EXIT_PROC;
    }

    if((buf[0]&0x20)== 0)   /* 12 hour system */
    {
        /* use 24 hour system */
        buf[0] = RS5C_ADDR(RS5C_REG_CTRL2);
        buf[1] |= (1<<5);
        err = i2c_master_send(client, buf, 2);
        if(0 != err)
        {
            goto EXIT_PROC;
        }
    }

    /* hours, minutes and seconds */
    buf[0] = bin2bcd(tm->tm_sec);
    buf[1] = bin2bcd(tm->tm_min);
    buf[2] = rs5c_hr2reg(tm->tm_hour);
    buf[3] = bin2bcd(tm->tm_wday & 0x07); //week 0~6
    buf[4] = bin2bcd(tm->tm_mday);
    buf[5] = bin2bcd(tm->tm_mon)+1;// 0~11
    tm->tm_year -= 100;
    buf[6] = bin2bcd(tm->tm_year % 100);// start at 1900  2018=>118

    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_SECS),   buf[0]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }
    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_MINS) ,  buf[1]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }
    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_HOURS) , buf[2]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }
    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_WDAY) ,  buf[3]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }
    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_DAY) ,   buf[4]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }
    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_MONTH) , buf[5]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }
    err = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C372_REG_YEAR) ,  buf[6]);
    if(0 != err)
    {
        goto EXIT_PROC;
    }

EXIT_PROC:

    return err;
}

/* The caller(rtc_mgr.ioctl) holds rtc->ops_lock */
static int bl5372_set_alarm(struct bl5372 *bl5372, struct rtc_wkalrm *t)
{
    int err_out = 0, err_ret;
    struct i2c_client *client = bl5372->client;
    u8 regData;

    dev_dbg(&(client->dev), "%s secs=%d, mins=%d, "
        "hours=%d, mday=%d, enabled=%d, pending=%d\n",
        "alarm set", t->time.tm_sec, t->time.tm_min,
        t->time.tm_hour, t->time.tm_mday,
        t->enabled, t->pending);

    /* clean interrupt flags */
    err_ret = bl5372_clear_int_flags(bl5372);
    if(0 != err_ret)
    {
        err_out = err_ret;
        goto EXIT_PROC;
    }

    if(t->enabled)
    {
        /* alarm_A.minute */
        regData = bin2bcd(t->time.tm_min);
        err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_ALARM_A_MIN), regData);
        if(0 != err_ret)
        {
            err_out = err_ret;
            pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
            goto EXIT_PROC;
        }
        /* alarm_A.hour */
        regData = bin2bcd(t->time.tm_hour);
        err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_ALARM_A_HOURS), regData);
        if(0 != err_ret)
        {
            err_out = err_ret;
            pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
            goto EXIT_PROC;
        }
    }

    /* enable ALARM A? */
    err_ret = i2c_smbus_read_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL1));
    if(0 > err_ret)
    {
        err_out = err_ret;
        pr_err("[%s:%d]i2c r err %d\n", __FUNCTION__, __LINE__, err_ret);
        goto EXIT_PROC;
    }
    regData = err_ret;
    if(t->enabled)
    {
        regData |= (0b1 << 7);
    }
    else
    {
        regData &= ~((0b1 << 7));
    }
    err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL1), regData);
    if(0 != err_ret)
    {
        err_out = err_ret;
        pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
        goto EXIT_PROC;
    }

EXIT_PROC:

    return err_out;
}

#define RTC_WKALM_SET_DIRECTLY 1  // 如果没有定义，手动定义一个值

/* caller holds rtc->ops_lock */
static int bl5372_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    struct bl5372 *bl5372 = i2c_get_clientdata(to_i2c_client(dev));
    void __user *uarg = (void __user *)arg;

    switch (cmd) {
        case RTC_WKALM_SET_DIRECTLY: {
            struct rtc_wkalrm alarm;
            if (copy_from_user(&alarm, uarg, sizeof(alarm))){
                return -EFAULT;
            }
            err = bl5372_set_alarm(bl5372, &alarm);
            break;
        }
    default:
        err = -ENOIOCTLCMD;
    }

    return err;
}

/* caller holds rtc->ops_lock */
static int bl5372_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
    return bl5372_get_datetime(to_i2c_client(dev), tm);
}

/* caller holds rtc->ops_lock */
static int bl5372_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
    return bl5372_set_datetime(to_i2c_client(dev), tm);
}

/* caller holds rtc->ops_lock */
static int bl5372_rtc_getalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
    // struct bl5372 *bl5372 = i2c_get_clientdata(to_i2c_client(dev));
    return 0;
}

/* caller holds rtc->ops_lock */
static int bl5372_rtc_setalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
    // struct bl5372 *bl5372 = i2c_get_clientdata(to_i2c_client(dev));
    return 0;
}

/* caller holds rtc->ops_lock */
static int bl5372_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
    // struct bl5372 *bl5372 = i2c_get_clientdata(to_i2c_client(dev));

    return 0;
}

static int bl5372_clear_int_flags(struct bl5372 * bl5372)
{
    int err_out = 0, err_ret;
    u8 regData;
    struct i2c_client * client = bl5372->client;

    /* clean ALARM A interrupt flags */
    err_ret = i2c_smbus_read_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL2));
    if(0 > err_ret)
    {
        err_out = err_ret;
        pr_err("[%s:%d]i2c r err %d\n", __FUNCTION__, __LINE__, err_ret);
        goto EXIT_PROC;
    }
    regData = err_ret;
    regData &= ~((0b1 << 2) | (0b1 << 1));
    err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL2), regData);
    if(0 != err_ret)
    {
        err_out = err_ret;
        pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
        goto EXIT_PROC;
    }

EXIT_PROC:

    return err_out;
}

/* thread context, should lock rtc->ops_lock */
static irqreturn_t bl5372_irq(int irq, void *dev_id)
{
    struct bl5372       *bl5372 = dev_id;
    int err;
    struct mutex        *lock = &(bl5372->rtc->ops_lock);

    dev_dbg(&(bl5372->client->dev), "bl5372_irq\n");

    mutex_lock(lock);

    err = bl5372_clear_int_flags(bl5372);
    if(0 > err)
    {
        dev_err(&(bl5372->client->dev), "clr int err %d\n", err);
    }

    mutex_unlock(lock);

    return IRQ_HANDLED;
}

static const struct rtc_class_ops bl5372_rtc_ops = {
    .ioctl      = bl5372_rtc_ioctl,
    .read_time  = bl5372_rtc_read_time,
    .set_time   = bl5372_rtc_set_time,
    .read_alarm             = bl5372_rtc_getalarm,
    .set_alarm              = bl5372_rtc_setalarm,
    .alarm_irq_enable       = bl5372_rtc_alarm_irq_enable
};

static int bl5372_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err_ret = 0;
    struct bl5372 *bl5372 = NULL;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        err_ret = -ENODEV;
        goto EXIT_PROC;
    }

    /* check whether the device exists */
    {
        unsigned char buf[2];
        buf[0] = 0x00;
        struct i2c_msg detect_msg[] = {
            {
                .addr = client->addr,
                .len = 1,
                .buf = buf
            },
            {
                .addr = client->addr,
                .flags = I2C_M_RD,
                .len = 1,
                .buf = buf
            },
        };
        /* read registers */
        err_ret = i2c_transfer(client->adapter, detect_msg, sizeof(detect_msg)/sizeof(detect_msg[0]));
        if(sizeof(detect_msg)/sizeof(detect_msg[0]) != err_ret)
        {
            err_ret = -ENODEV;
            goto EXIT_PROC;
        }
        err_ret = 0;
    }

    /* Memory allocated with this function is automatically freed on driver detach */
    bl5372 = devm_kzalloc(&client->dev, sizeof(*bl5372), GFP_KERNEL);
    if (!bl5372)
    {
        err_ret = -ENOMEM;
        goto EXIT_PROC;
    }

    device_init_wakeup(&client->dev, 1);
    bl5372->client = client;
    i2c_set_clientdata(client, bl5372);

    /* The rtc_device returned from this function are automatically freed on driver detach. */
    bl5372->rtc = devm_rtc_device_register(&client->dev, DRIVER_NAME, &bl5372_rtc_ops, THIS_MODULE);
    if (IS_ERR(bl5372->rtc))
    {
        err_ret = PTR_ERR(bl5372->rtc);
        goto EXIT_PROC;
    }

    /* set 24hour system, no 32KHz output, clean interrupt flags */
    err_ret = i2c_smbus_read_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL2));
    if(0 > err_ret)
    {
        pr_err("[%s:%d]i2c r err %d\n", __FUNCTION__, __LINE__, err_ret);
        goto EXIT_PROC;
    }
    u8 regData = err_ret;
    regData |= (0b1 << 5) | (0b1 << 3);
    regData &= ~((0b1 << 2) | (0b1 << 1));
    err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL2), regData);
    if(0 != err_ret)
    {
        pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
        goto EXIT_PROC;
    }

    /* set alarm base param */
    do {
        // week 1~7
        err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_ALARM_A_WDAY), 0x7F);
        if(0 != err_ret)
        {
            pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
            break;
        }
        // alarm A intterrupt to INTRA, no periodic output; low active level as interrupt level;
        err_ret = i2c_smbus_read_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL1));
        if(0 > err_ret)
        {
            pr_err("[%s:%d]i2c r err %d\n", __FUNCTION__, __LINE__, err_ret);
            break;
        }
        u8 regData = err_ret;
        regData |= (0b11 << 4);
        regData &= ~((0b111));  /* periodic interrupt pin outputs high level. */
        err_ret = i2c_smbus_write_byte_data(client, RS5C_ADDR(RS5C_REG_CTRL1), regData);
        if(0 != err_ret)
        {
            pr_err("[%s:%d]i2c w err %d\n", __FUNCTION__, __LINE__, err_ret);
            break;
        }
    }while(false);
    err_ret = 0;    /* alarm err can be ignored. */

    dev_dbg(&(client->dev), "irq: %d\n", client->irq);
    if(0 < client->irq)
    {
        /* IRQs requested with this function will be automatically freed on driver detach */
        err_ret = devm_request_threaded_irq(&(client->dev), client->irq, NULL, bl5372_irq,
            (IRQF_SHARED | IRQF_ONESHOT | IRQF_TRIGGER_FALLING), client->name, bl5372);
        if(0 != err_ret)
        {
            dev_err(&(client->dev), "unable to request IRQ(%d),err %d!\n", client->irq, err_ret);
            goto EXIT_PROC;
        }
        bl5372->irq_registered = true;
    }

EXIT_PROC:

    if(0 != err_ret)
    {
    }

    return err_ret;
}

static void bl5372_remove(struct i2c_client *client)
{
    int err_ret;
    struct bl5372 *bl5372 = i2c_get_clientdata(client);

    if(true == bl5372->irq_registered)
    {
        devm_free_irq(&(client->dev), client->irq, bl5372);
        bl5372->irq_registered = false;
    }

    /* clean interrupt flags */
    err_ret = bl5372_clear_int_flags(bl5372);
    if(0 != err_ret)
    {
    }
}

static const struct i2c_device_id bl5372_id[] = {
    { "bl5372", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bl5372_id);

#ifdef CONFIG_OF
static const struct of_device_id bl5372_of_match[] = {
    { .compatible = "beilin,bl5372" },
    {}
};
MODULE_DEVICE_TABLE(of, bl5372_of_match);
#endif

static struct i2c_driver bl5372_driver = {
    .driver     = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(bl5372_of_match),
    },
    .probe      = bl5372_probe,
    .remove     = bl5372_remove,
    .id_table   = bl5372_id,
};

module_i2c_driver(bl5372_driver);

MODULE_AUTHOR("nomivision");
MODULE_DESCRIPTION("Beilin BL5372 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
