/* linux/drivers/amlogic/hdmi/hdmi_tx/amlogic_cec.c
 *
 * Copyright (c) 2016 Gerald Dachs
 *
 * CEC interface file for Amlogic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/atomic.h>

#include <asm/uaccess.h>
#include <asm/delay.h>
#include <mach/am_regs.h>
#include <mach/power_gate.h>
#include <linux/amlogic/tvin/tvin.h>

#include <mach/gpio.h>
#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>
#include <mach/hdmi_tx_reg.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_cec.h>

#define CONFIG_TV_DEBUG

MODULE_AUTHOR("Gerald Dachs");
MODULE_DESCRIPTION("Amlogic CEC driver");
MODULE_LICENSE("GPL");

#define CEC_IOC_MAGIC        'c'
#define CEC_IOC_SETLADDR     _IOW(CEC_IOC_MAGIC, 0, unsigned int)
#define CEC_IOC_GETPADDR     _IO(CEC_IOC_MAGIC, 1)

#define VERSION   "1.0" /* Driver version number */
#define CEC_MINOR 243	/* Major 10, Minor 242, /dev/cec */

/* CEC Rx buffer size */
#define CEC_RX_BUFF_SIZE            16
/* CEC Tx buffer size */
#define CEC_TX_BUFF_SIZE            16

#define DRV_NAME "amlogic_cec"
#ifndef tvout_dbg
#ifdef CONFIG_TV_DEBUG
#define tvout_dbg(fmt, ...)					\
		printk(KERN_INFO "[%s] %s(): " fmt,		\
		DRV_NAME, __func__, ##__VA_ARGS__)
#else
#define tvout_dbg(fmt, ...)
#endif
#endif

static atomic_t hdmi_on = ATOMIC_INIT(0);

bool cec_msg_dbg_en = 1;
cec_global_info_t cec_global_info;

static hdmitx_dev_t* hdmitx_device = NULL;
static struct workqueue_struct *cec_workqueue = NULL;

struct cec_rx_list {
	u8 buffer[CEC_RX_BUFF_SIZE];
	unsigned char size;
	struct list_head list;
};

struct cec_rx_struct {
	spinlock_t lock;
	wait_queue_head_t waitq;
	atomic_t state;
	struct list_head list;
};

enum cec_state {
	STATE_RX,
	STATE_TX,
	STATE_DONE,
	STATE_ERROR
};

static char banner[] __initdata =
    "Amlogic CEC Driver, (c) 2016 Gerald Dachs\n";

static struct cec_rx_struct cec_rx_struct;

unsigned short cec_log_addr_to_dev_type(unsigned char log_addr)
{
    unsigned short us = CEC_UNREGISTERED_DEVICE_TYPE;
    if ((1 << log_addr) & CEC_DISPLAY_DEVICE)
    {
        us = CEC_DISPLAY_DEVICE_TYPE;
    }
    else if ((1 << log_addr) & CEC_RECORDING_DEVICE)
    {
        us = CEC_RECORDING_DEVICE_TYPE;
    }
    else if ((1 << log_addr) & CEC_PLAYBACK_DEVICE)
    {
        us = CEC_PLAYBACK_DEVICE_TYPE;
    }
    else if ((1 << log_addr) & CEC_TUNER_DEVICE)
    {
        us = CEC_TUNER_DEVICE_TYPE;
    }
    else if ((1 << log_addr) & CEC_AUDIO_SYSTEM_DEVICE)
    {
        us = CEC_AUDIO_SYSTEM_DEVICE_TYPE;
    }

    return us;
}

void cec_report_physical_address_smp(void)
{
    unsigned char msg[5];
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    tvout_dbg("cec_report_physical_address_smp: enter\n");

    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_REPORT_PHYSICAL_ADDRESS;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;
    msg[4] = cec_log_addr_to_dev_type(index);

    cec_ll_tx(msg, 5);

    tvout_dbg("cec_report_physical_address_smp: leave\n");
}

void cec_node_init(hdmitx_dev_t* hdmitx_device)
{
    unsigned long cec_phy_addr;

    tvout_dbg("cec node init: enter\n");

    cec_phy_addr = (((hdmitx_device->hdmi_info.vsdb_phy_addr.a) & 0xf) << 12)
                 | (((hdmitx_device->hdmi_info.vsdb_phy_addr.b) & 0xf) << 8)
                 | (((hdmitx_device->hdmi_info.vsdb_phy_addr.c) & 0xf) << 4)
                 | (((hdmitx_device->hdmi_info.vsdb_phy_addr.d) & 0xf) << 0);

    // If VSDB is not valid,use last or default physical address.
    if (hdmitx_device->hdmi_info.vsdb_phy_addr.valid == 0)
    {
	tvout_dbg("no valid cec physical address\n");
	if (aml_read_reg32 (P_AO_DEBUG_REG1))
	{
	    tvout_dbg("use last physical address\n");
	}
	else
	{
	    aml_write_reg32 (P_AO_DEBUG_REG1, 0x1000);
	    tvout_dbg("use default physical address\n");
	}
    }
    else
    {
	aml_write_reg32 (P_AO_DEBUG_REG1, cec_phy_addr);
    }
    tvout_dbg("physical address:0x%x\n", aml_read_reg32(P_AO_DEBUG_REG1));

    if (hdmitx_device->cec_init_ready != 0)
    {
	tvout_dbg("report physical address:0x%x\n", aml_read_reg32(P_AO_DEBUG_REG1));
	cec_report_physical_address_smp();
    }
    tvout_dbg("cec node init: cec features ok !\n");
}

static void amlogic_cec_set_rx_state(enum cec_state state)
{
    atomic_set(&cec_rx_struct.state, state);
}

static int amlogic_cec_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    if (atomic_read(&hdmi_on))
    {
	tvout_dbg("do not allow multiple open for tvout cec\n");
	ret = -EBUSY;
    }
    else
    {
	atomic_inc(&hdmi_on);
    }
    return ret;
}

static int amlogic_cec_release(struct inode *inode, struct file *file)
{
    atomic_dec(&hdmi_on);

    return 0;
}

static ssize_t amlogic_cec_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
    ssize_t retval;
    unsigned long spin_flags;
    struct cec_rx_list* entry = NULL;

    tvout_dbg("amlogic_cec_read: enter\n");

    if (wait_event_interruptible(cec_rx_struct.waitq,
				 atomic_read(&cec_rx_struct.state) == STATE_DONE))
    {
	tvout_dbg("error during wait on state change\n");
	return -ERESTARTSYS;
    }

    spin_lock_irqsave(&cec_rx_struct.lock, spin_flags);

    entry = list_first_entry_or_null(&cec_rx_struct.list, struct cec_rx_list, list);

    if (entry == NULL || entry->size > count)
    {
	tvout_dbg("entry is NULL, or empty\n");
	retval = -1;
	goto error_exit;
    }

    if (copy_to_user(buffer, entry->buffer, entry->size))
    {
	printk(KERN_ERR " copy_to_user() failed!\n");

	retval = -EFAULT;
	goto error_exit;
    }

    retval = entry->size;

    amlogic_cec_set_rx_state(STATE_RX);

error_exit:
    if (entry != NULL)
    {
    	list_del(&entry->list);
    	kfree(entry);
    }

    spin_unlock_irqrestore(&cec_rx_struct.lock, spin_flags);

    tvout_dbg("amlogic_cec_read: leave\n");

    return retval;
}

static ssize_t amlogic_cec_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
    int ret = 0;

    char data[CEC_TX_BUFF_SIZE];

    tvout_dbg("amlogic_cec_write: enter\n");

    /* check data size */
    if (count > CEC_TX_BUFF_SIZE || count == 0)
	return -1;

    if (copy_from_user(data, buffer, count))
    {
	printk(KERN_ERR " copy_from_user() failed!\n");
	return -EFAULT;
    }

    ret = cec_ll_tx(data, count);
    if (ret != 1)
    {
    	tvout_dbg("Message transmit failed, ret=%d\n", ret);
        return -1;
    }

    tvout_dbg("amlogic_cec_write: leave\n");

    return count;
}

static long amlogic_cec_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
    unsigned char logical_addr;

    switch(cmd) {
    case CEC_IOC_SETLADDR:
        if (get_user(logical_addr, (unsigned char __user *)arg))
        {
            tvout_dbg("Failed to get logical addr from user\n");
            return -EFAULT;
        }

        cec_global_info.my_node_index = logical_addr;
#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
        hdmi_wr_reg(CEC0_BASE_ADDR+CEC_LOGICAL_ADDR0, (0x1 << 4) | logical_addr);
#endif
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
        aocec_wr_reg(CEC_LOGICAL_ADDR0, (0x1 << 4) | logical_addr);
#endif
        tvout_dbg("Set logical address: %d\n", logical_addr);
        return 0;

    case CEC_IOC_GETPADDR:
    	return aml_read_reg32(P_AO_DEBUG_REG1);
    }

    return -EINVAL;
}

static u32 amlogic_cec_poll(struct file *file, poll_table *wait)
{
    poll_wait(file, &cec_rx_struct.waitq, wait);

    if (atomic_read(&cec_rx_struct.state) == STATE_DONE)
    {
	return POLLIN | POLLRDNORM;
    }
    return 0;
}

static const struct file_operations cec_fops = {
    .owner   = THIS_MODULE,
    .open    = amlogic_cec_open,
    .release = amlogic_cec_release,
    .read    = amlogic_cec_read,
    .write   = amlogic_cec_write,
    .unlocked_ioctl = amlogic_cec_ioctl,
    .poll    = amlogic_cec_poll,
};

static struct miscdevice cec_misc_device = {
    .minor = CEC_MINOR,
    .name  = "AmlogicCEC",
    .fops  = &cec_fops,
};

static irqreturn_t amlogic_cec_irq_handler(int irq, void *dummy)
{
    unsigned long spin_flags;
    struct cec_rx_list *entry;
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
    unsigned int intr_stat = 0;
#endif

    tvout_dbg("amlogic_cec_irq_handler: enter\n");

    udelay(100); //Delay execution a little. This fixes an issue when HDMI CEC stops working after a while.

#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
    tvout_dbg("cec TX status: rx: 0x%x; tx: 0x%x\n", hdmi_rd_reg(CEC0_BASE_ADDR+CEC_RX_MSG_STATUS), hdmi_rd_reg(CEC0_BASE_ADDR+CEC_TX_MSG_STATUS));
    if (hdmi_rd_reg(CEC0_BASE_ADDR+CEC_RX_MSG_STATUS) != RX_DONE)
    {
        return IRQ_HANDLED;
    }
#endif

#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
    intr_stat = aml_read_reg32(P_AO_CEC_INTR_STAT);
    tvout_dbg("aocec irq %x\n", intr_stat);
    if (intr_stat & (1<<1))
    {   // aocec tx intr
        tx_irq_handle();
        return IRQ_HANDLED;
    }
#endif

    if ((entry = kmalloc(sizeof(struct cec_rx_list), GFP_ATOMIC)) == NULL)
    {
        tvout_dbg("can't alloc cec_rx_list\n");
        return IRQ_HANDLED;
    }

    if ((-1) == cec_ll_rx(entry->buffer, &entry->size))
    {
        kfree(entry);
        tvout_dbg("amlogic_cec_irq_handler: nothing to read\n");
        return IRQ_HANDLED;
    }

    INIT_LIST_HEAD(&entry->list);

    spin_lock_irqsave(&cec_rx_struct.lock, spin_flags);
    list_add_tail(&entry->list, &cec_rx_struct.list);
    amlogic_cec_set_rx_state(STATE_DONE);
    spin_unlock_irqrestore(&cec_rx_struct.lock, spin_flags);

    wake_up_interruptible(&cec_rx_struct.waitq);

    tvout_dbg("amlogic_cec_irq_handler: leave\n");

    return IRQ_HANDLED;
}

static void amlogic_cec_delayed_init(struct work_struct *work)
{
    hdmitx_dev_t* hdmitx_device = (hdmitx_dev_t*)container_of(work, hdmitx_dev_t, cec_work);

    tvout_dbg("amlogic_cec_delayed_init: enter\n");

    msleep_interruptible(15000);

#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
    cec_gpi_init();
#endif

#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
    aml_set_reg32_bits(P_PERIPHS_PIN_MUX_1, 1, 25, 1);
    // Clear CEC Int. state and set CEC Int. mask
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR) | (1 << 23));    // Clear the interrupt
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK) | (1 << 23));            // Enable the hdmi cec interrupt

#endif
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
#if 1           // Please match with H/W cec config
// GPIOAO_12
    aml_set_reg32_bits(P_AO_RTI_PIN_MUX_REG, 0, 14, 1);       // bit[14]: AO_PWM_C pinmux                  //0xc8100014
    aml_set_reg32_bits(P_AO_RTI_PULL_UP_REG, 1, 12, 1);       // bit[12]: enable AO_12 internal pull-up   //0xc810002c
    aml_set_reg32_bits(P_AO_RTI_PIN_MUX_REG, 1, 17, 1);       // bit[17]: AO_CEC pinmux                    //0xc8100014
    ao_cec_init();
#else
// GPIOH_3
    aml_set_reg32_bits(P_PAD_PULL_UP_EN_REG1, 0, 19, 1);    // disable gpioh_3 internal pull-up
    aml_set_reg32_bits(P_PERIPHS_PIN_MUX_1, 1, 23, 1);      // gpioh_3 cec pinmux
#endif
    cec_arbit_bit_time_set(3, 0x118, 0);
    cec_arbit_bit_time_set(5, 0x000, 0);
    cec_arbit_bit_time_set(7, 0x2aa, 0);
#endif

    hdmitx_device->cec_init_ready = 1;
    cec_global_info.cec_flag.cec_init_flag = 0;

    tvout_dbg("amlogic_cec_delayed_init: leave\n");
}

static int amlogic_cec_init(void)
{
    extern hdmitx_dev_t * get_hdmitx_device(void);
    INIT_LIST_HEAD(&cec_rx_struct.list);

    printk(banner);

    hdmitx_device = get_hdmitx_device();
    tvout_dbg("CEC init\n");
    memset(&cec_global_info, 0, sizeof(cec_global_info_t));

#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_CLOCK_DIV_H, 0x00 );
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_CLOCK_DIV_L, 0xf0 );
#endif

    cec_global_info.cec_rx_msg_buf.rx_buf_size = sizeof(cec_global_info.cec_rx_msg_buf.cec_rx_message)/sizeof(cec_global_info.cec_rx_msg_buf.cec_rx_message[0]);
    cec_global_info.hdmitx_device = hdmitx_device;

#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
    if (request_irq(INT_HDMI_CEC, &amlogic_cec_irq_handler,
    IRQF_SHARED, "amhdmitx-cec",(void *)hdmitx_device))
    {
    	tvout_dbg("Can't register IRQ %d\n",INT_HDMI_CEC);
        return -EFAULT;
    }
#endif
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
    if (request_irq(INT_AO_CEC, &amlogic_cec_irq_handler,
    IRQF_SHARED, "amhdmitx-aocec",(void *)hdmitx_device))
    {
    	tvout_dbg("Can't register IRQ %d\n",INT_HDMI_CEC);
        return -EFAULT;
    }
#endif

    init_waitqueue_head(&cec_rx_struct.waitq);
    spin_lock_init(&cec_rx_struct.lock);

    if (misc_register(&cec_misc_device))
    {
	printk(KERN_WARNING " Couldn't register device 10, %d.\n", CEC_MINOR);
	return -EBUSY;
    }

    cec_workqueue = create_workqueue("cec_work");
    if (cec_workqueue == NULL)
    {
	printk("create work queue failed\n");
	return -EFAULT;
    }
    INIT_WORK(&hdmitx_device->cec_work, amlogic_cec_delayed_init);
    queue_work(cec_workqueue, &hdmitx_device->cec_work);    // for init

    tvout_dbg("hdmitx_device->cec_init_ready:0x%x\n", hdmitx_device->cec_init_ready);

    return 0;
}

static void amlogic_cec_exit(void)
{
    if (cec_global_info.cec_flag.cec_init_flag == 1)
    {

#if MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6
        aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK) & ~(1 << 23));            // Disable the hdmi cec interrupt
        free_irq(INT_HDMI_CEC, (void *)hdmitx_device);
#endif
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
        free_irq(INT_AO_CEC, (void *)hdmitx_device);
#endif
        cec_global_info.cec_flag.cec_init_flag = 0;
    }

	misc_deregister(&cec_misc_device);
}

module_init(amlogic_cec_init);
module_exit(amlogic_cec_exit);

