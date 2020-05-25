#include <linux/mfd/stm32-timers.h>
#include <linux/pinctrl/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/pps_kernel.h>

#define DEFAULT_CHANNEL 1

#define TIM_CR1_UDIS	BIT(1)	/* UEV disabled */

#define TIM_DIER_CC1IE	BIT(1)	/* CC1 IRQ enable */
#define TIM_DIER_CC2IE	BIT(2)	/* CC2 IRQ enable */
#define TIM_DIER_CC3IE	BIT(3)	/* CC3 IRQ enable */
#define TIM_DIER_CC4IE	BIT(4)	/* CC4 IRQ enable */

#define TIM_SR_CC1IF    BIT(1)  /* CC1 IRQ flag */
#define TIM_SR_CC2IF    BIT(2)  /* CC2 IRQ flag */
#define TIM_SR_CC3IF    BIT(3)  /* CC3 IRQ flag */
#define TIM_SR_CC4IF    BIT(4)  /* CC4 IRQ flag */

struct stm32_pps {
  struct regmap *regmap;
  struct clk *clk;
  struct pps_device *pps;
  struct pps_source_info info;
  int ready;
  u32 events;
  u32 count_at_interrupt[2];
  u32 ps_per_hz;
  u64 local_count;
  struct pps_event_time ts;
  struct timespec64 delta;
  int channel_reg_offset;
  u8 channel;
};

static ssize_t interrupt_delta_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct stm32_pps *ddata = dev_get_drvdata(dev);
  return sprintf(buf, "%lld.%09ld\n", (long long)ddata->delta.tv_sec, ddata->delta.tv_nsec);
}

static DEVICE_ATTR(interrupt_delta, S_IRUGO, interrupt_delta_show, NULL);

static ssize_t pps_ts_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct stm32_pps *ddata = dev_get_drvdata(dev);
  return snprintf(buf, PAGE_SIZE, "%lld.%09ld\n", (long long)ddata->ts.ts_real.tv_sec, ddata->ts.ts_real.tv_nsec);
}

static DEVICE_ATTR(pps_ts, S_IRUGO, pps_ts_show, NULL);

static ssize_t count_at_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct stm32_pps *ddata = dev_get_drvdata(dev);
  return snprintf(buf, PAGE_SIZE, "%u\n%u\n", ddata->count_at_interrupt[0], ddata->count_at_interrupt[1]);
}

static DEVICE_ATTR(count_at_interrupt, S_IRUGO, count_at_interrupt_show, NULL);

static ssize_t sysfs_show(struct device *dev, int register_offset, char *buf) {
  u32 register_value = 0;

  struct stm32_pps *ddata = dev_get_drvdata(dev);
  regmap_read(ddata->regmap, register_offset, &register_value);

  return snprintf(buf, PAGE_SIZE, "%u\n", register_value);
}

static ssize_t cnt_show(struct device *dev, struct device_attribute *attr, char *buf) {
  return sysfs_show(dev, TIM_CNT, buf);
}

static DEVICE_ATTR(cnt, S_IRUGO, cnt_show, NULL);

static ssize_t ccr1_show(struct device *dev, struct device_attribute *attr, char *buf) {
  return sysfs_show(dev, TIM_CCR1, buf);
}

static DEVICE_ATTR(ccr1, S_IRUGO, ccr1_show, NULL);

static ssize_t ccr2_show(struct device *dev, struct device_attribute *attr, char *buf) {
  return sysfs_show(dev, TIM_CCR2, buf);
}

static DEVICE_ATTR(ccr2, S_IRUGO, ccr2_show, NULL);

static ssize_t ccr3_show(struct device *dev, struct device_attribute *attr, char *buf) {
  return sysfs_show(dev, TIM_CCR3, buf);
}

static DEVICE_ATTR(ccr3, S_IRUGO, ccr3_show, NULL);

static ssize_t ccr4_show(struct device *dev, struct device_attribute *attr, char *buf) {
  return sysfs_show(dev, TIM_CCR4, buf);
}

static DEVICE_ATTR(ccr4, S_IRUGO, ccr4_show, NULL);

static ssize_t events_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct stm32_pps *ddata = dev_get_drvdata(dev);

  return snprintf(buf, PAGE_SIZE, "%u\n", ddata->events);
}

static DEVICE_ATTR(events, S_IRUGO, events_show, NULL);

static ssize_t local_count_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct stm32_pps *ddata = dev_get_drvdata(dev);

  return snprintf(buf, PAGE_SIZE, "%llu\n", ddata->local_count);
}

static DEVICE_ATTR(local_count, S_IRUGO, local_count_show, NULL);

static struct attribute *attrs[] = {
  &dev_attr_cnt.attr,
  &dev_attr_ccr1.attr,
  &dev_attr_ccr2.attr,
  &dev_attr_ccr3.attr,
  &dev_attr_ccr4.attr,
  &dev_attr_events.attr,
  &dev_attr_pps_ts.attr,
  &dev_attr_count_at_interrupt.attr,
  &dev_attr_interrupt_delta.attr,
  &dev_attr_local_count.attr,
  NULL,
};

static struct attribute_group attr_group = {
  .attrs = attrs,
};

static irqreturn_t stm32_pps_irq(int irq, void *devdata) {
  struct system_time_snapshot snap;
  struct stm32_pps *ddata = devdata;
  u32 sr, tmp, cnt1, cnt2;
  u16 count_at_capture, delta_count, read_delta; // TODO: 32bit vs 16bit
  // TIM_SR_CC1IF..CC4IF
  u32 cc_event = 1 << ddata->channel;

  regmap_read(ddata->regmap, TIM_SR, &sr);
  if(sr & cc_event) {
    // measure the time it takes to read the clock to adjust for that latency
    regmap_read(ddata->regmap, TIM_CNT, &cnt1);
    regmap_read(ddata->regmap, TIM_CNT, &cnt2);
    ktime_get_snapshot(&snap);

    ddata->ts.ts_real = ktime_to_timespec64(snap.real);
#ifdef CONFIG_NTP_PPS
    ddata->ts.ts_raw = ktime_to_timespec64(snap.raw);
#endif
    ddata->local_count = snap.cycles;

    regmap_read(ddata->regmap, ddata->channel_reg_offset, &tmp);
    count_at_capture = tmp;

    ddata->count_at_interrupt[0] = cnt1;
    ddata->count_at_interrupt[1] = cnt2;
    ddata->events++;

    delta_count = (u16)cnt2 - count_at_capture;

    // assume ktime_get_snapshot takes its time reading at the same spot as regmap_read
    read_delta = cnt2-cnt1;
    delta_count += read_delta;

    // at 209MHz, 16bits has 313us before overflowing
    ddata->delta.tv_sec = 0;
    ddata->delta.tv_nsec = (delta_count * ddata->ps_per_hz) / 1000;

    // adjust interrupt timestamp for when the PPS actually happened
    pps_sub_ts(&ddata->ts, ddata->delta);

    if(ddata->ready && ddata->pps)
      pps_event(ddata->pps, &ddata->ts, PPS_CAPTUREASSERT, NULL);

    // clear the input capture flag
    regmap_update_bits(ddata->regmap, TIM_SR, cc_event, 0);

    return IRQ_HANDLED;
  }

  return IRQ_NONE;
}

static u8 stm32_pps_get_channel(struct device *dev) {
  const __be32 *val;
  u32 channel;

  val = of_get_property(dev->of_node, "channel", NULL);
  if(!val)
    return DEFAULT_CHANNEL;

  channel = be32_to_cpup(val);
  if(channel < 1 || channel > 4)
    return DEFAULT_CHANNEL;

  return channel;
}

static int stm32_pps_set_ic(struct stm32_pps *ddata) {
  u32 channel_mask, channel_value;
  u32 enable_value;
  u32 channel_register;
  u32 irq_enable;

  switch(ddata->channel) {
    case 1:
    default:
      enable_value = TIM_CCER_CC1E;
      channel_mask = TIM_CCMR_CC1S_TI1|TIM_CCMR_CC1S_TI2;
      channel_value = TIM_CCMR_CC1S_TI1;
      channel_register = TIM_CCMR1;
      irq_enable = TIM_DIER_CC1IE;
      ddata->channel_reg_offset = TIM_CCR1;
      break;
    case 2:
      enable_value = TIM_CCER_CC2E;
      channel_mask = TIM_CCMR_CC2S_TI1|TIM_CCMR_CC2S_TI2;
      channel_value = TIM_CCMR_CC2S_TI1;
      channel_register = TIM_CCMR1;
      irq_enable = TIM_DIER_CC2IE;
      ddata->channel_reg_offset = TIM_CCR2;
      break;
    case 3:
      enable_value = TIM_CCER_CC3E;
      channel_mask = TIM_CCMR_CC1S_TI1|TIM_CCMR_CC1S_TI2;
      channel_value = TIM_CCMR_CC1S_TI1;
      channel_register = TIM_CCMR2;
      irq_enable = TIM_DIER_CC3IE;
      ddata->channel_reg_offset = TIM_CCR3;
      break;
    case 4:
      enable_value = TIM_CCER_CC4E;
      channel_mask = TIM_CCMR_CC2S_TI1|TIM_CCMR_CC2S_TI2;
      channel_value = TIM_CCMR_CC2S_TI1;
      channel_register = TIM_CCMR2;
      irq_enable = TIM_DIER_CC4IE;
      ddata->channel_reg_offset = TIM_CCR4;
      break;
  }
  regmap_update_bits(ddata->regmap, TIM_CCER, enable_value, 0);
  regmap_update_bits(ddata->regmap, channel_register, channel_mask, channel_value);
  regmap_update_bits(ddata->regmap, TIM_CCER, enable_value, enable_value);
  regmap_update_bits(ddata->regmap, TIM_DIER, irq_enable, irq_enable);

  return 0;
}
static int stm32_pps_register(struct platform_device *pdev, struct stm32_pps *ddata) {
  ddata->info.mode = PPS_CAPTUREASSERT | PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
  ddata->info.owner = THIS_MODULE;
  snprintf(ddata->info.name, PPS_MAX_NAME_LEN - 1, "%s", pdev->name);

  ddata->pps = pps_register_source(&ddata->info, PPS_CAPTUREASSERT);
  if (ddata->pps == NULL) {
    dev_err(&pdev->dev, "failed to register PPS source\n");
    return -EINVAL;
  }

  return 0;
}

static int stm32_pps_probe(struct platform_device *pdev) {
  struct device *dev = &pdev->dev;
  struct stm32_timers *pdata = dev_get_drvdata(pdev->dev.parent);
  struct stm32_pps *ddata;
  struct pinctrl *pinctrl;
  int ret, irq;

  ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
  if (!ddata)
    return -ENOMEM;

  ddata->regmap = pdata->regmap;
  ddata->clk = pdata->clk;
  ddata->channel = stm32_pps_get_channel(dev);
  ddata->events = 0;
  ddata->ready = 0;

  // use picoseconds per hz to avoid floating point
  // adds under 2ns/10us error @ 208MHz
  ddata->ps_per_hz = 1000000000 / (clk_get_rate(pdata->clk) / 1000);

  platform_set_drvdata(pdev, ddata);

  ret = clk_enable(ddata->clk);
  if (ret) {
    dev_err(dev, "failed to enable counter clock\n");
    return ret;
  }

  irq = platform_get_irq(pdev, 0);
  if (irq <= 0) {
    dev_err(dev, "failed to get irq: %d\n", irq);
    return irq;
  }

  ret = devm_request_irq(dev, irq, stm32_pps_irq, 0, 
      pdev->name, ddata);
  if (ret) {
    dev_err(dev, "irq%d request failed: %d\n", irq, ret);
    return ret;
  }

  // start counter
  regmap_write(ddata->regmap, TIM_ARR, UINT_MAX);
  regmap_update_bits(ddata->regmap, TIM_CR1, TIM_CR1_CEN|TIM_CR1_UDIS, TIM_CR1_CEN|TIM_CR1_UDIS);

  stm32_pps_register(pdev, ddata);

  stm32_pps_set_ic(ddata);

  pinctrl = devm_pinctrl_get_select_default(dev);
  if (IS_ERR(pinctrl))
    dev_warn(dev, "failed to setup gpio pins\n");

  if(sysfs_create_group(&dev->kobj, &attr_group)) {
    dev_err(dev, "sysfs_create_group failed\n");
  }

  ddata->ready = 1;

  return 0;
}

static int stm32_pps_remove(struct platform_device *pdev) {
  struct stm32_pps *ddata = dev_get_drvdata(&pdev->dev);

  ddata->ready = 0;

  sysfs_remove_group(&pdev->dev.kobj, &attr_group);

  regmap_update_bits(ddata->regmap, TIM_DIER, 
      (TIM_DIER_CC1IE|TIM_DIER_CC2IE|TIM_DIER_CC3IE|TIM_DIER_CC4IE), 0);

  if(ddata->pps) {
    pps_unregister_source(ddata->pps);
    ddata->pps = NULL;
  }

  return 0;
}

static const struct of_device_id stm32_pps_of_match[] = {
  { .compatible = "st,stm32-timer-pps", },
  { /* end node */ },
};
MODULE_DEVICE_TABLE(of, stm32_pps_of_match);

static struct platform_driver stm32_pps_driver = {
  .probe	= stm32_pps_probe,
  .remove	= stm32_pps_remove,
  .driver	= {
    .name = "stm32-pps",
    .of_match_table = stm32_pps_of_match,
  },
};
module_platform_driver(stm32_pps_driver);

MODULE_AUTHOR("Daniel Drown <dan-android@drown.org>");
MODULE_ALIAS("platform:stm32-pps");
MODULE_DESCRIPTION("STM32 Timer Clocksource");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.0.1");
