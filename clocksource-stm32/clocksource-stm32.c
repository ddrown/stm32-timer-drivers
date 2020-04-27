#include <linux/mfd/stm32-timers.h>
#include <linux/pinctrl/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/bits.h>
#include <linux/clocksource.h>

#define TIM_CR1_UDIS	 BIT(1)	/* UEV disabled */
#define TIM_SMCR_TS_TI1  BIT(6)|BIT(4)
#define TIM_SMCR_TS_TI2  BIT(6)|BIT(5)
#define TIM_SMCR_TS_ETRF BIT(6)|BIT(5)|BIT(4)
#define TIM_SMCR_ECE     BIT(14) // external clock mode 2
#define TIM_SMCR_SMS_EM1 BIT(2)|BIT(1)|BIT(0)

struct stm32_clocksource {
  struct regmap *regmap;
  struct clk *clk;
  struct clocksource clksrc;
  u32 rate;
  int channel;
};

static ssize_t sysfs_show(struct device *dev, int register_offset, char *buf) {
  u32 register_value = 0;

  struct stm32_clocksource *ddata = dev_get_drvdata(dev);
  regmap_read(ddata->regmap, register_offset, &register_value);

  return snprintf(buf, PAGE_SIZE, "%u\n", register_value);
}

static ssize_t cnt_show(struct device *dev, struct device_attribute *attr, char *buf) {
  return sysfs_show(dev, TIM_CNT, buf);
}

static DEVICE_ATTR(cnt, S_IRUGO, cnt_show, NULL);

static struct attribute *attrs[] = {
  &dev_attr_cnt.attr,
  NULL,
};

static struct attribute_group attr_group = {
  .attrs = attrs,
};

static inline struct stm32_clocksource *from_clocksource(struct clocksource *c)
{
  return container_of(c, struct stm32_clocksource, clksrc);
}

static u64 stm32_clocksource_read(struct clocksource *cs) {
  u32 cnt = 0;

  regmap_read(from_clocksource(cs)->regmap, TIM_CNT, &cnt);

  return (u64)cnt;
}

static void stm32_clocksource_init(struct platform_device *pdev) {
  struct stm32_clocksource *ddata = dev_get_drvdata(&pdev->dev);

  ddata->clksrc.name = pdev->name;
  ddata->clksrc.rating = 250;
  ddata->clksrc.read = stm32_clocksource_read;
  ddata->clksrc.mask = CLOCKSOURCE_MASK(32);
  ddata->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

  clocksource_register_hz(&ddata->clksrc, ddata->rate);
}

static int stm32_clocksource_get_channel(struct platform_device *pdev) {
  const __be32 *val;
  struct pinctrl *pinctrl;
  u32 channel;

  pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
  if (IS_ERR(pinctrl)) {
    dev_warn(&pdev->dev, "failed to setup gpio pins\n");
    return 0;
  }

  val = of_get_property(pdev->dev.of_node, "clock-source", NULL);
  if(!val) {
    dev_info(&pdev->dev, "using internal clock\n");
    return 0;
  }

  channel = be32_to_cpup(val);
  if(channel == 1 || channel == 2)
    return channel;

  dev_warn(&pdev->dev, "unsupported channel %u, using internal clock\n", channel);
  return 0;
}

static int stm32_clocksource_set_channel(struct platform_device *pdev, struct stm32_clocksource *ddata) {
  const __be32 *val;
  u32 rate;

  ddata->rate = clk_get_rate(ddata->clk);
  regmap_update_bits(ddata->regmap, TIM_SMCR, TIM_SMCR_SMS|TIM_SMCR_ECE, 0); // switch to internal clock

  if(ddata->channel == 0) {
    return 0;
  }

  val = of_get_property(pdev->dev.of_node, "clock-frequency", NULL);
  if(!val) {
    dev_warn(&pdev->dev, "no clock frequency set, using internal clock");
    return 0;
  }

  rate = be32_to_cpup(val);
  ddata->rate = rate;

  regmap_update_bits(ddata->regmap, TIM_SMCR, TIM_SMCR_TS, (ddata->channel == 1) ? TIM_SMCR_TS_TI1 : TIM_SMCR_TS_TI2);
  regmap_update_bits(ddata->regmap, TIM_SMCR, TIM_SMCR_SMS, TIM_SMCR_SMS_EM1);
  dev_info(&pdev->dev, "using channel %u, rate %u\n", ddata->channel, ddata->rate);

  return 0;
}

static int stm32_clocksource_probe(struct platform_device *pdev) {
  struct device *dev = &pdev->dev;
  struct stm32_timers *ddata = dev_get_drvdata(pdev->dev.parent);
  struct stm32_clocksource *priv;
  int ret;

  priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
  if (!priv)
    return -ENOMEM;

  priv->regmap = ddata->regmap;
  priv->clk = ddata->clk;

  ret = clk_enable(priv->clk);
  if (ret) {
    dev_err(dev, "failed to enable counter clock\n");
    return ret;
  }

  platform_set_drvdata(pdev, priv);

  priv->channel = stm32_clocksource_get_channel(pdev);
  stm32_clocksource_set_channel(pdev, priv);

  // start counter
  regmap_write(priv->regmap, TIM_ARR, UINT_MAX);
  regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN|TIM_CR1_UDIS, TIM_CR1_CEN|TIM_CR1_UDIS);

  stm32_clocksource_init(pdev);

  if(sysfs_create_group(&dev->kobj, &attr_group)) {
    dev_err(dev, "sysfs_create_group failed\n");
  }

  return 0;
}

static int stm32_clocksource_remove(struct platform_device *pdev) {
  struct stm32_clocksource *ddata = dev_get_drvdata(&pdev->dev);

  clocksource_unregister(&ddata->clksrc);

  sysfs_remove_group(&pdev->dev.kobj, &attr_group);

  // stop counter
  regmap_update_bits(ddata->regmap, TIM_CR1, TIM_CR1_CEN, 0);

  return 0;
}

static const struct of_device_id stm32_clocksource_of_match[] = {
  { .compatible = "st,stm32-timer-clocksource", },
  { /* end node */ },
};
MODULE_DEVICE_TABLE(of, stm32_clocksource_of_match);

static struct platform_driver stm32_clocksource_driver = {
  .probe	= stm32_clocksource_probe,
  .remove	= stm32_clocksource_remove,
  .driver	= {
    .name = "stm32-clocksource",
    .of_match_table = stm32_clocksource_of_match,
  },
};
module_platform_driver(stm32_clocksource_driver);

MODULE_AUTHOR("Daniel Drown <dan-android@drown.org>");
MODULE_ALIAS("platform:stm32-clocksource");
MODULE_DESCRIPTION("STM32 Timer Clocksource");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.0.1");
