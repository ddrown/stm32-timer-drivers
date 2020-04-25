#include <linux/mfd/stm32-timers.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/bits.h>
#include <linux/clocksource.h>

#define TIM_CR1_UDIS	BIT(1)	/* UEV disabled */

struct stm32_clocksource {
  struct regmap *regmap;
  struct clk *clk;
  struct clocksource clksrc;
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
  u32 rate;
  struct stm32_clocksource *ddata = dev_get_drvdata(&pdev->dev);

  rate = clk_get_rate(ddata->clk);

  ddata->clksrc.name = pdev->name;
  ddata->clksrc.rating = 250;
  ddata->clksrc.read = stm32_clocksource_read;
  ddata->clksrc.mask = CLOCKSOURCE_MASK(32);
  ddata->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

  clocksource_register_hz(&ddata->clksrc, rate);
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

  // start counter
  regmap_write(priv->regmap, TIM_ARR, UINT_MAX);
  regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN|TIM_CR1_UDIS, TIM_CR1_CEN|TIM_CR1_UDIS);

  stm32_clocksource_init(pdev);

  return 0;
}

static int stm32_clocksource_remove(struct platform_device *pdev) {
  struct stm32_clocksource *ddata = dev_get_drvdata(&pdev->dev);

  clocksource_unregister(&ddata->clksrc);

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
