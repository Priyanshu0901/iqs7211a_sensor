#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "iqs7211a.h"

#define SW0_NODE DT_ALIAS(sw0)
#define I2C_NODE DT_NODELABEL(iqs7211a)

iqs7211a_dev iqs7211a_sensor;

const struct gpio_dt_spec intr_pin = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
const struct i2c_dt_spec i2c_handle = I2C_DT_SPEC_GET(I2C_NODE);

int main(void)
{
  iqs7211a_sensor.interrupt_pin = intr_pin;
  iqs7211a_sensor.i2c_handle = i2c_handle;

  iqs7211a_init(&iqs7211a_sensor);

  while (1)
  {
  }
}
