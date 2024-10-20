#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "iqs7211a.h"

#define SW0_NODE DT_ALIAS(sw0)
#define I2C_NODE DT_NODELABEL(iqs7211a)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

void iqs7211a_ready_interrupt(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
  printf("Button is pressed\n");
}

void iqs7211a_begin()
{

  static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
  if (!device_is_ready(dev_i2c.bus))
  {
    printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
    return;
  }

  if (!gpio_is_ready_dt(&button))
  {
    return;
  }

  int ret;

  ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
  if (ret < 0)
  {
    return;
  }

  ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0)
  {
    return;
  }
  static struct gpio_callback pin_cb_data;
  (void)gpio_init_callback(&pin_cb_data, iqs7211a_ready_interrupt, BIT(button.pin));
  if (ret < 0)
  {
    return;
  }
  gpio_add_callback(button.port, &pin_cb_data);
}

/* Check if one of the gesture flags is set and display which one */
void printGesture(void)
{
  // iqs7211a_gestures_e tempGestures = get_gesture_event();

  // switch (tempGestures)
  // {
  //   case IQS7211A_GESTURE_SINGLE_TAP:
  //     printf("Single Tap\t\t");
  //   break;

  //   case IQS7211A_GESTURE_PRESS_HOLD:
  //     printf("Press and Hold\t\t");
  //   break;

  //   case IQS7211A_GESTURE_SWIPE_X_POSITIVE:
  //     printf("Swipe X +\t\t");
  //   break;

  //   case IQS7211A_GESTURE_SWIPE_X_NEGATIVE:
  //     printf("Swipe X -\t\t");
  //   break;

  //   case IQS7211A_GESTURE_SWIPE_Y_POSITIVE:
  //     printf("Swipe Y +\t\t");
  //   break;

  //   case IQS7211A_GESTURE_SWIPE_Y_NEGATIVE:
  //     printf("Swipe Y -\t\t");
  //   break;

  //   default:
  //     printf("-\t\t\t");
  //   break;
  }

int main(void)
{

  iqs7211a_begin();

  while (1)
  {
  }
}
