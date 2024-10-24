#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "iqs7211a.h"

#define SW0_NODE DT_ALIAS(sw0)
#define I2C_NODE DT_NODELABEL(iqs7211a)

iqs7211a_dev iqs7211a_sensor;
iqs7211a_power_modes running_power_mode = IQS7211A_IDLE;
uint16_t running_x_output = 65535;
uint16_t running_y_output = 65535;

const struct gpio_dt_spec intr_pin = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
const struct i2c_dt_spec i2c_handle = I2C_DT_SPEC_GET(I2C_NODE);

void printPowerMode(void);
void printHeading(void);
bool printData(void);
void printCoordinates(void);

int main(void)
{
  iqs7211a_sensor.interrupt_pin = intr_pin;
  iqs7211a_sensor.i2c_handle = i2c_handle;

  iqs7211a_init();

  while (1)
  {
    iqs7211a_run();
    if (iqs7211a_sensor.new_data_available)
  {

      if (printData())
      {
        printCoordinates();
        printPowerMode();
      }
    /* Display the heading when a finger is lifted from the trackpad */
    printHeading();
    /* Set this flag to false to indicate that the new data was already displayed/used */
    iqs7211a_sensor.new_data_available = false;

    k_msleep(100);
  }
}
}

/* Check Power mode and print out the current power mode */
void printPowerMode(void)
{
  iqs7211a_power_modes buffer = IQS7211A_getPowerMode(); // read current power mode

  switch (buffer)
  {
    case IQS7211A_ACTIVE:
      printf("Active\n");
      break;

    case IQS7211A_IDLE_TOUCH:
      printf("Idle-Touch\n");
      break;

    case IQS7211A_IDLE:
      printf("Idle\n");
      break;

    case IQS7211A_LP1:
      printf("Low Power 1\n");
      break;

    case IQS7211A_LP2:
      printf("Low Power 2\n");
      break;

    default:
      printf("Unknown\n");
      break;
    }

    /* Update the running state */
    running_power_mode = buffer;
}

/** Function to print heading on the release of a previous event
 *  See if there are no fingers on the trackpad
 */
void printHeading(void)
{
  /* Check if it is necessary to display the heading */
  if (IQS7211A_getNumFingers() == 0)
  {
    printf("\nGesture:\t\tFinger 1 X:\tFinger 1 Y:\tPower Mode:\n");
  }
}

/* Function to determine if it is necessary to print all the relevant data 
in the serial terminal */
bool printData(void)
{
/* See if it is necessary to display the button state, power mode or gestures */
  if(
      (
        (IQS7211A_getPowerMode()              != running_power_mode)
      ||(IQS7211A_getAbsXCoordinate(FINGER_1) != running_x_output)
      ||(IQS7211A_getAbsYCoordinate(FINGER_1) != running_y_output)
      )
    )
  {
    return true; // Let the main loop know to display the latest values
  }
  else
  {
    return false; // Let the main loop know to not display the latest values
  }

  return false;
}

/* Function to return X and Y coordinates of finger 1 */
void printCoordinates(void)
{
  uint16_t ui16TempX = IQS7211A_getAbsXCoordinate(FINGER_1);
  uint16_t ui16TempY = IQS7211A_getAbsYCoordinate(FINGER_1);

  printf("%u\t\t%u\t\t",ui16TempX,ui16TempY);

  running_x_output = ui16TempX;
  running_y_output = ui16TempY;
}

