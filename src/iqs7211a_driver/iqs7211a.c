#include "iqs7211a.h"
#include "board_param.h"
#include <zephyr/sys/printk.h>

int iqs7211a_init();
void iqs7211a_run();
iqs7211a_power_modes IQS7211A_getPowerMode(void);
uint8_t IQS7211A_getNumFingers(void);

int iqs7211a_start_up();
int iqs7211a_read(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle);
int iqs7211a_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle);
void iqs7211a_ready_interrupt(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pin);
bool checkReset();

void IQS7211A_writeMM();
uint8_t IQS7211A_setBit(uint8_t data, uint8_t bit_number);
bool IQS7211A_getBit(uint8_t data, uint8_t bit_number);
uint8_t IQS7211A_clearBit(uint8_t data, uint8_t bit_number);
void IQS7211A_enableTPEvent();
void IQS7211A_disableTPEvent();
uint16_t IQS7211A_getAbsXCoordinate(uint8_t fingerNum);
uint16_t IQS7211A_getAbsYCoordinate(uint8_t fingerNum);
void IQS7211A_updateAbsCoordinates(uint8_t fingerNum);
void IQS7211A_SW_Reset();
void IQS7211A_setEventMode();
void IQS7211A_updateInfoFlags();
void IQS7211A_acknowledgeReset();
bool IQS7211A_checkReset();
void IQS7211A_queueValueUpdates();
bool IQS7211A_readATIactive();
void IQS7211A_ReATI();


int iqs7211a_init()
{

  struct i2c_dt_spec dev_i2c_handle = iqs7211a_sensor.i2c_handle;
  struct gpio_dt_spec interrupt_pin = iqs7211a_sensor.interrupt_pin;

  if (!device_is_ready(dev_i2c_handle.bus))
  {
    printk("I2C bus %s is not ready!\n", dev_i2c_handle.bus->name);
    return -1;
  }

  if (!gpio_is_ready_dt(&interrupt_pin))
  {
    return -1;
  }

  int ret;

  ret = gpio_pin_configure_dt(&interrupt_pin, GPIO_INPUT);
  if (ret < 0)
  {
    return -1;
  }

  ret = gpio_pin_interrupt_configure_dt(&interrupt_pin, GPIO_INT_EDGE_BOTH);
  if (ret < 0)
  {
    return -1;
  }
  static struct gpio_callback pin_cb_data;
  (void)gpio_init_callback(&pin_cb_data, iqs7211a_ready_interrupt, BIT(interrupt_pin.pin));
  if (ret < 0)
  {
    return -1;
  }
  gpio_add_callback(interrupt_pin.port, &pin_cb_data);
  iqs7211a_sensor.iqs7211a_deviceRDY = false;

  iqs7211a_sensor.dev_state.state      = IQS7211A_STATE_START;
  iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_VERIFY_PRODUCT;
  return 1;
}

void iqs7211a_ready_interrupt(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pin)
{
  printf("Interrupt Received\n");
  // Read the state of the interrupt pin
  int pin_state = gpio_pin_get(dev, pin);

  // Check the state and update iqs7211a_deviceRDY
  if (pin_state > 0) {
    iqs7211a_sensor.iqs7211a_deviceRDY = false;
      printf("Rising edge\n");
  } else {
    iqs7211a_sensor.iqs7211a_deviceRDY = true;
      printf("Falling edge\n");
  }
}

void iqs7211a_run()
{
  switch (iqs7211a_sensor.dev_state.state)
  {
  /* After a hardware reset, this is the starting position of the running state
     machine */
  case IQS7211A_STATE_START:
    printf("IQS7211A Initialization:");
    iqs7211a_sensor.dev_state.state = IQS7211A_STATE_INIT;
    break;

  /* Perform the initialization routine on the IQS7211A */
  case IQS7211A_STATE_INIT:
    if(iqs7211a_start_up())
    {
      printf("IQS7211A Initialization complete!\n");
      iqs7211a_sensor.dev_state.state = IQS7211A_STATE_RUN;
    }
    break;

  /* Send an I2C software reset in the next RDY window */
  case IQS7211A_STATE_SW_RESET:
    if (iqs7211a_sensor.iqs7211a_deviceRDY)
    {
      IQS7211A_SW_Reset();
      iqs7211a_sensor.dev_state.state = IQS7211A_STATE_RUN;
    }
    break;

  /* Continuous reset monitoring state, ensure no reset event has occurred
    for data to be valid */
  case IQS7211A_STATE_CHECK_RESET:
    if (checkReset())
    {
      printf("Reset occurred!\n");
      iqs7211a_sensor.new_data_available = false;
      iqs7211a_sensor.dev_state.state = IQS7211A_STATE_START;
    }
    /* A reset did not occur, move to the run state and wait for a new RDY window */
    else
    {
      iqs7211a_sensor.new_data_available = true; /* No reset, thus data is valid */
      iqs7211a_sensor.dev_state.state = IQS7211A_STATE_RUN;
    }
    break;

  /* If a RDY Window is open, read the latest values from the IQS7211A */
  case IQS7211A_STATE_RUN:
    if (iqs7211a_sensor.iqs7211a_deviceRDY)
    {
      // queueValueUpdates();
      iqs7211a_sensor.iqs7211a_deviceRDY = false;
      // new_data_available = false;
      iqs7211a_sensor.dev_state.state = IQS7211A_STATE_CHECK_RESET;
    }
    break;
  default:
    break;
  }
}

bool checkReset()
{
  /* Perform a bitwise AND operation inside the IQS7211A_getBit() function with the
  SHOW_RESET_BIT to return the reset status */
  return false; // IQS7211A_getBit(iqs7211a_dev->IQS_memory_map.INFO_FLAGS[0], IQS7211A_SHOW_RESET_BIT);
}

int iqs7211a_start_up()
{
  switch (iqs7211a_sensor.dev_state.init_state)
  {
  /* Verifies product number to determine if the correct device is connected
  for this example */
  case IQS7211A_INIT_VERIFY_PRODUCT:
  if(iqs7211a_sensor.iqs7211a_deviceRDY){
    struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
    int ret;

    uint8_t data[2];
    memset(data, 0, 2);

    ret = iqs7211a_read(IQS7211A_MM_PROD_NUM, data, 2, &i2c_handle);
    if (ret < 0)
    {
      return -1;
    }
    uint16_t productNumber = (uint16_t)data[0] | (uint16_t)(data[1] << 8);

    memset(data, 0, 2);

    ret = iqs7211a_read(IQS7211A_MM_MAJOR_VERSION_NUM, data, 2, &i2c_handle);
    if (ret < 0)
    {
      return -1;
    }
    uint8_t ver_maj = data[0];

    memset(data, 0, 2);

    ret = iqs7211a_read(IQS7211A_MM_MINOR_VERSION_NUM, data, 2, &i2c_handle);
    if (ret < 0)
    {
      return -1;
    }
    uint8_t ver_min = data[0];
    printf("\t\tProduct number is: %d v%d.%d\n", productNumber, ver_maj, ver_min);

    if (productNumber == IQS7211A_PRODUCT_NUM)
    {
      printf("\t\tIQS7211A Release UI Confirmed!\n");
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_READ_RESET;
    }
    else
    {
      printf("\t\tDevice is not a IQS7211A!\n");
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_NONE;
    }
    }

    break;

  /* Verify if a reset has occurred */
  case IQS7211A_INIT_READ_RESET:
  if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_READ_RESET");
      IQS7211A_updateInfoFlags();
      if (checkReset())
      {
        printf("\t\tReset event occurred.\n");
        iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_UPDATE_SETTINGS;
      }
      else
      {
        printf("\t\t No Reset Event Detected - Request SW Reset\n");
        iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_CHIP_RESET;
      }}
    break;

  /* Perform SW Reset */
  case IQS7211A_INIT_CHIP_RESET:
  if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_CHIP_RESET\n");

      // Perform SW Reset
      IQS7211A_SW_Reset();
      printf("\t\tSoftware Reset Bit Set.\n");
      k_msleep(100);
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_READ_RESET;
  }
    break;

  /* Write all settings to IQS7211A from .h file */
  case IQS7211A_INIT_UPDATE_SETTINGS:
  if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_UPDATE_SETTINGS\n");
      IQS7211A_writeMM();
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_ACK_RESET;
    }
    break;

  /* Acknowledge that the device went through a reset */
  case IQS7211A_INIT_ACK_RESET:
    if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_ACK_RESET\n");
      IQS7211A_acknowledgeReset();
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_ATI;
    }
    break;

  /* Run the ATI algorithm to recalibrate the device with newly added settings */
  case IQS7211A_INIT_ATI:
    if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_ATI\n");
      IQS7211A_ReATI();
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_WAIT_FOR_ATI;
      printf("\tIQS7211A_INIT_WAIT_FOR_ATI\n");
    }
    break;

  /* Read the ATI Active bit to see if the rest of the program can continue */
  case IQS7211A_INIT_WAIT_FOR_ATI:
    if(iqs7211a_sensor.iqs7211a_deviceRDY){
      if (!IQS7211A_readATIactive())
      {
        printf("\t\tDONE");
        iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_READ_DATA;
      }
    }
    break;

  /* Read the latest data from the iqs7211a */
  case IQS7211A_INIT_READ_DATA:
    if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_READ_DATA\n");
      IQS7211A_queueValueUpdates();
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_ACTIVATE_EVENT_MODE;
    }
    break;

  /* Turn on I2C Event mode */
  case IQS7211A_INIT_ACTIVATE_EVENT_MODE:
    if(iqs7211a_sensor.iqs7211a_deviceRDY){
      printf("\tIQS7211A_INIT_ACTIVATE_EVENT_MODE\n");
      IQS7211A_setEventMode();
      iqs7211a_sensor.dev_state.init_state = IQS7211A_INIT_DONE;
    }
    break;

  /* Turn on I2C Stream mode */
  case IQS7211A_INIT_ACTIVATE_STREAM_MODE:
    {
    }
    break;

  /* If all operations have been completed correctly, the RDY pin can be set
   * up as an interrupt to indicate when new data is available */
  case IQS7211A_INIT_DONE:
    printf("\tIQS7211A_INIT_DONE\n");
    iqs7211a_sensor.new_data_available = true;
    return true;
    break;

  default:
    break;
  }
  return false;
}

void IQS7211A_writeMM()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  // To put reset

  uint8_t transferBytes[30]; // Temporary array which holds the bytes to be transferred.
  memset(transferBytes,0,sizeof(transferBytes));

  /* Change the ATI Settings */
  /* Memory Map Position 0x30 - 0x3D */
  transferBytes[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[2] = TP_COMPENSATION_DIV_0;
  transferBytes[3] = TP_COMPENSATION_DIV_1;
  transferBytes[4] = TP_ATI_TARGET_0;
  transferBytes[5] = TP_ATI_TARGET_1;
  transferBytes[6] = TP_REF_DRIFT_LIMIT_0;
  transferBytes[7] = TP_REF_DRIFT_LIMIT_1;
  transferBytes[8] = TP_MIN_COUNT_REATI_0;
  transferBytes[9] = TP_MIN_COUNT_REATI_1;
  transferBytes[10] = REATI_RETRY_TIME_0;
  transferBytes[11] = REATI_RETRY_TIME_1;
  transferBytes[12] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[13] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[14] = ALP_COMPENSATION_DIV_0;
  transferBytes[15] = ALP_COMPENSATION_DIV_1;
  transferBytes[16] = ALP_ATI_TARGET_0;
  transferBytes[17] = ALP_ATI_TARGET_1;
  transferBytes[18] = ALP_LTA_DRIFT_LIMIT_0;
  transferBytes[19] = ALP_LTA_DRIFT_LIMIT_1;
  printf("\t\t1. Write ATI Settings\n");

  /* Change the ALP ATI Compensation */
  /* Memory Map Position 0x3A - 0x3D */
  transferBytes[20] = ALP_COMPENSATION_A_0;
  transferBytes[21] = ALP_COMPENSATION_A_1;
  transferBytes[22] = ALP_COMPENSATION_B_0;
  transferBytes[23] = ALP_COMPENSATION_B_1;
  iqs7211a_write(IQS7211A_MM_TP_ATI_MIR, transferBytes, 24, &i2c_handle);
  printf("\t\t2. Write ALP Compensation Settings\n");

  /* Change the Report Rates and Timing */
  /* Memory Map Position 0x40 - 0x4A */
  transferBytes[0] = ACTIVE_MODE_REPORT_RATE_0;
  transferBytes[1] = ACTIVE_MODE_REPORT_RATE_1;
  transferBytes[2] = IDLE_TOUCH_MODE_REPORT_RATE_0;
  transferBytes[3] = IDLE_TOUCH_MODE_REPORT_RATE_1;
  transferBytes[4] = IDLE_MODE_REPORT_RATE_0;
  transferBytes[5] = IDLE_MODE_REPORT_RATE_1;
  transferBytes[6] = LP1_MODE_REPORT_RATE_0;
  transferBytes[7] = LP1_MODE_REPORT_RATE_1;
  transferBytes[8] = LP2_MODE_REPORT_RATE_0;
  transferBytes[9] = LP2_MODE_REPORT_RATE_1;
  transferBytes[10] = ACTIVE_MODE_TIMEOUT_0;
  transferBytes[11] = ACTIVE_MODE_TIMEOUT_1;
  transferBytes[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
  transferBytes[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
  transferBytes[14] = IDLE_MODE_TIMEOUT_0;
  transferBytes[15] = IDLE_MODE_TIMEOUT_1;
  transferBytes[16] = LP1_MODE_TIMEOUT_0;
  transferBytes[17] = LP1_MODE_TIMEOUT_1;
  transferBytes[18] = REF_UPDATE_TIME_0;
  transferBytes[19] = REF_UPDATE_TIME_1;
  transferBytes[20] = I2C_TIMEOUT_0;
  transferBytes[21] = I2C_TIMEOUT_1;
  iqs7211a_write(IQS7211A_MM_ACTIVE_MODE_RR, transferBytes, 22, &i2c_handle);
  printf("\t\t3. Write Report rates and timings\n");

  /* Change the System Settings */
  /* Memory Map Position 0x50 - 0x5B */
  transferBytes[0] = SYSTEM_CONTROL_0;
  transferBytes[1] = SYSTEM_CONTROL_1;
  transferBytes[2] = CONFIG_SETTINGS0;
  transferBytes[3] = CONFIG_SETTINGS1;
  transferBytes[4] = OTHER_SETTINGS_0;
  transferBytes[5] = OTHER_SETTINGS_1;
  transferBytes[6] = TRACKPAD_TOUCH_SET_THRESHOLD;
  transferBytes[7] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
  transferBytes[8] = ALP_THRESHOLD_0;
  transferBytes[9] = ALP_THRESHOLD_1;
  transferBytes[10] = OPEN_0_0;
  transferBytes[11] = OPEN_0_1;
  transferBytes[12] = ALP_SET_DEBOUNCE;
  transferBytes[13] = ALP_CLEAR_DEBOUNCE;
  transferBytes[14] = OPEN_1_0;
  transferBytes[15] = OPEN_1_1;
  transferBytes[16] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[17] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[18] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[19] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[20] = TRACKPAD_HARDWARE_SETTINGS_0;
  transferBytes[21] = TRACKPAD_HARDWARE_SETTINGS_1;
  transferBytes[22] = ALP_HARDWARE_SETTINGS_0;
  transferBytes[23] = ALP_HARDWARE_SETTINGS_1;
  iqs7211a_write(IQS7211A_MM_SYSTEM_CONTROL, transferBytes, 24, &i2c_handle);
  printf("\t\t4. Write System control settings\n");

  /* Change the Trackpad Settings */
  /* Memory Map Position 0x60 - 0x69 */
  transferBytes[0] = TRACKPAD_SETTINGS_0_0;
  transferBytes[1] = TRACKPAD_SETTINGS_0_1;
  transferBytes[2] = TRACKPAD_SETTINGS_1_0;
  transferBytes[3] = TRACKPAD_SETTINGS_1_1;
  transferBytes[4] = X_RESOLUTION_0;
  transferBytes[5] = X_RESOLUTION_1;
  transferBytes[6] = Y_RESOLUTION_0;
  transferBytes[7] = Y_RESOLUTION_1;
  transferBytes[8] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
  transferBytes[9] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
  transferBytes[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
  transferBytes[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
  transferBytes[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
  transferBytes[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
  transferBytes[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
  transferBytes[15] = FINGER_SPLIT_FACTOR;
  transferBytes[16] = X_TRIM_VALUE_0;
  transferBytes[17] = X_TRIM_VALUE_1;
  transferBytes[18] = Y_TRIM_VALUE_0;
  transferBytes[19] = Y_TRIM_VALUE_1;
  iqs7211a_write(IQS7211A_MM_TP_SETTINGS_0, transferBytes, 20, &i2c_handle);
  printf("\t\t5. Write Hardware settings\n");

  /* Change the ALP Settings */
  /* Memory Map Position 0x70 - 0x74 */
  transferBytes[0] = ALP_COUNT_FILTER_BETA_0;
  transferBytes[1] = OPEN_0;
  transferBytes[2] = ALP_LTA_BETA_LP1;
  transferBytes[3] = ALP_LTA_BETA_LP2;
  transferBytes[4] = ALP_SETUP_0;
  transferBytes[5] = ALP_SETUP_1;
  transferBytes[6] = ALP_TX_ENABLE_0;
  transferBytes[7] = ALP_TX_ENABLE_1;

  /* Change the Settings Version Numbers */
  /* Memory Map Position 0x74 - 0x75 */
  transferBytes[8] = MINOR_VERSION;
  transferBytes[9] = MAJOR_VERSION;
  iqs7211a_write(IQS7211A_MM_ALP_COUNT_FILTER_BETA, transferBytes, 10, &i2c_handle);
  printf("\t\t5. Write Filter Betas\n");

  /* Change the Gesture Settings */
  /* Memory Map Position 0x80 - 0x8F */
  transferBytes[0] = GESTURE_ENABLE_0;
  transferBytes[1] = GESTURE_ENABLE_1;
  transferBytes[2] = TAP_TIME_0;
  transferBytes[3] = TAP_TIME_1;
  transferBytes[4] = TAP_DISTANCE_0;
  transferBytes[5] = TAP_DISTANCE_1;
  transferBytes[6] = HOLD_TIME_0;
  transferBytes[7] = HOLD_TIME_1;
  transferBytes[8] = SWIPE_TIME_0;
  transferBytes[9] = SWIPE_TIME_1;
  transferBytes[10] = SWIPE_X_DISTANCE_0;
  transferBytes[11] = SWIPE_X_DISTANCE_1;
  transferBytes[12] = SWIPE_Y_DISTANCE_0;
  transferBytes[13] = SWIPE_Y_DISTANCE_1;
  transferBytes[14] = SWIPE_ANGLE_0;
  transferBytes[15] = GESTURE_OPEN_0;
  iqs7211a_write(IQS7211A_MM_GESTURE_ENABLE, transferBytes, 16, &i2c_handle);
  printf("\t\t6. Write Gesture Settings\n");

  /* Change the RxTx Mapping */
  /* Memory Map Position 0x90 - 0x9C */
  transferBytes[0] = RX_TX_MAP_0;
  transferBytes[1] = RX_TX_MAP_1;
  transferBytes[2] = RX_TX_MAP_2;
  transferBytes[3] = RX_TX_MAP_3;
  transferBytes[4] = RX_TX_MAP_4;
  transferBytes[5] = RX_TX_MAP_5;
  transferBytes[6] = RX_TX_MAP_6;
  transferBytes[7] = RX_TX_MAP_7;
  transferBytes[8] = RX_TX_MAP_8;
  transferBytes[9] = RX_TX_MAP_9;
  transferBytes[10] = RX_TX_MAP_10;
  transferBytes[11] = RX_TX_MAP_11;
  transferBytes[12] = RX_TX_MAP_12;
  iqs7211a_write(IQS7211A_MM_RXTX_MAPPING_1_0, transferBytes, 13, &i2c_handle);
  printf("\t\t7. Write TP Settings\n");

  /* Change the Allocation of channels into cycles 0-9 */
  /* Memory Map Position 0xA0 - 0xBD */
  transferBytes[0] = PLACEHOLDER_0;
  transferBytes[1] = CH_1_CYCLE_0;
  transferBytes[2] = CH_2_CYCLE_0;
  transferBytes[3] = PLACEHOLDER_1;
  transferBytes[4] = CH_1_CYCLE_1;
  transferBytes[5] = CH_2_CYCLE_1;
  transferBytes[6] = PLACEHOLDER_2;
  transferBytes[7] = CH_1_CYCLE_2;
  transferBytes[8] = CH_2_CYCLE_2;
  transferBytes[9] = PLACEHOLDER_3;
  transferBytes[10] = CH_1_CYCLE_3;
  transferBytes[11] = CH_2_CYCLE_3;
  transferBytes[12] = PLACEHOLDER_4;
  transferBytes[13] = CH_1_CYCLE_4;
  transferBytes[14] = CH_2_CYCLE_4;
  transferBytes[15] = PLACEHOLDER_5;
  transferBytes[16] = CH_1_CYCLE_5;
  transferBytes[17] = CH_2_CYCLE_5;
  transferBytes[18] = PLACEHOLDER_6;
  transferBytes[19] = CH_1_CYCLE_6;
  transferBytes[20] = CH_2_CYCLE_6;
  transferBytes[21] = PLACEHOLDER_7;
  transferBytes[22] = CH_1_CYCLE_7;
  transferBytes[23] = CH_2_CYCLE_7;
  transferBytes[24] = PLACEHOLDER_8;
  transferBytes[25] = CH_1_CYCLE_8;
  transferBytes[26] = CH_2_CYCLE_8;
  transferBytes[27] = PLACEHOLDER_9;
  transferBytes[28] = CH_1_CYCLE_9;
  transferBytes[29] = CH_2_CYCLE_9;
  iqs7211a_write(IQS7211A_MM_CYCLE_SETUP_0_9, transferBytes, 30, &i2c_handle);
  printf("\t\t8. Write Cycle 0 - 9 Settings\n");

  /* Change the Allocation of channels into cycles 10-17 */
  /* Memory Map Position 0xB0 - 0xCA */
  transferBytes[0] = PLACEHOLDER_10;
  transferBytes[1] = CH_1_CYCLE_10;
  transferBytes[2] = CH_2_CYCLE_10;
  transferBytes[3] = PLACEHOLDER_11;
  transferBytes[4] = CH_1_CYCLE_11;
  transferBytes[5] = CH_2_CYCLE_11;
  transferBytes[6] = PLACEHOLDER_12;
  transferBytes[7] = CH_1_CYCLE_12;
  transferBytes[8] = CH_2_CYCLE_12;
  transferBytes[9] = PLACEHOLDER_13;
  transferBytes[10] = CH_1_CYCLE_13;
  transferBytes[11] = CH_2_CYCLE_13;
  transferBytes[12] = PLACEHOLDER_14;
  transferBytes[13] = CH_1_CYCLE_14;
  transferBytes[14] = CH_2_CYCLE_14;
  transferBytes[15] = PLACEHOLDER_15;
  transferBytes[16] = CH_1_CYCLE_15;
  transferBytes[17] = CH_2_CYCLE_15;
  transferBytes[18] = PLACEHOLDER_16;
  transferBytes[19] = CH_1_CYCLE_16;
  transferBytes[20] = CH_2_CYCLE_16;
  transferBytes[21] = PLACEHOLDER_17;
  transferBytes[22] = CH_1_CYCLE_17;
  transferBytes[23] = CH_2_CYCLE_17;
  iqs7211a_write(IQS7211A_MM_CYCLE_SETUP_10_17, transferBytes, 24, &i2c_handle);
  printf("\t\t9. Write Cycle 10 - 17 Settings\n");

}

int iqs7211a_read(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle)
{
  int ret = i2c_write_read(i2c_handle->bus, i2c_handle->addr, (void *)&reg_addr, 1, (void *)reg_data, length);
#ifdef I2C_LINE_DEBUG

  printf("Read request\tSending from HAL:\tReg_addr : 0x%02x ,Command Length : %u\t\nRead Data :\t", reg_addr, length);

  for (int i = 0; i < length; i++)
  {
    printf("0x%02x ", reg_data[i]);
  }
  printf("\n");

#endif
  return ret;
}

int iqs7211a_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle)
{
  uint8_t buffer[length + 1];
  buffer[0] = reg_addr;
  memcpy(&buffer[1], reg_data, length);

  int ret = i2c_write(i2c_handle->bus, buffer, sizeof(buffer), i2c_handle->addr);
#ifdef I2C_LINE_DEBUG
  printf("Write request\tSending from HAL:\tReg_addr : 0x%02x,Command Length : %u\t\nFull_Data : ", reg_addr, length);
  for (int i = 0; i < sizeof(buffer); i++)
  {
    printf("0x%02x ", buffer[i]);
  }
  printf("\n");
#endif
  return ret;
}

/**
 * @name   IQS7211A_setBit
 * @brief  A method that returns the chosen bit value of the provided byte.
 * @param  data       -> byte of which a given bit value needs to be calculated.
 * @param  bit_number -> a number between 0 and 7 representing the bit in question.
 * @retval Returns an 8-bit unsigned integer value of the given data byte with
 *         the requested bit set.
 */
uint8_t IQS7211A_setBit(uint8_t data, uint8_t bit_number)
{
  return (data |= 1UL << bit_number);
}

/**
  * @name   IQS7211A_getBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval The boolean value of the specific bit requested. 
  */
bool IQS7211A_getBit(uint8_t data, uint8_t bit_number)
{
  return (data & ( 1 << bit_number )) >> bit_number;
}
/**
 * @name   IQS7211A_clearBit
 * @brief  A method that returns the chosen bit value of the provided byte.
 * @param  data       -> byte of which a given bit value needs to be calculated.
 * @param  bit_number -> a number between 0 and 7 representing the bit in question.
 * @retval Returns an 8-bit unsigned integer value of the given data byte with
 *         the requested bit cleared.
 */
uint8_t IQS7211A_clearBit(uint8_t data, uint8_t bit_number)
{
  return (data &= ~(1UL << bit_number));
}

/**
 * @name   enableTPEvent
 * @brief  A method to enable TP Events. Trackpad finger movement or finger
 *         up/down will trigger an event.
 * @param  stopOrRestart -> Specifies whether the communications window must
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   All other bits at the IQS7211A_MM_CONFIG_SETTINGS register address
 *         are preserved.
 */
void IQS7211A_enableTPEvent()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
  memset(transferBytes,0,sizeof(transferBytes));

  // First read the bytes at the memory address so that they can be preserved.
  iqs7211a_read(IQS7211A_MM_CONFIG_SETTINGS, transferBytes, 2, &i2c_handle);
  // Set the TP_EVENT_BIT in CONFIG_SETTINGS
  transferBytes[1] = IQS7211A_setBit(transferBytes[1], IQS7211A_TP_EVENT_BIT);
  // Write the bytes back to the device
  iqs7211a_write(IQS7211A_MM_CONFIG_SETTINGS, transferBytes, 2, &i2c_handle);
}

/**
 * @name   disableTPEvent
 * @brief  A method to disable TP Events. Trackpad finger movement or finger
 *         up/down will not trigger an event.
 * @param  stopOrRestart -> Specifies whether the communications window must
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   All other bits at the IQS7211A_MM_CONFIG_SETTINGS register address
 *         are preserved.
 */
void IQS7211A_disableTPEvent()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
  memset(transferBytes,0,sizeof(transferBytes));

  // First read the bytes at the memory address so that they can be preserved.
  iqs7211a_read(IQS7211A_MM_CONFIG_SETTINGS, transferBytes, 2, &i2c_handle);
  // Clear the TP_EVENT_BIT in CONFIG_SETTINGS
  transferBytes[1] = IQS7211A_clearBit(transferBytes[1], IQS7211A_TP_EVENT_BIT);
  // Write the bytes back to the device
  iqs7211a_write(IQS7211A_MM_CONFIG_SETTINGS, transferBytes, 2, &i2c_handle);
}

/**
 * @name	  getAbsXCoordinate
 * @brief   A method that returns the constructed 16bit coordinate value
 * @param   fingerNum     ->  Specifies the finger number. Finger 1 is the
 *                            first finger to touch the trackpad, finger 2 is
 *                            the second to touch the trackpad.
 * @retval  Returns 16-bit coordinate value.
 */
uint16_t IQS7211A_getAbsXCoordinate(uint8_t fingerNum)
{
  /* The 16-bit return value. */
  uint16_t absXCoordReturn = 0;

  /* Construct the 16-bit return value. */
  if (fingerNum == FINGER_1)
  {
    absXCoordReturn = (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_1_X[0]);
    absXCoordReturn |= (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_1_X[1] << 8);
  }
  else if (fingerNum == FINGER_2)
  {
    absXCoordReturn = (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_2_X[0]);
    absXCoordReturn |= (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_2_X[1] << 8);
  }
  /*- Return the coordinate value.
    - Note that a value of 65535 (0xFFFF) means there is no touch. */
  return absXCoordReturn;
}

/**
 * @name	  getAbsYCoordinate
 * @brief   A method that returns the constructed 16-bit coordinate value
 * @param   fingerNum     ->  Specifies the finger number. Finger 1 is the  
 *                            first finger to touch the trackpad, finger 2 is 
 *                            the second to touch the trackpad.
 * @retval  Returns 16-bit coordinate value.
 */
uint16_t IQS7211A_getAbsYCoordinate(uint8_t fingerNum)
{
  /* The 16-bit return value. */
  uint16_t absYCoordReturn = 0; 

  /* Construct the 16-bit return value. */
  if (fingerNum == FINGER_1)
  {
    absYCoordReturn = (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_1_Y[0]);
    absYCoordReturn |= (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_1_Y[1] << 8);
  }
  else if (fingerNum == FINGER_2)
  {
    absYCoordReturn = (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_2_Y[0]);
    absYCoordReturn |= (uint16_t)(iqs7211a_sensor.IQS_memory_map.FINGER_2_Y[1] << 8);
  }
  /* Return the coordinate value. Note that a value of 65535 (0xFFFF) means 
    there is no touch. */
  return absYCoordReturn;
}


/**
 * @name   updateAbsCoordinates
 * @brief  A method which reads the IQS7211A x and y coordinates and assigns
 *         them to FINGER_1_X and FINGER_1_X in the local memory map.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or closed after retrieving the information.
 *                          Use the STOP and RESTART definitions.
 * @param  fingerNum     -> Specifies the finger number. Finger 1 is the first
 *                          finger to touch the trackpad, finger 2 is the second
 *                          to touch the trackpad.
 * @retval None.
 * @note   The FINGER_1_X and FINGER_1_Y local memory map variables are altered
 *         with the new values from the IQS7211A device. The user can use the
 *         getAbsXCoordinate and getAbsYCoordinate methods to return the value.
 */
void IQS7211A_updateAbsCoordinates(uint8_t fingerNum)
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  /* The temporary address which will hold the bytes from the
  IQS7211A_MM_FINGER_1_X register address. */
  uint8_t transferBytes[4];
  memset(transferBytes,0,sizeof(transferBytes));
  if (fingerNum == FINGER_1)
  {
    /* Read the bytes using the iqs7211a_read method to read bytes at the
       Finger 1 address. */
    iqs7211a_read(IQS7211A_MM_FINGER_1_X, transferBytes, 4, &i2c_handle);
    /*  Assign the bytes to the union. */
    iqs7211a_sensor.IQS_memory_map.FINGER_1_X[0] = transferBytes[0];
    iqs7211a_sensor.IQS_memory_map.FINGER_1_X[1] = transferBytes[1];
    iqs7211a_sensor.IQS_memory_map.FINGER_1_Y[0] = transferBytes[2];
    iqs7211a_sensor.IQS_memory_map.FINGER_1_Y[1] = transferBytes[3];
  }
  else if (fingerNum == FINGER_2)
  {
    /* Read the bytes using the iqs7211a_read method to read bytes at the
      Finger 2 address. */
    iqs7211a_read(IQS7211A_MM_FINGER_2_X, transferBytes, 4, &i2c_handle);
    /*  Assign the bytes to the union. */
    iqs7211a_sensor.IQS_memory_map.FINGER_2_X[0] = transferBytes[0];
    iqs7211a_sensor.IQS_memory_map.FINGER_2_X[1] = transferBytes[1];
    iqs7211a_sensor.IQS_memory_map.FINGER_2_Y[0] = transferBytes[2];
    iqs7211a_sensor.IQS_memory_map.FINGER_2_Y[1] = transferBytes[3];
  }
}

/**
 * @name   SW_Reset
 * @brief  A method that sets the SW RESET bit to force the IQS7211A device
 *         to do a SW reset.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   To perform SW Reset, IQS7211A_SW_RESET_BIT in SYSTEM_CONTROL is set.
 */
void IQS7211A_SW_Reset()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferByte[2]; // Array to store the bytes transferred.
  memset(transferByte,0,sizeof(transferByte));

  iqs7211a_read(IQS7211A_MM_SYSTEM_CONTROL, transferByte, 2, &i2c_handle);
  /* Set the SW_RESET_BIT in the SYSTEM_CONTROL register */
  transferByte[1] = IQS7211A_setBit(transferByte[1], IQS7211A_SW_RESET_BIT);
  /* Write the new byte to the required device. */
  iqs7211a_write(IQS7211A_MM_SYSTEM_CONTROL, transferByte, 2, &i2c_handle);
}

/**
  * @name   setEventMode
  * @brief  A method to set the IQS7211A device into event mode.
  * @param  stopOrRestart -> Specifies whether the communications window must be 
  *                          kept open or must be closed after this action.
  *                          Use the STOP and RESTART definitions.
  * @retval None.
  * @note   All other bits at this register address are preserved.
  */

void IQS7211A_setEventMode()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferByte[2]; // The array which will hold the bytes which are transferred.
  memset(transferByte,0,sizeof(transferByte));

  /* First read the bytes at the memory address so that they can be preserved */
  iqs7211a_read(IQS7211A_MM_CONFIG_SETTINGS,transferByte,  2, &i2c_handle);
  /* Set/CLear the INTERFACE_SELECTION_BITS in CONTROL_SETTINGS */
  transferByte[1] = IQS7211A_setBit(transferByte[1], IQS7211A_EVENT_MODE_BIT);
  /* Write the bytes back to the device */
  iqs7211a_write(IQS7211A_MM_CONFIG_SETTINGS, transferByte,  2, &i2c_handle);
}

/**
 * @name   updateInfoFlags
 * @brief  A method which reads the IQS7211A info flags and assigns them to the 
 *         local memory map.
 * @param  stopOrRestart -> Specifies whether the communications window must be 
 *                          kept open or must be closed after this action.
 *              			      Use the STOP and RESTART definitions.
 * @retval None.
 */
void IQS7211A_updateInfoFlags()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  /* The array which will hold the bytes to be transferred. */
  uint8_t transferBytes[2]; 
  memset(transferBytes,0,sizeof(transferBytes));

  /* Read the info flags. */
  iqs7211a_read(IQS7211A_MM_INFOFLAGS, transferBytes, 2, &i2c_handle);
  /* Assign the info flags to the info flags union. */
  iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0] = transferBytes[0];
  iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[1] = transferBytes[1];
}

/**
 * @name	acknowledgeReset
 * @brief  A method that clears the Show Reset bit by setting the Ack Reset bit
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   -  If a reset has occurred the device settings should be reloaded
 *            using the begin function.
 * 		     -  After new device settings have been reloaded this method should
 *            be used to clear the reset bit.
 */
void IQS7211A_acknowledgeReset()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferByte[2]; // A temporary array to hold the bytes to be transferred.
  memset(transferByte,0,sizeof(transferByte));
  /* Read the System Flags from the IQS7211A, these must be read first in order
    not to change any settings. */
  iqs7211a_read(IQS7211A_MM_SYSTEM_CONTROL, transferByte, 2, &i2c_handle);
  /* Write the Ack Reset bit to bit 1 to clear the Reset Event Flag. */
  transferByte[0] = IQS7211A_setBit(transferByte[0], IQS7211A_ACK_RESET_BIT);
  /* Write the new byte to the System Flags address. */
  iqs7211a_write(IQS7211A_MM_SYSTEM_CONTROL, transferByte, 2, &i2c_handle);
}

/**
 * @name	  checkReset
 * @brief   A method that checks if the device has reset and returns the reset
 *          status.
 * @param   None.
 * @retval  Returns true if a reset has occurred, false if no reset has occurred.
 * @note    - If a reset has occurred the device settings should be reloaded using
 *            the begin function.
 * 		      - After new device settings have been reloaded the acknowledgeReset()
 *            function can be used to clear the reset flag.
 */
bool IQS7211A_checkReset()
{
  /* Perform a bitwise AND operation inside the IQS7211A_getBit() function with the
  SHOW_RESET_BIT to return the reset status */
  return IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0], IQS7211A_SHOW_RESET_BIT);
}
/**
 * @name    queueValueUpdates
 * @brief   All I2C read operations in the queueValueUpdates method will be
 *          performed each time the IQS7211A opens a RDY window.
 * @param   None.
 * @retval  None.
 * @note    Any address in the memory map can be read from here. This is where
 *          data read from the chip gets updated.
 */
void IQS7211A_queueValueUpdates()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferBytes[10]; // The array which will hold the bytes to be transferred.
  memset(transferBytes,0,sizeof(transferBytes));
  /* Read the gesture and info flags. */
  iqs7211a_read(IQS7211A_MM_INFOFLAGS, transferBytes, 4, &i2c_handle);

  /* Assign the info flags to the info flags */
  iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0] = transferBytes[0];
  iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[1] = transferBytes[1];

  /* Assign the gesture flags to the gesture flags */
  iqs7211a_sensor.IQS_memory_map.GESTURES[0] = transferBytes[2];
  iqs7211a_sensor.IQS_memory_map.GESTURES[1] = transferBytes[3];

  /* Read Finger 1 x and y coordinate. */
  iqs7211a_read(IQS7211A_MM_FINGER_1_X, transferBytes, 4, &i2c_handle);

  /* Read Finger 1 x and y coordinate. */
  iqs7211a_sensor.IQS_memory_map.FINGER_1_X[0] = transferBytes[0];
  iqs7211a_sensor.IQS_memory_map.FINGER_1_X[1] = transferBytes[1];
  iqs7211a_sensor.IQS_memory_map.FINGER_1_Y[0] = transferBytes[2];
  iqs7211a_sensor.IQS_memory_map.FINGER_1_Y[1] = transferBytes[3];

  /* Read Finger 2 x and y coordinate. */
  iqs7211a_read(IQS7211A_MM_FINGER_2_X, transferBytes, 4, &i2c_handle);

  iqs7211a_sensor.IQS_memory_map.FINGER_2_X[0] = transferBytes[0];
  iqs7211a_sensor.IQS_memory_map.FINGER_2_X[1] = transferBytes[1];
  iqs7211a_sensor.IQS_memory_map.FINGER_2_Y[0] = transferBytes[2];
  iqs7211a_sensor.IQS_memory_map.FINGER_2_Y[1] = transferBytes[3];
}

/**
 * @name	  readATIactive
 * @brief  A method that checks if the ATI routine is still active
 * @param  None.
 * @retval Returns true if the ATI_ACTIVE_BIT is cleared, false if the
 *         ATI_ACTIVE_BIT is set.
 * @note   If the ATI routine is active the channel states (NONE, PROX, TOUCH)
 *         might exhibit unwanted behaviour. Thus it is advised to wait for
 *         the routine to complete before continuing.
 */
bool IQS7211A_readATIactive()
{
  /* Read the Info flags from the IQS7211A*/
  IQS7211A_updateInfoFlags();

  if (IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0], IQS7211A_RE_ATI_OCCURRED_BIT))
  {
    return false;
  }

  return true;
}

/**
 * @name   ReATI
 * @brief  A method that sets the ReATI bit to force the IQS7211A device
 *         to run the Automatic Tuning Implementation (ATI) routine on 
 *         the TP channels.
 * @param  stopOrRestart -> Specifies whether the communications window must 
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   To force ATI, the IQS7211A_TP_RE_ATI_BIT in the System Control 
 *         register is set.
 */
void IQS7211A_ReATI()
{
  struct i2c_dt_spec i2c_handle = iqs7211a_sensor.i2c_handle;
  uint8_t transferByte[2]; // Array to store the bytes transferred.
  memset(transferByte,0,sizeof(transferByte));

  iqs7211a_read(IQS7211A_MM_SYSTEM_CONTROL, transferByte, 2, &i2c_handle);
  /* Set the RE_ATI_BIT in the SYSTEM_CONTROL register */
  transferByte[0] = IQS7211A_setBit(transferByte[0], IQS7211A_TP_RE_ATI_BIT); 
  /* Write the new byte to the required device. */
  iqs7211a_write(IQS7211A_MM_SYSTEM_CONTROL, transferByte, 2 ,&i2c_handle);
}

/**
  * @name   getPowerMode
  * @brief  A method which reads the INFO_FLAGS from the memory map and returns 
  *         the current power mode.
  * @param  void
  * @retval Returns the current iqs7211a_power_modes state the device is in.
  * @note   See Datasheet on power mode options and timeouts. 
  *         Normal Power, Low Power and Ultra Low Power (ULP).
  */
iqs7211a_power_modes IQS7211A_getPowerMode(void)
{
  uint8_t buffer = IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0], IQS7211A_CHARGING_MODE_BIT_0);
  buffer += IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0], IQS7211A_CHARGING_MODE_BIT_1) << 1;
  buffer += IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[0], IQS7211A_CHARGING_MODE_BIT_2) << 2;

  if(buffer == IQS7211A_ACTIVE_BITS)
  {
    return IQS7211A_ACTIVE;
  }
  else if(buffer == IQS7211A_IDLE_TOUCH_BITS)
  {
    return IQS7211A_IDLE_TOUCH;
  }
  else if(buffer == IQS7211A_IDLE_BITS)
  {
    return IQS7211A_IDLE;
  }
    else if(buffer == IQS7211A_LP1_BITS)
  {
    return IQS7211A_LP1;
  }
    else if(buffer == IQS7211A_LP2_BITS)
  {
    return IQS7211A_LP2;
  }
  else
  {
    return IQS7211A_POWER_UNKNOWN;
  }
}

/**
  * @name   getNumFingers
  * @brief  A method that returns the number of fingers active on the trackpad. 
  * @param  None.
  * @retval Returns an 8-bit unsigned integer value of the number of fingers 
  *         on the trackpad. 
  */
uint8_t IQS7211A_getNumFingers(void)
{
  uint8_t buffer = IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[1], IQS7211A_NUM_FINGERS_BIT_0);
  buffer += IQS7211A_getBit(iqs7211a_sensor.IQS_memory_map.INFO_FLAGS[1], IQS7211A_NUM_FINGERS_BIT_1);

  return buffer;
}