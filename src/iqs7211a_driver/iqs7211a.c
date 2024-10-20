#include "iqs7211a.h"
#include "board_param.h"
#include <zephyr/sys/printk.h>

int iqs7211a_init(iqs7211a_dev *dev_handle);
int iqs7211a_read(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle);
int iqs7211a_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle);
void iqs7211a_ready_interrupt(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void printGesture(iqs7211a_dev *dev_handle);
void iqs7211a_run(iqs7211a_dev *dev_handle);
bool checkReset(iqs7211a_dev *dev_handle);

int iqs7211a_init(iqs7211a_dev *dev_handle)
{

  struct i2c_dt_spec dev_i2c_handle = dev_handle->i2c_handle;
  struct gpio_dt_spec interrupt_pin = dev_handle->interrupt_pin;

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

  ret = gpio_pin_interrupt_configure_dt(&interrupt_pin, GPIO_INT_EDGE_TO_ACTIVE);
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
  dev_handle->iqs7211a_deviceRDY = false;
  return 1;
}

void iqs7211a_ready_interrupt(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
  printf("Button is pressed\n");
}

void iqs7211a_run(iqs7211a_dev *dev_handle)
{
  switch (dev_handle->dev_state)
  {
  /* After a hardware reset, this is the starting position of the running state
     machine */
  case IQS7211A_STATE_START:
    printf("IQS7211A Initialization:");
    dev_handle->dev_state = IQS7211A_STATE_INIT;
    break;

  /* Perform the initialization routine on the IQS7211A */
  case IQS7211A_STATE_INIT:
    // if(config_iqs7211a())
    // {
    //   printf("IQS7211A Initialization complete!\n");
    //   dev_handle->dev_state = IQS7211A_STATE_RUN;
    // }
    break;

  /* Send an I2C software reset in the next RDY window */
  case IQS7211A_STATE_SW_RESET:
    if (dev_handle->iqs7211a_deviceRDY)
    {
      // SW_Reset(STOP);
      dev_handle->dev_state = IQS7211A_STATE_RUN;
    }
    break;

  /* Continuous reset monitoring state, ensure no reset event has occurred
    for data to be valid */
  case IQS7211A_STATE_CHECK_RESET:
    if (checkReset(dev_handle))
    {
      printf("Reset occurred!\n");
      // dev_handle->new_data_available = false;
      dev_handle->dev_state = IQS7211A_STATE_START;
    }
    /* A reset did not occur, move to the run state and wait for a new RDY window */
    else
    {
      // new_data_available = true; /* No reset, thus data is valid */
      dev_handle->dev_state = IQS7211A_STATE_RUN;
    }
    break;

  /* If a RDY Window is open, read the latest values from the IQS7211A */
  case IQS7211A_STATE_RUN:
    if (dev_handle->iqs7211a_deviceRDY)
    {
      // queueValueUpdates();
      dev_handle->iqs7211a_deviceRDY = false;
      // new_data_available = false;
      dev_handle->dev_state = IQS7211A_STATE_CHECK_RESET;
    }
    break;
  default:
    break;
  }
}
bool checkReset(iqs7211a_dev *dev_handle)
{
  /* Perform a bitwise AND operation inside the getBit() function with the
  SHOW_RESET_BIT to return the reset status */
  return false; // getBit(iqs7211a_dev->IQS_memory_map.INFO_FLAGS[0], IQS7211A_SHOW_RESET_BIT);
}

/* Check if one of the gesture flags is set and display which one */
void printGesture(iqs7211a_dev *dev_handle)
{
  iqs7211a_gestures_e tempGestures;

  switch (tempGestures)
  {
  case IQS7211A_GESTURE_SINGLE_TAP:
    printf("Single Tap\t\t");
    break;

  case IQS7211A_GESTURE_PRESS_HOLD:
    printf("Press and Hold\t\t");
    break;

  case IQS7211A_GESTURE_SWIPE_X_POSITIVE:
    printf("Swipe X +\t\t");
    break;

  case IQS7211A_GESTURE_SWIPE_X_NEGATIVE:
    printf("Swipe X -\t\t");
    break;

  case IQS7211A_GESTURE_SWIPE_Y_POSITIVE:
    printf("Swipe Y +\t\t");
    break;

  case IQS7211A_GESTURE_SWIPE_Y_NEGATIVE:
    printf("Swipe Y -\t\t");
    break;

  default:
    printf("-\t\t\t");
    break;
  }
}

int start_up(iqs7211a_dev *dev_handle)
{
  struct i2c_dt_spec i2c_handle = dev_handle->i2c_handle;
  int ret;

  uint8_t data[2];
  memset(data,0,2);

  ret = iqs7211a_read(IQS7211A_MM_PROD_NUM,data,2,&i2c_handle);
  if(ret != 1){
    return -1;
  }
  uint16_t productNumber = (uint16_t)data[0] | (uint16_t)(data[1]<<8);

  memset(data,0,2);

  ret = iqs7211a_read(IQS7211A_MM_MAJOR_VERSION_NUM,data,2,&i2c_handle);
  if(ret != 1){
    return -1;
  }
  uint8_t ver_maj = data[0];

  memset(data,0,2);

  ret = iqs7211a_read(IQS7211A_MM_MINOR_VERSION_NUM,data,2,&i2c_handle);
  if(ret != 1){
    return -1;
  }
  uint8_t ver_min = data[0];
  printf("\t\tProduct number is: %d v%d.%d\n",productNumber,ver_maj,ver_min);

  if(productNumber == IQS7211A_PRODUCT_NUM){
    printf("\t\tIQS7211A Release UI Confirmed!\n");
  }
  else{
    printf("\t\tDevice is not a IQS7211A!\n");
    return -1;
  }

  k_msleep(100);

  //To put reset

  uint8_t transferBytes[30];	// Temporary array which holds the bytes to be transferred.
    
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
  iqs7211a_write(IQS7211A_MM_TP_ATI_MIR,transferBytes,24,&i2c_handle);
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
  iqs7211a_write(IQS7211A_MM_ACTIVE_MODE_RR,transferBytes,22,&i2c_handle);   
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
  iqs7211a_write(IQS7211A_MM_SYSTEM_CONTROL,transferBytes,24,&i2c_handle); 
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
  iqs7211a_write(IQS7211A_MM_TP_SETTINGS_0,transferBytes,20,&i2c_handle); 
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
  iqs7211a_write(IQS7211A_MM_ALP_COUNT_FILTER_BETA,transferBytes,10,&i2c_handle); 
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
  iqs7211a_write(IQS7211A_MM_GESTURE_ENABLE,transferBytes,16,&i2c_handle);
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
  iqs7211a_write(IQS7211A_MM_RXTX_MAPPING_1_0,transferBytes,13,&i2c_handle);
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
  iqs7211a_write(IQS7211A_MM_CYCLE_SETUP_0_9,transferBytes,30,&i2c_handle);
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
  iqs7211a_write(IQS7211A_MM_CYCLE_SETUP_10_17,transferBytes,24,&i2c_handle);
  printf("\t\t9. Write Cycle 10 - 17 Settings\n");

  return 1;
}

int iqs7211a_read(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle)
{
  return -1;
}

int iqs7211a_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct i2c_dt_spec *i2c_handle)
{
  return -1;
}