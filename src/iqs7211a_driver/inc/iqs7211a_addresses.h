/******************************************************************************
 *                                                                            *
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 *  Refer to IQS7211A datasheet for more information, available here:         *
 * - https://www.azoteq.com/design/datasheets/                                *
 * ========================================================================== *
 *                       IQS7211A - Registers & Memory Map                    *
******************************************************************************/

#ifndef __IQS7211A_ADDRESSES_H
#define __IQS7211A_ADDRESSES_H

/* Device Information - Read Only */

/* All Banks: 0x00 - 0x26 */
#define IQS7211A_MM_PROD_NUM                    0x00
#define IQS7211A_MM_MAJOR_VERSION_NUM           0x01
#define IQS7211A_MM_MINOR_VERSION_NUM           0x02
#define IQS7211A_MM_INFOFLAGS                   0x10
#define IQS7211A_MM_GESTURES                    0x11
#define IQS7211A_MM_RELATIVE_X                  0x12
#define IQS7211A_MM_RELATIVE_Y                  0x13
#define IQS7211A_MM_FINGER_1_X                  0x14
#define IQS7211A_MM_FINGER_1_Y                  0x15
#define IQS7211A_MM_FINGER_1_TOUCH_STRENGTH     0x16
#define IQS7211A_MM_FINGER_1_AREA               0x17
#define IQS7211A_MM_FINGER_2_X                  0x18
#define IQS7211A_MM_FINGER_2_Y                  0x19
#define IQS7211A_MM_FINGER_2_TOUCH_STRENGTH     0x1A
#define IQS7211A_MM_FINGER_2_AREA               0x1B

#define IQS7211A_MM_TOUCH_STATUS_ROW0           0x20
#define IQS7211A_MM_TOUCH_STATUS_ROW1           0x21
#define IQS7211A_MM_TOUCH_STATUS_ROW2           0x22
#define IQS7211A_MM_ALP_CH_COUNT                0x23
#define IQS7211A_MM_ALP_CH_LTA                  0x24
#define IQS7211A_MM_ALP_COUNT_A                 0x25
#define IQS7211A_MM_ALP_COUNT_B                 0x26

/* Trackpad Settings 0x30 - 0x35 */
#define IQS7211A_MM_TP_ATI_MIR                  0x30
#define IQS7211A_MM_TP_PCC_DIV                  0x31
#define IQS7211A_MM_TP_ATI_TARGET               0x32
#define IQS7211A_MM_TP_REF_DRIFT_LIMIT          0x33
#define IQS7211A_MM_TP_MIN_CT_ATI_VALUE         0x34
#define IQS7211A_MM_TP_ATI_RETRY_TIME           0x35

/* ALP Channel Settings 0x36 - 0x3B */
#define IQS7211A_MM_ALP_ATI_MIR                 0x36
#define IQS7211A_MM_ALP_PCC_DIV                 0x37
#define IQS7211A_MM_ALP_ATI_TARGET              0x38
#define IQS7211A_MM_ALP_LTA_DRIFT_LIMIT         0x39
#define IQS7211A_MM_ALP_PCC_A                   0x3A
#define IQS7211A_MM_ALP_PCC_B                   0x3B

/* Report rates and timeouts 0x40 - 0x4A */
#define IQS7211A_MM_ACTIVE_MODE_RR              0x40
#define IQS7211A_MM_IDLE_TOUCH_MODE_RR          0x41
#define IQS7211A_MM_IDLE_MODE_RR                0x42
#define IQS7211A_MM_LP1_MODE_RR                 0x43
#define IQS7211A_MM_LP2_MODE_RR                 0x44
#define IQS7211A_MM_ACTIVE_MODE_TIMEOUT         0x45
#define IQS7211A_MM_IDLE_TOUCH_MODE_TIMEOUT     0x46
#define IQS7211A_MM_IDLE_MODE_TIMEOUT           0x47
#define IQS7211A_MM_LP1_MODE_TIMEOUT            0x48
#define IQS7211A_MM_REF_UPDATE_TIME             0x49
#define IQS7211A_MM_RDY_TIMEOUT                 0x4A

/* System Control and Hardware Settings 0x50 - 0xA0 */
#define IQS7211A_MM_SYSTEM_CONTROL              0x50
#define IQS7211A_MM_CONFIG_SETTINGS             0x51
#define IQS7211A_MM_OTHER_SETTINGS              0x52
#define IQS7211A_MM_TP_TOUCH_THRESHOLD          0x53
#define IQS7211A_MM_ALP_PROX_THRESHOLD          0x54
#define IQS7211A_MM_ALP_TOUCH_THRESHOLD         0x55
#define IQS7211A_MM_ALP_PROX_DEBOUNCE           0x56
#define IQS7211A_MM_ALP_TOUCH_DEBOUNCE          0x57
#define IQS7211A_MM_TP_CONVERSION_FREQ          0x58
#define IQS7211A_MM_ALP_CONVERSION_FREQ         0x59
#define IQS7211A_MM_HW_SETTINGS                 0x5A


#define IQS7211A_MM_TP_SETTINGS_0               0x60
#define IQS7211A_MM_TP_SETTINGS_1               0x61
#define IQS7211A_MM_X_RESOLUTION                0x62
#define IQS7211A_MM_Y_RESOLUTION                0x63
#define IQS7211A_MM_XY_DYN_FILTER_BOTTOM_SPEED  0x64
#define IQS7211A_MM_XY_DYN_FILTER_TOP_SPEED     0x65
#define IQS7211A_MM_XY_DYN_FILTER_BOTSTAT_BETA  0x66
#define IQS7211A_MM_TRAVEL_THRESH_FING_SPLIT    0x67
#define IQS7211A_MM_X_TRIM                      0x68
#define IQS7211A_MM_Y_TRIM                      0x69

#define IQS7211A_MM_ALP_COUNT_FILTER_BETA       0x70
#define IQS7211A_MM_ALP_LTA_LP_BETA             0x71
#define IQS7211A_MM_ALP_SETUP                   0x72
#define IQS7211A_MM_ALP_TX_ENABLE               0x73
#define IQS7211A_MM_SETTINGS_VERSION            0x74

#define IQS7211A_MM_GESTURE_ENABLE              0x80
#define IQS7211A_MM_TAP_TIME                    0x81
#define IQS7211A_MM_TAP_DISTANCE                0x82
#define IQS7211A_MM_HOLD_TIME                   0x83
#define IQS7211A_MM_SWIPE_INIT_TIME             0x84
#define IQS7211A_MM_SWIPE_INIT_X_DISTANCE       0x85
#define IQS7211A_MM_SWIPE_INIT_Y_DISTANCE       0x86
#define IQS7211A_MM_SWIPE_ANGLE                 0x87

#define IQS7211A_MM_RXTX_MAPPING_1_0            0x90
#define IQS7211A_MM_RXTX_MAPPING_3_2            0x91
#define IQS7211A_MM_RXTX_MAPPING_5_4            0x92
#define IQS7211A_MM_RXTX_MAPPING_7_6            0x93
#define IQS7211A_MM_RXTX_MAPPING_9_8            0x94
#define IQS7211A_MM_RXTX_MAPPING_11_10          0x95
#define IQS7211A_MM_RXTX_MAPPING_NA_12          0x96

#define IQS7211A_MM_CYCLE_SETUP_0_9             0xA0
#define IQS7211A_MM_CYCLE_SETUP_10_17           0xB0

/* Extended addressing options 0xE0 - 0xE3 */
#define IQS7211A_MM_TRACKPAD_COUNT_VALUES       0xE0
#define IQS7211A_MM_TRACKPAD_REF_VALUES         0xE1
#define IQS7211A_MM_TRACKPAD_DELTA_VALUES       0xE2
#define IQS7211A_MM_TRACKPAD_PCC_VALUES         0xE3

#define IQS7211A_MM_FORCE_COMMS                 0xFF

  
#endif /* __IQS7211A_ADDRESSES_H */