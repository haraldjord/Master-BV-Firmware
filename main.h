


#ifndef _MAIN_H
#define _MAIN_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "PID.h"
#include "PWM.h"
#include "SDcard.h"
#include "TWIM.h"
#include "SAADC.h"
#include "motor.h"
#include "FSM.h"
#include "timers.h"
#include "mission.h"
#include "menu.h"


// SDK drivers
#include "nordic_common.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_conn_state.h"
#include "ble_link_ctx_manager.h"

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdm.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_rtt.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_error.h"
#include "nrf_drv_twi.h"

#include "nrfx_saadc.h"


#include "app_error.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "bsp_btn_ble.h"
#include "sdk_errors.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sensorsim.h"
#include "sdk_macros.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


/** @file
 *
 * @defgroup main main project files
 * @{
 * @ingroup mainModule
 *
 * @brief Main project file that contain core program handling and initialization.
 */


#define DEVICE_NAME                     "Buoyancy"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NTNU"                                  /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)         /**< Minimum acceptable connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)         /**< Maximum acceptable connection interval (75 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    6                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/**< UART FIFO BUFFER SIZE */
#define UART_TX_BUF_SIZE                256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                     /**< UART RX buffer size. */

/**< UART */
#define TXD_PIN (6UL)               /**< White wire. */
#define RXD_PIN (8UL)               /**< Gray/brown wire */


/**< GPIOTE input Interrupt */
#define MOTION_INT      13UL        /**< ICM motion sensor interrupt */
#define TMP_ALERT       15UL        /**< TMP117 temperature sensor interrupt */
#define HALLEFFECT_INT  17UL        /**< Hall Effect sensor interrupt */
#define TOP_LIMIT       29UL        /**< Top Limit switch    - Max Depth */
#define BOTTOM_LIMIT    28UL        /**< Bottom Limit switch - Water Surface */
#define PRESSURE_SWITCH  20UL        /**< Signal to switch on or off pressure sensor power */

#define LOW_POWER_THRESHOLD 8700  /**< SAADC value at 12.8 V - Go to LOWPOWER state */


void advertising_start(bool);
void assert_nrf_callback(uint16_t, const uint8_t *);
void nus_send(uint8_t*, uint16_t);
void printLine(uint8_t data[], uint16_t);

void anotherPrint(void);
void handleRXdata(ble_evt_t const *);
void uart_error_handle(app_uart_evt_t*);
void sleep_mode_enter(void);
void idle_state_handle(void);
void BLEdisconnect(void);
void disableAdvOnDisconnect(void);
void enableAdvOnDisconnect(void);
bool BLEconnected(void);
void stopAdvertising(void);
void advLEDoff(void);
void readTMP117(uint8_t *, uint8_t *);
void enablePressureSensor(void);
void disablePressureSensor(void);

#endif  // _MAIN_H

/** @} */