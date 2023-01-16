/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */



#include "main.h"
#include "FSM.h"

/** @file
 *
 * @defgroup main main project files
 * @{
 * @ingroup mainModule
 *
 * @brief Main project file that contain core program handling and initialization.
 */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                               /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                         /**< Initialize connection to 'invalid', meaning no connection established. */
static ble_uuid_t m_adv_uuids[]          =                                       /**< Universally unique service identifier for Nordic UART Service. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

bool getValue = false;          /**< Menu is waiting for value from BLE application                 */
bool transferDataFlag = false;  /**< Menu is in the process of transfering data to BLE application  */
bool updateFSM = true;          /**< Init to true in order to access state machine                  */
bool sendNUS = false;           /**< Flag to signal Nordic UART Service to send data                */
bool motorStopped = false;      /**< Flag to signal if motor is stopped or need to be stopped       */
bool isAdvertising = false;     /**< Flag to signal if advertising or not                           */
bool bottomLimit = false;       /**< Flag to signal when bottom limit switch is reached             */

mission_t mission;              /**< Create mission struct instance */
FSM_t fsm;                      /**< Create Finite State Machine struct instance*/
enum menu currentMenu;          /**< Create menu enumeration instance to keep track of menu operations*/




/**@brief Hall Effect button Interrupt handler.
*/
void hallEffectInterrupt_handler(void){

  NRF_LOG_ERROR("HallEffect_Interrupt_HANDLER");
  fsm.hallEffectButton = true;                        /**< Notify FSM that hall effect interrupt has occured*/
}

/**@brief Bottom limit switch Interrupt handler.
*/
void limitSwitchBottom_handler(void){

  NRF_LOG_INFO("Limit Switch BOTTOM_Interrupt_HANDLER");  
  nrf_delay_ms(300);  /**< Debounce delay. */
  bottomLimit = true;

}

/**@brief Upper limit switch Interrupt handler.
*
* @note at this point firmware is not yet written to do anything upper limit switch interrupt.
*/
void limitSwitchTop_handler(void){

  NRF_LOG_INFO("Limit Switch TOP_Interrupt_HANDLER");

}

/**@brief TMP117 Temperature Alert Interrupt handler.
*
* @note at this stage TMP117 temperature sensor is not yet configured,
*       and firmware is not yet written to do anything on temperature alert interrupt.
*/
void TMP_temp_Alert_Interrupt_handler(void){

  NRF_LOG_INFO("TMP117_ALERT_Interrupt_HANDLER");

}

/**@brief ICM20948 Motion Sensor Interrupt handler.
*
* @note at this stage ICM20948 motion sensor is not yet configured,
* and firmware is not yet written to do anything on motion sensor interrupt.
*/
void motionSensorInterrupt_handler(void){

  NRF_LOG_INFO("MotionSensor_Interrupt_HANDLER");

}





/**@breif Declaration of BLE event handler.
*/
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice,
 *          @ref FAILUREstate is called to float buoyancy vehicle to surface and change LED to PINK.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("SoftDevice ASSERT")
    FAILUREstate();                                    
    app_error_handler(DEAD_BEEF, line_num, p_file_name);  
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}



/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("NUS Data length is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}




/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error. @ref FAILUREstate() is called to float buoyancy vehicle to surface and change LED.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("Queued write error");
    FAILUREstate();                /**< Go to failure state to change LED to PINK and float to surface. */
    APP_ERROR_HANDLER(nrf_error);
}





/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 * @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @details In case of connection parameter error @ref FAILUREstate()
 * is called to float vehicle to surface and change LED before error handler is called.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("Conn_params_error_handler")
    FAILUREstate(); /**< Go to failure state to change LED to PINK and float to surface */
    APP_ERROR_HANDLER(nrf_error);
    
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service
 *          and send it to its appropriate configurations.
 *           Received data to handle the menu system.
 *           The user type operate menu by numeric integer values,
 *           and set configuration values in integer or float values.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA){

        if(!getValue && !transferDataFlag){  /**< GetValue: configuration value for mission (depth and time), PID, and threshold. transferDataFlag: to send file number to @ref transfer_menu()*/
          int option = atoi(p_evt->params.rx_data.p_data);
          switch(currentMenu){
              case MAINMENU:
                mainMenu(option);
                break;
              case MISSIONDATA:
                 missinDataMenu(option);
                 break;
              case CONFIGVEHICLE:
                 configVehicleMenu(option);
                 break;
              case TRANSFERDATA:
                transferDataMenu(option);
                break;
              default: NRF_LOG_INFO("UNKNOWN MENU");
              }
          }else if(transferDataFlag) transferData(p_evt->params.rx_data.p_data);
           else setConfigValue(p_evt->params.rx_data.p_data);

      }
    else if(p_evt->type == BLE_NUS_EVT_TX_RDY)
      { //NRF_LOG_ERROR("nus_data_handler: BLE_NUS_EVT_TX_RDY");
      }
    else if(p_evt->type == BLE_NUS_EVT_COMM_STARTED)
      { NRF_LOG_ERROR("nus_data_handler: BLE_NUS_EVT_COMM_STARTED: Notification has been enabled");
       currentMenu = MAINMENU;  /**< Send MainMenu as soon as new connection is established*/
       sendNUS = true;
       }
    
    else if(p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
      { NRF_LOG_ERROR("nus_data_handler: BLE_NUS_EVT_COMM_STOPPED: Notification has been disabled"); }
    
    else NRF_LOG_ERROR("nus_data_handler: default - p_evt->type: 0x%x",p_evt->type);

}
/**@snippet [Handling the data received over BLE] */



/************************
*
* BLE application handler
*
***********************/


/**@brief Function for handling BLE events to application.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE_GAP_EVT_CONNECTED");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            isAdvertising = false;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE_GAP_EVT_DISCONNECTED");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            NRF_LOG_INFO("BLE_GAP_EVT_PHY_UPDATE_REQUEST");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;


        case  BLE_GAP_EVT_PHY_UPDATE:
           NRF_LOG_INFO("BLE_GAP_EVT_PHY_UPDATE_REQUEST - PHY Update Procedure is complete");
          break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
          NRF_LOG_INFO("BLE_GAP_EVT_CONN_PARAM_UPDATE - Connection Parameters updated");
          break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST -");
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            NRF_LOG_INFO("BLE_GATTS_EVT_SYS_ATTR_MISSING - Bonding is used, so soft device peer Manager should handle this");
            // No system attributes have been stored.
            //err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            //APP_ERROR_CHECK(err_code);
            break;
        
        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_INFO("BLE_GATTS_EVT_WRITE -- Application");
            break;



         case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            {
            NRF_LOG_INFO("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST");
            ble_gatts_rw_authorize_reply_params_t p_rw_authorize_reply_params;
            memset(&p_rw_authorize_reply_params, 0, sizeof( p_rw_authorize_reply_params ) );
            
            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_READ)
              {
              NRF_LOG_DEBUG(BLE_GATTS_AUTHORIZE_TYPE_READ)
                p_rw_authorize_reply_params.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                p_rw_authorize_reply_params.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
                p_rw_authorize_reply_params.params.read.offset = 0;
                p_rw_authorize_reply_params.params.read.update = 1;
              
                err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle, &p_rw_authorize_reply_params);
                NRF_LOG_DEBUG(err_code);
                APP_ERROR_CHECK(err_code);
              }
            else if(p_ble_evt->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
              {
              NRF_LOG_DEBUG("BLE_GATTS_AUTHORIZE_TYPE_WRITE type??");
              NRF_LOG_INFO("p_ble_evt->evt.gatts_evt.params.authorize_request.type: 0x%x",p_ble_evt->evt.gatts_evt.params.authorize_request.type);
                p_rw_authorize_reply_params.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                p_rw_authorize_reply_params.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                p_rw_authorize_reply_params.params.write.offset = 0;
                p_rw_authorize_reply_params.params.write.update = 1;
              

                err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &p_rw_authorize_reply_params);
                NRF_LOG_DEBUG("sd_ble_gatts_rw_authorize_reply error: 0x%x)",err_code);
                APP_ERROR_CHECK(err_code);
              }

            }break;

        case BLE_GATTC_EVT_TIMEOUT:           /**< Disconnect on GATT Client timeout event.*/
             NRF_LOG_INFO("BLE_GATTC_EVT_TIMEOUT");

            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:           /**< Disconnect on GATT Server timeout event.*/
             NRF_LOG_INFO("BLE_GATTS_EVT_TIMEOUT");

            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
             break;

        default:
            NRF_LOG_INFO("App event DEFAULT: p_ble_evt->header.evt_id: 0x%x",p_ble_evt->header.evt_id);
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}



/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive the reply message from motor controller interface, through the app_uart module, and append it to
 *          a data buffer array. The data buffer will be summed to verify correct checksum and sent to @ref receiveReply().
 *
 * @param[in] p_event UART event
 */
void uart_event_handle(app_uart_evt_t * p_event)
{

    static uint8_t rxBuffer[9];
    static uint8_t checksum = 0;
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:/**< UART Rx - Receive reply from motor controller.*/
          {  
            if(index < sizeof(rxBuffer)){
              app_uart_get(&rxBuffer[index]);
              index ++;
            }
            if( index >= sizeof(rxBuffer)){
              index = 0;
              /*
               NRF_LOG_INFO(" ");
               NRF_LOG_INFO("rxBuffer[Nr]");
              for(int i = 0; i < sizeof(rxBuffer); i++)
              {
                NRF_LOG_INFO("%d: Hex: 0x%x Dec: %d",i, rxBuffer[i], rxBuffer[i]);
              }NRF_LOG_INFO(" ");
              */

              for(int j = 0; j < 8; j++)
                checksum += rxBuffer[j];
              if(checksum != rxBuffer[8]) NRF_LOG_ERROR("Wrong checksum: %d", checksum);
              checksum = 0; 
              receiveReply(rxBuffer);
            }
          }break;

        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("APP_UART_COMMUNICATION_ERROR");
            FAILUREstate(); /**< Go to @ref FAILUREstate() to change LED to PINK and float to surface */
            APP_ERROR_HANDLER(p_event->data.error_communication);  
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("APP_UART_FIFO_ERROR");
            FAILUREstate(); /**< Go to @ref FAILUREstate() to change LED to PINK and float to surface */
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */



/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/******************************
*
*
* Initialization functions
*
*
********************************/

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);


    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT library.
*
* @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the device.
*/
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;  // Return_code_type: Error Code
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);


    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_BLE_OBSERVER(m_ble_nus_observer, APP_BLE_OBSERVER_PRIO, ble_nus_on_ble_evt, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}



/**@brief  Function for initializing the UART module.
 */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RXD_PIN,
        .tx_pin_no    = TXD_PIN,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_9600
#endif
    };


    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_HIGH,
                       err_code);
    APP_ERROR_CHECK(err_code);

}
/**@snippet [UART Initialization] */




/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    nrf_log_backend_rtt_init();

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing gpio input and output pins.
 */
static void gpio_init(void)
{
    uint32_t err_code;
    
    // Initializer for GPIO
    err_code =  nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // Configure specific input and/or output pin
    nrfx_gpiote_in_config_t activeLow_Interrupt = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);  /**< False: not IN_EVENT, to save power.*/
    activeLow_Interrupt.pull = NRF_GPIO_PIN_PULLUP;

    // Hall Effect Interrupt Configuration
    nrfx_gpiote_in_config_t hallEffect_Interrupt = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);  /**< False: not IN_EVENT, to save power.*/
    activeLow_Interrupt.pull = NRF_GPIO_PIN_PULLUP;

    nrfx_gpiote_in_config_t activeHigh_Interrupt = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    activeHigh_Interrupt.pull = NRF_GPIO_PIN_PULLDOWN;

    nrfx_gpiote_out_config_t pressure_switch_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);

    // Initialize specific pins, pin nr., pin configuration, and event handler (if applicable)
    err_code = nrfx_gpiote_in_init(MOTION_INT, &activeLow_Interrupt, motionSensorInterrupt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_gpiote_in_init(HALLEFFECT_INT, &hallEffect_Interrupt, hallEffectInterrupt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_gpiote_in_init(TMP_ALERT, &activeLow_Interrupt, TMP_temp_Alert_Interrupt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_gpiote_in_init(BOTTOM_LIMIT, &activeHigh_Interrupt, limitSwitchBottom_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_gpiote_in_init(TOP_LIMIT, &activeHigh_Interrupt, limitSwitchTop_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_gpiote_out_init(PRESSURE_SWITCH, &pressure_switch_config);
    APP_ERROR_CHECK(err_code);

    // Enable interrupt to call event handler
    nrfx_gpiote_in_event_disable(MOTION_INT);
    nrfx_gpiote_in_event_enable(HALLEFFECT_INT,true);
    nrfx_gpiote_in_event_enable(TMP_ALERT, true);
    nrfx_gpiote_in_event_enable(BOTTOM_LIMIT,true);
    nrfx_gpiote_in_event_enable(TOP_LIMIT, true);

}

/**@brief Function to enable pressure sensor.
 */
void enablePressureSensor(){
  nrfx_gpiote_out_set(PRESSURE_SWITCH);
}

/**@brief Function to disable pressure sensor.
 */
void disablePressureSensor(){
  nrfx_gpiote_out_clear(PRESSURE_SWITCH);
}



/******************************
*
*
* Application functions
*
*
********************************/


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function trigger system-off mode (this function will not return), wakeup will cause a reset
 */
void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = sd_power_system_off();
    while(1);   // For sleepMode while debugging debug, has no effect when not in debug.
    APP_ERROR_CHECK(err_code);
}




/**@brief Send data over BLE Nordic UART Service (NUS)
*
*@param[in] data string array to be sent over NUS
*@param[in] length Length of string array
*/
void nus_send(uint8_t data[], uint16_t length){

    volatile uint16_t index = 0, offset = 0;
    uint16_t pkglength = 0;
    ret_code_t err_code;

    uint8_t dataPkg[BLE_NUS_MAX_DATA_LEN] = {0};

    VERIFY_PARAM_NOT_NULL(&m_nus);
  
  for(int i = 0; i < length; i++)
  {
    dataPkg[index] = data[index+offset];
    index++;
    if((dataPkg[index-1] == '\n') ||    
       (dataPkg[index-1] == '\r') ||    
       (data[index+offset] == '\0') ||    
       (index >= BLE_NUS_MAX_DATA_LEN))
    {
      if(index > 1)
      {
        do 
        { 
          pkglength = index;
          err_code = ble_nus_data_send(&m_nus, dataPkg, &pkglength, m_conn_handle);
          if( (err_code != NRF_ERROR_INVALID_STATE) &&
              (err_code != NRF_ERROR_RESOURCES) &&
              (err_code != NRF_ERROR_NOT_FOUND))
          {
            APP_ERROR_CHECK(err_code);
          }
  
        }while(err_code == NRF_ERROR_RESOURCES && m_conn_handle != BLE_CONN_HANDLE_INVALID);
        offset += index;                   
        index = 0;
        memset(&dataPkg,0,sizeof(dataPkg));
      }
    }
  }
}




/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        NRF_LOG_INFO("%d",erase_bonds);
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
        isAdvertising = true;
    }
}




/**@brief Start HFCLK from crystal oscillator, this will give the PWM and SAADC higher accuracy.*/
void HFCLKstart()
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    // Wait for clock to start
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
}

/**@brief Disconnect BLE connection.*/
void BLEdisconnect(){
  ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  APP_ERROR_CHECK(err_code);
}

/**@brief disable restarted advertising upon disconnection.*/
void disableAdvOnDisconnect(void){
    m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = true;
}
/**@brief enable restarted advertising upon disconnection.*/
void enableAdvOnDisconnect(void){
    m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = false;
}

/**@brief Check if connected to client
*
* @return true If connected
* @return false If not connected
*/
bool BLEconnected(){
  
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
    return true;
  else
    return false;
}


/**@brief stop advertising
*
* @warning If called when not advertising SoftDevice will return NRF_ERROR_INVALID_STATE
*/
void stopAdvertising(){

  if(isAdvertising){
    ret_code_t err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    APP_ERROR_CHECK(err_code);
    isAdvertising = false;
    }

}

/**@brief Read TMP117 16 bit temperature register
*
* @param[out] tempMSB 8 most significant bits representing temperature
* @param[out] tempLSB 8 least significant bits representing temperature
*/
void readTMP117(uint8_t * tempMSB, uint8_t * tempLSB){

  uint8_t tempReg = 0x00; /**<Temperature register within TMP117 */

  // Get Temperature
  TWIMtxrx(TMP117, tempReg, tempMSB);
  while(isTWIMbusy());
  TWIMrx(TMP117,tempMSB);
  while(isTWIMbusy());
  TWIMrx(TMP117,tempLSB);
  while(isTWIMbusy());
 }



/**@brief Function for application main entry.
 */
int main(void)
{
    pwm_init(); /**< Initialize PWM before anything else to avoid the LED on full strength when uninitialized.*/
    float motorSpeed = 1;/**< variable to measure motor speed while returning to lower limit switch*/
    
    bool erase_bonds;
    uint32_t err_code;
    volatile uint16_t data = 0; // I2C data

    memset(&fsm,0,sizeof(fsm));
    memset(&mission,0,sizeof(mission));

    

  
    // Initialize Support Hardware for chip and BLE
    HFCLKstart();
    log_init();
    timers_init();
    gpio_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    uart_init();
    app_timer_stop_all();
    saadc_init();
motorEnableLimitSwitches(); /**< Enable limit switches as soon as possible to make sure they are enabled when motor is running*/


    
    SDcardInit();
    TWIMInit();
    motorInit();  // Initialize Motor
    
    missionInit();  // Initialize Mission

    fsm.state = INIT; // Initialize state machine

    while (1) 
    {
     
     if(updateFSM){
      updateFSM = false;
      FSM();  
        
      if(motorStopped == false){
        stopMotorAtSurface();
        setReferencePositionToZero();
        }
      }


      if(sendNUS){  /**< Call NUS data transmission by setting flag: sendNUS*/
        sendNUS = false;
        NRF_LOG_ERROR("currentMenu: %d",currentMenu);
        switch(currentMenu){
          case MAINMENU: printMainMenu(); break;
          case MISSIONDATA: printMissionDataMenu(); break;
          case CONFIGVEHICLE: printConfigVehicleMenu(); break;
          case TRANSFERDATA: printTransferDataMenu(); break;
          case TRANSFER_ALL_FILES: transferAllFiles(); break;
          case TRANSFER_ONE_FILE: TransferOneFile(); break;
          case DELETE_ALL_FILES: deleteAllFiles(); break;
          case DELETE_ONE_FILE: deleteFile(); break;
          default: NRF_LOG_INFO("Unknown send menu: %d",currentMenu);
          }
      }

     idle_state_handle();

    }
}

/** @} */