#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_nvmc.h"
#include "nrf_drv_power.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_srv_brux.h"
#include "app_mpu.h"

#define DEBUG
#define APP_BLE_CONN_CFG_TAG		    1

#define DEVICE_NAME                     "BRUXISMO"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "EDUARDO"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(1000)

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OPCODE_LENGTH			        1
#define HANDLE_LENGTH			        2

#define RTC_FREQUENCY                   32                                        //Determines the RTC frequency and prescaler
#define RTC_CC_VALUE                    8                                         //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
#define SAADC_SAMPLE_INTERVAL_MS        150                                       //Interval in milliseconds at which RTC times out and triggers SAADC sample task (
#define SAADC_CALIBRATION_INTERVAL      5                                         //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER         1                                         //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE                NRF_SAADC_OVERSAMPLE_DISABLED             //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE                0                                         //Set to 1 to enable BURST mode, otherwise set to 0.

#define SAMPLE_BUFFER                   2                                         //Buffer length for SAADC samples
#define PAGE_ADDR                       0x0007f000
#define END_PAGE                        0x00080000

typedef enum {
    DEVICE_IS_ADVERTISING,
    DEVICE_IS_CONNECTED,
    DEVICE_IS_IDLE,
    DEVICE_IS_STOPPING_ADV,
    DEVICE_IS_DISCONNECTING,
    DEVICE_WAS_DISCONNECTED
} device_status_t;

typedef struct saadc
{
    bool m_saadc_initialized;
    bool limit_exceeded;
    uint8_t samples_not_exceeded;
    uint32_t m_adc_evt_counter;
    bool notification_enabled;
    bool stop_sending;
} saadc_status_t;

typedef enum {
    MPU_SLEEP,
    MPU_ACCEL,
    MPU_GYRO,
    MPU_ACCEL_GYRO
} mpu_stages;

typedef struct {
    mpu_stages stage;
    mpu_stages prev_stage;
    bool status_changed;
    bool updating;
    bool hold;
} mpu_status_t;

static saadc_status_t          saadc_status  = {false, false, 20, 0, false, false};
static device_status_t         device_status = DEVICE_IS_IDLE;
static mpu_status_t            mpu_status = {MPU_SLEEP, false, false, false, MPU_SLEEP};

static nrf_saadc_value_t       m_buffer_pool[2][SAMPLE_BUFFER];
const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2);                     /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static uint32_t                rtc_ticks = RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS*1000, RTC_FREQUENCY);

void saadc_init(void);
void saadc_callback(nrf_drv_saadc_evt_t const *p_event);
static void advertising_start(bool erase_bonds);

BLE_BRUX_DEF(m_brux);
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

APP_TIMER_DEF(m_accel_notification);
APP_TIMER_DEF(m_gyro_notification);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint16_t	m_ble_brux_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BRUX_SERVICE_UUID, BLE_UUID_TYPE_BLE}
};

static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
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
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void notification_accel_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    //Talvez adicionar uma verificaçao se o mpu já foi configurado
    if (device_status == DEVICE_IS_CONNECTED && !mpu_status.status_changed) {
        mpu_status.updating = true;
        NRF_LOG_INFO("Reading Accel \r\n");
        MPU6050_read_accel();
        NRF_LOG_INFO("Accel Read\r\n");
        err_code = ble_brux_accel_update(&m_brux, m_sample_accel);
        APP_ERROR_CHECK(err_code);
        mpu_status.updating = false;
    }
}

static void notification_gyro_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    //Talvez adicionar uma verificaçao se o mpu já foi configurado
    if (device_status == DEVICE_IS_CONNECTED  && !mpu_status.status_changed) {
        mpu_status.updating = true;
        MPU6050_read_gyro();
        err_code = ble_brux_gyro_update(&m_brux, m_sample_gyro);
        APP_ERROR_CHECK(err_code);
        mpu_status.updating = false;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_accel_notification, APP_TIMER_MODE_REPEATED, notification_accel_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_gyro_notification, APP_TIMER_MODE_REPEATED, notification_gyro_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
}

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

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_brux_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_brux_max_data_len, m_ble_brux_max_data_len);
    } 
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);        
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void on_brux_evt(ble_brux_t	* p_brux_service,
			ble_brux_evt_t	* p_evt)
{
    ret_code_t err_code;
    
	switch(p_evt->evt_type)
	{
        case BLE_BRUX_ACCEL_NOTIFICATION_ENABLED:
            mpu_status.status_changed = true;
            
            mpu_status.stage = MPU_ACCEL_GYRO * (mpu_status.stage == MPU_GYRO) + MPU_ACCEL * (mpu_status.stage == MPU_SLEEP);   
            err_code = app_timer_start(m_accel_notification, NOTIFICATION_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            
            break;
        
        case BLE_BRUX_ACCEL_NOTIFICATION_DISABLED:
            while (mpu_status.updating);
            mpu_status.status_changed = true;
            mpu_status.stage = MPU_GYRO * (mpu_status.stage == MPU_ACCEL_GYRO) + MPU_SLEEP * (mpu_status.stage == MPU_ACCEL);
            
            err_code = app_timer_stop(m_accel_notification);
            APP_ERROR_CHECK(err_code);
            
            break;
        
        case BLE_BRUX_GYRO_NOTIFICATION_ENABLED:
            mpu_status.status_changed = true;
            mpu_status.stage = MPU_ACCEL_GYRO * (mpu_status.stage == MPU_ACCEL) + MPU_GYRO * (mpu_status.stage == MPU_SLEEP);
            
            err_code = app_timer_start(m_gyro_notification, NOTIFICATION_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            
            break;
        
        case BLE_BRUX_GYRO_NOTIFICATION_DISABLED:
            while (mpu_status.updating);
            mpu_status.status_changed = true;
            mpu_status.stage = MPU_ACCEL * (mpu_status.stage == MPU_ACCEL_GYRO) + MPU_SLEEP * (mpu_status.stage == MPU_GYRO);

            err_code = app_timer_stop(m_gyro_notification);
            APP_ERROR_CHECK(err_code);
            
            break;
		
        case BLE_BRUX_FORCE_NOTIFICATION_ENABLED:
            saadc_status.notification_enabled = true;
            break;
        
        case BLE_BRUX_FORCE_NOTIFICATION_DISABLED:
            saadc_status.notification_enabled = false;
            break;
        
        case BLE_BRUX_EVT_CONNECTED:
		    break;
		
        case BLE_BRUX_EVT_DISCONNECTED:
			break;
	 	
        default:
		    break;
	}
}	

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_brux_init_t    brux_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    
    //NRF_LOG_INFO("QWR");
    
    memset(&brux_init, 0, sizeof(brux_init));
    
    //NRF_LOG_INFO("MEMSET");
    
    brux_init.evt_handler	= on_brux_evt;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&brux_init.custom_value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&brux_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&brux_init.custom_value_char_attr_md.write_perm);
    
    //NRF_LOG_INFO("ANTES INIT");
    
    err_code = ble_brux_init(&m_brux, &brux_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
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
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    ret_code_t err_code;
	
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        
        if(!saadc_status.m_saadc_initialized)
        {
            saadc_init();                                              //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
        }
        saadc_status.m_saadc_initialized = true;                                    //Set SAADC as initialized
        
        nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
		
        /*    */
        err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
    }
}

/*static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();                        //Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}*/

static void rtc_config(void)
{

    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);                //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);                    //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                                   //Enable RTC
}

/**
 * @brief Função de inicialização do módulo SAADC
 * */
void saadc_init()
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;
    
    saadc_config.low_power_mode = true;                         //Enable low power mode
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;       //Set SAADC resolution to 12 bits
    saadc_config.oversample = SAADC_OVERSAMPLE;
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;              
    
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config.gain = NRF_SAADC_GAIN1_5;
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;

    if(SAADC_BURST_MODE)        channel_config.burst = NRF_SAADC_BURST_ENABLED;

    channel_config.pin_p = NRF_SAADC_INPUT_AIN0;
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

    nrfx_saadc_limits_set(0, NRF_DRV_SAADC_LIMITL_DISABLED, 250);
    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    channel_config.pin_p = NRF_SAADC_INPUT_AIN1;
    nrfx_saadc_limits_set(1, NRF_DRV_SAADC_LIMITL_DISABLED, 425);
    
    err_code = nrfx_saadc_channel_init(1, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLE_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLE_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Esta função será chamada sempre que algum evento for registado
 * pelo módulo SAADC
 * */
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    ret_code_t err_code;
    static int16_t sensor1, sensor2;
    
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        if ((saadc_status.m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)                         // Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            NRF_SAADC->EVENTS_CALIBRATEDONE = 0;                                                        // Clear the calibration event flag
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);                                     // Trigger calibration task
            while(!NRF_SAADC->EVENTS_CALIBRATEDONE);                                                    // Wait until calibration task is completed. The calibration tasks takes about 1000us with 10us acquisition time. Configuring shorter or longer acquisition time will make the calibration take shorter or longer respectively.
            while(NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));          // Set flag to trigger calibration in main context when SAADC is stopped
        }
        
        NRF_LOG_INFO("ADC event number: %d\r\n",(int)saadc_status.m_adc_evt_counter++);        // Print the event number on UART
        
        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLE_BUFFER);
        APP_ERROR_CHECK(err_code);

        sensor1 = p_event->data.done.p_buffer[0];
        sensor2 = p_event->data.done.p_buffer[1];
        
        if (!saadc_status.limit_exceeded) {
            if (!saadc_status.samples_not_exceeded && device_status != DEVICE_IS_IDLE) {
                device_status = DEVICE_IS_STOPPING_ADV * (device_status == DEVICE_IS_ADVERTISING) + DEVICE_IS_CONNECTED * (device_status == DEVICE_IS_CONNECTED);
                mpu_status.hold = true * (mpu_status.stage != MPU_SLEEP) + true * (mpu_status.hold);
                saadc_status.stop_sending = true * (saadc_status.notification_enabled);
            }
            else if (saadc_status.samples_not_exceeded)      {saadc_status.samples_not_exceeded--;}
        }
        else if (saadc_status.limit_exceeded){
            NRF_LOG_INFO("Value: %d\n", sensor1);
            NRF_LOG_INFO("Value: %d\n", sensor2);
            
            if (device_status == DEVICE_IS_IDLE) {
                bool erase_bonds;
                advertising_start(erase_bonds);
                device_status = DEVICE_IS_ADVERTISING;
            }

            saadc_status.limit_exceeded = false;
            saadc_status.samples_not_exceeded = 20;
        }

        nrf_drv_saadc_uninit();                                                                   // Uninitialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               // Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         // Clear the SAADC interrupt if set
        saadc_status.m_saadc_initialized = false;                                                 // Set SAADC as uninitialized

        if (saadc_status.notification_enabled && !saadc_status.stop_sending) {
            int16_t sensor_values[] = {sensor1, sensor2};
            ble_brux_force_update(&m_brux, sensor_values);
        }
    }

    if (p_event->type == NRF_DRV_SAADC_EVT_LIMIT)
    {
        NRF_LOG_INFO("YEEEES\r\n");
        if (mpu_status.hold) {
            mpu_status.status_changed = true;
            mpu_status.stage = mpu_status.prev_stage;
            mpu_status.prev_stage = MPU_SLEEP;
        }
        saadc_status.limit_exceeded = true;
        saadc_status.stop_sending = false;
    }
}
/**
 * @brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
/*static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}*/

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
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
            APP_ERROR_CHECK(err_code);
            device_status = DEVICE_IS_IDLE;
            //sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
	        device_status = DEVICE_IS_ADVERTISING;
            saadc_status.notification_enabled = false;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            mpu_status.status_changed = true;
            mpu_status.stage = MPU_SLEEP;
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            device_status = DEVICE_IS_CONNECTED;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
	    err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
	    APP_ERROR_CHECK(err_code);
	    break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
	    err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
	    APP_ERROR_CHECK(err_code);
    
    //case BLE_GAP_EVT_ADV_SET_TERMINATED:
    //    NRF_LOG_INFO("Event GAP occured\r\n");
    //    break;
    default:
        //NRF_LOG_INFO("Teste\r\n");    // No implementation needed.
        break;
    }
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
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            //sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
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


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

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


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    bool erase_bonds;
    /*  Utiliza o regulador DC/DC interno (melhor consumo energético) */
    NRF_POWER->DCDCEN = 1;

    // Initialize.
    log_init();
    //NRF_LOG_INFO("LOGS");
    
    timers_init();
    //NRF_LOG_INFO("TIMERS");
    
    buttons_leds_init(&erase_bonds);
    //NRF_LOG_INFO("BUTTONS");
    
    power_management_init();
    //NRF_LOG_INFO("POWER");
    
    ble_stack_init();
    //NRF_LOG_INFO("STACK");
    
    gap_params_init();
    //NRF_LOG_INFO("PARAMS");
    
    gatt_init();
    //NRF_LOG_INFO("GATT INIT");
    
    services_init();
    NRF_LOG_INFO("SERVICES");
    
    advertising_init();
    NRF_LOG_INFO("ADV");
    
    conn_params_init();
    NRF_LOG_INFO("CONN PARAMS");

    // Start execution.
    NRF_LOG_INFO("Template example started.");
    
    //lfclk_config();                                  // Configure low frequency 32kHz clock
    //NRF_LOG_INFO("lfcll");
    
    rtc_config();                                    // Configure RTC. The RTC will generate periodic interrupts. Requires 32kHz clock to operate.
    NRF_LOG_INFO("rtc");
    
    application_timers_start();
    NRF_LOG_INFO("application timers");

    //advertising_start(erase_bonds);

    twi_init();

    MPU6050_sleep_mode();

    // Enter main loop.
    for (;;)
    {
        if (device_status == DEVICE_IS_STOPPING_ADV) {
            NRF_LOG_INFO("Stop Adv\r\n");
            err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
            APP_ERROR_CHECK(err_code);
            device_status = DEVICE_IS_IDLE;
            NRF_LOG_INFO("Success\r\n");
        }

        if (mpu_status.status_changed) {
            switch (mpu_status.stage) {
                case MPU_SLEEP:
                    MPU6050_sleep_mode();
                    
                    err_code = app_timer_stop(m_accel_notification);
                    APP_ERROR_CHECK(err_code);
                    
                    err_code = app_timer_stop(m_gyro_notification);
                    APP_ERROR_CHECK(err_code);
                    break;
                case MPU_ACCEL:
                    MPU6050_accelerometer();

                    if (mpu_status.hold) {
                        err_code = app_timer_start(m_accel_notification, NOTIFICATION_INTERVAL, NULL);
                        APP_ERROR_CHECK(err_code);
                        mpu_status.hold = false;
                        NRF_LOG_INFO("RESTART MPU_ACCEL\r\n");
                    }

                    break;
                case MPU_GYRO:
                    MPU6050_gyroscope();

                    if (mpu_status.hold) {
                        err_code = app_timer_start(m_gyro_notification, NOTIFICATION_INTERVAL, NULL);
                        APP_ERROR_CHECK(err_code);
                        mpu_status.hold = false;
                    }
                    
                    break;
                case MPU_ACCEL_GYRO:
                    MPU6050_accel_and_gyro();
                    
                    if (mpu_status.hold) {
                        err_code = app_timer_start(m_accel_notification, NOTIFICATION_INTERVAL, NULL);
                        APP_ERROR_CHECK(err_code);

                        err_code = app_timer_start(m_gyro_notification, NOTIFICATION_INTERVAL, NULL);
                        APP_ERROR_CHECK(err_code);

                        mpu_status.hold = false;
                    }
                    break;
                default:
                    break;
            }
            mpu_status.status_changed = false;
        }

        if (mpu_status.hold && mpu_status.prev_stage == MPU_SLEEP) {
            mpu_status.prev_stage = mpu_status.stage;
            mpu_status.stage = MPU_SLEEP;

            MPU6050_sleep_mode();
                    
            err_code = app_timer_stop(m_accel_notification);
            APP_ERROR_CHECK(err_code);
            
            err_code = app_timer_stop(m_gyro_notification);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("HOLDING\r\n");
        }
        else {
            idle_state_handle();
        }
                
        NRF_LOG_FLUSH();
    }
}