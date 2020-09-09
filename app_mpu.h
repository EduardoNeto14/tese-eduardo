#include "nrf_drv_twi.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     		    0

#define MPU_I2C_ADDRESS			        0x68
#define MPU_POWER_REG			        0x6B
#define MPU_POWER_CYCLE			        0b00000000
#define MPU_READ_TIMEOUT		        2000
#define MPU_SAMP_FREQ			        250

#define MPU_GYRO_CFG_REG		        0x1B
#define MPU_GYRO_READ_REG		        0x43
#define MPU_GYRO_READ_REG_SIZE		    6
#define MPU_GYRO_CFG_250DEG		        0b00000000
#define MPU_GYRO_READINGSCALE_250DEG	0b00001000
#define MPU_CALIBRATE_READING_NUM	    2000

#define MPU_TEMP_READ_REG		        0x41
#define MPU_TEMP_READ_REG_SIZE		    2

#define MPU_ACCEL_CFG_REG		        0x1C
#define MPU_ACCEL_READ_REG		        0x3B
#define MPU_ACCEL_READ_REG_SIZE		    6
#define MPU_ACCEL_CFG_2G		        0b00000000
#define MPU_ACCEL_READINGSCALE_2G	    16384.0
#define MPU_ACCEL_CFG_4G		        0b00001000
#define MPU_ACCEL_READINGSCALE_4G	    8192.0
#define MPU_ACCEL_CFG_8G		        0b00010000
#define MPU_ACCEL_READINGSCALE_8G	    4096.0
#define MPU_ACCEL_CFG_16G		        0b00011000
#define MPU_ACCEL_READINGSCALE_16G	    2048.0

#define SCL_PIN             27    // SCL signal pin
#define SDA_PIN             26    // SDA signal pin

volatile bool m_xfer_done = false;

uint8_t m_sample_accel[MPU_ACCEL_READ_REG_SIZE];
uint8_t m_sample_gyro[MPU_GYRO_READ_REG_SIZE];

const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void MPU6050_set_mode(void)
{
    ret_code_t err_code;

    uint8_t reg[2] = {MPU_POWER_REG, MPU_POWER_CYCLE};
    err_code = nrf_drv_twi_tx(&m_twi, MPU_I2C_ADDRESS, &reg[0], sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    reg[0] = MPU_GYRO_CFG_REG;
    reg[1] =  MPU_GYRO_CFG_250DEG;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MPU_I2C_ADDRESS, &reg[0], sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    reg[0] = MPU_ACCEL_CFG_REG;
    reg[1] =  MPU_ACCEL_CFG_2G;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MPU_I2C_ADDRESS, &reg[0], sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            
	    if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                #ifdef DEBUG
                NRF_LOG_INFO("RX DONE");
                #endif

                m_xfer_done = true;
            }
            
	    m_xfer_done = true;
	    break;
        
	default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_mpu6050_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_mpu6050_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void MPU6050_read_accel()
{
    #ifdef DEBUG
    SEGGER_RTT_WriteString(0,"TX to ACCEL\n");
    #endif

    ret_code_t err_code;
    uint8_t reg[2]; 
    
    m_xfer_done = false;
    reg[0] = MPU_ACCEL_READ_REG;
    
    err_code = nrf_drv_twi_tx(&m_twi, MPU_I2C_ADDRESS, &reg[0], 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    
    m_xfer_done = false;
    
    #ifdef DEBUG
    SEGGER_RTT_WriteString(0, "DONE\n");
    #endif

    err_code = nrf_drv_twi_rx(&m_twi, MPU_I2C_ADDRESS, &m_sample_accel[0], sizeof(m_sample_accel));
    APP_ERROR_CHECK(err_code);

    while(m_xfer_done == false);
}

static void MPU6050_read_gyro()
{
    #ifdef DEBUG
    SEGGER_RTT_WriteString(0, "TX to GYRO\n");
    #endif

    ret_code_t err_code;
    uint8_t reg[2];

    m_xfer_done = false;
    reg[0] = MPU_GYRO_READ_REG;
    
    err_code = nrf_drv_twi_tx(&m_twi, MPU_I2C_ADDRESS, &reg[0], 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    #ifdef DEBUG
    SEGGER_RTT_WriteString(0, "DONE\n");
    #endif

    err_code = nrf_drv_twi_rx(&m_twi, MPU_I2C_ADDRESS, &m_sample_gyro[0], sizeof(m_sample_gyro));
    APP_ERROR_CHECK(err_code);
    
    while(m_xfer_done == false);
}


/**
 * @brief Function for reading data from MPU.
 
void read_sensor_data()
{
    ret_code_t err_code;
    m_xfer_done = false;

    MPU6050_read_accel();    
    
    m_xfer_done = false;
    
    MPU6050_read_gyro();    
    err_code = nrf_drv_twi_rx(&m_twi, MPU_I2C_ADDRESS, &m_sample[0], sizeof(m_sample));
    APP_ERROR_CHECK(err_code);

    while(m_xfer_done == false);
    #ifdef DEBUG
    SEGGER_RTT_WriteString(0, "NICE\n");
    #endif
}*/