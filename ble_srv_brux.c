#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_srv_brux.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

static uint32_t bruxism_force_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init);
static uint32_t bruxism_gyro_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init);
static uint32_t bruxism_battery_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init);
static uint32_t bruxism_accel_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init);
static void on_connect(ble_brux_t * p_brux, ble_evt_t const * p_ble_evt);
static void on_disconnect(ble_brux_t * p_brux, ble_evt_t const * p_ble_evt);

uint32_t ble_brux_init(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init)
{
	if (p_brux == NULL || p_brux_init == NULL)
		return NRF_ERROR_NULL;

	uint32_t	err_code;
	ble_uuid_t 	ble_uuid;

	p_brux->evt_handler	= p_brux_init->evt_handler;
	p_brux->conn_handle	= BLE_CONN_HANDLE_INVALID;
	ble_uuid128_t	base_uuid = {BRUX_BASE_UUID};
	
	NRF_LOG_INFO("SERVICE BRUX INIT");
	
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_brux->uuid_type);
	VERIFY_SUCCESS(err_code);
	
	NRF_LOG_INFO("SUCESSO");
	
	ble_uuid.type = p_brux->uuid_type;
	ble_uuid.uuid = BRUX_SERVICE_UUID;
	
	NRF_LOG_INFO("SERVICE BRUX ADD");
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_brux->service_handler);
	if (err_code != NRF_SUCCESS)
		return err_code;
	NRF_LOG_INFO("SUCESSO");
	
	err_code = bruxism_force_char_add(p_brux, p_brux_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = bruxism_accel_char_add(p_brux, p_brux_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	err_code = bruxism_gyro_char_add(p_brux, p_brux_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	return bruxism_battery_char_add(p_brux, p_brux_init);
}

static uint32_t bruxism_force_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init)
{
	uint32_t			err_code;
	ble_gatts_char_md_t	char_md;
	ble_gatts_attr_md_t	cccd_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_brux_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read 	= 1;
	char_md.char_props.write	= 1;
	char_md.char_props.notify	= 1;
	
	static char user_desc[]         = "FORCE VALUE";

	char_md.p_char_user_desc        = (uint8_t *) user_desc;
	char_md.char_user_desc_size     = strlen(user_desc);
	char_md.char_user_desc_max_size = strlen(user_desc);

	char_md.p_char_pf		= NULL;
	char_md.p_user_desc_md	= NULL;
	char_md.p_cccd_md		= &cccd_md;
	char_md.p_sccd_md		= NULL;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm	= p_brux_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm	= p_brux_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc		= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth		= 0;
	attr_md.wr_auth		= 0;
	attr_md.vlen		= 0;

	ble_uuid.type		= p_brux->uuid_type;
	ble_uuid.uuid		= BRUX_FORCE_UUID;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= 2*sizeof(int16_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= 2*sizeof(int16_t);
	int16_t value[2]			= {0xFFFF, 0xFFFF};
	attr_char_value.p_value		= (uint8_t *) value;

	err_code = sd_ble_gatts_characteristic_add(p_brux->service_handler, &char_md, &attr_char_value, &p_brux->brux_force_handles);

	if (err_code != NRF_SUCCESS)
		return err_code;

	return NRF_SUCCESS;	
}

static uint32_t bruxism_accel_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init)
{
	uint32_t			err_code;
	ble_gatts_char_md_t	char_md;
	ble_gatts_attr_md_t	cccd_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_brux_init->custom_value_char_attr_md.cccd_write_perm;
    	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read 	= 1;
	char_md.char_props.write	= 1;
	char_md.char_props.notify	= 1;
	
	static char user_desc[]         = "ACCEL VALUE";

	char_md.p_char_user_desc        = (uint8_t *) user_desc;
	char_md.char_user_desc_size     = strlen(user_desc);
	char_md.char_user_desc_max_size = strlen(user_desc);

	char_md.p_char_pf		= NULL;
	char_md.p_user_desc_md	= NULL;
	char_md.p_cccd_md		= &cccd_md;
	char_md.p_sccd_md		= NULL;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm	= p_brux_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm	= p_brux_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc		= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth		= 0;
	attr_md.wr_auth		= 0;
	attr_md.vlen		= 0;

	ble_uuid.type		= p_brux->uuid_type;
	ble_uuid.uuid		= BRUX_ACCEL_UUID;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= 6*sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= 6*sizeof(uint8_t);
	uint8_t value[6]			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	attr_char_value.p_value		= value;

	err_code = sd_ble_gatts_characteristic_add(p_brux->service_handler, &char_md, &attr_char_value, &p_brux->brux_accel_handles);

	if (err_code != NRF_SUCCESS)
		return err_code;

	return NRF_SUCCESS;	
}

static uint32_t bruxism_gyro_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init)
{
	uint32_t			err_code;
	ble_gatts_char_md_t	char_md;
	ble_gatts_attr_md_t	cccd_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_brux_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read 	= 1;
	char_md.char_props.write	= 1;
	char_md.char_props.notify	= 1;
	
	static char user_desc[]         = "GYRO VALUE";

	char_md.p_char_user_desc        = (uint8_t *) user_desc;
	char_md.char_user_desc_size     = strlen(user_desc);
	char_md.char_user_desc_max_size = strlen(user_desc);
	
	char_md.p_char_pf		= NULL;
	char_md.p_user_desc_md	= NULL;
	char_md.p_cccd_md		= &cccd_md;
	char_md.p_sccd_md		= NULL;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm	= p_brux_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm	= p_brux_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc		= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth		= 0;
	attr_md.wr_auth		= 0;
	attr_md.vlen		= 0;

	ble_uuid.type		= p_brux->uuid_type;
	ble_uuid.uuid		= BRUX_GYRO_UUID;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= 6*sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= 6*sizeof(uint8_t);
	uint8_t value[6]		= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	attr_char_value.p_value		= value;

	err_code = sd_ble_gatts_characteristic_add(p_brux->service_handler, &char_md, &attr_char_value, &p_brux->brux_gyro_handles);

	if (err_code != NRF_SUCCESS)
		return err_code;

	return NRF_SUCCESS;	
}

static uint32_t bruxism_battery_char_add(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init)
{
	uint32_t		err_code;
	ble_gatts_char_md_t	char_md;
	ble_gatts_attr_md_t	cccd_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t		ble_uuid;
	ble_gatts_attr_md_t	attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_brux_init->custom_value_char_attr_md.cccd_write_perm;
    	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read 	= 1;
	char_md.char_props.write	= 0;
	char_md.char_props.notify	= 1;
	
	static char user_desc[]         = "BATTERY VALUE";

	char_md.p_char_user_desc        = (uint8_t *) user_desc;
	char_md.char_user_desc_size     = strlen(user_desc);
	char_md.char_user_desc_max_size = strlen(user_desc);
	
	char_md.p_char_pf		= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md		= &cccd_md;
	char_md.p_sccd_md		= NULL;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm	= p_brux_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm	= p_brux_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc		= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth		= 0;
	attr_md.wr_auth		= 0;
	attr_md.vlen		= 0;

	ble_uuid.type		= p_brux->uuid_type;
	ble_uuid.uuid		= BRUX_BATTERY_UUID;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint8_t);
	uint16_t value			= 100;
	attr_char_value.p_value		= (uint8_t *) &value;

	err_code = sd_ble_gatts_characteristic_add(p_brux->service_handler, &char_md, &attr_char_value, &p_brux->brux_battery_handles);

	if (err_code != NRF_SUCCESS)
		return err_code;

	return NRF_SUCCESS;	
}

static void on_write(ble_brux_t * p_brux, ble_evt_t const * p_ble_evt)
{
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if ((p_evt_write->handle == p_brux->brux_accel_handles.cccd_handle) && (p_evt_write->len == 2) )
	{
		
		if (p_brux->evt_handler != NULL)
        {
            ble_brux_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BRUX_ACCEL_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BRUX_ACCEL_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_brux->evt_handler(p_brux, &evt);
        }
	}
	
	else if ((p_evt_write->handle == p_brux->brux_gyro_handles.cccd_handle) && (p_evt_write->len == 2) )
	{
		
		if (p_brux->evt_handler != NULL)
        {
            ble_brux_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BRUX_GYRO_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BRUX_GYRO_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_brux->evt_handler(p_brux, &evt);
        }
	}

	else if ((p_evt_write->handle == p_brux->brux_force_handles.cccd_handle) && (p_evt_write->len == 2) )
	{
		
		if (p_brux->evt_handler != NULL)
        {
            ble_brux_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BRUX_FORCE_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BRUX_FORCE_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_brux->evt_handler(p_brux, &evt);
        }
	}
}


void ble_brux_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
	ble_brux_t * p_brux = (ble_brux_t *) p_context;

	if (p_brux == NULL || p_ble_evt == NULL)
		return;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_brux, p_ble_evt);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_brux, p_ble_evt);
			break;

		case BLE_GATTS_EVT_WRITE:
			on_write(p_brux, p_ble_evt);
			break;

		default:
			break;

	}

}

static void on_connect(ble_brux_t * p_brux, ble_evt_t const * p_ble_evt)
{
	p_brux->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_brux_t * p_brux, ble_evt_t const * p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_brux->conn_handle = BLE_CONN_HANDLE_INVALID;
}

uint32_t ble_brux_force_update(ble_brux_t * p_brux, int16_t * force_value)
{
    NRF_LOG_INFO("In ble_brux_force_update. \r\n");
    if (p_brux == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 2 * sizeof(int16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *) force_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_brux->conn_handle,
                                      p_brux->brux_force_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_brux->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_brux->brux_force_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_brux->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
    }


    return err_code;
}

uint32_t ble_brux_accel_update(ble_brux_t * p_brux, uint8_t * accel_values)
{
    NRF_LOG_INFO("In ble_brux_accel_update. \r\n");
    
	if (p_brux == NULL)
    {
        return NRF_ERROR_NULL;
    }

	NRF_LOG_INFO("[0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n", accel_values[0], accel_values[1], accel_values[2], accel_values[3], accel_values[4], accel_values[5]);
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 6*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *) accel_values;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_brux->conn_handle,
                                      p_brux->brux_accel_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_brux->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_brux->brux_accel_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_brux->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
    }


    return err_code;
}

uint32_t ble_brux_gyro_update(ble_brux_t * p_brux, uint8_t * gyro_values)
{
    NRF_LOG_INFO("In ble_brux_gyro_update. \r\n");
    if (p_brux == NULL)
    {
        return NRF_ERROR_NULL;
    }
		NRF_LOG_INFO("[0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n", gyro_values[0], gyro_values[1], gyro_values[2], gyro_values[3], gyro_values[4], gyro_values[5]);
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 6*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *) gyro_values;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_brux->conn_handle,
                                      p_brux->brux_gyro_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_brux->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_brux->brux_gyro_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_brux->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
    }


    return err_code;
}

uint32_t ble_brux_battery_update(ble_brux_t * p_brux, uint8_t battery_value)
{
    NRF_LOG_INFO("In ble_brux_force_update. \r\n");
    if (p_brux == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *) &battery_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_brux->conn_handle,
                                      p_brux->brux_battery_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_brux->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_brux->brux_battery_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_brux->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n");
    }


    return err_code;
}
