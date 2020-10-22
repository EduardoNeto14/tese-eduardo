#ifndef BLE_BRUX_H__
#define BLE_BRUX_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BRUX_BASE_UUID		{0x20, 0x4F, 0x20, 0x4D, 0x20, 0x53, 0x20, 0x49, 0x20, 0x58, 0x20, 0x55, 0x20, 0x52, 0x20, 0x42}

#define BRUX_SERVICE_UUID 	0x0001
#define BRUX_FORCE_UUID		0x1111
#define BRUX_ACCEL_UUID		0x2222
#define BRUX_GYRO_UUID		0x3333
#define BRUX_BATTERY_UUID	0x5555

#define BRUX_TEMP_UUID		0x4444
/**@brief Custom Service structure. This contains various status information for the service. */

#ifndef BLE_BRUX_BLE_OBSERVER_PRIO
#define BLE_BRUX_BLE_OBSERVER_PRIO 2
#endif

#define BLE_BRUX_DEF(_name) 			\
static ble_brux_t	_name;			\
NRF_SDH_BLE_OBSERVER(_name ## _obs , 		\
		BLE_BRUX_BLE_OBSERVER_PRIO, 	\
		ble_brux_on_ble_evt, &_name);


typedef enum
{
    BLE_BRUX_ACCEL_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_BRUX_ACCEL_NOTIFICATION_DISABLED,                             /**< Custom value notification disabled event. */
	BLE_BRUX_GYRO_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_BRUX_GYRO_NOTIFICATION_DISABLED,                             /**< Custom value notification disabled event. */
	BLE_BRUX_FORCE_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_BRUX_FORCE_NOTIFICATION_DISABLED,                             /**< Custom value notification disabled event. */
    //BLE_BRUX_EVT_DISCONNECTED,
    //BLE_BRUX_EVT_CONNECTED
} ble_brux_evt_type_t;

typedef struct
{
	ble_brux_evt_type_t evt_type;                                  /**< Type of event. */
} ble_brux_evt_t;

typedef struct ble_brux_s ble_brux_t;

typedef void (*ble_brux_evt_handler_t) (ble_brux_t * p_bas, ble_brux_evt_t * p_evt);

typedef struct
{
	ble_brux_evt_handler_t			evt_handler;	
	ble_srv_cccd_security_mode_t	custom_value_char_attr_md;
} ble_brux_init_t;

struct  ble_brux_s
{
	ble_brux_evt_handler_t		evt_handler;
	uint16_t					service_handler;
	ble_gatts_char_handles_t	brux_force_handles;
	ble_gatts_char_handles_t	brux_accel_handles;
	ble_gatts_char_handles_t	brux_gyro_handles;
	ble_gatts_char_handles_t	brux_battery_handles;
	uint16_t					conn_handle;
	uint8_t						uuid_type;
};

uint32_t ble_brux_init(ble_brux_t * p_brux, const ble_brux_init_t * p_brux_init);

void ble_brux_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_brux_force_update(ble_brux_t * p_brux, int16_t * force_value);

uint32_t ble_brux_accel_update(ble_brux_t * p_brux, uint8_t * accel_values);

uint32_t ble_brux_gyro_update(ble_brux_t * p_brux, uint8_t * gyro_values);
#endif