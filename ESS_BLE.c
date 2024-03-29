/*
 * This file has been automatically generated by the WICED 20719-B1 Designer.
 * Bluetooth Application.
 *
 */

/** ESS_BLE.c
 *
 */

#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_wdog.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_app_common.h"
#include "sparcommon.h"
#include "string.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#include "wiced_hal_pspi.h"
#include "ESS_BLE_db.h"
#include "wiced_bt_cfg.h"
#include "sensirion.h"


/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define TRANS_UART_BUFFER_SIZE  1024
#define TRANS_UART_BUFFER_COUNT 2
#define WICED_BT_TRACE_ENABLE

/*******************************************************************
 * Variable Definitions
 ******************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
// Transport pool for sending RFCOMM data to host
static wiced_transport_buffer_pool_t* transport_pool = NULL;

static wiced_timer_t sht_timer;
static wiced_timer_t sgp_timer;

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                   ess_ble_app_init               ( void );
static wiced_bt_dev_status_t  ess_ble_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   ess_ble_set_advertisement_data ( void );
static void                   ess_ble_advertisement_stopped  ( void );
static void                   ess_ble_reset_device           ( void );
/* GATT Registration Callbacks */
static wiced_bt_gatt_status_t ess_ble_write_handler          ( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id );
static wiced_bt_gatt_status_t ess_ble_read_handler           ( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id );
static wiced_bt_gatt_status_t ess_ble_connect_callback       ( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t ess_ble_server_callback        ( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data );
static wiced_bt_gatt_status_t ess_ble_event_handler          ( wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data );
static uint32_t               hci_control_process_rx_cmd     ( uint8_t* p_data, uint32_t len );
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                   ess_ble_trace_callback         ( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif

/*******************************************************************
 * Macro Definitions
 ******************************************************************/
// Macro to extract uint16_t from little-endian byte array
#define LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(byte_array) \
        (uint16_t)( ((byte_array)[0] | ((byte_array)[1] << 8)) )

/*******************************************************************
 * Transport Configuration
 ******************************************************************/
wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,              /**< Wiced transport type. */
    {
        WICED_TRANSPORT_UART_HCI_MODE, /**<  UART mode, HCI or Raw */
        HCI_UART_DEFAULT_BAUD          /**<  UART baud rate */
    },
    {
        TRANS_UART_BUFFER_SIZE,        /**<  Rx Buffer Size */
        TRANS_UART_BUFFER_COUNT        /**<  Rx Buffer Count */
    },
    NULL,                              /**< Wiced transport status handler.*/
    hci_control_process_rx_cmd,        /**< Wiced transport receive data handler. */
    NULL                               /**< Wiced transport tx complete callback. */
};

/*******************************************************************
 * GATT Initial Value Arrays
 ******************************************************************/
uint8_t ess_ble_generic_access_device_name[] = {'E','S','S','_','B','L','E'};
uint8_t ess_ble_generic_access_appearance[]  = {0x00,0x00};
uint8_t ess_ble_ess_service_button[]         = {0x00};

uint8_t ess_ble_ess_service_temperature[]                   = {0x00,0x01};
uint8_t ess_ble_ess_service_temperature_user_description[]  = "Temperature";
uint8_t ess_ble_ess_service_temperature_cccd[]              = {0x00,0x00};
uint8_t ess_ble_ess_service_humidity[]                      = {0x00,0x01};
uint8_t ess_ble_ess_service_humidity_user_description[]     = "Humidity";
uint8_t ess_ble_ess_service_humidity_cccd[]                 = {0x00,0x00};
uint8_t ess_ble_ess_service_eco2[]                          = {0x00,0x01};
uint8_t ess_ble_ess_service_eco2_user_description[]         = "eCO2";
uint8_t ess_ble_ess_service_eco2_cccd[]                     = {0x00,0x00};
uint8_t ess_ble_ess_service_tvoc[]                          = {0x00,0x01};
uint8_t ess_ble_ess_service_tvoc_user_description[]         = "TVOC";
uint8_t ess_ble_ess_service_tvoc_cccd[]                     = {0x00,0x00};

/*******************************************************************
 * GATT Lookup Table
 ******************************************************************/

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
gatt_db_lookup_table ess_ble_gatt_db_ext_attr_tbl[] =
{
    /* { attribute handle,                  maxlen, curlen, attribute data } */
    {HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE, 7,      7,      ess_ble_generic_access_device_name},
    {HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,  2,      2,      ess_ble_generic_access_appearance},
    {HDLC_ESS_SERVICE_BUTTON_VALUE,         1,      1,      ess_ble_ess_service_button},

    {HDLC_ESS_SERVICE_TEMPERATURE_VALUE,    2,      2,      ess_ble_ess_service_temperature},
    {HDLD_ESS_SERVICE_TEMPERATURE_USER_DESCRIPTION,   sizeof(ess_ble_ess_service_temperature_user_description)-1, sizeof(ess_ble_ess_service_temperature_user_description)-1, ess_ble_ess_service_temperature_user_description},
    {HDLD_ESS_SERVICE_TEMPERATURE_CLIENT_CONFIGURATION, 2,      2,      ess_ble_ess_service_temperature_cccd},

    {HDLC_ESS_SERVICE_HUMIDITY_VALUE,    2,      2,      ess_ble_ess_service_humidity},
    {HDLD_ESS_SERVICE_HUMIDITY_USER_DESCRIPTION,   sizeof(ess_ble_ess_service_humidity_user_description)-1, sizeof(ess_ble_ess_service_humidity_user_description)-1, ess_ble_ess_service_humidity_user_description},
    {HDLD_ESS_SERVICE_HUMIDITY_CLIENT_CONFIGURATION, 2,      2,      ess_ble_ess_service_humidity_cccd},

    {HDLC_ESS_SERVICE_ECO2_VALUE,    2,      2,      ess_ble_ess_service_eco2},
    {HDLD_ESS_SERVICE_ECO2_USER_DESCRIPTION,   sizeof(ess_ble_ess_service_eco2_user_description)-1, sizeof(ess_ble_ess_service_eco2_user_description)-1, ess_ble_ess_service_eco2_user_description},
    {HDLD_ESS_SERVICE_ECO2_CLIENT_CONFIGURATION, 2,      2,      ess_ble_ess_service_eco2_cccd},

    {HDLC_ESS_SERVICE_TVOC_VALUE,    2,      2,      ess_ble_ess_service_tvoc},
    {HDLD_ESS_SERVICE_TVOC_USER_DESCRIPTION,   sizeof(ess_ble_ess_service_tvoc_user_description)-1, sizeof(ess_ble_ess_service_tvoc_user_description)-1, ess_ble_ess_service_tvoc_user_description},
    {HDLD_ESS_SERVICE_TVOC_CLIENT_CONFIGURATION, 2,      2,      ess_ble_ess_service_tvoc_cccd},
};

// Number of Lookup Table Entries
const uint16_t ess_ble_gatt_db_ext_attr_tbl_size = ( sizeof ( ess_ble_gatt_db_ext_attr_tbl ) / sizeof ( gatt_db_lookup_table ) );
uint16_t connection_id = 0;
/*******************************************************************
 * Function Definitions
 ******************************************************************/

/******************************************************************************
 * This function reads the value from SHTC1 and prints it
 *****************************************************************************/
void sht_cb(uint32_t arg){
    int32_t temp;
    int32_t hum;
    int8_t res = sht_read(&temp, &hum);
    if(res){
        WICED_BT_TRACE("SHT Read failed: %d\r\n",res);
    }else{
        ess_ble_ess_service_temperature[0] = (uint8_t)(0xff&(temp>>8));
        ess_ble_ess_service_temperature[1] = (uint8_t)(0xff&temp);
        ess_ble_ess_service_humidity[0] = (uint8_t)(0xff&(hum>>8));
        ess_ble_ess_service_humidity[1] = (uint8_t)(0xff&hum);
        if(ess_ble_ess_service_temperature_cccd[0] & GATT_CLIENT_CONFIG_NOTIFICATION){
            wiced_bt_gatt_send_notification(connection_id, HDLC_ESS_SERVICE_TEMPERATURE_VALUE, sizeof(ess_ble_ess_service_temperature), ess_ble_ess_service_temperature);
            WICED_BT_TRACE("Sent Temperature Value %d\n\r", temp);
        }
        if(ess_ble_ess_service_humidity_cccd[0] & GATT_CLIENT_CONFIG_NOTIFICATION){
            wiced_bt_gatt_send_notification(connection_id, HDLC_ESS_SERVICE_HUMIDITY_VALUE, sizeof(ess_ble_ess_service_humidity), ess_ble_ess_service_humidity);
            WICED_BT_TRACE("Sent Humidity Value %d\n\r", hum);
        }
    }
    sht_measure();
}

/******************************************************************************
 * This function reads the value from SGP30 and prints it
 *****************************************************************************/
void sgp_cb(uint32_t arg){
    uint16_t eco2;
    uint16_t voc;
    int8_t res = sgp_read_iaq(&eco2, &voc);
    if(res){
        WICED_BT_TRACE("SGP Read failed: %d\r\n",res);
    }else{
        ess_ble_ess_service_eco2[0] = (uint8_t)(0xff&(eco2>>8));
        ess_ble_ess_service_eco2[1] = (uint8_t)(0xff&eco2);
        ess_ble_ess_service_tvoc[0] = (uint8_t)(0xff&(voc>>8));
        ess_ble_ess_service_tvoc[1] = (uint8_t)(0xff&voc);
        if(ess_ble_ess_service_eco2_cccd[0] & GATT_CLIENT_CONFIG_NOTIFICATION){
            wiced_bt_gatt_send_notification(connection_id, HDLC_ESS_SERVICE_ECO2_VALUE, sizeof(ess_ble_ess_service_eco2), ess_ble_ess_service_eco2);
            WICED_BT_TRACE("Sent eCO2 Value %d\n\r", eco2);
        }
        if(ess_ble_ess_service_tvoc_cccd[0] & GATT_CLIENT_CONFIG_NOTIFICATION){
            wiced_bt_gatt_send_notification(connection_id, HDLC_ESS_SERVICE_TVOC_VALUE, sizeof(ess_ble_ess_service_tvoc), ess_ble_ess_service_tvoc);
            WICED_BT_TRACE("Sent TVOC Value %d\n\r", voc);
        }
    }
}


/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
void application_start(void)
{
    /* Initialize the transport configuration */
    wiced_transport_init( &transport_cfg );

    /* Initialize Transport Buffer Pool */
    transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE, TRANS_UART_BUFFER_COUNT );

#if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
    /* Set the Debug UART as WICED_ROUTE_DEBUG_NONE to get rid of prints */
    //  wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );

    /* Set Debug UART as WICED_ROUTE_DEBUG_TO_PUART to see debug traces on Peripheral UART (PUART) */
      wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    /* Set the Debug UART as WICED_ROUTE_DEBUG_TO_WICED_UART to send debug strings over the WICED debug interface */
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#endif

    /* Initialize Bluetooth Controller and Host Stack */
    wiced_bt_stack_init(ess_ble_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void ess_ble_app_init(void)
{
    /* Initialize Application */
    wiced_bt_app_init();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set Advertisement Data */
    ess_ble_set_advertisement_data();

    /* Register with stack to receive GATT callback */
    wiced_bt_gatt_register( ess_ble_event_handler );

    /* Initialize GATT Database */
    wiced_bt_gatt_db_init( gatt_database, gatt_database_len );

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'wiced_bt_cfg.c' */
    /* TODO: Make sure that this is the desired behavior. */
    wiced_hal_i2c_init();

    if(sht_probe()){
        WICED_BT_TRACE("SHT Probe Failed\r\n");
    }else{
        WICED_BT_TRACE("SHT Probe Success\r\n");
    }
    if(sht_measure()){
        WICED_BT_TRACE("Failed to start SHT measurement\r\n");
    }
    if ( WICED_SUCCESS == wiced_init_timer( &sht_timer, &sht_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER )) {
        if ( WICED_SUCCESS != wiced_start_timer( &sht_timer, 1000 )) {
            WICED_BT_TRACE( "Seconds Timer Error\n\r" );
        }
    }
    if(sgp_init()){
        WICED_BT_TRACE("Failed to init SGP measurement\r\n");
    }
    if ( WICED_SUCCESS == wiced_init_timer( &sgp_timer, &sgp_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER )) {
        if ( WICED_SUCCESS != wiced_start_timer( &sgp_timer, 1000 )) {
            WICED_BT_TRACE( "Seconds Timer Error\n\r" );
        }
    }

    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
}

/* Set Advertisement Data */
void ess_ble_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[2] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0; 

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = strlen((const char*)BT_LOCAL_NAME);
    adv_elem[num_elem].p_data = BT_LOCAL_NAME;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/* This function is invoked when advertisements stop */
void ess_ble_advertisement_stopped( void )
{
    WICED_BT_TRACE("Advertisement stopped\n");

    /* TODO: Handle when advertisements stop */
}

/* TODO: This function should be called when the device needs to be reset */
void ess_ble_reset_device( void )
{
    /* TODO: Clear any additional persistent values used by the application from NVRAM */

    // Reset the device
    wiced_hal_wdog_reset_system( );
}

/* Bluetooth Management Event Handler */
wiced_bt_dev_status_t ess_ble_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */

#ifdef HCI_TRACE_OVER_TRANSPORT
        // There is a virtual HCI interface between upper layers of the stack and
        // the controller portion of the chip with lower layers of the BT stack.
        // Register with the stack to receive all HCI commands, events and data.
        wiced_bt_dev_register_hci_trace(ess_ble_trace_callback);
#endif

        WICED_BT_TRACE("Bluetooth Enabled (%s)\n",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

            /* Perform application-specific initialization */
            ess_ble_app_init();
        }
        break;
    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\n");
        break;
    case BTM_SECURITY_REQUEST_EVT:
        /* Security Request */
        WICED_BT_TRACE("Security Request\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Request for Pairing IO Capabilities (BLE) */
        WICED_BT_TRACE("BLE Pairing IO Capabilities Request\n");
        /* No IO Capabilities on this Platform */
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND|BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = 0;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        /* Pairing is Complete */
        p_ble_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE("Pairing Complete %d.\n", p_ble_info->reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        WICED_BT_TRACE("Paired Device Link Request Keys Event\n");
        /* Device/app-specific TODO: HANDLE PAIRED DEVICE LINK REQUEST KEY - retrieve from NVRAM, etc */
#if 0
        if (ess_ble_read_link_keys( &p_event_data->paired_device_link_keys_request ))
        {
            WICED_BT_TRACE("Key Retrieval Success\n");
        }
        else
#endif
        /* Until key retrieval implemented above, just fail the request - will cause re-pairing */
        {
            WICED_BT_TRACE("Key Retrieval Failure\n");
            status = WICED_BT_ERROR;
        }
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_adv_mode);
        if ( BTM_BLE_ADVERT_OFF == *p_adv_mode )
        {
            ess_ble_advertisement_stopped();
        }
        break;
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* Pairing request, TODO: handle confirmation of numeric compare here if desired */
        WICED_BT_TRACE("numeric_value: %d\n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;
    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event);
        break;
    }

    return status;
}

/* Get a Value */
wiced_bt_gatt_status_t ess_ble_get_value( uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t max_len, uint16_t *p_len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < ess_ble_gatt_db_ext_attr_tbl_size; i++)
    {
        if (ess_ble_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (ess_ble_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                // Value fits within the supplied buffer; copy over the value
                *p_len = ess_ble_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, ess_ble_gatt_db_ext_attr_tbl[i].p_data, ess_ble_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;

                // TODO: Add code for any action required when this attribute is read
                switch ( attr_handle )
                {
                case HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE:
                    break;
                case HDLC_GENERIC_ACCESS_APPEARANCE_VALUE:
                    break;
                case HDLC_ESS_SERVICE_BUTTON_VALUE:
                    break;
                }
            }
            else
            {
                // Value to read will not fit within the buffer
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // TODO: Add code to read value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully read then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
        default:
            // The read operation was not performed for the indicated handle
            WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\n", attr_handle);
            res = WICED_BT_GATT_READ_NOT_PERMIT;
            break;
        }
    }

    return res;
}

/* Set a Value */
wiced_bt_gatt_status_t ess_ble_set_value( uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < ess_ble_gatt_db_ext_attr_tbl_size; i++)
    {
        if (ess_ble_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Verify that size constraints have been met
            validLen = (ess_ble_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                ess_ble_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(ess_ble_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;

                // TODO: Add code for any action required when this attribute is written
                // For example you may need to write the value into NVRAM if it needs to be persistent
                switch ( attr_handle )
                {
                case HDLC_ESS_SERVICE_BUTTON_VALUE:
                    break;
                }
            }
            else
            {
                // Value to write does not meet size constraints
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // TODO: Add code to write value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully written then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
        default:
            // The write operation was not performed for the indicated handle
            WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\n", attr_handle);
            res = WICED_BT_GATT_WRITE_NOT_PERMIT;
            break;
        }
    }

    return res;
}

/* Handles Write Requests received from Client device */
wiced_bt_gatt_status_t ess_ble_write_handler( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = ess_ble_set_value(p_write_req->handle, conn_id, p_write_req->p_val, p_write_req->val_len);

    return status;
}

/* Handles Read Requests received from Client device */
wiced_bt_gatt_status_t ess_ble_read_handler( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = ess_ble_get_value(p_read_req->handle, conn_id, p_read_req->p_val, *p_read_req->p_val_len, p_read_req->p_val_len);

    return status;
}

/* GATT Connection Status Callback */
wiced_bt_gatt_status_t ess_ble_connect_callback( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            // Device has connected
            WICED_BT_TRACE("Connected : BDA '%B', Connection ID '%d'\n", p_conn_status->bd_addr, p_conn_status->conn_id );

            /* TODO: Handle the connection */
            connection_id = p_conn_status->conn_id;
        }
        else
        {
            // Device has disconnected
            WICED_BT_TRACE("Disconnected : BDA '%B', Connection ID '%d', Reason '%d'\n", p_conn_status->bd_addr, p_conn_status->conn_id, p_conn_status->reason );

            /* TODO: Handle the disconnection */
            connection_id = 0;
            /* restart the advertisements */
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/* GATT Server Event Callback */
wiced_bt_gatt_status_t ess_ble_server_callback( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch ( type )
    {
    case GATTS_REQ_TYPE_READ:
        status = ess_ble_read_handler( &p_data->read_req, conn_id );
        break;
    case GATTS_REQ_TYPE_WRITE:
        status = ess_ble_write_handler( &p_data->write_req, conn_id );
        break;
    }

    return status;
}

/* GATT Event Handler */
wiced_bt_gatt_status_t ess_ble_event_handler( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch ( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = ess_ble_connect_callback( &p_event_data->connection_status );
        break;
    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = ess_ble_server_callback( p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data );
        break;
    default:
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/* Handle Command Received over Transport */
uint32_t hci_control_process_rx_cmd( uint8_t* p_data, uint32_t len )
{
    uint8_t status = 0;
    uint8_t cmd_status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t opcode = 0;
    uint8_t* p_payload_data = NULL;
    uint8_t payload_length = 0;

    WICED_BT_TRACE("hci_control_process_rx_cmd : Data Length '%d'\n", len);

    // At least 4 bytes are expected in WICED Header
    if ((NULL == p_data) || (len < 4))
    {
        WICED_BT_TRACE("Invalid Parameters\n");
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {
        // Extract OpCode and Payload Length from little-endian byte array
        opcode = LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(p_data);
        payload_length = LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(&p_data[sizeof(uint16_t)]);
        p_payload_data = &p_data[sizeof(uint16_t)*2];

        // TODO : Process received HCI Command based on its Control Group
        // (see 'hci_control_api.h' for additional details)
        switch ( HCI_CONTROL_GROUP(opcode) )
        {
        default:
            // HCI Control Group was not handled
            cmd_status = HCI_CONTROL_STATUS_UNKNOWN_GROUP;
            wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, sizeof(cmd_status));
            break;
        }
    }

    // When operating in WICED_TRANSPORT_UART_HCI_MODE or WICED_TRANSPORT_SPI,
    // application has to free buffer in which data was received
    wiced_transport_free_buffer( p_data );
    p_data = NULL;

    return status;
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/* Handle Sending of Trace over the Transport */
void ess_ble_trace_callback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_transport_send_hci_trace( transport_pool, type, length, p_data );
}
#endif
