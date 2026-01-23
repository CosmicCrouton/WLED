#include "wled.h"

#include "clnp_protocol.h"
#include "clnp_input.h"

#include <cstring>
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"

uint8_t clnp_protocol::uart_num = 2;
clnp_device clnp_protocol::devices[CLNP_DEVICE_COUNT];

void clnp_protocol::setup()
{
    // Get the NVS namespace and key value
    nvs_handle_t nvs;

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Retry nvs_flash_init
        DEBUG_PRINTF("NVS partition was truncated, so erasing it");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );


    for(int i = 0; i < CLNP_DEVICE_COUNT; i++) {
        devices[i].nvs_namespace = "esp_clnp?" + std::to_string(i);
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(devices[i].nvs_namespace.c_str(), NVS_READONLY, &nvs));

        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(nvs, "address", &devices[i].group_membership_flags));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(nvs, "group", &devices[i].address));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(nvs, "terminated", &devices[i].termination_enabled));

        DEBUG_PRINTF("group: %04x, addr: %02x, term: %01x\n", devices[i].group_membership_flags, devices[i].address, devices[i].termination_enabled);

        devices[i].wled_segment_index = i;
        devices[i].fade_step_duration_ms = 100;
        devices[i].is_muted = false;

        esp_read_mac(&devices[i].serial_number[0], esp_mac_type_t::ESP_MAC_ETH);
        devices[i].serial_number[7] = i; //Make serial number unique per device index

        nvs_close(nvs);
    }

    uart_num = 2; //TODO: Make configurable
}

void clnp_protocol::print_array(std::string msg, const uint8_t* data, size_t size)
{
    DEBUG_PRINTF("%s:", msg.c_str());

    return;

    for (int i = 0; i < size; i++) {
      DEBUG_PRINTF("0x%x,", data[i]);
    }
    DEBUG_PRINTLN("\n");
}

uint16_t clnp_protocol::crc16(const uint8_t *data, size_t size)
{
    uint8_t  thisbyte, crcbit, databit;
    uint16_t poly = 0x1021; //Generator Polynomial
    uint16_t crc = 0;

    for (uint8_t i = 0; i < size; i++)
    {
        thisbyte = data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crcbit = crc & 0x8000 ? 1 : 0;
            databit = thisbyte & 0x80 ? 1 : 0;
            crc = crc << 1;
            if (crcbit != databit)
            {
                crc = crc ^ poly;
            }
            thisbyte = thisbyte << 1;
        }
    }
    return crc;
}

size_t clnp_protocol::get_addr_size(CLNPDestinationType destType) {
    size_t destination_field_len = -1;

    //Destination type determines length of destination field
    switch(destType) {
        case CLNPDestinationType::MASTER_BROADCAST_MESSAGE:
        case CLNPDestinationType::INTERNAL_CHAIN_MESSAGE_ONLY:
            destination_field_len = 0;
            break;                                  //destination field length = 0 bytes
        case CLNPDestinationType::SINGLE_CHAIN_MESSAGE:
            destination_field_len = 1;
            break;
        case CLNPDestinationType::GROUP_MASTER_MESSAGE:
            destination_field_len = 4;
            break;
        case CLNPDestinationType::SERIAL_NUMBER_MASTER_MESSAGE:
            destination_field_len = 8;
            break;
        case CLNPDestinationType::SINGLE_BOARD_MESSAGE:
            destination_field_len = 2;
            break;
        default:
            DEBUG_PRINTLN("Invalid destination address type!");
    }

    return destination_field_len;
}

bool clnp_device::address_matched(CLNPDestinationType destType, const uint8_t* addr) {
    uint32_t group_addr;

    //Destination type determines length of destination field
    for (uint8_t i = 0; i < CLNP_DEVICE_COUNT; i++) {

        switch(destType) {
            case CLNPDestinationType::MASTER_BROADCAST_MESSAGE:
                return true;

            case CLNPDestinationType::SINGLE_CHAIN_MESSAGE:
                if (addr[0] == this->address) {
                    return true;
                }
                break;

            case CLNPDestinationType::GROUP_MASTER_MESSAGE:
                group_addr = addr[0] << 24 | addr[1] << 16 | addr[2] << 8 | addr[3];

                if (group_addr & this->group_membership_flags) {
                    return true;
                }
                break;

            case CLNPDestinationType::SERIAL_NUMBER_MASTER_MESSAGE:
                if (memcmp(addr, this->serial_number, 8) == 0) {
                    return true;
                }
                break;

            case CLNPDestinationType::SINGLE_BOARD_MESSAGE:
            case CLNPDestinationType::INTERNAL_CHAIN_MESSAGE_ONLY:
                DEBUG_PRINTF("Unsupported destination type! (0x%x)", destType);
                break;

            default:
                DEBUG_PRINTLN("Invalid destination address type!");
        }
    }

    return false;
}

boolean clnp_device::route(CLNPDestinationType destType, uint8_t destAddr, const uint8_t* messageLayer, size_t messageLayerSize)
{
    size_t destination_addr_size = 0;
    uint8_t routeLayer[CLNP_PACKET_SIZE_MAX];

    destination_addr_size = clnp_protocol::get_addr_size(destType);

    if (this->is_muted) {
        DEBUG_PRINTLN("Device is muted, not sending packet.");
        return true;
    }

    if (destination_addr_size == -1) {
        DEBUG_PRINTF("Invalid destination type.");
        return false;
    }

    if (destination_addr_size >= 2)
    {
        DEBUG_PRINTF("Only destination addresses of size 0 or 1 byte are currently supported.");
        return false;
    }

    size_t routeLayerSize = messageLayerSize + destination_addr_size + 2 + 2;   //+2 for destType+srcAddr, +2 for CRC

    routeLayer[0] = static_cast<uint8_t>(destType);

    if (destination_addr_size >= 0) {
        routeLayer[1] = destAddr;
    }

    routeLayer[1 + destination_addr_size] = this->address;  //source address for outgoing message
    memcpy(&routeLayer[2+destination_addr_size], messageLayer, messageLayerSize);

    uint16_t crc = clnp_protocol::crc16(&routeLayer[0], routeLayerSize - 2);  //-2 for the CRC bytes
    routeLayer[routeLayerSize-2] = crc >> 8;
    routeLayer[routeLayerSize-1] = crc & 0xFF;

    clnp_protocol::print_array("tx route: ", routeLayer, routeLayerSize);

    return this->transport(routeLayer, routeLayerSize);
}

boolean clnp_device::transport(const uint8_t* routeLayer, size_t routeLayerSize)
{
    ESP_RETURN_ON_FALSE(routeLayerSize < CLNP_PACKET_SIZE_MAX, false, "CLNP", "");

    uint8_t transportLayer[CLNP_PACKET_SIZE_MAX];
    uint8_t transportLayerSize = 2;

    //Add escape characters (0x10 must be escaped in the routing layer)
    // adjust transportLayerSize since it will change when an escape byte is added
    for (int ri = 0; ri < routeLayerSize; ri++, transportLayerSize++)
    {
        if (routeLayer[ri] == CLNP_DLE) {
            transportLayer[transportLayerSize] = CLNP_DLE;
            transportLayerSize += 1;
        }

        transportLayer[transportLayerSize] = routeLayer[ri];
    }
    transportLayerSize += 2;  //+2 for the trailing DLE+ETX bytes.

    transportLayer[0] = CLNP_DLE;
    transportLayer[1] = CLNP_STX;
    transportLayer[transportLayerSize-2] = CLNP_DLE;
    transportLayer[transportLayerSize-1] = CLNP_ETX;

    //TODO_HJK: vary delay based on serial number (9-146 section 5.2.2)
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (uart_write_bytes(clnp_protocol::uart_num, transportLayer, transportLayerSize) != transportLayerSize) {
        DEBUG_PRINTF("Send data critical failure.");
        return false;
    }

    clnp_protocol::print_array("tx tranport: ", transportLayer, transportLayerSize);
    return true;
}

//remove escape characters and check for proper framing
size_t clnp_protocol::process_transport(const uint8_t *data, size_t size)
{
    ESP_RETURN_ON_FALSE(size >= 4, size, "CLNP", "process_transport: less than 4 bytes");

    //CLNP packet must start with DLE STX (0x10 0x02) and end with DLE ETX (0x10 0x03)
    if (data[0] != CLNP_DLE && data[1] != CLNP_ETX)
    {
        DEBUG_PRINTLN("process_transport: packet must start with DLE+ETX");
        return 2;
    }

    uint8_t routing_layer_data[270];
    std::size_t routing_layer_data_size = 0;
    bool complete_transport_packet_found = false;
    DEBUG_PRINTLN("DLE+ETX check passed");

    for (int i=2; i < size-1; i++) {
        //DLE+ETX (end of transport packet) encountered
        if (data[i] == CLNP_DLE && data[i+1] == CLNP_ETX) {
            complete_transport_packet_found = true;
            break;
        }

        //0x10 bytes (DLE) are escaped (doubled) within the transport layer, so undo that
        if (data[i] == CLNP_DLE && data[i+1] == CLNP_DLE) {
            i++;
        }

        routing_layer_data[routing_layer_data_size] = data[i];
        routing_layer_data_size++;
    }

    if (complete_transport_packet_found) {
        DEBUG_PRINTLN("Full frame received");
        clnp_protocol::proccess_route_layer(routing_layer_data, routing_layer_data_size);
    }
    else {
        DEBUG_PRINTLN("from_bytes: complete transport packet NOT found");
    }

    return routing_layer_data_size + 4; //+4 for DLE+STX and DLE+ETX
}

//check for address match and crc validity
bool clnp_protocol::proccess_route_layer(const uint8_t *data, size_t size)
{
    ESP_RETURN_ON_FALSE(size >= 6, false, "CLNP", "Route layer initial size invalid! Got: %i, Expected: %i\n", size, 6);

    uint8_t destination_data[8];
    uint8_t destination_field_len = 0;

    clnp_protocol::print_array("route_layer: ", data, size);

    CLNPDestinationType destination_type = static_cast<CLNPDestinationType>(data[0]);

    destination_field_len = clnp_protocol::get_addr_size(destination_type);

    //Double check required data length now that we know the destination field length
    ESP_RETURN_ON_FALSE(size >= 6 + destination_field_len, false, "CLNP",
                        "Route layer size invalid! Got: %i, Expected: %i\n", size, 6 + destination_field_len);

    memcpy(destination_data, &data[1], destination_field_len);
    uint8_t source_address = data[1+destination_field_len];

    //result should be 0 when the provided CRC matches the calculated CRC
    bool crc_pass = (clnp_protocol::crc16(data, size) == 0);

    if (crc_pass == false) {
        DEBUG_PRINTF("Routing layer CRC failed!\n");
        return false;
    }

    //Only support up to 4 segments/outputs
    int maxDeviceIndex = CLNP_DEVICE_COUNT;
    if (strip.getSegmentsNum() <= CLNP_DEVICE_COUNT) {
        maxDeviceIndex = strip.getSegmentsNum();
    }

    //Check route layer addressing
    for (int i = 0; i < maxDeviceIndex; i++) {
        clnp_device &device = clnp_protocol::devices[i];

        if (device.address_matched(destination_type, destination_data)) {
            device.process_message(destination_type, source_address,
                                    &data[2+destination_field_len], size - 4 - destination_field_len);
        }
    }

    return false;
}

//process message layer commands
bool clnp_device::process_message(CLNPDestinationType destination_type, uint8_t src_addr, const uint8_t *data, size_t size)
{
    ESP_RETURN_ON_FALSE(size >= 2, false, "CLNP", "FADE_IO: insufficient data length");

    clnp_protocol::print_array("message_layer: ", data, size);

    CLNPMessageFlags_t msgFlags = static_cast<CLNPMessageFlags_t>(data[0]);
    CLNPCmds msgCmd = static_cast<CLNPCmds>(data[1]);

    uint8_t messageLayer[255];
    uint8_t messageLayerSize = 0;
    uint8_t destinationAddr[8];

    CLNPParamAddress param_addr;
    nvs_handle_t nvs;
    std::string nvs_namespace;
    esp_err_t err;


    #define UART_NUM 2 //TODO

    bool isResponse = msgFlags & CLNPMessageFlags_t::COMMAND_RESPONSE;
    bool isAckRequested = msgFlags & CLNPMessageFlags_t::ACKNOWLEDGEMENT;
    bool isErrorLogged = msgFlags & CLNPMessageFlags_t::ERRORS_LOGGED;


    switch (msgCmd)
    {
        case CLNPCmds::PROTOCOL_QUERY:
            DEBUG_PRINTF("PROTOCOL_QUERY\n");
            if (isAckRequested) {
                messageLayer[0] = CLNPMessageFlags_t::ACKNOWLEDGEMENT;
                messageLayer[1] = static_cast<uint8_t>(msgCmd);
                memcpy(&messageLayer[2], this->serial_number, 8);
                messageLayerSize = 10;
                clnp_protocol::print_array("tx message: ", messageLayer, messageLayerSize);
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);
            }
            break;

        case CLNPCmds::GET_VERSION_NUMBERS:
            DEBUG_PRINTF("GET_VERSION_NUMBERS\n");
            if (isAckRequested) {
                messageLayer[0] = CLNPMessageFlags_t::ACKNOWLEDGEMENT | CLNPMessageFlags_t::COMMAND_RESPONSE;
                messageLayer[1] = static_cast<uint8_t>(msgCmd);
                memset(&messageLayer[2], 0, 8);  //TODO_HJK: fill with some sort of versioning info for WLED?
                messageLayerSize = 10;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);
            }
            break;

        //DLE+STX   DestType  SrcAddr  MsgFlags  MsgType   CRC-16  DLE+ETX
        //10 02     00        fe       01        e8        b7 44   10 03  (discover network msg)
        case CLNPCmds::SETUP_DISCOVER_NETWORK:
            DEBUG_PRINTF("SETUP_DISCOVER_NETWORK\n");
            if (destination_type != CLNPDestinationType::MASTER_BROADCAST_MESSAGE) {
                send_response(msgCmd, src_addr, CLNPErrorCode::GEN_ILLEGAL_DESTINATION_TYPE, false);
            }
            else if (isAckRequested) {
                messageLayer[0] = CLNPMessageFlags_t::COMMAND_RESPONSE | CLNPMessageFlags_t::ACKNOWLEDGEMENT;
                messageLayer[1] = static_cast<uint8_t>(CLNPCmds::SETUP_DISCOVER_NETWORK);
                memcpy(&messageLayer[2], this->serial_number, 8);
                messageLayer[10] = this->address;
                memcpy(&messageLayer[11], &this->group_membership_flags, 4);
                messageLayer[15] = 0x01;  //Chain address lock (enabled)
                messageLayerSize = 16;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);
            }
            break;

        case CLNPCmds::COMMON_LIGHT_CONTROL:
            process_common_light_control_message(src_addr, &data[2], size - 2);
            break;

        //DLE+STX   DestType  SrcAddr  MsgFlags  MsgType   CRC-16  DLE+ETX
        //10 02     00        fe       00        e9        XX XX   10 03  (mute chain cmd)
        case CLNPCmds::SETUP_MUTE_CHAIN:
            DEBUG_PRINTF("SETUP_MUTE_CHAIN\n");
            if (isAckRequested) {
                messageLayer[0] = CLNPMessageFlags_t::COMMAND_RESPONSE | CLNPMessageFlags_t::ACKNOWLEDGEMENT;
                messageLayer[1] = static_cast<uint8_t>(CLNPCmds::SETUP_MUTE_CHAIN);
                memcpy(&messageLayer[2], this->serial_number, 8);
                messageLayer[10] = this->address;
                memcpy(&messageLayer[11], &this->group_membership_flags, 4);
                messageLayer[15] = 0x01;  //Chain address lock (enabled)
                messageLayerSize = 16;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);
            }

            this->is_muted = true;
            break;

        //DLE+STX   DestType  SrcAddr  MsgFlags  MsgType   CRC-16  DLE+ETX
        //10 02     00        fe       00        ea        a4 37   10 03  (unmute chain cmd)
        case CLNPCmds::SETUP_UNMUTE_CHAIN:
            DEBUG_PRINTF("SETUP_UNMUTE_CHAIN\n");
            this->is_muted = false;
            if (isAckRequested) {
                messageLayer[0] = CLNPMessageFlags_t::COMMAND_RESPONSE | CLNPMessageFlags_t::ACKNOWLEDGEMENT;
                messageLayer[1] = static_cast<uint8_t>(CLNPCmds::SETUP_UNMUTE_CHAIN);
                memcpy(&messageLayer[2], this->serial_number, 8);
                messageLayer[10] = this->address;
                memcpy(&messageLayer[11], &this->group_membership_flags, 4);
                messageLayer[15] = 0x01;  //Chain address lock (enabled)
                messageLayerSize = 16;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);
            }
            break;

        //chain address = 0x11110 000, 1 bytes
        //group mask    = 0x11000 000-011, 4 bytes
        //01 02  F0    01   (set param unit address 1)
        //01 02  C0    00 00 00 00 (set param group mask 0x00000000)
        case CLNPCmds::SET_PARAM:
            param_addr = static_cast<CLNPParamAddress>(data[2]);
            DEBUG_PRINTF("SET_PARAM: 0x%x\n", param_addr);

            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(this->nvs_namespace.c_str(), NVS_READWRITE, &nvs));

            if (param_addr == CLNPParamAddress::GROUP_MASK) {
                this->group_membership_flags = ((uint32_t)data[3] << 24) ||
                                               ((uint32_t)data[4] << 16) ||
                                               ((uint32_t)data[5] << 8)  ||
                                                (uint32_t)data[6];

                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(nvs, "group", this->group_membership_flags));
                send_response(msgCmd, src_addr, CLNPErrorCode::NONE, true);

            } else if (param_addr == CLNPParamAddress::CHAIN_ADDRESS) {
                this->address = data[3];
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(nvs, "address", this->address));

                send_response(msgCmd, src_addr, CLNPErrorCode::NONE, true);
            } else if (param_addr == CLNPParamAddress::TERM_ENABLE) {
                this->termination_enabled = data[3];
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(nvs, "terminated", this->termination_enabled));

                send_response(msgCmd, src_addr, CLNPErrorCode::NONE, true);
            } else {
                send_response(msgCmd, src_addr, CLNPErrorCode::PARAM_UNSUPPORTED_PARAM_ID, false);
            }

            nvs_commit(nvs);
            nvs_close(nvs);
            break;

        case CLNPCmds::GET_PARAM:
            param_addr = static_cast<CLNPParamAddress>(data[2]);
            DEBUG_PRINTF("GET_PARAM: 0x%x\n", param_addr);

            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(nvs_namespace.c_str(), NVS_READONLY, &nvs));

            messageLayer[0] = CLNPMessageFlags_t::COMMAND_RESPONSE | CLNPMessageFlags_t::ACKNOWLEDGEMENT;
            messageLayer[1] = static_cast<uint8_t>(CLNPCmds::GET_PARAM);
            messageLayer[2] = static_cast<uint8_t>(param_addr);

            if (param_addr == CLNPParamAddress::GROUP_MASK) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(nvs, "group", &this->group_membership_flags));

                memcpy(&messageLayer[3], &this->group_membership_flags, 4);
                messageLayerSize = 7;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);

            } else if (param_addr == CLNPParamAddress::CHAIN_ADDRESS) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(nvs, "address", &this->address));

                messageLayer[3] = this->address;
                messageLayerSize = 4;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);
            } else if (param_addr == CLNPParamAddress::TERM_ENABLE) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(nvs, "termination", &this->termination_enabled));

                messageLayer[3] = this->termination_enabled;
                messageLayerSize = 4;
                this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, src_addr,
                            messageLayer, messageLayerSize);

            } else {
                send_response(msgCmd, src_addr, CLNPErrorCode::PARAM_UNSUPPORTED_PARAM_ID, false);
            }

            nvs_close(nvs);
            break;

        case CLNPCmds::SETUP_EUMERATION:
        case CLNPCmds::SETUP_REPORT_SLAVES:
        case CLNPCmds::SETUP_RESTORE_FACTORY_DEFAULTS:
            send_response(msgCmd, src_addr, CLNPErrorCode::GEN_UNSUPPORTED_MESSAGE_TYPE, false);
            break;

        case CLNPCmds::GET_LOGGED_EVENTS:
        case CLNPCmds::CLEAR_LOGGED_EVENTS:
            DEBUG_PRINTF("CLNP_LOG_MSG\n");
            send_response(msgCmd, src_addr, CLNPErrorCode::LOGGING_NOT_AVAILABLE, false);
            break;
    }

    return true;
}

void clnp_device::send_response(CLNPCmds cmd, uint8_t dest_addr, CLNPErrorCode error_code, bool is_acknowledgement)
{
    uint8_t messageLayer[4];
    uint8_t messageLayerSize;

    messageLayer[0] = CLNPMessageFlags_t::COMMAND_RESPONSE;

    if (is_acknowledgement) {
        messageLayer[0] |= CLNPMessageFlags_t::ACKNOWLEDGEMENT;
    }

    messageLayer[1] = static_cast<uint8_t>(cmd);

    if (error_code != CLNPErrorCode::NONE) {
        messageLayerSize = 2;
    } else {
        messageLayerSize = 4;
        messageLayer[2] = static_cast<uint16_t>(error_code) >> 8;
        messageLayer[3] = static_cast<uint16_t>(error_code) & 0xFF;
    }

    this->route(CLNPDestinationType::SINGLE_CHAIN_MESSAGE, dest_addr,
                messageLayer, messageLayerSize);
}

bool clnp_device::process_common_light_control_message(uint8_t src_addr, const uint8_t *data, size_t size)
{
    float x, y;
    uint8_t intensity, color_temp, fadeCounts;
    uint8_t rgb[4] = {0};
    bool isOn;

    ESP_RETURN_ON_FALSE(size >= 1, false, "CLNP", "process_common_light_control_message: insufficient data length");

    CLNPOpCommonLight op_code = static_cast<CLNPOpCommonLight>(data[0]);

    DEBUG_PRINTF("common light control op code: 0x%x\n", op_code);

    switch (op_code)
    {
        case CLNPOpCommonLight::FADE_IO:
            ESP_RETURN_ON_FALSE(size >= 3, false, "CLNP", "FADE_IO: insufficient data length");

            fadeCounts = data[1];
            isOn = data[2] ? true : false;

            wled_set_onoff(fadeCounts, isOn);
            break;

        case CLNPOpCommonLight::FADE_INTENSITY:
            ESP_RETURN_ON_FALSE(size >= 4, false, "CLNP", "FADE_INTENSITY: insufficient data length");

            fadeCounts = data[1];
            intensity = static_cast<uint8_t>(data[2]);  //intensity (Y), ignore LSB

            clnp_device::wled_set_intensity_fade(fadeCounts, intensity);
            break;

        case CLNPOpCommonLight::FADE_TO_PALETTE_COLOR_AND_INT:
            DEBUG_PRINTLN("FADE_TO_PALETTE_COLOR_AND_INT");
            send_response(CLNPCmds::COMMON_LIGHT_CONTROL, src_addr, CLNPErrorCode::GEN_UNSUPPORTED_MESSAGE_TYPE, false);
            break;

        case CLNPOpCommonLight::FADE_TO_COLOR_TEMP_AND_INT:
            ESP_RETURN_ON_FALSE(size >= 6, false, "CLNP", "FADE_TO_COLOR_TEMP_AND_INT: insufficient data length");

            fadeCounts = data[1];
            color_temp = static_cast<uint16_t>((data[2] << 8) | data[3]); //color temperature in Kelvin
            intensity = static_cast<uint8_t>(data[4]);  //intensity (Y), ignore LSB

            colorKtoRGB(color_temp, rgb);

            clnp_device::wled_set_color_fade(fadeCounts, intensity, rgb[0], rgb[1], rgb[2], rgb[3]);
            break;

        //        opcode   fade    X       Y     INT
        //00 1e   04       ff      3c 29   98 51 ff ff
        case CLNPOpCommonLight::FADE_TO_COLOR_AND_INT:
            ESP_RETURN_ON_FALSE(size >= 8, false, "CLNP", "FADE_TO_COLOR_TEMP_AND_INT: insufficient data length");

            //convert from CIE 1931 xyY to RGB
            fadeCounts = data[1];
            //Skip LSB of each x/y/Y value since WLED uses 8 bit colors
            x = static_cast<float>((data[2] << 8) | data[3]) / static_cast<float>(UINT16_MAX);  //chromaticity (x)
            y = static_cast<float>((data[4] << 8) | data[5]) / static_cast<float>(UINT16_MAX);  //chromaticity (y)
            intensity = static_cast<uint8_t>(data[6]);  //intensity (Y), ignore LSB

            colorXYtoRGB(x, y, rgb);

            clnp_device::wled_set_color_fade(fadeCounts, intensity, rgb[0], rgb[1], rgb[2], rgb[3]);
            break;

        default:
            DEBUG_PRINTLN("Unsupported COMMON_LIGHT_CONTROL op code");
            send_response(CLNPCmds::COMMON_LIGHT_CONTROL, src_addr, CLNPErrorCode::GEN_UNSUPPORTED_MESSAGE_TYPE, false);
            return false;
    }

    return true;
}

void clnp_device::wled_set_color_fade(uint16_t fadeCounts, uint8_t intensity, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    uint16_t fade_duration = static_cast<uint16_t>(this->fade_step_duration_ms) * fadeCounts;

    Segment &seg = strip.getSegment(this->wled_segment_index);

    uint32_t color = RGBW32(r, g, b, w);

    if (seg.opacity != intensity || seg.colors[0] != color)
    {
        seg.startTransition(fade_duration, blendingStyle != BLEND_STYLE_FADE); // start transition prior to change
        seg.options |=   0x01 << SEG_OPTION_ON;
        seg.colors[0] = color;
        seg.opacity = intensity;
        stateChanged = true;
    }
}

void clnp_device::wled_set_intensity_fade(uint16_t fadeCounts, uint8_t intensity)
{
    uint16_t fade_duration = static_cast<uint16_t>(this->fade_step_duration_ms) * fadeCounts;

    Segment &seg = strip.getSegment(this->wled_segment_index);

    if (seg.opacity != intensity) {
        seg.startTransition(fade_duration, blendingStyle != BLEND_STYLE_FADE);
        seg.options |=   0x01 << SEG_OPTION_ON;
        seg.opacity = intensity;
        stateChanged = true;
    }
}

void clnp_device::wled_set_onoff(uint16_t fadeCounts, bool isOn)
{
    uint16_t fade_duration = static_cast<uint16_t>(this->fade_step_duration_ms) * fadeCounts;

    Segment &seg = strip.getSegment(this->wled_segment_index);

    bool prev = (seg.options >> SEG_OPTION_ON) & 0x01;
    if (isOn != prev) {
        seg.startTransition(fade_duration, blendingStyle != BLEND_STYLE_FADE); // start transition prior to change

        if (isOn)
            seg.options |=   0x01 << SEG_OPTION_ON;
        else
            seg.options &= ~(0x01 << SEG_OPTION_ON);

        stateChanged = true; // send UDP/WS broadcast
    }
}
