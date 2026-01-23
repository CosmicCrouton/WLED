#include <cstdint>
#include <string>

#define CLNP_DLE 0x10
#define CLNP_STX 0x02
#define CLNP_ETX 0x03

#ifndef CLNP_DEVICE_COUNT
#define CLNP_DEVICE_COUNT 4
#endif

//CLNP 9-146 section 1.6.1
typedef enum CLNPMessageFlags_t: uint8_t {
    COMMAND_RESPONSE =  0b10000000, //0 = command, 1 = response
    //0b01111100 reserved for future use
    ERRORS_LOGGED    =  0b00000010, //0 = no errors, 1 = errors logged
    ACKNOWLEDGEMENT  =  0b00000001  //0 = command failed, 1 = command succeeded
} CLNPMessageFlags_t;

//CLNP 9-146 section 1.5.1
enum class CLNPDestinationType : uint8_t {
    MASTER_BROADCAST_MESSAGE        = 0x00,
    SINGLE_CHAIN_MESSAGE            = 0x01,
    GROUP_MASTER_MESSAGE            = 0x02,
    SERIAL_NUMBER_MASTER_MESSAGE    = 0x03,
    SINGLE_BOARD_MESSAGE            = 0x10,
    INTERNAL_CHAIN_MESSAGE_ONLY     = 0x7E
};

enum class CLNPCmds : uint8_t {

    PROTOCOL_QUERY      = 0x00,
    GET_PARAM           = 0x01,
    SET_PARAM           = 0x02,
    GET_PARAM_BOUNDS    = 0x03,
    GET_VERSION_NUMBERS = 0x04,
    GET_LOGGED_EVENTS   = 0x05,
    CLEAR_LOGGED_EVENTS = 0x06,

    COMMON_LIGHT_CONTROL        = 0x1E,

    SETUP_EUMERATION                = 0xE0,
    SETUP_DISCOVER_NETWORK          = 0xE8,
    SETUP_MUTE_CHAIN                = 0xE9,
    SETUP_UNMUTE_CHAIN              = 0xEA,
    SETUP_REPORT_SLAVES             = 0xEB,
    SETUP_RESTORE_FACTORY_DEFAULTS  = 0xEF,
    //0xF0-0xFF reserved for internal (emteq master units) use
    CHAIN_HEARTBEAT = 0xF0, //Receiving this cmd may indicate a wiring issue
};

enum class CLNPOpCommonLight : uint8_t {

    FADE_IO = 0x00,
    FADE_INTENSITY = 0x01,
    FADE_TO_PALETTE_COLOR_AND_INT = 0x02,
    FADE_TO_COLOR_TEMP_AND_INT = 0x03,
    FADE_TO_COLOR_AND_INT = 0x04,
    SET_ONOFF_STATE = 0x0A,
};


enum class CLNPErrorCode : uint16_t {
    GEN_INVALID_CRC = 0x0000,
    GEN_ILLEGAL_SOURCE = 0x0001,
    GEN_ILLEGAL_DESTINATION_TYPE = 0x0002,
    GEN_UNKNOWN_MESSAGE_TYPE = 0x0003,
    GEN_UNSUPPORTED_MESSAGE_TYPE = 0x0004,
    GEN_INSUFFICIENT_DATA = 0x0005,
    GEN_INVALID_DATA = 0x0006,
    GEN_BUFFER_OVERFLOW = 0x0007,
    GEN_UNSPECIFIED_ERROR = 0x00FE,
    GEN_LOSS_OF_HEARTBEAT = 0x00FF,

    PARAM_UNKNOWN_PARAM_ID = 0x0100,
    PARAM_UNSUPPORTED_PARAM_ID = 0x0101,
    PARAM_ILLEGAL_PARAM_ID = 0x0102,
    PARAM_SET_VALUE_RANGE = 0x0103,
    PARAM_REQ_VALUE_UNAVAILABLE = 0x0104,
    PARAM_IS_AN_ENUM_OR_BITFIELD = 0x0105,
    PARAM_IS_READ_ONLY = 0x0106,

    LOGGING_NOT_AVAILABLE = 0x0200,

    BOOTLOADER_NOT_IN_BOOTLOADER_MODE = 0x0300,

    ELECTRICAL_LOSS_OF_SYNC = 0x0400,

    //0x05 - 0x0F reserved for future use

    SETUP_NOT_IN_CONFIG_MODE = 0x1000,

    DTL_TIMEOUT_WAIT_FOR_DATA = 0xC000,
    DTL_UNEXPECTED_START_OF_MSG = 0xC001,
    DTL_UNEXPECTED_END_OF_MSG = 0xC002,
    DTL_UNEXPECTED_DLE_SEQ = 0xC003,

    DRL_UNKNOWN_DEST_TYPE = 0xC100,
    DRL_INVALID_SOURCE_ADDRESS = 0xC101,

    CAT_WATCHDOG_FAIL = 0xFF00,
    CAT_INVALID_STATE = 0xFF01,

    NONE = 0xFFFF,
};

enum class CLNPParameterErrorCodes : uint8_t {
    UNKNOWN_PARAMETER_ID = 0x00,
    UNSUPPORTED_PARAM = 0x01,
    ILLEGAL_PARAM_ID = 0x02,
    SET_VALUE_OUT_OF_RANGE = 0x03,
    REQUESTED_VALUE_UNAVAILABLE = 0x04,
    PARAM_IS_AN_ENUM_OR_BITFIELD = 0x05,
    PARAM_IS_READ_ONLY = 0x06,
};


enum class CLNPParamAddress : uint8_t {
    GROUP_MASK = 0b11000000,
    CHAIN_ADDRESS = 0b11110000,
    COMM_RATE = 0b11110100,
    DATA_BITS = 0b11110101,
    TERM_ENABLE = 0b11110110,
    LOG_LEVEL = 0b11110111,
};

class clnp_device {
    public:
        bool address_matched(CLNPDestinationType destType, const uint8_t* addr);

        bool route(CLNPDestinationType destType,
                          uint8_t destAddr,
                          const uint8_t* messageLayer, size_t messageLayerSize);

        bool transport(const uint8_t* routeLayer, size_t routeLayerSize);

        bool process_message(CLNPDestinationType destination_type, uint8_t dest_addr,
                                    const uint8_t *data, size_t size);

        bool process_command(CLNPCmds cmd,
                                    CLNPMessageFlags_t flags,
                                    const uint8_t *data, size_t size);

        bool process_common_light_control_message(uint8_t src_addr, const uint8_t *data, size_t size);

        void send_response(CLNPCmds cmd, uint8_t dest_addr, CLNPErrorCode error_code, bool is_acknowledgement);

        void wled_set_color_fade(uint16_t fadeCounts, uint8_t intensity, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
        void wled_set_intensity_fade(uint16_t fadeCounts, uint8_t intensity);
        void wled_set_onoff(uint16_t fadeCounts, bool isOn);

    public:
        uint8_t wled_segment_index;
        uint8_t serial_number[8];
        uint8_t address;
        uint8_t termination_enabled;
        uint32_t group_membership_flags;
        uint8_t fade_step_duration_ms;
        bool is_muted;

        std::string nvs_namespace;
};

class clnp_protocol {
    public:
        static uint8_t uart_num;

        static clnp_device devices[CLNP_DEVICE_COUNT];

    public:
        static void print_array(std::string msg, const uint8_t* data, size_t size);

        static size_t process_transport(const uint8_t *data, size_t size);
        static bool proccess_route_layer(const uint8_t *data, size_t size);

        static uint16_t crc16(const uint8_t *data, size_t size);
        static size_t get_addr_size(CLNPDestinationType destType);

        static void setup();
};







