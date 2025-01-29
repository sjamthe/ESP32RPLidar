
#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <stdint.h>
#include "driver/uart.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include "util.h"
#include "freertos/ringbuf.h"

// Structure for scan measurement data
struct MeasurementData {
	float angle;        // In degrees
	float distance;     // In millimeters
	uint8_t quality;    // Quality of measurement
	bool startFlag;     // Start flag for new scan
};

//DMA variables. TODO: replace NUM, TX, RX later
#define UART_NUM UART_NUM_2
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD_RATE 115200
#define RX_TIMEOUT_MS 1
#define RING_BUFFER_SIZE (4*1024)
#define UART_RX_BUF_SIZE 256

typedef uint32_t sl_result;

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT    (0x1<<15)
 // Express scan payload sync bytes
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1     0xA
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2     0x5

typedef struct _sl_lidar_response_measurement_node_t {
    uint8_t sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    uint16_t   angle_q6_checkbit; // check_bit:1;angle_q6:15;
	uint16_t   distance_q2;
} __attribute__((packed)) sl_lidar_response_measurement_node_t;

typedef struct sl_lidar_response_measurement_node_hq_t
{
    uint16_t   angle_z_q14;
    uint32_t   dist_mm_q2;
    uint8_t    quality;
    uint8_t    flag;
} __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;

typedef struct _sl_lidar_response_ultra_cabin_nodes_t
{
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    uint32_t combined_x3;
} __attribute__((packed)) sl_lidar_response_ultra_cabin_nodes_t;

typedef struct _sl_lidar_response_ultra_capsule_measurement_nodes_t
{
    uint8_t                             s_checksum_1; // see [s_checksum_1]
    uint8_t                             s_checksum_2; // see [s_checksum_1]
    uint16_t                            start_angle_sync_q6;
    sl_lidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
} __attribute__((packed)) sl_lidar_response_ultra_capsule_measurement_nodes_t;

class RPLidar {
public:
    // Constants for commands
    static const uint8_t CMD_SYNC_BYTE = 0xA5;
    static const uint8_t CMD_STOP = 0x25;
    static const uint8_t CMD_RESET = 0x40;
    static const uint8_t CMD_SCAN = 0x20;
    static const uint8_t CMD_EXPRESS_SCAN = 0x82;
    static const uint8_t CMD_FORCE_SCAN = 0x21; // Boost?
    static const uint8_t CMD_GET_INFO = 0x50;
    static const uint8_t CMD_GET_HEALTH = 0x52;
    static const uint8_t GET_SAMPLERATE = 0x59;
    static const uint8_t GET_LIDAR_CONF = 0x84;

	// Response Mode
	static const uint8_t SINGLE_RESP_MODE = 0x00;
	static const uint8_t MULTI_RESP_MODE = 0x01;

    // Express Scan payload
    static const uint8_t EXPRESS_TYPE_LEGACY = 0x00;
    static const uint8_t EXPRESS_TYPE_EXTENDED = 0x02;
	static const uint8_t EXPRESS_MEASUREMENTS_PER_SCAN = 96; // Number of Measurements per scan call.

	// Response types
    static const uint8_t RESP_TYPE_INFO = 0x04;
    static const uint8_t RESP_TYPE_HEALTH = 0x06;
    static const uint8_t RESP_TYPE_SCAN_RATE = 0x15;
    static const uint8_t RESP_TYPE_SCAN = 0x81;
    static const uint8_t RESP_TYPE_EXPRESS_LEGACY_SCAN = 0x82;
    static const uint8_t RESP_TYPE_EXPRESS_EXTENDED_SCAN = 0x84;
    static const uint8_t RESP_TYPE_EXPRESS_DENSE_SCAN = 0x85;

    // Response descriptor sync bytes
    static const uint8_t RESP_SYNC_BYTE1 = 0xA5;
    static const uint8_t RESP_SYNC_BYTE2 = 0x5A;

    // Measurement quality threshold
    static const uint8_t MIN_QUALITY = 0;
    static const uint16_t READ_TIMEOUT_MS = 200;  
    static const uint16_t READ_EXP_TIMEOUT_MS = 100; 

    // Structure for device info
    struct DeviceInfo {
        uint8_t model;             // Should show 24 from raw byte 0x18
        uint8_t firmware_major;    // Should be 1
        uint8_t firmware_minor;    // Should be 24
        uint8_t hardware;          // Should be 5
        uint8_t serialnum[16];
    };

    // Structure for device health
    struct DeviceHealth {
        uint8_t status;
        uint16_t error_code;
    };

    // Structure for device scan rate in microSecs
    struct DeviceScanRate {
        uint16_t standard;
        uint16_t express;
    };

	struct ResponseDescriptor {
        uint32_t length;   // 30-bit length
        uint8_t mode;      // 2-bit mode
        uint8_t dataType;  // Data type byte
    };

    // Constructor
    RPLidar(HardwareSerial& serial, int rxPin, int txPin, int motorPin = -1);

    // Basic operations
    bool begin(unsigned long baud = 115200);
    void end();
    
    // Core commands
    bool stop();
    bool reset();
    bool startScan();
    bool startExpressScan(uint8_t expressScanType = 2);
    bool forceScan();
    
    // Information commands
    bool getHealth(DeviceHealth& health);
    bool getInfo(DeviceInfo& info);
    bool getSampleRate(DeviceScanRate &scanRate); // Rate is in uSecs
    
    // Data retrieval
    // Declare a function pointer for commin name.
    sl_result readMeasurement(MeasurementData*, size_t&);
    void ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t &capsule, MeasurementData *measurements, size_t &nodeCount);

    
    // Motor control
    void startMotor(uint8_t pwm = 255);
    void stopMotor();

private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    int _motorPin;
    bool _motorEnabled;
    bool _isConnected;
    bool _scanning;
    uint8_t _scanResponseMode; // Stores the current scan response.
    ResponseDescriptor _responseDescriptor;  // Store the last response descriptor
    bool _is_previous_capsuledataRdy;
    sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;

    // Helper functions
    bool waitResponseHeader();
    void flushInput();
    void sendCommand(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payloadSize = 0);
    uint8_t checksum(const uint8_t* data, uint8_t len);
	  bool verifyResponseDescriptor(uint8_t expectedMode, uint8_t expectedType, uint32_t expectedLength);
    sl_result readMeasurementTypeScan(MeasurementData*, size_t&);
    sl_result readMeasurementTypeExpExtended(MeasurementData*, size_t&);
    sl_result readMeasurementTypeExpLegacy(MeasurementData*, size_t&);
    sl_result _waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t& node, uint32_t timeout = READ_EXP_TIMEOUT_MS);
    uint8_t readByte(); 
    size_t readBytes(uint8_t* buffer, size_t length);
    size_t available() ;
    void writeByte(uint8_t byte);
    void writeBytes(const uint8_t* data, size_t length);
};

// Definition of the variable bit scale encoding mechanism
#define SL_LIDAR_VARBITSCALE_X2_SRC_BIT  9
#define SL_LIDAR_VARBITSCALE_X4_SRC_BIT  11
#define SL_LIDAR_VARBITSCALE_X8_SRC_BIT  12
#define SL_LIDAR_VARBITSCALE_X16_SRC_BIT 14

#define SL_LIDAR_VARBITSCALE_X2_DEST_VAL 512
#define SL_LIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define SL_LIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define SL_LIDAR_VARBITSCALE_X16_DEST_VAL 3328

#define SL_LIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_) \
    (  (((0x1<<(_BITS_)) - SL_LIDAR_VARBITSCALE_X16_DEST_VAL)<<4) + \
       ((SL_LIDAR_VARBITSCALE_X16_DEST_VAL - SL_LIDAR_VARBITSCALE_X8_DEST_VAL)<<3) + \
       ((SL_LIDAR_VARBITSCALE_X8_DEST_VAL - SL_LIDAR_VARBITSCALE_X4_DEST_VAL)<<2) + \
       ((SL_LIDAR_VARBITSCALE_X4_DEST_VAL - SL_LIDAR_VARBITSCALE_X2_DEST_VAL)<<1) + \
       SL_LIDAR_VARBITSCALE_X2_DEST_VAL - 1)


#define SL_RESULT_OK                     (sl_result)0
#define SL_RESULT_FAIL_BIT               (sl_result)0x80000000
#define SL_RESULT_ALREADY_DONE           (sl_result)0x20
#define SL_RESULT_INVALID_DATA           (sl_result)(0x8000 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_FAIL         (sl_result)(0x8001 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_TIMEOUT      (sl_result)(0x8002 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_STOP         (sl_result)(0x8003 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_NOT_SUPPORT  (sl_result)(0x8004 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_FORMAT_NOT_SUPPORT     (sl_result)(0x8005 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_INSUFFICIENT_MEMORY    (sl_result)(0x8006 | SL_RESULT_FAIL_BIT)

#define SL_IS_OK(x)    ( ((x) & SL_RESULT_FAIL_BIT) == 0 )
#define SL_IS_FAIL(x)  ( ((x) & SL_RESULT_FAIL_BIT) )

typedef void (*node_callback_t)(MeasurementData* , size_t );

typedef struct {
	RPLidar *lidar;
    node_callback_t callback;
    // Add other parameters if needed
} task_params_t;

void process_nodes(MeasurementData* measurements, size_t count);

#endif // RPLIDAR_H