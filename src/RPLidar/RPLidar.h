#include <stdint.h>
// RPLidar.h
#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <Arduino.h>
#include <HardwareSerial.h>


#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT    (0x1<<15)


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

// Structure for scan measurement data
struct MeasurementData {
	float angle;        // In degrees
	float distance;     // In millimeters
	uint8_t quality;    // Quality of measurement
	bool startFlag;     // Start flag for new scan
	unsigned long errorCount;
};

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

    // Express scan payload sync bytes
    static const uint8_t RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 = 0xA;
    static const uint8_t RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2 = 0x5;

    // Measurement quality threshold
    static const uint8_t MIN_QUALITY = 0;
    static const uint16_t READ_TIMEOUT_MS = 200;  // 200ms timeout for reading

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
    bool readMeasurement(MeasurementData&);
    
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
    uint8_t _scanResponseMode; // Stores the current scan response.
    ResponseDescriptor _responseDescriptor;  // Store the last response descriptor

    // Helper functions
    bool waitResponseHeader();
    void flushInput();
    void sendCommand(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payloadSize = 0);
    uint8_t checksum(const uint8_t* data, uint8_t len);
	  bool verifyResponseDescriptor(uint8_t expectedMode, uint8_t expectedType, uint32_t expectedLength);
    bool readMeasurementTypeScan(MeasurementData&);
    bool readMeasurementTypeExpExtended(MeasurementData&);
    bool readMeasurementTypeExpLegacy(MeasurementData&);
    bool _waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t& node, uint32_t timeout = READ_TIMEOUT_MS);
};

#endif // RPLIDAR_H