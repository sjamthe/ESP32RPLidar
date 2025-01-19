// RPLidar.h
#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <Arduino.h>
#include <HardwareSerial.h>

class RPLidar {
public:
    // Constants for commands
    static const uint8_t CMD_SYNC_BYTE = 0xA5;
    static const uint8_t CMD_STOP = 0x25;
    static const uint8_t CMD_RESET = 0x40;
    static const uint8_t CMD_SCAN = 0x20;
    static const uint8_t CMD_EXPRESS_SCAN = 0x82;
    static const uint8_t CMD_FORCE_SCAN = 0x21;
    static const uint8_t CMD_GET_INFO = 0x50;
    static const uint8_t CMD_GET_HEALTH = 0x52;

	// Response Mode
	static const uint8_t SINGLE_RESP_MODE = 0x00;
	static const uint8_t MULTI_RESP_MODE = 0x01;

	// Response types
    static const uint8_t RESP_TYPE_INFO = 0x04;
    static const uint8_t RESP_TYPE_HEALTH = 0x06;
    static const uint8_t RESP_TYPE_SCAN = 0x81;

    // Response descriptor sync bytes
    static const uint8_t RESP_SYNC_BYTE1 = 0xA5;
    static const uint8_t RESP_SYNC_BYTE2 = 0x5A;

    // Measurement quality threshold
    static const uint8_t MIN_QUALITY = 0;
    static const uint16_t READ_TIMEOUT_MS = 200;  // 200ms timeout for reading

    // Structure for scan measurement data
    struct MeasurementData {
        float angle;        // In degrees
        float distance;     // In millimeters
        uint8_t quality;    // Quality of measurement
        bool startFlag;     // Start flag for new scan
    };

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
    bool startExpressScan();
    bool forceScan();
    
    // Information commands
    bool getHealth(DeviceHealth& health);
    bool getInfo(DeviceInfo& info);
    
    // Data retrieval
    bool readMeasurement(MeasurementData& measurement);
    
    // Motor control
    void startMotor(uint8_t pwm = 255);
    void stopMotor();

private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    int _motorPin;
    bool _motorEnabled;
    ResponseDescriptor _responseDescriptor;  // Store the last response descriptor

    // Helper functions
    bool waitResponseHeader();
    void flushInput();
    void sendCommand(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payloadSize = 0);
    uint8_t checksum(const uint8_t* data, uint8_t len);
	bool verifyResponseDescriptor(uint8_t expectedMode, uint8_t expectedType, uint32_t expectedLength);
};

#endif // RPLIDAR_H