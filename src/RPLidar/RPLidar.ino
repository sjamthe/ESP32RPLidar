#include "RPLidar.h"

RPLidar lidar(Serial2, 16, 17, 5); 
bool scanning = false;

void  getLidarInfo() {
// Get device info
    RPLidar::DeviceInfo info;
    if (lidar.getInfo(info)) {
        Serial.println("RPLidar Info:");
        Serial.printf("  Model: %02d ", info.model);
        switch(info.model) {
            case 0x18: Serial.println("(RPLIDAR A-series)"); break;
            default: Serial.println("(Unknown model)"); break;
        }
        Serial.printf("  Firmware Version: %d.%02d\n", info.firmware_major, info.firmware_minor);
        Serial.printf("  Hardware Version: 0x%02d\n", info.hardware);
        Serial.print("  Serial Number: ");
        for (int i = 0; i < 16; i++) {
            Serial.printf("%02X", info.serialnum[i]);
        }
        Serial.println();
    } else {
        Serial.println("Error: Failed to get device info");
        delay(10000); 
        ESP.restart();
        return;
    }

    // Get health status
    RPLidar::DeviceHealth health;
    if (lidar.getHealth(health)) {
        Serial.println("RPLidar Health:");
        Serial.printf("  Status: %d\n", health.status);
        Serial.printf("  Error Code: 0x%04X\n", health.error_code);
        if (health.status == 2) {  // Error state
            Serial.println("Device is in error state!");
            delay(10000);  // Give time for error message to be sent
            ESP.restart(); // Restart on error
            return;
        }
    } else {
        Serial.println("Error: Failed to get device health");
        delay(10000); 
        ESP.restart(); // Restart on error
        return;
    }

    // Get Scan Rate
    RPLidar::DeviceScanRate scanRate;
    if (lidar.getSampleRate(scanRate)) {
        Serial.println("RPLidar Scan Rate:");
        Serial.printf("  Standard Scan Rate: %d usecs\n", scanRate.standard);
        Serial.printf("  Express Scan Rate: %d usecs\n", scanRate.express);
    } else {
        Serial.println("Error: Failed to get scan rate");
        delay(10000); 
        ESP.restart(); // Restart on error
        return;
    }

}

void startLidarScan() {
        // Reset device before starting
    Serial.println("Resetting RPLidar...");
    lidar.resetLidar();
    delay(2000);  // Give it time to reset
    
    // Start motor with a clean delay sequence
    Serial.println("Starting motor...");
    lidar.startMotor();
    delay(1000);  // Give motor time to reach speed

    // Start scan
    Serial.println("Starting scan...");
    
    if (!lidar.startExpressScan()) {
        Serial.println("Failed to start scan");
        delay(1000);  
        return;
    }
    Serial.println("Scan started successfully");
	scanning = true;
    // Wait for measurements to start
    delay(200);
}

void setup() {
    // Start USB serial for debugging and wait for port to be ready
    Serial.begin(115200);
    delay(2000);  // Give time for USB serial to properly initialize
    Serial.println("\n\nRPLidar Test Starting...");
    delay(500);   // Additional delay to ensure stability
    
    // Initialize RPLidar
    Serial.println("Initializing RPLidar...");
    if (!lidar.begin()) {
        Serial.println("Failed to start RPLidar");
        return;
    }
    
    getLidarInfo();

    startLidarScan();
}

unsigned long startmillis = millis();
void loop() {
    // start & stop every minute
    if (scanning && (millis() - startmillis) > 1 * 60 * 1000) {
        lidar.stopScan();
		scanning = false;
        Serial.printf("Scan stopped at %d ms\n", millis());
    } else if (!scanning && (millis() - startmillis) > 2 * 60 * 1000) {
        if (lidar.startExpressScan()) {
			scanning = true;
            startmillis = millis();
    		Serial.println("Scan restarted successfully after 3 minutes");
		}
    }
    delay(100);
}