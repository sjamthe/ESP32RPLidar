/*
 * Expect output for getInfo
model: 24
firmware: 1.24
hardware: 5
serialNumber (HEX): 81 53 9D F1 C3 E3 9A C4 C3 E6 98 F9 71 84 34 0D 
 *
 */
#include "RPLidar.h"

RPLidar lidar(Serial2, 16, 17, 25); 

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
    
    // Get device info
    RPLidar::DeviceInfo info;
    if (lidar.getInfo(info)) {
        Serial.println("RPLidar Info:");
        Serial.printf("  Model: %02d ", info.model);
        switch(info.model) {
            case 0x18: Serial.println("(RPLIDAR A-series)"); break;
            default: Serial.println("(Unknown model)"); break;
        }
        // Swap minor and major for correct display
        Serial.printf("  Firmware Version: %d.%02d\n", info.firmware_major, info.firmware_minor);
        Serial.printf("  Hardware Version: 0x%02d\n", info.hardware);
        Serial.print("  Serial Number: ");
        for (int i = 0; i < 16; i++) {
            Serial.printf("%02X", info.serialnum[i]);
        }
        Serial.println();
    }

    // Get health status
    RPLidar::DeviceHealth health;
    if (lidar.getHealth(health)) {
        Serial.println("RPLidar Health:");
        Serial.printf("  Status: %d\n", health.status);
        Serial.printf("  Error Code: 0x%04X\n", health.error_code);
        if (health.status == 2) {  // Error state
            Serial.println("Device is in error state!");
            delay(1000);  // Give time for error message to be sent
            //ESP.restart(); // Restart on error
            return;
        }
    } else {
        Serial.println("Error: Failed to get device health");
    }

    //Get Scan Rate
    RPLidar::DeviceScanRate scanRate;
    if (lidar.getSampleRate(scanRate)) {
        Serial.println("RPLidar Scan Rate:");
        Serial.printf("  Standard Scan Rate: %d usecs\n", scanRate.standard);
        Serial.printf("  Express Scan Rate: %d usecs\n", scanRate.express);
    } else {
        Serial.println("Error: Failed to get scan rate");
    }

    // Reset device before starting
    Serial.println("Resetting RPLidar...");
    lidar.reset();
    delay(2000);  // Give it time to reset
    
    // Start motor with a clean delay sequence
    Serial.println("Starting motor...");
    lidar.startMotor();
    delay(1000);  // Give motor time to reach speed

    // Start scan
    Serial.println("Starting scan...");
    //if (!lidar.startScan()) {
    if (!lidar.startExpressScan()) {
        Serial.println("Failed to start scan");
        delay(1000);  // Give time for error message to be sent
        //ESP.restart(); // Restart on scan failure (Should we restart lidar instead of ESP?)
        return;
    }
    Serial.println("Scan started successfully");

    // Wait for measurements to start
    delay(200);
}

unsigned long startMillis = 0;
unsigned long measurementCount = 0;
unsigned long rpsCount = 0;
unsigned long errorCount = 0;
unsigned long timeoutCount = 0;
bool firstMeasurement = true;

void loop1() {

}

void loop() {
    MeasurementData measurements[lidar.EXPRESS_MEASUREMENTS_PER_SCAN];
    size_t count = 0;
    
	sl_result ans = lidar.readMeasurement(measurements, count);
    if (ans == SL_RESULT_OK) {
        if (firstMeasurement) {
            Serial.println("First measurement received!");
            startMillis = millis();
            firstMeasurement = false;
        }
             
		measurementCount+= count;

		for (int i=0; i<count; i++) {
			if(measurements[i].startFlag) {
			rpsCount++;
			}
		}		
		// Print stats every second
		unsigned long now = millis();
		if ((now - startMillis) > 10000) {
			Serial.printf("Errors: %u. Timeouts: %u, Measurements: %u, Measurements per second: %04.0f, rps: %04.0f\n",
				errorCount, timeoutCount, measurementCount, measurementCount/((now-startMillis)/1000.0), rpsCount/((now-startMillis)/1000.0));
			Serial.printf("Last measurement - Angle: %.2f°, Distance: %.2fmm, Quality: %d\n", 
							measurements[0].angle, measurements[0].distance, measurements[0].quality);
			startMillis = millis();
			measurementCount = 0;
			rpsCount = 0;
			errorCount = 0;
		}
    } 
    else {
        if(ans == SL_RESULT_OPERATION_TIMEOUT)
			timeoutCount++;
		else
		 	errorCount++;
    }
}