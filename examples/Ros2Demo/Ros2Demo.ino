
#include <WiFi.h>
#include <ESPmDNS.h>

#include "WifiSetup.h" // Includes wifi network information

//Ros variables
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#define ROS_AGENT_IP "192.168.86.36" // IP of machine running micro-ROS agent, TODO: Ad webapi to set this.
#define ROS_AGENT_PORT 8888 // Port of machine running micro-ROS agent. TODO: Ad webapi to set this.
#define ROS_NODE_NAME "esp32_lidar_node"
#define ROS_NAMESPACE ""

static bool ros_init_status = false;
static struct micro_ros_agent_locator locator;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//Lidar variables
#include "RingLaserScanBatchPool.h"
#include "RPLidar.h"

RingLaserScanBatchPool* batchPool = nullptr;
RPLidar* lidar = nullptr;
TaskHandle_t publishTaskHandle = NULL;
bool scanning = false;

void setupWifi() {
		
	WiFi.mode(WIFI_STA);
	WiFi.setHostname(hostname); 

	// Disconnect if we're already connected
	WiFi.disconnect(true);
	delay(1000);

	// Add these before WiFi.begin() to potentially improve stability
	WiFi.persistent(true);
	WiFi.setSleep(false);  // Disable WiFi sleep mode
	
	WiFi.begin(ssid, password);
	//IMPORTANT: Below line was needed for ESP32S3 to connect properly (after begin)
	WiFi.setTxPower(WIFI_POWER_8_5dBm);

	WiFi.setAutoReconnect(true);
	WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
	WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

	// More graceful connection handling
	int attempts = 0;
	while (WiFi.status() != WL_CONNECTED && attempts < 3) {
		delay(5000);
		attempts++;
		Serial.printf("Connection attempt %d/3, Status: %d\n", attempts, WiFi.status());
	}

	if (WiFi.status() != WL_CONNECTED) {
		Serial.printf("Error: Failed to connect after 3 attempts. WiFi status: %d\n", WiFi.status());
		// Now you can decide whether to restart or handle the failure differently
		ESP.restart();
	}

	// Enable mDNS responder
	if(!MDNS.begin(hostname)) {
		Serial.printf("Error: Error setting up MDNS responder!\n");
	}

	Serial.printf("WiFi connected to: %s\n", String(ssid));
	Serial.printf("IP address: %s\n", WiFi.localIP().toString());
	Serial.printf("Hostname: %s\n", WiFi.getHostname());
}

/* 
* Synchronize clock with the agent.
*/
void syncClock() {
	// Sync timeout
	const int timeout_ms = 1000;
	//Set Pacific timezone
	setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1);

	// Synchronize time with the agent
	rmw_uros_sync_session(timeout_ms);
	// After successful synchronization
	if (rmw_uros_epoch_synchronized()) {
		int64_t time_ns = rmw_uros_epoch_nanos();
		
		struct timeval tv;
		tv.tv_sec = time_ns / 1000000000LL;
		tv.tv_usec = (time_ns % 1000000000LL) / 1000;
		
		settimeofday(&tv, NULL);
	}
	struct tm timeinfo;
		char timeStringBuff[50];  // Make sure this is large enough for your format
	// Get and print local time
		if (getLocalTime(&timeinfo)) {
			strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S %Z\n", &timeinfo);
			Serial.printf("Current time: %s", timeStringBuff);  
		} else {
			Serial.printf("Failed to obtain time\n");
		}
	Serial.printf("Clock synchronized with Micro-ROS agent\n");
}

void setupMicroROS() {
	Serial.printf("Starting MicroROS connecting to agent on IP %s, port %d\n",ROS_AGENT_IP, ROS_AGENT_PORT);
	locator.address.fromString(ROS_AGENT_IP);
	locator.port = ROS_AGENT_PORT;

	// Set the agent IP and port
	if(rmw_uros_set_custom_transport(
			false,
			(void *) &locator,
			arduino_wifi_transport_open,
			arduino_wifi_transport_close,
			arduino_wifi_transport_write,
			arduino_wifi_transport_read
		) != RMW_RET_OK) {
		Serial.printf("Error establishing micro-ROS wifi support.\n");
			ros_init_status = false;
			return;
	}
	Serial.printf("Established ROS support\n");


		// Get the default allocator
	allocator = rcl_get_default_allocator();
	if (!rcutils_allocator_is_valid(&allocator)) {
		Serial.printf("Error: Failed to get Micro-ROS allocator\n");
		return;
	}
	Serial.printf("MicroROS got allocator\n");

	// Initialize micro-ROS support
	if (rmw_uros_ping_agent(100, 10) == RMW_RET_OK) {
		rcl_ret_t retval = rclc_support_init(&support, 0, NULL, &allocator);
		if(retval != RCL_RET_OK) {
			Serial.printf("Error(%d) initializing micro-ROS support.\n", retval);
			ros_init_status = false;
			vTaskDelay(pdMS_TO_TICKS(1000));
			return;
		}
	} else {
		Serial.printf(
		"Agent not responding. Make sure  micro_ros_agent is running on IP %s, port %d\n",ROS_AGENT_IP, ROS_AGENT_PORT);
		ros_init_status = false;
		vTaskDelay(pdMS_TO_TICKS(1000));
		return;
	}
		ros_init_status = true;
	Serial.printf("MicroROS support started connecting to agent on IP %s, port %d\n",ROS_AGENT_IP, ROS_AGENT_PORT);

	syncClock();

	// Create node
	node = rcl_get_zero_initialized_node(); // get zero initialized node
	rcl_ret_t retval = rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support);
	if(retval != RCL_RET_OK) {
		Serial.printf("Error(%d) creating micro-ROS node. %s\n",retval, rcutils_get_error_string());
		rcl_reset_error();
		return;
	}
	Serial.printf("MicroROS Node %s started\n", ROS_NODE_NAME);

}

bool getLidarInfo() {
// Get device info
    RPLidar::DeviceInfo info;
    if (lidar->getInfo(info)) {
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
        delay(10); 
        return false;
    }
	return true;
}

void setupLidar() {
    // Create batch pool first
    batchPool = new RingLaserScanBatchPool(10);
    if (!batchPool) {
        Serial.println("Failed to create batch pool");
        return;
    }
    Serial.println("Batch pool created");
    
    // Then create lidar with the pool
    lidar = new RPLidar(UART_NUM_2, 39, 40, 41, batchPool);
    if (!lidar) {
        Serial.println("Failed to create lidar");
        delete batchPool;
        return;
    }
    Serial.println("Lidar created");
    
        // Initialize lidar
    if (!lidar->begin()) {
        Serial.println("Failed to initialize lidar");
        delete lidar;
        delete batchPool;
        return;
    }
    
    Serial.println("Initialization complete");

	int attempts = 0;
	while (getLidarInfo() != true && attempts < 3) {
		delay(5000);
		attempts++;
		Serial.printf("Lidar getInfo attempt %d/3\n", attempts);
	}
}

void setup() {

	Serial.begin(115200);
	while(!Serial) {
		delay(100);
	}
	Serial.println("Starting wifi...");
	setupWifi();

	setupMicroROS();
	setupLidar();
}

void loop() {

}