#include <WiFi.h>             // Header File for WiFi
//#include <WiFiClientSecure.h> // Header File for secured WiFi client
#include "BLEUtils.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "BLEDevice.h"


//-----------------Preprocessor-------------------------//

#define PORT_NUMBER_HTTP     80
#define SERIAL_PORT_NUM      115200

#define DELAY_SEND_RES_RET   500

#define DELAY_TIME_WIFI_CONN 500
#define DELAY_TIME_BLE_SCAN  600
#define DELAY_TIME_BLE_START 2000
#define DELAY_TIME_MAIN_LOOP 3000
#define DELAY_TIME_BLE_CONN  5000
#define DELAY_TIME_BLE_DATA  60000

#define DELAY_WAIT_CONNECT   1000
#define DELAY_WAIT_RESPONSE  5000

#define DELAY_LIMIT_WIFI_CLI 15000

#define AP_SSID              "esp32"

#define BLUE_TOOTH_SCAN_TIME 50
#define BLE_SCAN_WINDOW_SIZE 99
#define BLE_SCAN_INTERVAL    100

#define TARGET_DEVICE_NAME   "BLE_Gamma:lsj4"
#define CHAR_UUID_CTL        "00001524-1212-efde-1523-785feabcd123"
#define CHAR_UUID_MEAS       "00001525-1212-efde-1523-785feabcd123"
#define CHAR_UUID_LOG        "00001526-1212-efde-1523-785feabcd123"

#define REQ_RESULT_RET_SIZE  1
#define REQ_LOG_INFOR_SIZE   1
#define REQ_LOG_DATA_SIZE    1
#define REQ_START_STOP_SIZE  4
#define REQ_DATE_SET_SIZE    8

#define ALLOCATE_SIZE_RESULT 9
#define ALLOCATE_SIZE_LOG    11

#define SIZE_OF_UINT8        sizeof(uint8_t)

//------------------------------------------------------//


//-----------------Global variable----------------------//

BLEScan* pBLEScan;             // Bluetooth Scanner
BLEClient* pClient;            // BluetoothClient
BLEAdvertisedDevice* myDevice; // Bluetooth Advertised device

char *ssid = "YOUR_WIFI_SSID";
char *pw = "YOUR_WIFI_PW";
int portNo = PORT_NUMBER_HTTP;
int scanTime = BLUE_TOOTH_SCAN_TIME;
char *host = "www.your_host_url";

String hostStr = String(host);
String url = "/collect"; //the target URL

uint8_t resultData[ALLOCATE_SIZE_RESULT] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

static boolean doConnect = false;
static boolean connected = false;

static BLEUUID WP_CTL_UUID(CHAR_UUID_CTL);
static BLEUUID WP_MEAS_UUID(CHAR_UUID_MEAS);
static BLEUUID WP_LOG_UUID(CHAR_UUID_LOG);
static BLEUUID SERVICE_UUID("00001523-0000-1000-8000-00805f9b34fb");

static BLERemoteCharacteristic* characteristic_cmd_control;
static BLERemoteCharacteristic* characteristic_cmd_measurement;
static BLERemoteCharacteristic* characteristic_cmd_log;

static uint8_t cmd_RESULT_RETURN = 0xA0;
static uint8_t cmd_BLE_STARTSTOP = 0x10;

static uint8_t cmd_BLE_Date_Time_Set = 0x20;
static uint8_t cmd_LOG_INFO_QUERY = 0x21;
static uint8_t cmd_LOG_DATA_SEND = 0xA1;

static uint8_t cmd_BLE_USER_NAME_SET = 0x30;
static uint8_t cmd_BLE_USER_NAME_QUERY = 0x31;

//------------------------------------------------------//


//-----------------Function prototype-------------------//

void setupWiFi();
void connectWiFi();
void initBLE();
void connectBlueToothDevice();
int scanDevices();
int sendData(String dataString);
bool connectToServer();
void scanBlueToothDevice();
void handshake_setup_BLE();
inline void readMeasureResult();
inline void startMeasurement(uint8_t log_storage_interval);
inline void stopMeasurement(uint8_t log_storage_interval);
void readData();
void checkRawData(uint8_t *rawData);

//------------------------------------------------------//


/**
 * The aim of this class is to implement a custom event listener to detect the advertised device that we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

    /**
     * Eventlistener that activate only when it finds the advertising device.
     *
     * @param {advertisedDevice} An advertised device instance
     */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());

        //Name: BLE_Gamma:lsj4, Address: d7:6d:da:40:be:6a, appearance: 0
        if (advertisedDevice.getName().compare(TARGET_DEVICE_NAME) == 0) {
            BLEDevice::getScan()->stop();

            // free the pre-allocated memory (if exist), and create new BLEAdvertisedDevice instance
            if (myDevice != nullptr)
                delete myDevice;
            myDevice = new BLEAdvertisedDevice(advertisedDevice);

            doConnect = true;
        }
    }
};

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        connected = true;
        Serial.println("onConnect() activated");
    }

    void onDisconnect(BLEClient* pclient) {
        connected = false;
        Serial.println("onDisconnect");
    }
};


static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData,size_t length, bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
}


/* Initialise things before starting the main loop */
void setup() {
    Serial.begin(SERIAL_PORT_NUM); //Start Serial monitor in 9600
    initBLE(); // initialise the BLE scanner
    scanBlueToothDevice(); //scan a bluetooth device
    connectBlueToothDevice(); //connect a bluetooth device
    setupWiFi(); //initialise and connect to the WiFi
    handshake_setup_BLE();
}


/* The main loop of the arduino. */
void loop() {
    // check if the BLE scanner found the target device.
    if (doConnect) {
        // check if the BLEClient is connected to the server
        if (connected && pClient->isConnected()) {
            // read the measured data from device via BLE communication
            readMeasureResult();
            delay(DELAY_TIME_BLE_DATA);
        } else {
            Serial.println("Bluetooth disconnected!!");
            doConnect = false;
            connected = false;
            delay(DELAY_TIME_MAIN_LOOP);
        }
    } else {
        Serial.println("Bluetooth not connected..");
        Serial.println("Start scanning to reconnect...");
        scanBlueToothDevice();    //scan ble devices
        connectBlueToothDevice(); //connect to the BLEServer

        delay(DELAY_WAIT_CONNECT);

        handshake_setup_BLE(); //do handshake to set up the ble connection

        delay(DELAY_TIME_MAIN_LOOP);
    }
}

/**
 * Read the measured data from the device.
 */
inline void readMeasureResult() {
    Serial.println("\nSend cmd_Result_Return");

    characteristic_cmd_control->writeValue(&cmd_RESULT_RETURN, REQ_RESULT_RET_SIZE, true); //send cmd_Result_Return

    delay(DELAY_WAIT_RESPONSE);

    uint8_t *rawData = characteristic_cmd_measurement->readRawData();
    checkRawData(rawData);

    Serial.println("readMeasureResult() finish");
}

void setDateTime(uint8_t *dateTimeBuffer) {
    //TODO dummy data....
    dateTimeBuffer += 2;
    *dateTimeBuffer = 0x13;
    dateTimeBuffer++;
    *dateTimeBuffer = 7;
    dateTimeBuffer++;
    *dateTimeBuffer = 4;
    dateTimeBuffer++;
    *dateTimeBuffer = 1;
    dateTimeBuffer++;
    *dateTimeBuffer = 1;
    dateTimeBuffer++;
    *dateTimeBuffer = 1;
}

/**
 * Send the cmd_BLE_STARTSTOP to the device to either start or stop the measurement process.
 *
 * @param {new_stat} Should be either 0x01 or 0x00. 0x01 for start, and 0x00 for stop.
 * @param {log_storage_interval} Log interval.
 */
void changeMeasurementStatus(uint8_t new_stat, uint8_t log_storage_interval) {
    uint8_t logInterval;
    uint8_t bleStat;

    if (log_storage_interval <= 0x0A)
        logInterval = 0x0A;
    else
        logInterval = 0x3c;

    Serial.println("\nSend cmd_BLE_STARTSTOP");
    uint8_t *startStopBuffer = (uint8_t *) calloc(REQ_START_STOP_SIZE, SIZE_OF_UINT8);

    startStopBuffer[0] = cmd_BLE_STARTSTOP;
    startStopBuffer[1] = 2;
    startStopBuffer[2] = new_stat;
    startStopBuffer[3] = logInterval;

    characteristic_cmd_control->writeValue(startStopBuffer, REQ_START_STOP_SIZE, true); //send cmd_BLE_STARTSTOP
    delay(DELAY_SEND_RES_RET);
    free(startStopBuffer);
    Serial.println("Finish sending cmd_BLE_STARTSTOP");
}

/**
 * Send cmd_BLE_STARTSTOP to make the device start the measurement process.
 *
 * @param {log_storage_interval} Log interval.
 */
inline void startMeasurement(uint8_t log_storage_interval) {
    changeMeasurementStatus(0x01, log_storage_interval);
}

/**
 * Send cmd_BLE_STARTSTOP to make the device stop the measurement process.
 *
 * @param {log_storage_interval} Log interval.
 */
inline void stopMeasurement(uint8_t log_storage_interval) {
    changeMeasurementStatus(0x00, log_storage_interval);
}

inline void iterateRawData(uint8_t *rawData, int len) {
    Serial.print("Received data :");
    for (int i = 0; i < len; i++) {
        Serial.printf(" %d", *rawData);
        rawData++;
    }
    Serial.println("");
}

void iterateLogInfo(uint8_t *rawData) {
    Serial.println("Iterate Log Info");
    iterateRawData(rawData, 13);
}

void iterateReturnedResult(uint8_t *rawData) {
    Serial.println("Iterate Returned Result");
    iterateRawData(rawData, 11);

    if (*(rawData + 1) == 11) {
        if (*(rawData + 2) != 1) {
            //TODO need to check if this is the correct way of use the cmd_BLE_START_STOP
            startMeasurement(0x0A); //log interval could be either 0x0A or 0x3C
            delay(DELAY_TIME_BLE_START);
        }
    }
}


/**
 * Check the first byte of the raw data to identify the response.
 *
 * @param {rawData} The pointer that points to the raw data.
 */
void checkRawData(uint8_t *rawData) {
    switch (*rawData) {
        case 0x21 : //cmd_LOG_INFO_QUERY
            iterateLogInfo(rawData);
            break;
        case 0xA0 : //cmd_RESULT_RETURN
            iterateReturnedResult(rawData);
            break;
        default :
            Serial.printf("received = {%d}\n", *rawData);
    }

}

/**
 * Reads the raw data.
 */
void readData() {
    uint8_t *rawData_log = characteristic_cmd_log->readRawData();
    uint8_t *rawData_msr = characteristic_cmd_measurement->readRawData();

    checkRawData(rawData_log);
    checkRawData(rawData_msr);
}

/**
 * This function does a handshake to set up the ble connection.
 */
void handshake_setup_BLE() {
    Serial.println("\n\nHandshake start\n");

    if (pClient->isConnected()) {
        //buffer for the date time data
        Serial.println("\nSend date time");
        uint8_t *dateTimeBuffer = (uint8_t*) calloc(REQ_DATE_SET_SIZE, SIZE_OF_UINT8);

        dateTimeBuffer[0] = cmd_BLE_Date_Time_Set;
        dateTimeBuffer[1] = 6; // 6 bytes for date time

        setDateTime(dateTimeBuffer); //get current date time and store it in the date time buffer.

        characteristic_cmd_control->writeValue(dateTimeBuffer, REQ_DATE_SET_SIZE, true); //send cmd_Date_Time_Set

        delay(DELAY_SEND_RES_RET);

        free(dateTimeBuffer); //free the allocated memory

        readMeasureResult(); //read the measured data from the device via BLE communication

        //TODO characteristic_cmd_log
        characteristic_cmd_control->writeValue(&cmd_LOG_INFO_QUERY, REQ_LOG_INFOR_SIZE, true); //send cmd_Log_Infor_QUERY

        delay(DELAY_WAIT_RESPONSE);
        readData();
    } else {
        Serial.println("- Not connected!\n");
    }
}

/**
 * Initialise the BLE settings so that the ESP32 device could scan the BLE advertising devices.
 */
void initBLE() {
    Serial.println("Initialise the BLE module");

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(BLE_SCAN_INTERVAL);
    pBLEScan->setWindow(BLE_SCAN_WINDOW_SIZE); // less than or equal to setInterval value

    Serial.println(" - Create client");
    pClient  = BLEDevice::createClient();
    Serial.println(" - Client created\n\n");

    pClient->setClientCallbacks(new MyClientCallback());
}

/**
 * Function that helps the device to set up the WiFi environment.
 */
void setupWiFi() {
    WiFi.begin(ssid, pw); //Open the WiFi connection so that ESP32 could send data to server via HTTP/HTTPS protocol.
    connectWiFi();
}

/**
 * Function that connects the device to the target WiFi.
 */
void connectWiFi() {
    Serial.print("\n\nConnecting to WiFi...");

    int count = 0;

    // use while loop to wait until the device is connected to the WiFi
    while (WiFi.status() != WL_CONNECTED) {
        if (count >= DELAY_LIMIT_WIFI_CLI)
            break;

        count += DELAY_TIME_WIFI_CONN;
        delay(DELAY_TIME_WIFI_CONN);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nConnection failed :(");
    } else {
        Serial.println("\nConnected!");
        Serial.print("\nConnected to:\n\t\tIP address = ");
        Serial.println(WiFi.localIP()); //print out the ip address.
    }
}

/**
 * Connect to the target bluetooth device.
 */
void connectBlueToothDevice() {
    Serial.println("\n\nBLE connection start");
    Serial.printf("\nConnect to the Device: %s \n", myDevice->toString().c_str());
    connected = connectToServer();
    doConnect = connected;
    Serial.printf("Connected Device: %s \n", myDevice->toString().c_str());
}

/**
 * Scan the nearby bluetooth devices.
 * Run the infinite loop to scan until it finds the device that we are looking for.
 */
void scanBlueToothDevice() {
    while (true) {
        scanDevices();

        if (doConnect)
            break;

        delay(DELAY_TIME_BLE_SCAN);
    }
}

/**
 * Start scanning and check if there is the device that we are looking for.
 * Returns the number of found devices.
 *
 * @return The number of found devices.
 */
int scanDevices() {
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    pBLEScan->clearResults(); //delete results from BLEScan buffer to release memory
    return foundDevices.getCount();
}

/**
 * Check if the given characteristic is readable and notifiable.
 *
 * @param {pRemoteCharacteristic} a pointer that points to the characteristic instance
 */
void checkCharacteristic(BLERemoteCharacteristic* pRemoteCharacteristic) {
    Serial.println(" - Found our characteristic");
    Serial.println(pRemoteCharacteristic->toString().c_str());

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
        std::string value = pRemoteCharacteristic->readValue();
    }

    // check if the target characteristic is notifiable.
    if(pRemoteCharacteristic->canNotify())
        pRemoteCharacteristic->registerForNotify(notifyCallback);
}

/**
 * Look for the characteristics that we are interested in from the given BLERemoteService instance.
 *
 * @param {pRemoteService} pointer that points to the service instance
 */
void getCharacteristicFromService(BLERemoteService* pRemoteService) {
    if (pRemoteService == nullptr) {
        Serial.println("Null pointer!");
        return;
    }

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    BLERemoteCharacteristic* temp1 = pRemoteService->getCharacteristic(WP_CTL_UUID);
    BLERemoteCharacteristic* temp2 = pRemoteService->getCharacteristic(WP_MEAS_UUID);
    BLERemoteCharacteristic* temp3 = pRemoteService->getCharacteristic(WP_LOG_UUID);

    if (temp1 != nullptr) {
        checkCharacteristic(temp1);
        characteristic_cmd_control = temp1;
    }

    if (temp2 != nullptr) {
        checkCharacteristic(temp2);
        characteristic_cmd_measurement = temp2;
    }

    if (temp3 != nullptr) {
        checkCharacteristic(temp3);
        characteristic_cmd_log = temp3;
    }
}

/**
 * Connect to the BLE server.
 */
bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    // Connect to the remove BLE Server.
    BLEAddress myDevice_address = myDevice->getAddress();
    Serial.printf("target address: %s\n", myDevice_address.toString().c_str());

    esp_ble_addr_type_t ble_addr_type = myDevice->getAddressType();
    Serial.printf("target address type: %d\n", ble_addr_type);

    bool isConnected = pClient->connect(myDevice_address);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)

    if (!isConnected) {
        Serial.println("Connection failed");
        return false;
    }

    Serial.println("\n - Connected to server");

    delay(DELAY_TIME_BLE_CONN);

    Serial.println("Get services - BLEClient::getServices()");

    std::map<std::string, BLERemoteService*> *services = pClient->getServices();
    std::map<std::string, BLERemoteService*>::iterator it = services->begin();

    if (services->empty())
        Serial.println("No service...");

    Serial.println("Iterate services - start for loop in connectServer()");

    // free the pre-allocated memory (if exist), and create new BLERemoteCharacteristic instance
    if (characteristic_cmd_control != nullptr)
        delete characteristic_cmd_control;
    if (characteristic_cmd_measurement != nullptr)
        delete characteristic_cmd_measurement;
    if (characteristic_cmd_log != nullptr)
        delete characteristic_cmd_log;

    for (it = services->begin(); it != services->end(); ++it) {
        BLERemoteService *ble_service = it->second;
        getCharacteristicFromService(ble_service);
    }
}

/**
 * The function that sends the data via network.
 *
 * @param {queryString} The query string that will be used for url.
 * @return Returns 1 if success. Otherwise, returns 0.
 */
int sendData(String queryString) {
    WiFiClient client;

    Serial.print("\nConnecting to ");
    Serial.println(hostStr);

    //check if the http client is connected.
    if (!client.connect(host, PORT_NUMBER_HTTP)) {
        Serial.print("Connection failed...");
        return 0;
    }

    String header = "GET " + String(url) + "?" + String(queryString) + " HTTP/1.1";

    client.println(header);
    client.println("User-Agent: ESP32_TEMS");
    client.println("Host: " + hostStr);
    client.println("Connection: closed");
    client.println();

    Serial.println("Data sending process success!\n"); //to debug

    return 1;
}