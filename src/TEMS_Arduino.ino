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
#define DELAY_TIME_MAIN_LOOP 3000
#define DELAY_TIME_BLE_CONN  5000

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
#define REQ_STAT_STOP_SIZE   4
#define REQ_DATE_SET_SIZE    8

//TODO result size re-check!!!!
#define RES_LOG_DATA_SIZE    1
#define RES_RESULT_RET_SIZE  11
#define RES_LOG_INFOR_SIZE   13

#define ALLOCATE_SIZE_RESULT 9
#define ALLOCATE_SIZE_LOG    11

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
        Serial.println("onConnect");
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
    Serial.print("data: ");
    Serial.println((char*)pData);
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
    if (doConnect) {
        if (connected && pClient->isConnected()) {

            // read the measured data from device via BLE communication
            readMeasureResult();

        } else {
            Serial.println("Bluetooth disconnected!!");
            doConnect = false;
            connected = false;
        }
    } else {
        Serial.println("Bluetooth not connected..");
        Serial.println("Start scanning to reconnect...");
        scanBlueToothDevice();
        connectBlueToothDevice();

        delay(DELAY_WAIT_CONNECT);

        handshake_setup_BLE(); //do handshake to set up the ble connection
    }

    delay(DELAY_TIME_MAIN_LOOP);
}

/**
 * Loop until the given characteristic is readable.
 *
 * @param {characteristic} A pointer that points to the target characteristic.
 */
inline void waitUntilReadable_characteristic(BLERemoteCharacteristic *characteristic) {
    while (!characteristic->canRead()) delay(DELAY_WAIT_RESPONSE);
}

/**
 * Read the measured data from the device.
 */
inline void readMeasureResult() {
    characteristic_cmd_control->writeValue(&cmd_RESULT_RETURN, REQ_RESULT_RET_SIZE, true); //send cmd_Result_Return

    waitUntilReadable_characteristic(characteristic_cmd_measurement); //delay until the characteristic_cmd_measurement is readable

    uint8_t *rawData = characteristic_cmd_control->readRawData(); //get raw data (hex bytes)

    rawData += 2;

    int i;
    for (i = 0; i < ALLOCATE_SIZE_RESULT; i++) {
        Serial.printf("%d\n", *rawData); //print out data
        resultData[i] = *rawData;
        rawData++;
    }
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
    *dateTimeBuffer = 0;
    dateTimeBuffer++;
    *dateTimeBuffer = 0;
    dateTimeBuffer++;
    *dateTimeBuffer = 0;
}

/**
 * This function does a handshake to set up the ble connection.
 */
void handshake_setup_BLE() {
    if (pClient->isConnected()) {
        //buffer for the date time data
        uint8_t *dateTimeBuffer = (uint8_t*) calloc(REQ_DATE_SET_SIZE, sizeof(uint8_t));

        dateTimeBuffer[0] = cmd_BLE_Date_Time_Set;
        dateTimeBuffer[1] = 6; // 6 bytes for date time

        setDateTime(dateTimeBuffer); //get current date time and store it in the date time buffer.

        characteristic_cmd_control->writeValue(dateTimeBuffer, REQ_DATE_SET_SIZE, false); //send cmd_Date_Time_Set

        delay(DELAY_SEND_RES_RET);

        free(dateTimeBuffer); //free the allocated memory

        readMeasureResult(); //read the measured data from the device via BLE communication

        characteristic_cmd_log->writeValue(&cmd_LOG_INFO_QUERY, REQ_LOG_INFOR_SIZE, true); //send cmd_Log_Infor_QUERY
        waitUntilReadable_characteristic(characteristic_cmd_log); //delay until the cmd_LOG_INFO_QUERY is readable
        uint8_t *logRawData = characteristic_cmd_log->readRawData(); //get raw log data (hex bytes)
        //TODO log data
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

void connectBlueToothDevice() {
    Serial.println("\n\nBLE connection start");
    Serial.printf("\nConnect to the Device: %s \n", myDevice->toString().c_str());
    connected = connectToServer();
    doConnect = connected;
    Serial.printf("Connected Device: %s \n", myDevice->toString().c_str());
}

void scanBlueToothDevice() {
    while (true) {
        scanDevices();

        if (doConnect)
            break;

        delay(DELAY_TIME_BLE_SCAN);
    }
}

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
        Serial.println("Failed to connect");
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