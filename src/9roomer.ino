#include <DFRobot_SHT20.h>
#include <WiFi.h>
#include "BLEUtils.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "SPIFFS.h"
#include "FS.h"


//-----------------Preprocessors------------------------//

#define PORT_NUMBER          8090
#define SERIAL_PORT_NUM      115200

#define DELAY_SEND_RES_RET   200

#define DELAY_TIME_WIFI_CONN 500
#define DELAY_TIME_MAIN_LOOP 500
#define DELAY_TIME_BLE_SCAN  600
#define DELAY_TIME_BLE_START 2000
#define DELAY_TIME_BLE_CONN  5000

#define DELAY_INIT_SHT20     200

#define DELAY_WAIT_CONNECT   1000
#define DELAY_WAIT_RESPONSE  1000
#define DELAY_WAIT_MAIN      300000

#define DELAY_LIMIT_WIFI_CLI 15000

#define WIFI_CLIENT_TIMEOUT  5000

#define BLUE_TOOTH_SCAN_TIME 50
#define BLE_SCAN_WINDOW_SIZE 99
#define BLE_SCAN_INTERVAL    100

#define CHAR_UUID_CTL        "00001524-1212-efde-1523-785feabcd123"
#define CHAR_UUID_MEAS       "00001525-1212-efde-1523-785feabcd123"
#define CHAR_UUID_LOG        "00001526-1212-efde-1523-785feabcd123"

#define WIFI_SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b" //uuif for wifi name service
#define WIFI_PW_SERVICE_UUID "4fafc202-1fb5-459e-8fcc-c5c9c331914b" //uuid for wifi pw service
#define WIFI_NAME_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8" //uuid for wifi name characteristic
#define WIFI_PW_CHAR_UUID    "bed5483e-36e1-4688-b7f5-ea07361b26a8" //uuid for wifi pw characteristic

#define D_NAME_SERVICE_UUID  "4fafc203-1fb5-459e-8fcc-c5c9c331914b" //uuid for the device name setting service
#define D_NAME_CHAR_UUID     "bec5483e-36e1-4688-b7f5-ea07361b26a8" //uuid for the device name setting characteristic

#define BLE_SERVER_LIFE_TIME 10000 //180000
#define BLE_SERVER_ADD_TIME  120000
#define BLE_SERVER_SUB_TIME  2000

#define REQ_RESULT_RET_SIZE  1
#define REQ_LOG_INFOR_SIZE   1
#define REQ_LOG_DATA_SIZE    1
#define REQ_START_STOP_SIZE  4
#define REQ_DATE_SET_SIZE    8

#define ALLOCATE_SIZE_RESULT 9
#define ALLOCATE_SIZE_LOG    11

#define SIZE_OF_UINT8        sizeof(uint8_t)

#define INIT_TARGET_DEV_NAME "BLE_Gamma:0123"

//------------------------------------------------------//

class MyServerCallbacks;

//-----------------Global variables---------------------//

const std::string DEVICE_NUMBER = "u518";
const String DEV_NUM = String(DEVICE_NUMBER.c_str());
const std::string BLE_DEVICE_NAME = "9room-" + DEVICE_NUMBER;

int bleServerLifeTime;


BLEScan* pBLEScan = nullptr;             // Bluetooth Scanner
BLEClient* pClient = nullptr;            // BluetoothClient
BLEAdvertisedDevice* myDevice = nullptr; // Bluetooth Advertised device

static std::string *targetName = nullptr;

static BLEServer *pServer = nullptr;
static BLEService *pServerService_wifi = nullptr;
static BLEService *pServerService_pw = nullptr;
static BLECharacteristic *pCharacteristic_wifi = nullptr;
static BLECharacteristic *pCharacteristic_pw = nullptr;
static BLEAdvertising *pAdvertising = nullptr;
static MyServerCallbacks *serverCallback = nullptr;

static BLEService *pServerService_deviceName = nullptr;
static BLECharacteristic *pCharacteristic_deviceName = nullptr;

const char *ssid = "YOUR_WIFI_SSID";
const char *pw = "YOUR_WIFI_PW";

std::string *ssid_str = nullptr;
std::string *pw_str = nullptr;

int scanTime = BLUE_TOOTH_SCAN_TIME;

//char *host = "ec2-15-164-218-172.ap-northeast-2.compute.amazonaws.com";
char *host = "groom.techtest.shop";
String hostStr = String(host);
String url_g = "/logoneg";
String url_th = "/logoneth";
String url_time = "/time";

uint8_t resultData[ALLOCATE_SIZE_RESULT] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

bool needToChangeStatus = true;
bool handshake;

static bool doConnect = false;
static bool connected = false;

int LED_BUILTIN = 2;

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

static int serialNum_g = 0;
static int serialNum_th = 0;

static DFRobot_SHT20 sht20;

//------------------------------------------------------//

//--------------Function prototypes---------------------//

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData,size_t length, bool isNotify);
inline void sendMeasureRequest();
void setDateTime(uint8_t *dateTimeBuffer);
void changeMeasurementStatus(uint8_t new_stat, uint8_t log_storage_interval);
inline void startMeasurement(uint8_t log_storage_interval);
inline void stopMeasurement(uint8_t log_storage_interval);
inline void iterateRawData(uint8_t *rawData, int len);
void iterateLogInfo(uint8_t *rawData);
void iterateReturnedResult(uint8_t *rawData);
void checkRawData(uint8_t *rawData);
void sendData_BLE();
void handshake_setup_BLE();
void initBLEServer();
void setupWiFi();
void connectWiFi();
void connectBlueToothDevice();
void scanBlueToothDevice();
int scanDevices();
void checkCharacteristic(BLERemoteCharacteristic* pRemoteCharacteristic);
void getCharacteristicFromService(BLERemoteService* pRemoteService);
bool connectToServer();
bool validate_sht20(float val);
int sendViaHTTP(String queryString, String url);
int sendGeiger(float measuredVal, int deviceNum);
void removeAdvertising();
int sendTemperatureAndHumidity(float temperature, float humidity, int deviceNum);
void readFile(fs::FS &fs, const char * path, int type);
void writeFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char * path);

//------------------------------------------------------//


/**
 * The aim of this class is to implement a custom event listener to get the wifi password.
 */
class WiFiPasswordCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            if (pw_str != nullptr) delete pw_str;

            pw_str = new std::string(value);

            deleteFile(SPIFFS, "/wifi_pw.txt");
            writeFile(SPIFFS, "/wifi_pw.txt", value.c_str());

            Serial.println("*********");
            Serial.print("New value: ");
            for (int i = 0; i < value.length(); i++)
                Serial.print(value[i]);

            Serial.println();
            Serial.println("*********");
        }
    }
};

/**
 * The aim of this class is to implement a custom event listener to get the wifi name.
 */
class WiFiNameCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            if (ssid_str != nullptr) delete ssid_str;
            ssid_str = new std::string(value);

            deleteFile(SPIFFS, "/wifi_name.txt");
            writeFile(SPIFFS, "/wifi_name.txt", value.c_str());

            Serial.println("*********");
            Serial.print("Received Wi-Fi ssid: ");
            for (int i = 0; i < value.length(); i++)
                Serial.print(value[i]);

            Serial.println("\n*********");
        }
    }
};

/**
 * The ble server's event listener that helps the esp32 to store the target device name.
 */
class DeviceNameCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            if (targetName != nullptr) delete targetName;

            targetName = new std::string(value);

            deleteFile(SPIFFS, "/device_name.txt");
            writeFile(SPIFFS, "/device_name.txt", value.c_str());

            Serial.println("*********");
            Serial.print("New BLE device name: ");
            for (int i = 0; i < value.length(); i++)
                Serial.print(value[i]);

            Serial.println("\n*********");
        }
    }
};

/**
 * The aim of this class is to implement a custom event listener for the local BLE server.
 */
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("- MyServerCallbacks::onConnect() executed");

        bleServerLifeTime += BLE_SERVER_ADD_TIME;
    };

    void onDisconnect(BLEServer* pServer) {
        Serial.println("- MyServerCallbacks::onDisconnect() executed");
    }
};


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

        if (advertisedDevice.getName().compare(*targetName) == 0) {
            BLEDevice::getScan()->stop();

            // free the pre-allocated memory (if exist), and create new BLEAdvertisedDevice instance
            if (myDevice != nullptr)
                delete myDevice;
            myDevice = new BLEAdvertisedDevice(advertisedDevice);

            doConnect = true;
        }
    }
};

/**
 * BLE client instance's callback that helps the ESP32 to check if the ESP32 is connected to the BLE server that we are looking for.
 */
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        connected = true;
        Serial.println("onConnect() activated");
    }

    void onDisconnect(BLEClient* pclient) {
        connected = false;
        Serial.println("onDisconnect() activated");
    }
};


static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData,size_t length, bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);

    delay(DELAY_WAIT_RESPONSE);

    checkRawData(pData);
}


/* Initialise things before starting the main loop */
void setup() {
    targetName = new std::string(INIT_TARGET_DEV_NAME);

    Serial.begin(SERIAL_PORT_NUM); //Start Serial monitor in 115200

    Serial.println("Initialise the BLE module");
    BLEDevice::init(BLE_DEVICE_NAME);

    if (!SPIFFS.begin(true)) {
        Serial.println("An error has occurred while mounthing SPIFFS\n");
    } else {

        readFile(SPIFFS, "/wifi_name.txt", 1);
        readFile(SPIFFS, "/wifi_pw.txt", 2);
        readFile(SPIFFS, "/device_name.txt", 3);

        initBLEServer();

        bleServerLifeTime = BLE_SERVER_LIFE_TIME;

        pinMode (LED_BUILTIN, OUTPUT);

        while (bleServerLifeTime > 0) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
            bleServerLifeTime -= BLE_SERVER_SUB_TIME;
            Serial.printf("BLE server mode remaining time: %d sec\n", bleServerLifeTime / 1000);
        }

        Serial.println("\nTerminating BLE server mode");
        removeAdvertising();

        Serial.println("BLE server mode terminated!\nBLE client mode start!\n\n");

        digitalWrite(LED_BUILTIN, HIGH);
        delay(3000);
        digitalWrite(LED_BUILTIN, LOW);
    }

    initBLE(); // initialise the BLE scanner
    scanBlueToothDevice(); //scan a bluetooth device
    connectBlueToothDevice(); //connect a bluetooth device
    handshake = false;
    setupWiFi(); //initialise and connect to the WiFi
    handshake_setup_BLE();

    //set up DFRobot_SHT20
    sht20.initSHT20();
    delay(DELAY_INIT_SHT20);
    sht20.checkSHT20();
}


/* The main loop of the arduino. */
void loop() {
    int waitTime = DELAY_WAIT_MAIN;

    // check if the BLE scanner found the target device.
    if (doConnect) {

        //If WiFi is not connected
        if (WiFi.status() != WL_CONNECTED) {

            Serial.println("WiFi is not connected!");
            setupWiFi();
            waitTime = DELAY_TIME_MAIN_LOOP;

        } else {

            // check if the BLEClient is connected to the server
            if (connected && pClient->isConnected()) {
                if (!handshake)
                    handshake_setup_BLE(); //do handshake to set up the ble connection

                float humd = sht20.readHumidity();
                Serial.print("humidity: ");
                Serial.println(humd);
                bool humidityIsValid = validate_sht20(humd);

                float temp = sht20.readTemperature();
                Serial.print("temperature: ");
                Serial.println(temp);
                bool temperatureIsValid = validate_sht20(temp);

                // send tempearture and humidity only when both humidity value and temperature value are valid (non error).
                if (humidityIsValid && temperatureIsValid) {
                    sendTemperatureAndHumidity(temp, humd, DEV_NUM);
                }

                serialNum_th += 1;

                // read the data from device via BLE communication
                sendMeasureRequest();
            } else {
                doConnect = false;
                connected = false;
                Serial.println("Bluetooth disconnected!!");
                waitTime = DELAY_TIME_MAIN_LOOP;
            }

        }
    } else {
        Serial.println("Bluetooth not connected..");
        Serial.println("Start scanning to reconnect...");
        scanBlueToothDevice();    //scan ble devices
        connectBlueToothDevice(); //connect to the BLEServer

        delay(DELAY_WAIT_CONNECT);

        handshake_setup_BLE(); //do handshake to set up the ble connection
        waitTime = DELAY_TIME_MAIN_LOOP;
    }

    delay(waitTime);
}

void readFile(fs::FS &fs, const char * path, int type) {
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);

    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");

    std::string readLine = "";

    while(file.available()){
        readLine += char(file.read());
    }

    Serial.println(readLine.c_str());

    switch (type) {
        case 1 :
            if (ssid_str != nullptr) delete ssid_str;
            ssid_str = new std::string(readLine);
            break;
        case 2 :
            if (pw_str != nullptr) delete pw_str;
            pw_str = new std::string(readLine);
            break;
        case 3 :
            if (targetName != nullptr) delete targetName;
            targetName = new std::string(readLine);
            break;
        default :
            Serial.println("Invalid type!");
    }

    Serial.println("\nFinished reading file\n\n");
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written\n");
    } else {
        Serial.println("- frite failed\n");
    }
}

void deleteFile(fs::FS &fs, const char * path) {
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted\n");
    } else {
        Serial.println("- delete failed\n");
    }
}


bool validate_sht20(float val) {
    int i = (int) val;
    bool isValid = false;
    switch (i) {
        case 998:
            Serial.println("Error::I2C_Time_Out - Sensor not detected");
            break;
        case 999:
            Serial.println("Error::BAD_CRC - CRC bad");
            break;
        default:
            isValid = true;
    }

    return isValid;
}

/**
 * Read the measured data from the device.
 */
inline void sendMeasureRequest() {
    Serial.println("\nSend cmd_Result_Return");
    characteristic_cmd_control->writeValue(&cmd_RESULT_RETURN, REQ_RESULT_RET_SIZE, true); //send cmd_Result_Return
    Serial.println("sendMeasureRequest() finish");
}

/**
 * This function helps the ESP32 to set the date time by getting current date time, and use the received data to set the current date time of the geiger device.
 */
void setDateTime(uint8_t *dateTimeBuffer) {
    WiFiClient client;

    Serial.print("\nConnecting to ");
    Serial.println(hostStr);

    //check if the http client is connected.
    if (!client.connect(host, PORT_NUMBER)) {
        Serial.print("Connection failed...");
        return;
    }

    String header = "GET " + url_time + " HTTP/1.1";

    client.println(header);
    client.println("User-Agent: ESP32_9roomer");
    client.println("Host: " + hostStr);
    client.println("Connection: closed");
    client.println();

    unsigned long timeout = millis();

    // use the while loop to wait until the server sends the HTTP response
    while (client.available() == 0) {
        if (millis() - timeout > WIFI_CLIENT_TIMEOUT) {
          Serial.println(">>> Client Timeout !");
          client.stop();
          return;
        }
    }


    String line;

    // use the while loop to read HTTP response
    while (client.available()) {
        line = client.readStringUntil('\r');
        Serial.print(line);
    }
    line.trim();

    uint8_t year = line.substring(0, 4).toInt() - 2000;
    uint8_t month = line.substring(5, 7).toInt();
    uint8_t date = line.substring(8, 10).toInt();
    uint8_t hour = line.substring(11, 13).toInt();
    uint8_t minute = line.substring(14, 16).toInt();
    uint8_t second = line.substring(17, 19).toInt();

    Serial.println("\nData sending process success!\n"); //to debug

    client.stop();

    dateTimeBuffer += 2;
    *dateTimeBuffer = year;
    dateTimeBuffer++;
    *dateTimeBuffer = month;
    dateTimeBuffer++;
    *dateTimeBuffer = date;
    dateTimeBuffer++;
    *dateTimeBuffer = hour;
    dateTimeBuffer++;
    *dateTimeBuffer = minute;
    dateTimeBuffer++;
    *dateTimeBuffer = second;
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
        if (*(rawData + 2) != 0x01) {
            needToChangeStatus = true;
        } else {
            //TODO needToChangeStatus = false;
        }
    }

    int val_msb = rawData[7];
    int val_lsb = rawData[8];

    float measuredVal = ((float) (val_lsb << 8) + val_msb) / 100.0;
    Serial.printf("measured value = %1.2f\n", measuredVal);
    sendGeiger(measuredVal, DEV_NUM);

    serialNum_g += 1;
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
 * Sends the raw data.
 */
void sendData_BLE() {
    if (needToChangeStatus) {
        startMeasurement(0x0A); //log interval could be either 0x0A or 0x3C
    } else {
        stopMeasurement(0x0A);
    }

    delay(DELAY_SEND_RES_RET);

    sendMeasureRequest(); //read the measured data from the device via BLE communication

    delay(DELAY_SEND_RES_RET);

    if (needToChangeStatus) {
        uint8_t logBuffer[1] = {cmd_LOG_INFO_QUERY};
        characteristic_cmd_control->writeValue(logBuffer, REQ_LOG_INFOR_SIZE, true); //send cmd_Log_Infor_QUERY
    }
}

/**
 * This function does a handshake to set up the ble connection.
 */
void handshake_setup_BLE() {
    Serial.println("\n\nHandshake start\n");

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected...\nTerminate handshake");
        return;
    }

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

        sendMeasureRequest(); //read the measured data from the device via BLE communication

        delay(DELAY_WAIT_RESPONSE);

        sendData_BLE();

        handshake = true;
    } else {
        Serial.println("- Not connected!\n");
    }
}

void removeAdvertising() {
    pAdvertising->stop();

    if (pAdvertising != nullptr) delete pAdvertising;

    if (pCharacteristic_wifi != nullptr) delete pCharacteristic_wifi;
    if (pCharacteristic_pw != nullptr) delete pCharacteristic_pw;
    if (pCharacteristic_deviceName != nullptr) delete pCharacteristic_deviceName;

    if (pServerService_wifi != nullptr) delete pServerService_wifi;
    if (pServerService_pw != nullptr) delete pServerService_pw;
    if (pServerService_deviceName != nullptr) delete pServerService_deviceName;

    if (serverCallback != nullptr) delete serverCallback;

    BLEDevice::removeServer();
}

/**
 * Initialise the BLE server.
 */
void initBLEServer() {
    if (pServer != nullptr) delete pServer;
    pServer = BLEDevice::createServer();
    serverCallback = new MyServerCallbacks();
    pServer->setCallbacks(serverCallback);

    if (pServerService_wifi != nullptr) delete pServerService_wifi;
    if (pServerService_pw != nullptr) delete pServerService_pw;
    if (pServerService_deviceName != nullptr) delete pServerService_deviceName;

    pServerService_wifi = pServer->createService(WIFI_SERVICE_UUID);
    pServerService_pw = pServer->createService(WIFI_PW_SERVICE_UUID);
    pServerService_deviceName = pServer->createService(D_NAME_SERVICE_UUID);

    if (pCharacteristic_wifi != nullptr) delete pCharacteristic_wifi;
    if (pCharacteristic_pw != nullptr) delete pCharacteristic_pw;
    if (pCharacteristic_deviceName != nullptr) delete pCharacteristic_deviceName;

    pCharacteristic_wifi = pServerService_wifi->createCharacteristic(
                      WIFI_NAME_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
    pCharacteristic_pw = pServerService_pw->createCharacteristic(
                      WIFI_PW_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
    pCharacteristic_deviceName = pServerService_deviceName->createCharacteristic(
                      D_NAME_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

    pCharacteristic_wifi->setCallbacks(new WiFiNameCallbacks());
    pCharacteristic_pw->setCallbacks(new WiFiPasswordCallbacks());

    pCharacteristic_deviceName->setCallbacks(new DeviceNameCallbacks());

    pServerService_wifi->start();
    pServerService_pw->start();
    pServerService_deviceName->start();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("Local BLE server created!!\n");
}

/**
 * Initialise the BLE settings so that the ESP32 device could scan the BLE advertising devices.
 */
void initBLE() {
    Serial.println("Generate BLE Scanner to scan ble devices");
    if(pBLEScan != nullptr) delete(pBLEScan);
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(BLE_SCAN_INTERVAL);
    pBLEScan->setWindow(BLE_SCAN_WINDOW_SIZE); // less than or equal to setInterval value

    if(pClient != nullptr) delete(pClient);
    Serial.println(" - Create client");
    pClient  = BLEDevice::createClient();
    Serial.println(" - Client created\n\n");

    pClient->setClientCallbacks(new MyClientCallback());
}

/**
 * Function that helps the device to set up the WiFi environment.
 */
void setupWiFi() {
    if (WiFi.status() != WL_CONNECTED) WiFi.disconnect();
    WiFi.mode(WIFI_STA); //init wifi mode

    if (ssid_str != nullptr) {
        if (pw_str == nullptr) {
            WiFi.begin(ssid_str->c_str());
        } else {
            WiFi.begin(ssid_str->c_str(), pw_str->c_str());
        }
    } else {
        if (pw == "") {
            WiFi.begin(ssid); //Open the WiFi connection so that ESP32 could send data to server via HTTP/HTTPS protocol.
        } else {
            WiFi.begin(ssid, pw); //Open the WiFi connection so that ESP32 could send data to server via HTTP/HTTPS protocol.
        }
    }
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
 * Send the collected data to the TEMS server via HTTP.
 */
int sendViaHTTP(String queryString, String url) {
    WiFiClient client;

    Serial.print("\nConnecting to ");
    Serial.println(hostStr);

    //check if the http client is connected.
    if (!client.connect(host, PORT_NUMBER)) {
        Serial.print("Connection failed...");
        return 0;
    }

    String header = "GET " + url + queryString + " HTTP/1.1";

    client.println(header);
    client.println("User-Agent: ESP32-groomer");
    client.println("Host: " + hostStr);
    client.println("Connection: closed");
    client.println();

    Serial.println("Data sending process success!\n"); //to debug

    client.stop();

    return 1;
}

/**
 * Send the collected geiger data to the server
 */
int sendGeiger(float measuredVal, String deviceNum) {
    String queryString = "?s=" + String(serialNum_g) + "&g=" + String(measuredVal) + "&u=" + deviceNum;
    return sendViaHTTP(queryString, url_g);
}

/**
 * Send the temperature data and humidity data to the server.
 */
int sendTemperatureAndHumidity(float temperature, float humidity, String deviceNum) {
    String queryString = "?s=" + String(serialNum_th) + "&t=" + String(temperature) + "&h=" + String(humidity) + "&u=" + deviceNum;
    return sendViaHTTP(queryString, url_th);
}