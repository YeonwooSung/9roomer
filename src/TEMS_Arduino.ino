#include <WiFi.h>             // Header File for WiFi
//#include <WiFiClientSecure.h> // Header File for secured WiFi client
#include <BLEDevice.h>
//#include <BLEUtils.h>
//#include <BLEScan.h>
//#include <BLEAdvertisedDevice.h>


//-----------------Preprocessor-------------------------//

#define DELAY_TIME_BLE_INPUT 50
#define DELAY_TIME_WIFI_CONN 500
#define DELAY_TIME_BLE_SCAN  600
#define DELAY_TIME_MAIN_LOOP 3000
#define DELAY_LIMIT_WIFI_CLI 15000
#define AP_SSID              "esp32"
#define PORT_NUMBER_HTTP     80
#define BLUE_TOOTH_SCAN_TIME 50
#define SERIAL_PORT_NUM      115200

//------------------------------------------------------//


//-----------------Global variable----------------------//

BLEScan* pBLEScan;             // Bluetooth Scanner
BLEAdvertisedDevice* myDevice; // Bluetooth Advertised device
char *ssid = "YOUR_WIFI_SSID";
char *pw = "YOUR_WIFI_PW";
int portNo = PORT_NUMBER_HTTP;
int scanTime = BLUE_TOOTH_SCAN_TIME;
char *host = "www.your_host_url";
String hostStr = String(host);
String url = "/collect"; //the target URL
static boolean doConnect = false;
static boolean connected = false;
static BLEUUID serviceUUID; // The remote service we wish to connect to.
static BLERemoteCharacteristic* pRemoteCharacteristic;

//------------------------------------------------------//


//-----------------Function prototype-------------------//

void setupWiFi();
void connectWiFi();
void initBLE();
void connectBlueToothDevice();
int scanDevices();
int sendData(String dataString);
bool connectToServer();

//------------------------------------------------------//


class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());

        //Name: BLE_Gamma:lsj4, Address: d7:6d:da:40:be:6a, appearance: 0
        if (advertisedDevice.getName().compare("BLE_Gamma:lsj4") == 0) {
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);

            serviceUUID = myDevice->getServiceUUID();
            connected = connectToServer();
            doConnect = connected;

            Serial.printf("Connected Device: %s \n", advertisedDevice.toString().c_str());
        }
    }
};

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        //TODO
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
    connectBlueToothDevice(); //scan and connect a bluetooth device
    setupWiFi(); //initialise and connect to the WiFi
}


/* The main loop of the arduino. */
void loop() {
    if (doConnect) {
        //TODO read and send data
        if (connected) {
            if (pRemoteCharacteristic->canRead()) {
                std::string readStr = pRemoteCharacteristic->readValue();
                const char *readStr_cStr = readStr.c_str();
                String readData = String(readStr_cStr);
                Serial.println(readData);
            }
        } else {
            Serial.println("Bluetooth disconnected!!");
            doConnect = false;
        }
    } else {
        Serial.println("Bluetooth not connected..");
        Serial.println("Start scanning to reconnect...");
        connectBlueToothDevice();
    }

    delay(DELAY_TIME_MAIN_LOOP);
}


void initBLE() {
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99); // less than or equal to setInterval value
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
    Serial.print("Connecting to WiFi...");

    int count = 0;

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
    while (1) {
        scanDevices();

        if (doConnect)
            break;

        delay(DELAY_TIME_BLE_SCAN);
    }
}

int scanDevices() {
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);

    Serial.println("Scan done!");
    pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

    return foundDevices.getCount();
}

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);

    if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }

    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(serviceUUID);

    if (pRemoteCharacteristic == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        pClient->disconnect();
        return false;
    }

    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
        std::string value = pRemoteCharacteristic->readValue();
        Serial.print("The characteristic value was: ");
        Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
        pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
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
    client.println("User-Agent: ESP32"); //TODO add device id
    client.println("Host: " + hostStr);
    client.println("Connection: closed");
    client.println();

    Serial.println("Data sending process success!\n"); //to debug

    return 1;
}
