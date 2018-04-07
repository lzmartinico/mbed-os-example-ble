#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "LocalisationService.h"
#include "LSM9DS1.h"
// #include "mbed_mem_trace.h"

DigitalOut alivenessLED(LED1, 1);
const static char     DEVICE_NAME[] = "LP";
static const uint16_t uuid16_list[] = {LocalisationService::LOCALISATION_SERVICE_UUID};
LocalisationService *localisationService;
LSM9DS1 imu(p7, p30);
static EventQueue eventQueue(/* event count */ 32 * EVENTS_EVENT_SIZE);

uint8_t mac5_list[10] = {0xcd, 0xd7, 0x17, 0x51, 0x43, 0xb8, 0x2a, 0xf8, 0x3d, 0x62};
uint8_t mac4_list[10] = {0x75, 0xb6, 0x2d, 0x88, 0xc9, 0x00, 0x80, 0x0c, 0xea, 0x88};

void inertialCallback() {
    imu.readAccel();
    imu.readGyro();
    imu.readMag();

    // TODO update characteristic
    localisationService->updateIMU(&imu.gx_raw);
}

void periodicCallback(void) {
    alivenessLED = !alivenessLED; /* Do blinky on LED1 while we're waiting for BLE events */
}

void printDevice(uint8_t mac5, uint8_t mac4, int8_t rssi) {
    printf(":%02x:%02x %d\r\n", mac4, mac5, rssi);
}

bool isNearBeacon(const Gap::AdvertisementCallbackParams_t *params) {
    uint8_t mac5 = params->peerAddr[0];
    uint8_t mac4 = params->peerAddr[1];
    if (params->rssi >= -93) {
        for (size_t i = 0; i < 10; i++) {
            if (mac5_list[i] == mac5 && mac4_list[i] == mac4) return true;
        }
    }
    return false;
}

void advertisementCallback(const Gap::AdvertisementCallbackParams_t *params) {

    if (isNearBeacon(params)) {
        eventQueue.call(printDevice, params->peerAddr[0], params->peerAddr[1], params->rssi);
        localisationService->updateBeaconRssi(params->rssi, params->peerAddr[0]);
    }
}

void onBleInitError(BLE &ble, ble_error_t error)
{
   /* Initialization error handling should go here */
}

void printMacAddress()
{
    /* Print out device MAC address to the console*/
    Gap::AddressType_t addr_type;
    Gap::Address_t address;
    BLE::Instance().gap().getAddress(&addr_type, address);
    printf("DEVICE MAC ADDRESS: ");
    for (int i = 5; i >= 1; i--){
        printf("%02x:", address[i]);
    }
    printf("%02x\r\n", address[0]);
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{
    (void) params;
    BLE::Instance().gap().setScanParams(500, 500);
    BLE::Instance().gap().startScan(advertisementCallback);
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    (void) params;
    BLE::Instance().gap().stopScan();
    BLE::Instance().gap().startAdvertising();
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onConnection(connectionCallback);
    ble.gap().onDisconnection(disconnectionCallback);

    localisationService = new LocalisationService(ble);

    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms. */
    ble.gap().startAdvertising();

    printMacAddress();
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

//Init LSM9DS1 chip
void inertialSetup()
{
    // Use the begin() function to initialize the LSM9DS0 library.
    // You can either call it with no parameters (the easy way):
    uint16_t status = imu.begin();
 
    //Make sure communication is working
    printf("LSM9DS1 WHO_AM_I's returned: 0x%X\r\n", status);
    printf("Should be 0x683D\r\n");
}

int main()
{
    // mbed_mem_trace_set_callback(mbed_mem_trace_default_callback);

    eventQueue.call_every(500, periodicCallback);
    eventQueue.call_every(500, inertialCallback);

    inertialSetup();

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
