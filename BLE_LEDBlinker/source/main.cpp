#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "LocalisationService.h"
// #include "mbed_mem_trace.h"

DigitalOut alivenessLED(LED1, 1);
static const char PEER_NAME[] = "LED";
static EventQueue eventQueue(/* event count */ 32 * EVENTS_EVENT_SIZE);

uint8_t mac5_list[10] = {0xcd, 0xd7, 0x17, 0x51, 0x43, 0xb8, 0x2a, 0xf8, 0x3d, 0x62};
uint8_t mac4_list[10] = {0x75, 0xb6, 0x2d, 0x88, 0xc9, 0x00, 0x80, 0x0c, 0xea, 0x88};

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

    // scan interval: 400ms and scan window: 400ms.
    // Every 400ms the device will scan for 400ms
    // This means that the device will scan continuously.
    ble.gap().setScanParams(500, 500);
    ble.gap().startScan(advertisementCallback);

    printMacAddress();
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

int main()
{
    // mbed_mem_trace_set_callback(mbed_mem_trace_default_callback);

    eventQueue.call_every(500, periodicCallback);

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
