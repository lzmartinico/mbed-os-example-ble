/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "LEDService.h"
#include "DistanceService.h"
#include <string>

DigitalOut alivenessLED(LED1, 0);
DigitalOut actuatedLED(LED2, 0);

/*const char** mac_addrs = (const char *[]){
    "ed23c0d875cd",
    "e7311a8eb6d7",
    "c7bc919b2d17",
    "ec75a5ed8851",
    "fe12def2c943",
    "c03b5cfa00b8",
    "e0b83a2f802a",
    "f15576cb0cf8",
    "f17fb178ea3d",
    "fd8185988862"
};*/
static const char PEER_NAME[] = "tin76";
const static char     DEVICE_NAME[] = "LED";
static const uint16_t uuid16_list[] = {LEDService::LED_SERVICE_UUID};

static EventQueue eventQueue(/* event count */ 10 * EVENTS_EVENT_SIZE);

LEDService *ledServicePtr;
DistanceService *distanceServicePtr;

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    (void) params;
    BLE::Instance().gap().startAdvertising();
}

void blinkCallback(void)
{
    alivenessLED = !alivenessLED; /* Do blinky on LED1 to indicate system aliveness. */
}

/**
 * This callback allows the LEDService to receive updates to the ledState Characteristic.
 *
 * @param[in] params
 *     Information about the characterisitc being updated.
 */
void advertisementCallback(const Gap::AdvertisementCallbackParams_t *params) {
    // parse the advertising payload, looking for data type COMPLETE_LOCAL_NAME
    // The advertising payload is a collection of key/value records where
    // byte 0: length of the record excluding this byte
    // byte 1: The key, it is the type of the data
    // byte [2..N] The value. N is equal to byte0 - 1
    printf("Received advertisment\r\n");
    for (uint8_t i = 0; i < params->advertisingDataLen; ++i) {

        const uint8_t record_length = params->advertisingData[i];
        if (record_length == 0) {
            continue;
        }
        const uint8_t type = params->advertisingData[i + 1];
        const uint8_t* value = params->advertisingData + i + 2;
        const uint8_t value_length = record_length - 1;
        //std::string ss;
        //ss.assign(value, value+value_length);

        //if(type == GapAdvertisingData::COMPLETE_LOCAL_NAME) {
            //if ((value_length == sizeof(PEER_NAME)) && (memcmp(value, PEER_NAME, value_length) == 0)) {
                printf(
                    "adv %s peerAddr[%02x %02x %02x %02x %02x %02x] rssi %d, isScanResponse %u, AdvertisementType %u\r\n", value_length, value, //ss.c_str(),
                    params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2],
                    params->peerAddr[1], params->peerAddr[0], params->rssi, params->isScanResponse, params->type
                );
                distanceServicePtr->updatedistance(params->rssi, *(params->peerAddr), 0);
                // TODO: exit somehow; this breaks the chance to reconnect 
                //BLE::Instance().gap().connect(params->peerAddr, Gap::ADDR_TYPE_RANDOM_STATIC, NULL, NULL);
                //break;
            //}
        //}
        i += record_length;
    }
}

void onConnectionCallback(const Gap::ConnectionCallbackParams_t *params) {
    printf("Connected to something\r\n");
    BLE::Instance().gap().startScan(advertisementCallback);
}

/**
 * This function is called when the ble initialization process has failled
 */
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

/**
 * Callback triggered when the ble initialization process has finished
 */
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
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);
    ble.gap().onConnection(onConnectionCallback);

    bool initialValueForLEDCharacteristic = false;
    distanceServicePtr = new DistanceService(ble);

    /* setup advertising */
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

int main()
{
    eventQueue.call_every(500, blinkCallback);

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
