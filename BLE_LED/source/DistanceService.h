/* ARM University Program Microcontroller Library
 * 
 * Distance Service
 *
 * This software is distributed under an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef __BLE_DISTANCE_SERVICE_H__
#define __BLE_DISTANCE_SERVICE_H__

#include "ble/BLE.h"

/* Distance Service (Custom)    */
/* Characteristics:     */
/*  - Distance (Custom) */


class DistanceService {
public:
    const static uint16_t DISTANCE_SERVICE_UUID        = 0x2000;
    const static uint16_t DISTANCE_CHARACTERISTIC_UUID = 0x3000;
    const static uint16_t ID_CHARACTERISTIC_UUID       = 0x3001;
    DistanceService(BLEDevice &_ble) :
        ble(_ble),
        distance(0),
        id(0),
        timestamp(0),
        distanceCharacteristic(DISTANCE_CHARACTERISTIC_UUID, &distance, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY), 
        idCharacteristic(ID_CHARACTERISTIC_UUID, &id, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
        timestampCharacteristic(GattCharacteristic::UUID_DATE_TIME_CHAR, &timestamp, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY) {

        static bool serviceAdded = false;
        if (serviceAdded) {
            return;
        }

        GattCharacteristic *charTable[] = {&distanceCharacteristic, &idCharacteristic, &timestampCharacteristic};
        GattService         DistanceService(0x2000, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        ble.addService(DistanceService);
        serviceAdded = true;
    }

    /**
     * @brief Update the battery dist with a new value. Valid values range from
     * 0..100. Anything outside this range will be ignored.
     *
     * @param newdist
     *              update to battery dist.
     */
    void updatedistance(uint8_t newdist, uint8_t newid, uint8_t newtimest) {
        distance = newdist;
        id = newid;
        timestamp = newtimest;
        ble.updateCharacteristicValue(distanceCharacteristic.getValueAttribute().getHandle(), &distance, 1);
        ble.updateCharacteristicValue(idCharacteristic.getValueAttribute().getHandle(), &id, 1);
        ble.updateCharacteristicValue(timestampCharacteristic.getValueAttribute().getHandle(), (uint8_t *) &timestamp, 1);
    }

private:
    BLEDevice &ble;

    uint8_t    distance;
    uint8_t    id;
    uint64_t   timestamp;
    ReadOnlyGattCharacteristic<uint8_t> distanceCharacteristic;
    ReadOnlyGattCharacteristic<uint8_t> idCharacteristic;
    ReadOnlyGattCharacteristic<uint64_t> timestampCharacteristic;
};

#endif /* #ifndef __BLE_DISTANCE_SERVICE_H__*/


// *******************************ARM University Program Copyright \357\277\275 ARM Ltd 2015*************************************//
