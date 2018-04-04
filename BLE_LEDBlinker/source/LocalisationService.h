#ifndef __BLE_LOCALISATION_SERVICE_H__
#define __BLE_LOCALISATION_SERVICE_H__

#include "ble/BLE.h"


class LocalisationService {

public:
    const static uint16_t LOCALISATION_SERVICE_UUID       = 0x2000;
    const static uint16_t BEACON_RSSI_CHARACTERISTIC_UUID = 0x3000;

    LocalisationService(BLE &_ble) :
        ble(_ble),
        beaconRssiValue(0, 0),  // TODO set defaults
        beaconRssiCharacteristic(
            BEACON_RSSI_CHARACTERISTIC_UUID,
            &beaconRssiValue,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
        )
    {
        setupService();
    }

    void updateBeaconRssi(int8_t rssi, uint8_t beacon_id) {
        beaconRssiValue.rssi = rssi;
        beaconRssiValue.beacon_id = beacon_id;
        ble.gattServer().write(
            beaconRssiCharacteristic.getValueHandle(), 
            reinterpret_cast<uint8_t *>(&beaconRssiValue), 
            sizeof(BeaconRssiValue)
        );
    }

    void setupService(void) {
        GattCharacteristic *charTable[] = {
            &beaconRssiCharacteristic
        };
        GattService localisationService(
            0x2000,
            charTable,
            sizeof(charTable) / sizeof(GattCharacteristic*)
        );

        ble.gattServer().addService(localisationService);
    }

    // __attribute__((packed))
    struct BeaconRssiValue {
        BeaconRssiValue(int8_t _rssi, uint8_t _beacon_id) :
            rssi(_rssi),
            beacon_id(_beacon_id)
        {}

        int8_t rssi;
        uint8_t beacon_id;
    };

    BLE &ble;
    BeaconRssiValue beaconRssiValue;
    ReadOnlyGattCharacteristic<BeaconRssiValue> beaconRssiCharacteristic;
};

#endif /* #ifndef __BLE_LOCALISATION_SERVICE_H__*/
