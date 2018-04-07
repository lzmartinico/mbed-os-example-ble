#ifndef __BLE_LOCALISATION_SERVICE_H__
#define __BLE_LOCALISATION_SERVICE_H__

#include "ble/BLE.h"


class LocalisationService {

public:
    const static uint16_t LOCALISATION_SERVICE_UUID       = 0x2000;
    const static uint16_t BEACON_RSSI_CHARACTERISTIC_UUID = 0x3000;
    const static uint16_t IMU_CHARACTERISTIC_UUID         = 0x4000;

    LocalisationService(BLE &_ble) :
        ble(_ble),
        beaconRssiValue(0, 0),  // TODO set defaults
        IMU_vals(),
        beaconRssiCharacteristic(
            BEACON_RSSI_CHARACTERISTIC_UUID,
            &beaconRssiValue,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
        ),
        IMUCharacteristic(
            IMU_CHARACTERISTIC_UUID,
            &IMU_vals,
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
    
    void updateIMU(void *data) {
        memcpy(&IMU_vals, data, sizeof(IMU_vals));
        ble.gattServer().write(
            IMUCharacteristic.getValueHandle(), 
            reinterpret_cast<uint8_t *>(&IMU_vals), 
            sizeof(IMUValue)
        );
    }

    void setupService(void) {
        GattCharacteristic *charTable[] = {
            &beaconRssiCharacteristic,
            &IMUCharacteristic
        };
        GattService localisationService(
            LOCALISATION_SERVICE_UUID,
            charTable,
            sizeof(charTable) / sizeof(GattCharacteristic*)
        );

        ble.gattServer().addService(localisationService);
    }

    struct IMUValue{
        int16_t gx_raw, gy_raw, gz_raw; // x, y, and z axis readings of the gyroscope
        int16_t ax_raw, ay_raw, az_raw; // x, y, and z axis readings of the accelerometer
        int16_t mx_raw, my_raw, mz_raw; // x, y, and z axis readings of the magnetometer
    };

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
    IMUValue IMU_vals; 
    ReadOnlyGattCharacteristic<BeaconRssiValue> beaconRssiCharacteristic;
    ReadOnlyGattCharacteristic<IMUValue> IMUCharacteristic;
};

#endif /* #ifndef __BLE_LOCALISATION_SERVICE_H__*/
