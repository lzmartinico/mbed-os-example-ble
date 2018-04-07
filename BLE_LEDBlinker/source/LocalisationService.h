#ifndef __BLE_LOCALISATION_SERVICE_H__
#define __BLE_LOCALISATION_SERVICE_H__

#include "ble/BLE.h"
#include "LSM9DS1.h"


class LocalisationService {

public:
    const static uint16_t LOCALISATION_SERVICE_UUID       = 0x2000;
    const static uint16_t BEACON_RSSI_CHARACTERISTIC_UUID = 0x3000;
    const static uint16_t IMU_CHARACTERISTIC_UUID         = 0x4000;

    LocalisationService(BLE &_ble) :
        ble(_ble),
        beaconRssiValue(0, 0),  // TODO set defaults
        imuValues(),
        beaconRssiCharacteristic(
            BEACON_RSSI_CHARACTERISTIC_UUID,
            &beaconRssiValue,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
        ),
        IMUCharacteristic(
            IMU_CHARACTERISTIC_UUID,
            &imuValues,
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
    
    void updateIMU(LSM9DS1 &imu) {
        imuValues.ax_raw = imu.ax_raw;
        imuValues.ay_raw = imu.ay_raw;
        imuValues.az_raw = imu.az_raw;
        imuValues.gx_raw = imu.gx_raw;
        imuValues.gy_raw = imu.gy_raw;
        imuValues.gz_raw = imu.gz_raw;
        imuValues.mx_raw = imu.mx_raw;
        imuValues.my_raw = imu.my_raw;
        imuValues.mz_raw = imu.mz_raw;
        ble.gattServer().write(
            IMUCharacteristic.getValueHandle(), 
            reinterpret_cast<uint8_t *>(&imuValues), 
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
    IMUValue imuValues; 
    ReadOnlyGattCharacteristic<BeaconRssiValue> beaconRssiCharacteristic;
    ReadOnlyGattCharacteristic<IMUValue> IMUCharacteristic;
};

#endif /* #ifndef __BLE_LOCALISATION_SERVICE_H__*/
