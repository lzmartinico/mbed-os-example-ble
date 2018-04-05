// LSM9DS91 Demo
 
#include "mbed.h"
#include "LSM9DS1.h"
 
// refresh time. set to 500 for part 2 and 50 for part 4
#define REFRESH_TIME_MS 1000
 
// Verify that the pin assignments below match your breadboard
LSM9DS1 imu(p7, p30);
 
// Serial pc(USBTX, USBRX);
 
//Init Serial port and LSM9DS1 chip
void setup()
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

    setup();  //Setup sensor and Serial
    printf("------ LSM9DS1 Demo -----------\r\n");
 
    while (true)
    {
        
        imu.readAccel();
    
        printf("A: %2f, %2f, %2f\r\n", imu.ax, imu.ay, imu.az);
 
        imu.readGyro();
        
        printf("G: %2f, %2f, %2f\r\n", imu.gx, imu.gy, imu.gz);
 
        imu.readMag();
        
        printf("M: %2f, %2f, %2f\r\n\r\n", imu.mx, imu.my, imu.mz);
       
        wait_ms(REFRESH_TIME_MS);
    }
}