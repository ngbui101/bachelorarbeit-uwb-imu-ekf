#include "imu.h"
#include <sys/time.h>
#include "mqtt.h"
BNO08x imu;

unsigned long previous_millis_imu = 0;
unsigned long current_millis_imu = 0;
int millis_since_last_ts;

uint64_t ts_acc = 0;
uint64_t ts_ori = 0;

uint8_t report_id = 0x00;
float ax = 0.0;
float ay = 0.0;
float az = 0.0;
float qx = 0.0;
float qy = 0.0;
float qz = 0.0;
float qw = 0.0;

bool hasAcc, hasOri = false;
uint64_t getNanosecondTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000000L + (uint64_t)tv.tv_usec * 1000L;
}
void send_imu_data(uint64_t timestamp)
{

    const int IMU_PAYLOAD_SIZE = 255;
    char imu_payload[IMU_PAYLOAD_SIZE];
    snprintf(imu_payload, IMU_PAYLOAD_SIZE, "%llu;%f,%f,%f;%f,%f,%f,%f", timestamp, ax, ay, az, qw, qx, qy, qz);
    while (!mqtt_publish("imu/data", imu_payload))
    {
    };
}
void enable_reading()
{
    imu.enableARVRStabilizedGameRotationVector(12);
    imu.enableLinearAccelerometer(12);
    delay(100);
}

void start_imu()
{
    Wire.flush();
    Wire.begin(IMU_SDA, IMU_SCL, 400000);
    delay(1000);
    if (imu.begin(IMU_ADDR, Wire) == false)
    {
        Serial.println("IMU not found");
        while (1)
            ;
    }
    Serial.println("BNO086 started");
    enable_reading();
}

void imu_handler()
{
    if (imu.wasReset())
    {
        enable_reading();
    }
    while (!hasAcc | !hasOri)
    {
        if (imu.getSensorEvent() == true)
        {
            report_id = imu.getSensorEventID();
            if (report_id == SENSOR_REPORTID_LINEAR_ACCELERATION)
            {
                ax = imu.getLinAccelX();
                ay = imu.getLinAccelY();
                az = imu.getLinAccelZ();
                hasAcc = true;
            }
            if (report_id ==
                SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
            {
                qx = imu.getGameQuatI();
                qy = imu.getGameQuatJ();
                qz = imu.getGameQuatK();
                qw = imu.getGameQuatReal();
                hasOri = true;
            }
        }
        delayMicroseconds(1);
    }
    hasAcc = false;
    hasOri = false;
    uint64_t ts = getNanosecondTimestamp();
    send_imu_data(ts);
}