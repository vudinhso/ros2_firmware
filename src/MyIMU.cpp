#include "MyIMU.h"

extern double quat[4];
extern double gyr[3];
extern double acc[3];
extern double quat_calib[4];
extern double gyr_calib[3];
extern double acc_calib[3];

ICM_20948_I2C myICM;

void IMUBegin()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(2000);
    delay(100);

    do
    {
        myICM.begin(Wire, 0);
        if (myICM.status != ICM_20948_Stat_Ok)
            delay(10);
    } while (myICM.status != ICM_20948_Stat_Ok);

    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    IMUCalibrate();
}

void IMUCalibrate()
{
    for (size_t i = 0; i < 100; i++)
    {
        IMUGetData_Uncalibrated();
        quat_calib[0] += quat[0];
        quat_calib[1] += quat[1];
        quat_calib[2] += quat[2];
        quat_calib[3] += quat[3];
        gyr_calib[0] += gyr[0];
        gyr_calib[1] += gyr[1];
        gyr_calib[2] += gyr[2];
        acc_calib[0] += acc[0];
        acc_calib[1] += acc[1];
        acc_calib[2] += acc[2];
    }
    quat_calib[0] /= 100.0;
    quat_calib[1] /= 100.0;
    quat_calib[2] /= 100.0;
    quat_calib[3] /= 100.0;
    gyr_calib[0] /= 100.0;
    gyr_calib[1] /= 100.0;
    gyr_calib[2] /= 100.0;
    acc_calib[0] /= 100.0;
    acc_calib[1] /= 100.0;
    acc_calib[2] /= 100.0;
    acc_calib[2] -= 9.80665f;

    Serial.println("IMU Calibrated");
    Serial.print("quat_calib: ");
    Serial.print(quat_calib[0], 6);
    Serial.print(", ");
    Serial.print(quat_calib[1], 6);
    Serial.print(", ");
    Serial.print(quat_calib[2], 6);
    Serial.print(", ");
    Serial.println(quat_calib[3], 6);
    Serial.print("gyr_calib: ");
    Serial.print(gyr_calib[0], 6);
    Serial.print(", ");
    Serial.print(gyr_calib[1], 6);
    Serial.print(", ");
    Serial.println(gyr_calib[2], 6);
    Serial.print("acc_calib: ");
    Serial.print(acc_calib[0], 6);
    Serial.print(", ");
    Serial.print(acc_calib[1], 6);
    Serial.print(", ");
    Serial.println(acc_calib[2], 6);
}

void IMUGetData_Uncalibrated()
{
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        // Quaternion
        if ((data.header & DMP_header_bitmap_Quat6) > 0)
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            double n = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

            quat[0] = q1 / n;
            quat[1] = q2 / n;
            quat[2] = q3 / n;
            quat[3] = q0 / n;
        }
        // Gyroscope
        if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
        {
            float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
            float y = (float)data.Raw_Gyro.Data.Y;
            float z = (float)data.Raw_Gyro.Data.Z;

            constexpr float Deg2Rad = 3.1416f / 180.0f;
            constexpr float LSB_PER_DPS = 16.4f; // dps2000 is currently used: FS = ±500 dps
            gyr[0] = x / LSB_PER_DPS * Deg2Rad;  // Store the gyro values in the global array
            gyr[1] = y / LSB_PER_DPS * Deg2Rad;
            gyr[2] = z / LSB_PER_DPS * Deg2Rad;
        }
        // Accelerometer
        if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
        {
            constexpr float G = 9.80665f;
            constexpr float LSB_PER_G = 8192.0f; // gpm4 is currently used: FS = ±4 g

            float acc_x = (float)data.Raw_Accel.Data.X;
            float acc_y = (float)data.Raw_Accel.Data.Y;
            float acc_z = (float)data.Raw_Accel.Data.Z;
            acc[0] = acc_x / LSB_PER_G * G; // Store the accel values in the global array
            acc[1] = acc_y / LSB_PER_G * G;
            acc[2] = acc_z / LSB_PER_G * G;
        }
    }
}

void IMUGetData()
{
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        // Quaternion
        if ((data.header & DMP_header_bitmap_Quat6) > 0)
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            double n = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

            quat[0] = q1 / n;
            quat[1] = q2 / n;
            quat[2] = q3 / n;
            quat[3] = q0 / n;
        }
        // Gyroscope
        if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
        {
            float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
            float y = (float)data.Raw_Gyro.Data.Y;
            float z = (float)data.Raw_Gyro.Data.Z;

            constexpr float Deg2Rad = 3.1416f / 180.0f;
            constexpr float LSB_PER_DPS = 16.4f;               // dps2000 is currently used: FS = ±500 dps
            gyr[0] = x / LSB_PER_DPS * Deg2Rad - gyr_calib[0]; // Store the gyro values in the global array
            gyr[1] = y / LSB_PER_DPS * Deg2Rad - gyr_calib[1];
            gyr[2] = z / LSB_PER_DPS * Deg2Rad - gyr_calib[2];
        }
        // Accelerometer
        if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
        {
            constexpr float G = 9.80665f;
            constexpr float LSB_PER_G = 8192.0f; // gpm4 is currently used: FS = ±4 g

            float acc_x = (float)data.Raw_Accel.Data.X;
            float acc_y = (float)data.Raw_Accel.Data.Y;
            float acc_z = (float)data.Raw_Accel.Data.Z;
            acc[0] = acc_x / LSB_PER_G * G - acc_calib[0] ; // Store the accel values in the global array
            acc[1] = acc_y / LSB_PER_G * G - acc_calib[1] ;
            acc[2] = acc_z / LSB_PER_G * G - acc_calib[2] ;
        }
    }
}