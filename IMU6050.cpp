/***********************************************************************
 *  ====================================================================
 *    --------------------------------
 *      IMU 6050
 *    --------------------------------
 *    
 *    Author    : Jogesh S Nanda
 *    Date      : 02.01.2019
 *    
 *    Revision  : 07.01.2019
 *    
 *    Description   :   " Contains the Functions for IMU.6050 "
 *    
 *  @2019 Copyright reserved to Toboids Automata Pvt. (Ltd)
 *  ====================================================================
 */

 #include "IMU6050.h"
 #include "Arduino.h"

/************************************************************************
 * Setting the cofficient for Coefficient filters.
 * 
 *    Filtering Formula used.
 *      filtered_angle = (0.02 * accel) + (0.98 * gyro)
 * 
 *    Syntax  ::  MPU6050 mpu6050(wire, a, g);
 *      a = accelorometer cofficient
 *      g = gyroscope cofficient
 *  
 *  eg.
 *    To set acceloromet cofficient to 0.1 and gyro cofficient to 0.9
 *    MPU mpu6050(wire, 0.1, 0.9);
 *  
 *  default value is 0.02 and 0.98
 */
MPU6050::MPU6050(TwoWire &w){
    wire = &w;
    accCoef = 0.02f;
    gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
    wire = &w;
    accCoef = aC;
    gyroCoef = gC;
}

/************************************************************************
 *  MPU 6050 Register/Settings 
 *  
 *    Calls this function in Void Setup().
 */
void MPU6050::begin(){

    /**************************************************************************
     *    Sample Rate Divider
     *  ----------------------
     *  
     *  Register : 19          SMPLTR_DIV [7:0]       Type :  Read/Write
     *  
     *  Descrption :
     *    Sample Rate = (Gyro Output Rate)/(1 + SMPLRT_DIV)
     *    
     *    Gyro Output rate ---> see register 26.
     */
    writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);

    /**************************************************************************
     *    Configuration
     *  ----------------------
     *  
     *  Register : 26          DLPF_CFG [2:0]       Type :  Read/Write
     *  
     *  Descrption :
     *    Sets the Low-Pass Filter Bandwidth
     *    
     *    DLFP_CFG  |    BandWidth  |   Acc (fs)  |  Gyro (fs)
     *    ----------------------------------------------------
     *        0             260 Hz          1 kHz       8 kHz
     *        1             184             1           1
     *        2              94             1           1
     *        3              44             1           1
     *        4              21             1           1
     *        5              10             1           1
     *        6               5             1           1
     *        7        [------------  Reserved  ------------]
     */
    writeMPU6050(MPU6050_CONFIG, 0x00);

    /**************************************************************************
     *    Gyro Configuration
     *  ----------------------
     *  
     *  Register : 1B          FS_SEL [4:3]       Type :  Read/Write
     *  
     *  Descrption :
     *    Sets the Gyro Full Scale Range.
     *    
     *            FS_SEL    |     Range
     *          -----------------------------
     *              0         +/- 250 degree/s
     *              1         +/- 500 degree/s
     *              2         +/- 1000 degree/s
     *              3         +/- 2000 degree/s
     */
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);    //  + 500 degree/s

    /**************************************************************************
     *    Accel Configuration
     *  -------------------------
     *  
     *  Register : 1C          AFS_SEL [4:3]       Type :  Read/Write
     *  
     *  Descrption :
     *    Sets the Accel. Full Scale Range.
     *    
     *           AFS_SEL    |     Range
     *          -----------------------------
     *              0            +/- 2 g
     *              1            +/- 4 g
     *              2            +/- 8 g
     *              3            +/- 16 g
     */
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x10);     //  +/- 8 g

    /**************************************************************************
     *    Clock Selection and Power Management 
     *  ----------------------------------------
     *  
     *  Register : 6B          CLK_SEL [2:0]       Type :  Read/Write
     *  
     *  Descrption :
     *    Sets the clock Speed and PLL Referrence.
     *    
     *           CLK_SEL    |     Clock Selection
     *          -------------------------------------
     *              0           internal 8 Mhz Oscillator
     *              1           PLL with X Gyro Refference
     *              2           PLL with Y Gyro Refference
     *              3           PLL with Z Gyro Refference
     *              4           PLL with external 32.78 kHz Refference
     *              5           PLL with external 19.2 MHz Refference
     *              6           Reserved
     *              7           Stops and puts to sleep.
     *    
     *    SLEEP     [bit 6]    -   set '1' to put MPU 6050 to Sleep.
     *    CYCLE     [bit 5]    -   set '1' and Sleep is disabled.
     *    TEMP_DIS  [bit 3]    -   set '1' to disable Temp. Sensor
     *    DEV_RESET [bit 7]    -   set to '1' to reset the internal Register.
     */
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x02);   // PLL with Y axis Gyro
    this->update();
    
    angleGyroX = 0;
    angleGyroY = 0;
    
    angleX = this->getAccAngleX();
    angleY = this->getAccAngleY();
    preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data){
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);
    wire->write(data);
    wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);
    wire->endTransmission(true);
    wire->requestFrom((uint8_t)MPU6050_ADDR, (size_t)2/*length*/);
    byte data =  wire->read();
    wire->read();
    return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
    gyroXoffset = x;
    gyroYoffset = y;
    gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console){
    float x = 0, y = 0, z = 0;
    int16_t rx, ry, rz;

    delay(1000);
    if(console){
        Serial.println();
        Serial.println("========================================");
        Serial.println("calculate gyro offsets");
        Serial.print("DO NOT MOVE A MPU6050");
    }
    
    for(int i = 0; i < 3000; i++){
      if(console && i % 1000 == 0){
        Serial.print(".");
      }
    
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x3B);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);

    wire->read() << 8 | wire->read();
    wire->read() << 8 | wire->read();
    wire->read() << 8 | wire->read();
    wire->read() << 8 | wire->read();
    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
    }
    
    gyroXoffset = x / 3000;
    gyroYoffset = y / 3000;
    gyroZoffset = z / 3000;

    if(console){
      Serial.println();
      Serial.println("Done!!!");
      Serial.print("X : ");Serial.println(gyroXoffset);
      Serial.print("Y : ");Serial.println(gyroYoffset);
      Serial.print("Z : ");Serial.println(gyroZoffset);
      Serial.println("Program will start after 3 seconds");
      Serial.print("========================================");
      delay(3000);
    }
}

void MPU6050::update(){
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x3B);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);

    rawAccX = wire->read() << 8 | wire->read();
    rawAccY = wire->read() << 8 | wire->read();
    rawAccZ = wire->read() << 8 | wire->read();
    rawTemp = wire->read() << 8 | wire->read();
    rawGyroX = wire->read() << 8 | wire->read();
    rawGyroY = wire->read() << 8 | wire->read();
    rawGyroZ = wire->read() << 8 | wire->read();

    temp = (rawTemp + 12412.0) / 340.0;

    accX = ((float)rawAccX) / 16384.0;
    accY = ((float)rawAccY) / 16384.0;
    accZ = ((float)rawAccZ) / 16384.0;

    angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
    angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

    gyroX = ((float)rawGyroX) / 65.5;
    gyroY = ((float)rawGyroY) / 65.5;
    gyroZ = ((float)rawGyroZ) / 65.5;

    gyroX -= gyroXoffset;
    gyroY -= gyroYoffset;
    gyroZ -= gyroZoffset;

    interval = (millis() - preInterval) * 0.001;

    angleGyroX += gyroX * interval;
    angleGyroY += gyroY * interval;
    angleGyroZ += gyroZ * interval;

    angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
    angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
    angleZ = angleGyroZ;

    preInterval = millis();

}
