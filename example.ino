#include "IMU6050.h"
#include <Wire.h>

/************************************************************************
 *  Variable Declaration
 */
  
  int Yaw, Pitch, Roll = 0;

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
MPU6050 mpu6050(Wire);


void setup() {

  Serial.begin(9600);

  /**********************************************************************
   * Start the i2C communication with
   *    Master  --- Arduino
   *    Slave   --- MPU
   */
  Wire.begin();

  /**********************************************************************
   * Start MPU Library and FIFO
   */
  mpu6050.begin();

/************************************************************************
 * use calcGyroOffsets() in setup(), it will calculate calibration of the gyroscope, and the value of the gyroscope will calibrated.
 * 
 *    NOTE  : DO NOT MOVE THE MPU DURING CALIBRATION
 * 
 *  This can be operated in two modes.
 *    calcGyroOffsets()       ---->   Calibrate the MPU
 *    calcGyroOffsets(true)   ---->   Calibrate the MPU and display the values in the Serial monitor
 *  
 *  Programme will the wait for 3 sec.
 *  Once the values are known use 
 *        setGyroOffsets() to continue.
 *        
 *        Syntax  ::    mpu6050.setGyroOffsets(x, y, z);
 *            X - value obtained from " X : ____ "
 *            Y - value obtained from " Y : ____ "
 *            Z - value obtained from " Z : ____ "
 */
  mpu6050.calcGyroOffsets(true);

}

void loop() {

  /**********************************************************************
   * Get the data of MPU execute .update()
   */
  mpu6050.update();

  Roll = mpu6050.getAngleX();
  Pitch = mpu6050.getAngleY();
  Yaw = mpu6050.getAngleZ();
 
  
  /*********************************************************************
   *  Serial Monitor/ Display 
   */
  Serial.print("\t Roll : ");
  Serial.print(Roll);
  Serial.print("\t Pitch : ");
  Serial.print(Pitch);
  Serial.print("\t Yaw : ");
  Serial.println(Yaw);
  
  
}
