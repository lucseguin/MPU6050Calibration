#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;
int64_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void calibrate_mpu6050();

#define OFFSETS_FILE "offsets.txt"

File offset_file;


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);//TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);

  Serial.print("Initializing SD card...");
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  Serial.println("Initializing MPU6050");
  mpu.initialize();
  if (mpu.testConnection() ) {
    Serial.println("MPU6050 connected");
  } else {
    Serial.println("MPU6050 ERROR");
  }
  calibrate_mpu6050();

  if (SD.exists(OFFSETS_FILE)) {
    SD.remove(OFFSETS_FILE);
  }

  offset_file = SD.open(OFFSETS_FILE, FILE_WRITE);
  if (offset_file) {
    offset_file.print("MPU6050-Offsets:");
    offset_file.print((int)ax_offset);
    offset_file.print(",");
    offset_file.print((int)ay_offset);
    offset_file.print(",");
    offset_file.print((int)az_offset);
    offset_file.print(",");
    offset_file.print((int)gx_offset);
    offset_file.print(",");
    offset_file.print((int)gy_offset);
    offset_file.print(",");
    offset_file.println((int)gz_offset);

    offset_file.close();

    Serial.print("MPU6050-Offsets:");
    Serial.print((int)ax_offset);
    Serial.print(",");
    Serial.print((int)ay_offset);
    Serial.print(",");
    Serial.print((int)az_offset);
    Serial.print(",");
    Serial.print((int)gx_offset);
    Serial.print(",");
    Serial.print((int)gy_offset);
    Serial.print(",");
    Serial.println((int)gz_offset);

    Serial.println("Offsets saved to file.");
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("OFFSETS_FILE did not open for writing!");
    while (1) {}
  }
}

void loop() {

}


///////////////////////////////////   MPU6050 Calibration   ////////////////////////////////////
int64_t buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 6;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int64_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
void calibrate_mpu6050() {
  Serial.println("Calibrating MPU6050");
  //ApplicationMonitor.IAmAlive();

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  meansensors();
  delay(1000);
  calibration();
  delay(1000);
  meansensors();

  Serial.println("MPU6050 Calibrated!");

  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
}
int64_t i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
void meansensors() {
  i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  Serial.println("Calculating Mean Sensors");
  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    //ApplicationMonitor.IAmAlive();
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//Serial.print("ax:");Serial.print(ax);Serial.print("ay:");Serial.print(ay);Serial.print("az:");Serial.print(az);
//Serial.print("gx:");Serial.print(gx);Serial.print("gy:");Serial.print(gy);Serial.print("gz:");Serial.println(gz);   
 
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + (int64_t)ax;
      buff_ay = buff_ay + (int64_t)ay;
      buff_az = buff_az + (int64_t)az;
      buff_gx = buff_gx + (int64_t)gx;
      buff_gy = buff_gy + (int64_t)gy;
      buff_gz = buff_gz + (int64_t)gz;
      if ((i - 100) % 100 == 0) {
        Serial.print(".");
      }
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
      Serial.println(";");
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  Serial.println("Calibrating");
  //ApplicationMonitor.IAmAlive();

  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax) <= acel_deadzone) {
      Serial.println("ax_offset ready");
      ready++;
    } else {
      Serial.print("mean_ax:");Serial.println((int)mean_ax);
      ax_offset = ax_offset - mean_ax / acel_deadzone;
    }

    if (abs(mean_ay) <= acel_deadzone) {
      Serial.println("ay_offset ready");
      ready++;
    }
    else {
      Serial.print("mean_ay:");Serial.println((int)mean_ay);
      ay_offset = ay_offset - mean_ay / acel_deadzone;
    }

    if (abs(16384 - mean_az) <= acel_deadzone) {
      Serial.println("az_offset ready");
      ready++;
    }
    else {
      Serial.print("mean_az:");Serial.println((int)mean_az);
      az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    }

    if (abs(mean_gx) <= giro_deadzone) {
      Serial.println("gx_offset ready");
      ready++;
    }
    else {
      Serial.print("mean_gx:");Serial.println((int)mean_gx);
      gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
    }

    if (abs(mean_gy) <= giro_deadzone) {
      Serial.println("gy_offset ready");
      ready++;
    }
    else {
      Serial.print("mean_gy:");Serial.println((int)mean_gy);
      gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
    }

    if (abs(mean_gz) <= giro_deadzone) {
      Serial.println("gz_offset ready");
      ready++;
    }
    else {
      Serial.print("mean_gz:");Serial.println((int)mean_gz);
      gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    }

    if (ready == 6) break;
  }
}

