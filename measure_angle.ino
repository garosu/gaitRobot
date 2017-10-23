#include <Wire.h>
#include "MsTimer2.h"

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;  //가속도 평균값 저장 변수
float baseGyX, baseGyY, baseGyZ;  //자이로 평균값 저장 변수

unsigned long t_now;  //현재 측정 주기 시간
unsigned long t_prev; //이전 측정 주기 시간

int16_t IntegralGyroZ = 0;
// IntegralGyroZ = GyZ * dt + IntegralGyroZ;

void setup() {
  initMPU6050(); //MPU-6050 센서에 대한 초기 설정 함수
  Serial.begin(115200); //Serial 통신 시작
  calibAccelGyro(); //센서 보정
  initDT(); //시간 간격에 대한 초기화 -> 현재 시각 저장
//  MsTimer2::set(1000, dataMeasure);
//  MsTimer2::start();  
}

void dataMeasure()
{
}

void loop() {
  readAccelGyro(); //가속도, 자이로 센서 값 읽어드림
  calcDT();
  calcAccelYPR();
  calcGyroYPR();

  //static int cnt;
  //cnt++;
  //if(cnt%2 == 0)
    SendDataToProcessing(); //위에 동일한 함수는 주석처리!
  //측정 주기 시간이 짝수(2ms 단위로 하기 위해서)이면 프로세싱으로 보낸다.

  delay(2000);
}

void initMPU6050(){
  Wire.begin(); //I2C 통신 시작 아림
  Wire.beginTransmission(MPU_addr); //0x68번지 값을 가지는 MPU-6050과 I2C 통신
  Wire.write(0x6B);
  Wire.write(0); //잠자는 MPU-6050을 깨우고 있다.
  Wire.endTransmission(true); //I2C 버스 제어권에서 손 놓음
}

void readAccelGyro(){
  Wire.beginTransmission(MPU_addr); //0x68번지 값을 가지는 MPU-6050과 I2C 통신 시작
  Wire.write(0x3B); //0x3B번지에 저장
  Wire.endTransmission(false); //데이터 전송 후 재시작 메새지 전송(연결은 계속 지속)
  Wire.requestFrom(MPU_addr, 14, true); //0x68 번지에 0x3B 부터 48까지 총 14바이트 저장
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}


void SendDataToProcessing(){
  //Serial.print(F("DEL:"));
  //Serial.print(dt,DEC);
  Serial.print(F("ACCEL:   "));
  Serial.print(accel_angle_x, 2);
  Serial.print(F(",   "));
  Serial.print(accel_angle_y, 2);
  Serial.print(F(",   "));
  Serial.println(accel_angle_z, 2);
  
  Serial.print(F("GYRO_:   "));
  Serial.print(gyro_angle_x, 2);
  Serial.print(F(",   "));
  Serial.print(gyro_angle_y, 2);
  Serial.print(F(",   "));
  Serial.println(gyro_angle_z, 2);
  Serial.println("");
/*
  Serial.print(F("#FIL:"));
  Serial.print(filtered_angle_x, 2);
  Serial.print(F(","));
  Serial.print(filtered_angle_y, 2);
  Serial.print(F(","));
  Serial.print(filtered_angle_z, 2);
  Serial.println(F(""));
*/
  delay(5);
}

void calibAccelGyro(){
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  readAccelGyro(); //가속도 자이로 센서 읽어들임

  //평균값 구하기
  for(int i=0; i<10; i++){
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(10);
  }
  baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}


void initDT(){
  t_prev = millis();
}

void calcDT(){
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0; //millis()로 얻은 값은 밀리초 단위이니까!!!!
  t_prev = t_now;
}

float gyro_x, gyro_y, gyro_z;

void calcGyroYPR()
{
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131; // 각속도를 저장하는 변수

  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;

  //gyro_angle_x += gyro_x * dt;
  //gyro_angle_y += gyro_y * dt;
  //gyro_angle_z += gyro_z * dt;
  gyro_angle_x = gyro_x;
  gyro_angle_y = gyro_y;
  gyro_angle_z = gyro_z;
}

void calcAccelYPR(){
  float accel_x, accel_y, accel_z; //가속도 센서의 최종적인 보정값!!!
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;

  accel_x = AcX - baseAcX; // 가속도(직선) X축에 대한 현재 값 - 가속도 센서의 평균값
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);

  //직석 +X축이 기울어진 각도 구함
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;

  accel_angle_z = 0;
}

