#include <Kalman.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <M5StickC.h>


/**
 * Stand up detection define
 */
#define THRESHOLD_COUNT 150
#define THRESHOLD_DEGREE 50
#define CALIBRATION_SAMPLE_COUNT 500
#define OVER_THRESHOLD 1
#define NOT_OVER_THRESHOLD 0
#define UPDATE_CYCLE 20

/**
 * Firebase define
 */
#define FIREBASE_HOST ""    // Need to set your information
#define FIREBASE_AUTH ""    // Need to set your information
#define WIFI_SSID ""        // Need to set your information
#define WIFI_PASSWORD ""    // Need to set your information
#define STICK_NO 1

/**
 * Stand up detection global variable
 */
float acc_x;
float acc_y;
float acc_z;
float acc_offset_x;
float acc_offset_y;
float acc_offset_z;
float gyro_x;
float gyro_y;
float gyro_z;
float gyro_offset_x;
float gyro_offset_y;
float gyro_offset_z;
float kalAngleX;
float kalAngleY;
int dispColor = BLACK;
Kalman kalmanX;
Kalman kalmanY;
long lastMs = 0;
long tick = 0;
int standUpCount = 0;
int isOverThreshold = NOT_OVER_THRESHOLD;


/**
 * Firebase global variable
 */
FirebaseData firebaseData;
FirebaseJson json;
String const firebaseDetectionPath = "/Detection";

void setup() {
  setupForFirebase();
  setupForStandUpDetection();
}

void loop() {
  loopForStandUpDetection();
  
  delay(2);
}

/**
 * Firebase Library
 */
void setupForFirebase(){
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println("Firebase connection start");

  // Wifi Connection
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Firebase Connection
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set datase read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}
void setDataToDB(int data){
  if (Firebase.setInt(firebaseData, firebaseDetectionPath + "/UID" + STICK_NO, data)){
    Serial.println("PASSED");
    Serial.println();
  } else {
    M5.Lcd.setCursor(0,0);
    M5.Lcd.println("Push Data Error");
    Serial.println("Failed");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println();
  }
}



/**
 * Stand up detection Library
 */
void setupForStandUpDetection(){
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(dispColor);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("  X       Y       Z");
  M5.MPU6886.Init();

  M5.Lcd.setCursor(0,30);
  M5.Lcd.println("Calibrating...");
  delay(500);
  calibration();

  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("%7.2f %7.2f %7.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
  M5.Lcd.setCursor(0, 75);
  M5.Lcd.printf("%7.2f %7.2f %7.2f", acc_offset_x*1000, acc_offset_y*1000, acc_offset_z*1000);
  
  getData();
  kalmanX.setAngle(getRoll());
  kalmanY.setAngle(getPitch());
  lastMs = micros();
}
void loopForStandUpDetection(){
  getData();
  applyCalibration();
  float dt = (micros() - lastMs) / 1000000.0;
  lastMs = micros();
  float roll = getRoll();
  float pitch = getPitch();
  
  kalAngleX = kalmanX.getAngle(roll, gyro_x, dt);
  kalAngleY = kalmanY.getAngle(pitch, gyro_y, dt);

  // ThresholdCount
  int tempOverThreshold = checkThreshold();

  // UpdateDB
  if(tempOverThreshold != isOverThreshold){
    setDataToDB(tempOverThreshold);
  }
  isOverThreshold = tempOverThreshold;

  tick++;
  if(tick % UPDATE_CYCLE == 0){
    tick = 0;
    
    draw(tempOverThreshold);
  }
}
void calibration(){

  // Caliculate offset
  float gyroSum[3];
  float accSum[3];

  for(int i = 0; i < CALIBRATION_SAMPLE_COUNT; i++){
    getData();
    gyroSum[0] += gyro_x;
    gyroSum[1] += gyro_y;
    gyroSum[2] += gyro_z;
    accSum[0] += acc_x;
    accSum[1] += acc_y;
    accSum[2] += acc_z;
    delay(2);
  }
  gyro_offset_x = gyroSum[0]/CALIBRATION_SAMPLE_COUNT;
  gyro_offset_y = gyroSum[1]/CALIBRATION_SAMPLE_COUNT;
  gyro_offset_z = gyroSum[2]/CALIBRATION_SAMPLE_COUNT;
  acc_offset_x = accSum[0]/CALIBRATION_SAMPLE_COUNT;
  acc_offset_y = accSum[1]/CALIBRATION_SAMPLE_COUNT;
  acc_offset_z = accSum[2]/CALIBRATION_SAMPLE_COUNT - 1.0;    // gravity
}
void applyCalibration(){
  gyro_x -= gyro_offset_x;
  gyro_y -= gyro_offset_y;
  gyro_z -= gyro_offset_z;
  acc_x -= acc_offset_x;
  acc_y -= acc_offset_y;
  acc_z -= acc_offset_z;
}
void draw(int isOverThreshold){
  if(isOverThreshold){
    if(dispColor != ORANGE){
      dispColor = ORANGE;
      M5.Lcd.fillScreen(ORANGE);
    }
  } else {
    if(dispColor != BLACK){
      dispColor = BLACK;
      M5.Lcd.fillScreen(BLACK);
    }
  }
  M5.Lcd.setCursor(0, 15);
  M5.Lcd.printf("%7.2f %7.2f %7.2f", gyro_x, gyro_y, gyro_z);
  M5.Lcd.setCursor(140, 15);
  M5.Lcd.print("o/s");
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%7.2f %7.2f %7.2f", acc_x * 1000, acc_y * 1000, acc_z * 1000);
  M5.Lcd.setCursor(145, 30);
  M5.Lcd.print("mg");
  M5.Lcd.setCursor(0, 45);
  M5.Lcd.printf("%7.2f %7.2f", kalAngleX, kalAngleY);
  M5.Lcd.setCursor(140, 45);
  M5.Lcd.print("deg");
}
int checkThreshold(){
  int ret = NOT_OVER_THRESHOLD;
  if(kalAngleX > THRESHOLD_DEGREE){
    standUpCount++;
    if(standUpCount > THRESHOLD_COUNT){
      standUpCount = THRESHOLD_COUNT;
      ret = OVER_THRESHOLD;
    } else {
      ret = NOT_OVER_THRESHOLD;
    }
  } else {
    standUpCount--;
    if(standUpCount < 0){
      standUpCount = 0;
      ret = NOT_OVER_THRESHOLD;
    } else {
      ret = OVER_THRESHOLD;
    }
  }

  return ret;

}
void getData(){
  M5.MPU6886.getGyroData(&gyro_x, &gyro_y, &gyro_z);
  M5.MPU6886.getAccelData(&acc_x, &acc_y, &acc_z);
}
float getRoll(){
  return atan2(acc_y, acc_z) * RAD_TO_DEG;
}
float getPitch(){
  return atan(-acc_x / sqrt(acc_y*acc_y + acc_z*acc_z)) * RAD_TO_DEG;
}
