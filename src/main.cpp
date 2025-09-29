#include <Arduino.h>
#include <flip.h>
#include <TFT_eSPI.h>
// #include <SPI.h>
#include <esp_heap_caps.h>
// #include "lis2ds12_reg.h"

uint32_t startTime, frame = 0; // For frames-per-second estimate
uint32_t lastStepTime;

// Pin definitions

#define PIN_LIS_SCK    14
#define PIN_LIS_MOSI   13
#define PIN_LIS_MISO   12
#define PIN_LIS_CS 25
// #define PIN_TFT_CS 5
/*
#define PIN_LIS_SCK    18
#define PIN_LIS_MOSI   23
#define PIN_LIS_MISO   19
#define PIN_LIS_CS 15
#define PIN_TFT_CS 5
*/
#define READ_BIT     0b10000000
#define WRITE_BIT    0b00000000
#define WHO_AM_I_REG 0x0F
#define WHO_AM_I     0x43
#define OUT_X_L      0x28
#define CTRL2        0x21

// Configuration
#define SCREENWIDTH 240
#define SCREENHEIGHT 320
#define GRID_SIZE 10
// const uint8_t RADIUS = GRID_DIMENSION / 4;
const float GRAVITY = 500.0f; // 7,382 pixels per meter, 72,416 pixels per 9.81 m

struct vec2{
  float x;
  float y;
};

SPIClass mySPI = SPIClass(HSPI);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);
FlipFluid flip;

float accelerationG[3] = {0, 0, 0};
SPISettings lisSettings(25000000, MSBFIRST, SPI_MODE0);

void drawFluid();
void printMemoryInfo();
int platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void simulateTask(void *pvParameters);
void otherTask(void *pvParameters);
void updateAccel(float *accelerationG);
int32_t densityToBlueHex(float density, float maxDensity);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("ESP32 Booted");
  pinMode(PIN_LIS_CS, OUTPUT);
  mySPI.begin(PIN_LIS_SCK, PIN_LIS_MISO, PIN_LIS_MOSI, PIN_LIS_CS);
  // --- Read WHO_AM_I ---
  digitalWrite(PIN_LIS_CS, LOW);
  mySPI.beginTransaction(lisSettings);
  mySPI.transfer(WHO_AM_I_REG | READ_BIT);
  uint8_t whoami = mySPI.transfer(0x00);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  if (whoami != WHO_AM_I) {
    Serial.println("lis2ds12 not found!");
    while(1) {}
  } else {
    Serial.println("lis2ds12 found!");
  }
  mySPI.endTransaction();
  digitalWrite(PIN_LIS_CS, HIGH);
  digitalWrite(PIN_LIS_CS, LOW);
  mySPI.beginTransaction(lisSettings); // Write to CTRL1
  mySPI.transfer(0x20);           // CTRL1 address, write
  mySPI.transfer(0b01110010);     // Data
  mySPI.endTransaction();
  digitalWrite(PIN_LIS_CS, HIGH);

  tft.init();
  tft.fillScreen(TFT_ORANGE);
  // img.createSprite(SCREENWIDTH - 1, SCREENHEIGHT - 1);
  // img.createSprite(239, 300);
  img.setColorDepth(8);
  // if (img.createSprite(239, 319) == nullptr) { Serial.println("Sprite alloc failed"); }
  if (img.createSprite(240, 320) == nullptr) { Serial.println("Sprite alloc failed"); }
  // uint16_t screenWidth, uint16_t screenHeight, uint16_t cellSize, float pRadius, float flip, float driftConst, int projectionIterations, float overrelaxation
  flip.init(SCREENWIDTH, SCREENHEIGHT, GRID_SIZE, 3.0f, 0.65f, 1.0f, 50, 1.6f);
  flip.initializeBoundaries();
  flip.initializeParticles();
  // flip.initializeCellTypeCircle(120.0f, 120.0f, 120.0f);
  // flip.initializeParticlesInRadius(115.0f, 120.0f, 120.0f, 1);
  // flip.initializeCellTypeCircle(115.0f, 120.0f, 120.0f);
  startTime = millis();
  lastStepTime = startTime;

  
  // Create simulation task (core 0)
  xTaskCreatePinnedToCore(
    simulateTask,      // Task function
    "SimTask",         // Name
    8192,              // Stack size
    NULL,              // Parameters
    2,                 // Priority
    NULL,              // Task handle
    0                  // Core
  );
  
  // Create draw task (core 1)
  xTaskCreatePinnedToCore(
    otherTask,
    "otherTask",
    8192,
    NULL,
    1,
    NULL,
    1
  );
  Serial.println("end of setup()");
}

void loop() {
  /*
  // obtain deltaT
  uint32_t currentTime = millis();
  float deltaT = (currentTime - lastStepTime) / 1000.0f;  // Convert to seconds
  lastStepTime = currentTime;  // Update for next iteration
  Serial.print("FPS: ");
  Serial.println(1/deltaT, 4);
  */
  // float deltaT = 0.5f;
  // delay(100);
  // deltaT *= 10.0f; // speed up

  // flip.simulate(deltaT, GRAVITY);
  // drawFluid();
}

void simulateTask(void *pvParameters) {
  uint32_t lastSimTime = millis();
  uint32_t lastPrintTime = millis();
  uint32_t simCount = 0;
  while (1) {
    uint32_t now = millis();
    float localDeltaT = (now - lastSimTime) / 1000.0f;
    lastSimTime = now;
    float accel[] = {accelerationG[0] * GRAVITY, accelerationG[1] * GRAVITY, accelerationG[2] * GRAVITY};
    flip.simulate(localDeltaT, accel);
    simCount++;
    /*
    if (now - lastPrintTime >= 1000) {
      // UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.print("SimTask FPS: ");
      Serial.println(simCount);
      // Serial.print(" | Stack high water mark: ");
      // Serial.println(highWaterMark);
      simCount = 0;
      lastPrintTime = now;
    }
    */
    vTaskDelay(1);
  }
}

void otherTask(void *pvParameters) {
  const TickType_t frameDelay = 33 / portTICK_PERIOD_MS; // ~30 FPS
  TickType_t lastWakeTime = xTaskGetTickCount();
  int frameCount = 0;
  while (1) {
    drawFluid();
    updateAccel(accelerationG);
    /*
    if (frameCount % 20 == 0) {
      Serial.print("X: "); Serial.print(accelerationG[0]);
      Serial.print(" ");
      Serial.print("Y: "); Serial.print(accelerationG[1]);
      Serial.print(" ");
      Serial.print("Z: "); Serial.println(accelerationG[2]);
    }
    */
    frameCount++;
    vTaskDelayUntil(&lastWakeTime, frameDelay);
  }
}

void drawFluid() {
  img.fillScreen(TFT_BLACK);
  /*
  for (int c = 0; c < SCREEN_SIZE / GRID_SIZE; c++) {
    for (int r = 0; r < SCREEN_SIZE / GRID_SIZE; r++) {
      // fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
      img.fillRect(c * GRID_SIZE, r * GRID_SIZE, c * (GRID_SIZE + 1), r * (GRID_SIZE + 1), densityToBlueHex(flip.getParticleDensity(c, r), 8.0f));
    }
  }*/
  for (int i = 0; i < flip.getNumParticles(); i++) {
    img.fillCircle(int(flip.getParticlePosX()[i]), int(flip.getParticlePosY()[i]), int(flip.getParticleRadius()), TFT_BLUE);
  }
  img.pushSprite(0, 0);
}

void printMemoryInfo() {
    // Get total free memory
    uint32_t freeHeap = esp_get_free_heap_size();
    
    // Get minimum free memory (lowest point since boot)
    uint32_t minFreeHeap = esp_get_minimum_free_heap_size();
    
    // Get largest free block
    uint32_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    
    Serial.printf("Free Heap: %u bytes\n", freeHeap);
    Serial.printf("Minimum Free Heap: %u bytes\n", minFreeHeap);
    Serial.printf("Largest Free Block: %u bytes\n", largestFreeBlock);
    
    // For more detailed breakdown (optional)
    Serial.printf("DRAM Free: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.printf("PSRAM Free: %u bytes (if available)\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

int32_t densityToBlueHex(float density, float maxDensity) {
    // Clamp density to [0, maxDensity]
    if (density < 0) density = 0;
    if (density > maxDensity) density = maxDensity;

    // Normalize to [0, 1]
    float norm = density / maxDensity;

    // Blue channel: 0 to 255
    uint8_t blue = (uint8_t)(norm * 255.0f + 0.5f); // Round to nearest

    // Red and Green channels: always 0
    uint8_t red = 0;
    uint8_t green = 0;

    // Combine into 0xRRGGBB
    int32_t color = (red << 16) | (green << 8) | blue;

    return color;
}

void updateAccel(float* accelerationG) {
  // --- Read 6 bytes from OUT_X_L (auto-increment) ---
  uint8_t rawData[6];
  digitalWrite(PIN_LIS_CS, LOW);
  mySPI.beginTransaction(lisSettings);
  mySPI.transfer(0x28 | 0b10000000); // 0x80 = read, 0x40 = auto-increment
  for (int i = 0; i < 6; i++) {
    rawData[i] = mySPI.transfer(0x00);
  }
  mySPI.endTransaction();
  digitalWrite(PIN_LIS_CS, HIGH);
  accelerationG[0] = (float)(((int16_t)(rawData[1] << 8) | rawData[0]) >> 2) * 0.000244f; // X
  accelerationG[1] = (float)(((int16_t)(rawData[3] << 8) | rawData[2]) >> 2) * 0.000244f; // X
  accelerationG[2] = (float)(((int16_t)(rawData[5] << 8) | rawData[4]) >> 2) * 0.000244f; // X
}