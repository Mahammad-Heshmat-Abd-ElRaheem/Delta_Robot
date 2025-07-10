#include "esp_camera.h"
#include "Arduino.h"

// Select camera model (must match your hardware)
#define CAMERA_MODEL_AI_THINKER
#define PIN_OUT1 12
#define PIN_OUT2 13

// Pin definitions for ESP32-CAM AI-Thinker
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#endif

// 9 rolling history arrays, each with 11 elements
byte detectedHistory[9][11] = {0};
byte currentIndex[9] = {0};

// Reverse bits utility
uint16_t reverse_bits(uint16_t n) {
  uint16_t result = 0;
  for (int i = 0; i < 16; i++) {
    result <<= 1;
    result |= (n & 1);
    n >>= 1;
  }
  return result;
}

// Capture 9 pixels around center
void get_nine_pixels(uint16_t* pPix) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to capture image");
    return;
  }

  uint16_t *pixels = (uint16_t *)fb->buf;
  int y = fb->height / 2;
  int x = fb->width / 2;

  // 9 pixels: center + surrounding square
  int dx[9] = {0, -1, +1,  0, 0, -1, +1, -1, +1};
  int dy[9] = {0,  0,  0, -1, +1, -1, +1, +1, -1};

  for (int i = 0; i < 9; i++) {
    int px = x + dx[i];
    int py = y + dy[i];
    pPix[i] = reverse_bits(pixels[py * fb->width + px]);
  }

  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM 9 Pixel Detection with Rolling Memory");

  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  uint16_t pixels[9];
  get_nine_pixels(pixels);

  byte pixelDominantColor[9];

  for (int i = 0; i < 9; i++) {
    uint16_t colorCode = pixels[i];

    byte RedCode = (colorCode >> 11) & 0b00011111;
    byte GreenCode = (colorCode >> 5) & 0b00111111;
    byte BlueCode = colorCode & 0b00011111;

    RedCode *= 2;
    BlueCode *= 2;

    byte colorValue;
    if (GreenCode >= RedCode && GreenCode >= BlueCode) {
      colorValue = 1;
    } else if (RedCode >= GreenCode && RedCode >= BlueCode) {
      colorValue = 3;
    } else {
      colorValue = 2;
    }

    detectedHistory[i][currentIndex[i]] = colorValue;
    currentIndex[i]++;
    if (currentIndex[i] >= 11) currentIndex[i] = 0;

    byte greenCount = 0, blueCount = 0, redCount = 0;
    for (int j = 0; j < 11; j++) {
      if (detectedHistory[i][j] == 1) greenCount++;
      else if (detectedHistory[i][j] == 2) blueCount++;
      else if (detectedHistory[i][j] == 3) redCount++;
    }

    byte dominantColor;
    if (greenCount >= blueCount && greenCount >= redCount) {
      dominantColor = 1;
    } else if (blueCount >= greenCount && blueCount >= redCount) {
      dominantColor = 2;
    } else {
      dominantColor = 3;
    }

    pixelDominantColor[i] = dominantColor;
  }

  byte totalGreen = 0, totalBlue = 0, totalRed = 0;
  for (int i = 0; i < 9; i++) {
    if (pixelDominantColor[i] == 1) totalGreen++;
    else if (pixelDominantColor[i] == 2) totalBlue++;
    else if (pixelDominantColor[i] == 3) totalRed++;
  }

  String finalColor = "Unknown";
  if (totalGreen >= totalBlue && totalGreen >= totalRed) {
    finalColor = "Green";
  } else if (totalBlue >= totalGreen && totalBlue >= totalRed) {
    finalColor = "Blue";
  } else if (totalRed >= totalGreen && totalRed >= totalBlue) {
    finalColor = "Red";
  }

  Serial.print("Final Detected Color (from 9 pixels): ");
  Serial.println(finalColor);

  // Set outputs based on detected color
  if (finalColor == "Green") {
    digitalWrite(PIN_OUT1, HIGH);
    digitalWrite(PIN_OUT2, HIGH);
  }
  else if (finalColor == "Blue") {
    digitalWrite(PIN_OUT1, LOW);
    digitalWrite(PIN_OUT2, LOW);
  }
  else if (finalColor == "Red") {
    digitalWrite(PIN_OUT1, HIGH);
    digitalWrite(PIN_OUT2, LOW);
  }
  else {
    digitalWrite(PIN_OUT1, LOW);
    digitalWrite(PIN_OUT2, LOW);
  }

  delay(200);
}

