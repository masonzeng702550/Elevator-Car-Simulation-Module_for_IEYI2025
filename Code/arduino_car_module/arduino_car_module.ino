#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <stdlib.h>  // 用於隨機數生成
#include <Fonts/FreeSans12pt7b.h>  // 使用內建字體

// 腳位定義
// WS2812B 燈條
#define LED_STRIP_A_PIN 2
#define LED_STRIP_B_PIN 3

// 電梯按鈕
#define BUTTON_EMERGENCY 4
#define BUTTON_FLOOR_3 5
#define BUTTON_FLOOR_2 6
#define BUTTON_FLOOR_1 7

// TFT螢幕腳位
#define TFT_CS    10    // Chip Select
#define TFT_DC    9     // Data/Command
#define TFT_RST   8     // Reset
#define TFT_MOSI  11    // SDA
#define TFT_SCLK  13    // SCL
#define TFT_BLK   12    // Backlight

// 燈條設定
#define LED_STRIP_A_COUNT 8
#define LED_STRIP_B_COUNT 8

// TFT 顯示設定
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

// 建立顯示器物件
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel ledStripA(LED_STRIP_A_COUNT, LED_STRIP_A_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledStripB(LED_STRIP_B_COUNT, LED_STRIP_B_PIN, NEO_GRB + NEO_KHZ800);

// 按鈕狀態變數
bool button1Pressed = false;
bool button2Pressed = false;
bool button3Pressed = false;
bool emergencyPressed = false;
bool emergencyMode = false;

// 防彈跳變數
unsigned long lastButtonTime = 0;
const unsigned long buttonDebounceTime = 200;

// TFT螢幕控制變數
unsigned long lastDisplayTime = 0;
//const unsigned long displayInterval = 1000; 
const unsigned long displayInterval = 500; // 顯示切換間隔 (毫秒) - 優化為更快的更新頻率
bool displayState = false; // false: 英文, true: 英文（保持一致性）

// 電梯狀態變數（從Python接收）
int currentFloor = 1;
int targetFloor = 1;
String elevatorDirection = "IDLE";
String elevatorStatus = "NORMAL"; // NORMAL, EMERGENCY, FULL

// 顯示狀態追蹤（避免不必要的重繪）
String lastFloorText = "";
String lastStatusText = "";
String lastElevatorStatus = "";
String lastElevatorDirection = "";

void setup() {
  Serial.begin(9600);
  Serial.println("ELEVATOR_SYSTEM_READY");
  
  // 初始化隨機數種子
  randomSeed(analogRead(0));
  
  // 初始化按鈕腳位
  pinMode(BUTTON_EMERGENCY, INPUT_PULLUP);
  pinMode(BUTTON_FLOOR_3, INPUT_PULLUP);
  pinMode(BUTTON_FLOOR_2, INPUT_PULLUP);
  pinMode(BUTTON_FLOOR_1, INPUT_PULLUP);
  
  // 初始化燈條
  ledStripA.begin();
  ledStripB.begin();
  ledStripA.setBrightness(40);   // 設定更柔和亮度 (約15%)
  ledStripB.setBrightness(40);   // 設定更柔和亮度 (約15%)
  
  // 設定燈條為最亮的黃白光
  setLightsToBrightWhite();

  // 初始化顯示器
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(2);  // 轉180度 (0=0度, 1=90度, 2=180度, 3=270度)
  uint16_t backgroundColor = tft.color565(0, 0,25);  // RGB(0, 0, 25) 深藍色背景
  tft.fillScreen(backgroundColor);
  
  // 設定文字屬性
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  
  Serial.println("電梯系統初始化完成！");
  
  // 顯示初始狀態
  updateTFTDisplay();
  
  Serial.println("車廂運作模組初始化完成");
  Serial.println("ARDUINO_READY");
}

void loop() {
  // 處理按鈕輸入
  handleButtons();
  
  // 處理序列埠通訊
  handleSerialCommunication();
  
  // 處理TFT顯示更新
  handleTFTDisplay();
  
  delay(50); // 短暫延遲避免過度讀取
}

void setLightsToBrightWhite() {
  // 設定為溫暖柔和的黃光 (255, 180, 80)
  uint32_t warmYellow = ledStripA.Color(255, 180, 80);
  
  // 設定燈條 A
  for(int i = 0; i < LED_STRIP_A_COUNT; i++) {
    ledStripA.setPixelColor(i, warmYellow);
  }
  ledStripA.show();
  
  // 設定燈條 B
  for(int i = 0; i < LED_STRIP_B_COUNT; i++) {
    ledStripB.setPixelColor(i, warmYellow);
  }
  ledStripB.show();
}

void handleButtons() {
  unsigned long currentTime = millis();
  
  // 檢查是否超過防彈跳時間
  if (currentTime - lastButtonTime < buttonDebounceTime) {
    return;
  }
  
  // 讀取按鈕狀態
  int button1State = digitalRead(BUTTON_FLOOR_1);
  int button2State = digitalRead(BUTTON_FLOOR_2);
  int button3State = digitalRead(BUTTON_FLOOR_3);
  int emergencyState = digitalRead(BUTTON_EMERGENCY);
  
  // 檢查一樓按鈕
  if (button1State == LOW && !button1Pressed) {
    button1Pressed = true;
    lastButtonTime = currentTime;
    Serial.println("BUTTON:1");
    Serial.println("DEBUG: Button 1 pressed (LOW)");
  } else if (button1State == HIGH) {
    button1Pressed = false;
  }
  
  // 檢查二樓按鈕
  if (button2State == LOW && !button2Pressed) {
    button2Pressed = true;
    lastButtonTime = currentTime;
    Serial.println("BUTTON:2");
    Serial.println("DEBUG: Button 2 pressed (LOW)");
  } else if (button2State == HIGH) {
    button2Pressed = false;
  }
  
  // 檢查三樓按鈕
  if (button3State == LOW && !button3Pressed) {
    button3Pressed = true;
    lastButtonTime = currentTime;
    Serial.println("BUTTON:3");
    Serial.println("DEBUG: Button 3 pressed (LOW)");
  } else if (button3State == HIGH) {
    button3Pressed = false;
  }
  
  // 檢查緊急按鈕
  if (emergencyState == LOW && !emergencyPressed) {
    emergencyPressed = true;
    lastButtonTime = currentTime;
    emergencyMode = !emergencyMode; // 切換緊急模式
    
    if (emergencyMode) {
      Serial.println("BUTTON:EMERGENCY_ON");
      Serial.println("PLAY_SOUND:em.mp3");
      Serial.println("EMERGENCY MODE ACTIVATED");
    } else {
      Serial.println("BUTTON:EMERGENCY_OFF");
      Serial.println("EMERGENCY MODE DEACTIVATED");
    }
  } else if (emergencyState == HIGH) {
    emergencyPressed = false;
  }
  
  // 添加緊急按鈕狀態除錯
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 2000) { // 每2秒輸出一次狀態
    lastDebugTime = currentTime;
    Serial.print("Emergency Button State: ");
    Serial.print(emergencyState);
    Serial.print(", Emergency Mode: ");
    Serial.println(emergencyMode);
  }
}

void handleSerialCommunication() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // 解析來自 Python 的指令
    if (command.startsWith("STATUS:")) {
      // 處理狀態更新指令
      Serial.print("RECEIVED: ");
      Serial.println(command);
      
      // 解析狀態指令
      if (command.indexOf("STATUS:FULL") != -1) {
        // Python 要求進入自動緊急模式
        elevatorStatus = "FULL";
        Serial.println("AUTO EMERGENCY MODE ACTIVATED BY PYTHON");
      } else if (command.indexOf("STATUS:EMERGENCY") != -1) {
        // Python 要求進入手動緊急模式
        elevatorStatus = "EMERGENCY";
        Serial.println("MANUAL EMERGENCY MODE ACTIVATED BY PYTHON");
      } else if (command.indexOf("STATUS:NORMAL") != -1) {
        // Python 要求解除緊急模式
        elevatorStatus = "NORMAL";
        Serial.println("NORMAL MODE ACTIVATED BY PYTHON");
      }
    }
    
    // 解析樓層和方向資訊
    if (command.indexOf("FLOOR:") != -1) {
      int floorStart = command.indexOf("FLOOR:") + 6;
      int floorEnd = command.indexOf(";", floorStart);
      if (floorEnd != -1) {
        String floorStr = command.substring(floorStart, floorEnd);
        currentFloor = floorStr.toInt();
      }
    }
    
    if (command.indexOf("DIR:") != -1) {
      int dirStart = command.indexOf("DIR:") + 4;
      int dirEnd = command.indexOf(";", dirStart);
      if (dirEnd != -1) {
        elevatorDirection = command.substring(dirStart, dirEnd);
      }
    }
    
    // 解析目標樓層
    if (command.indexOf("TARGET:") != -1) {
      int targetStart = command.indexOf("TARGET:") + 7;
      int targetEnd = command.indexOf(";", targetStart);
      if (targetEnd != -1) {
        String targetStr = command.substring(targetStart, targetEnd);
        targetFloor = targetStr.toInt();
      }
    }
  }
}

void handleTFTDisplay() {
  unsigned long currentTime = millis();
  
  // 每秒更新一次顯示
  if (currentTime - lastDisplayTime >= displayInterval) {
    lastDisplayTime = currentTime;
    displayState = !displayState; // 切換顯示模式
    updateTFTDisplay();
  }
}

void updateTFTDisplay() {
  // 檢查是否需要更新顯示
  String currentFloorText = getFloorDisplayText();
  String currentStatusText = getStatusDisplayText();
  
  // 只有在內容改變時才重繪螢幕
  if (currentFloorText != lastFloorText || 
      currentStatusText != lastStatusText ||
      elevatorStatus != lastElevatorStatus ||
      elevatorDirection != lastElevatorDirection) {
    
    // 清空螢幕並設定背景色為深藍色 RGB(0, 0, 25)
    uint16_t backgroundColor = tft.color565(0, 0, 25);  // RGB(0, 0, 25) 深藍色
    tft.fillScreen(backgroundColor);
    
    // 上方2/3區域顯示樓層資訊
    int upperHeight = (240 * 2) / 3;  // 160
    
    // 繪製上方區域（深紫色背景）
    tft.fillRect(0, 0, 240, upperHeight, backgroundColor);
    
    // 顯示樓層資訊
    displayFloorInfo();
    
    // 下方1/3區域顯示運轉狀態
    int lowerHeight = 240 - upperHeight;  // 80
    int lowerY = upperHeight;
    
    // 繪製下方區域
    displayStatusInfo(lowerY, lowerHeight);
    
    // 更新追蹤變數
    lastFloorText = currentFloorText;
    lastStatusText = currentStatusText;
    lastElevatorStatus = elevatorStatus;
    lastElevatorDirection = elevatorDirection;
  }
}

String getFloorDisplayText() {
  if (elevatorDirection == "IDLE") {
    return String(currentFloor);
  } else if (elevatorDirection == "UP") {
    return "^" + String(targetFloor);  // 修正為目標樓層
  } else if (elevatorDirection == "DOWN") {
    return "v" + String(targetFloor);  // 修正為目標樓層
  }
  return String(currentFloor);
}

String getStatusDisplayText() {
  if (elevatorStatus == "EMERGENCY") {
    return "EMERGENCY";
  } else if (elevatorStatus == "FULL") {
    return "EXPRESS";
  } else {
    return "NORMAL";
  }
}

void displayFloorInfo() {
  tft.setTextSize(10);  // 進一步放大樓層字體
  tft.setTextColor(ST77XX_WHITE);
  
  String floorText = getFloorDisplayText();
  
  // 居中顯示文字 - 對齊上面2/3區域的中間點
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(floorText, 0, 0, &x1, &y1, &w, &h);
  int textX = (240 - w) / 2;
  int upperHeight = (240 * 2) / 3;  // 160
  int textY = 65;
  
  tft.setCursor(textX, textY);
  tft.print(floorText);
}

void displayStatusInfo(int y, int height) {
  // 根據狀態決定顏色和文字
  uint16_t boxColor, textColor;
  String statusText = getStatusDisplayText();
  
  if (elevatorStatus == "EMERGENCY") {
    // 手動緊急：紅色
    boxColor = ST77XX_RED;
    textColor = ST77XX_RED;
  } else if (elevatorStatus == "FULL") {
    // 自動緊急：綠色
    boxColor = ST77XX_GREEN;
    textColor = ST77XX_GREEN;
  } else {
    // 正常：白色
    boxColor = ST77XX_WHITE;
    textColor = ST77XX_WHITE;
  }
  
  // 繪製邊框
  tft.drawRect(10, y + 10, 220, height - 20, boxColor);
  
  // 設定文字屬性 - 使用較大的字體
  tft.setTextSize(4);
  tft.setTextColor(textColor);
  
  // 居中顯示文字 - 對齊下面1/3區域的中間點
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(statusText, 0, 0, &x1, &y1, &w, &h);
  int textX = (240 - w) / 2;
  int textY = 185;
  
  tft.setCursor(textX, textY);
  tft.print(statusText);
}