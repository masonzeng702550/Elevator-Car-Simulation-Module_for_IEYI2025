# TFT螢幕中文字體支援說明

## 當前狀態
目前的Arduino程式使用英文顯示，因為標準的Adafruit_GFX庫不支援中文字體。

## 顯示內容
- **樓層顯示**：上方2/3區域顯示目前樓層或目標樓層
  - 靜止時：顯示數字（1、2、3）
  - 上升時：顯示 ↑2、↑3
  - 下降時：顯示 ↓2、↓1

- **狀態顯示**：下方1/3區域顯示運轉狀態
  - 手動緊急：紅色框 + "EMERGENCY" / "Emergency"
  - 自動緊急：綠色框 + "EXPRESS" / "Express"  
  - 正常運轉：白色框 + "NORMAL" / "Normal"

## 如需顯示中文
如果您希望顯示中文（如"緊急運転"、"直通運転"、"通常運転"），需要安裝中文字體庫：

### 選項1：使用Adafruit_GFX_Chinese庫
1. 在Arduino IDE中安裝 "Adafruit GFX Chinese Font Library"
2. 修改程式碼：
   ```cpp
   #include <Adafruit_GFX_Chinese.h>
   ```
3. 使用中文字符串：
   ```cpp
   statusText = displayState ? "Emergency" : "緊急運転";
   statusText = displayState ? "Express" : "直通運転";
   statusText = displayState ? "Normal" : "通常運転";
   ```

### 選項2：使用點陣字體
1. 下載中文字體點陣檔案
2. 將字體檔案放入Arduino專案的fonts資料夾
3. 在程式碼中載入字體

### 選項3：使用圖形化字體
1. 將中文字符轉換為點陣圖
2. 使用drawBitmap()函數顯示

## 建議
目前使用英文顯示已能清楚表達電梯狀態，如果不需要中文顯示，建議保持現狀。

如需中文顯示，推薦使用選項1（Adafruit_GFX_Chinese庫），安裝簡單且相容性好。 