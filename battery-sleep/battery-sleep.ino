#define batPin 34
#define LED 2

// Cấu hình thời gian ngủ (tính bằng micro giây)
#define TIME_TO_SLEEP  10        // Ngủ trong 10 giây
#define uS_TO_S_FACTOR 1000000ULL // Hệ số chuyển từ giây sang micro giây

void setup() {
  Serial.begin(115200);
  
  // Cấu hình GPIO
  pinMode(batPin, INPUT);
  pinMode(LED, OUTPUT);

  // 1. LẤY MẪU ĐO PIN (Nên lấy trung bình 10 lần cho ổn định)
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(batPin);
    delay(2);
  }
  int rawADC = sum / 10;

  // 2. TÍNH % PIN THỰC TẾ 
  int batPercent = map(rawADC, 2640, 4095, 0, 100);
  batPercent = constrain(batPercent, 0, 100);

  Serial.printf("Battery Percent: %d%%\n", batPercent);

  // 3. BẬT/TẮT LED
  if (batPercent <= 20) {
    digitalWrite(LED, HIGH);
    // Nếu pin yếu, có thể nhấp nháy LED cảnh báo nhanh trước khi ngủ
    delay(500); 
  } else {
    digitalWrite(LED, LOW);
  }

  // 4. CẤU HÌNH HẸN GIỜ THỨC DẬY VÀ VÀO DEEP SLEEP
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Going to sleep now...");
  Serial.flush(); // Đảm bảo in hết Serial trước khi ngủ
  
  esp_deep_sleep_start();
}

void loop() {
  // Để trống vì khi dậy từ Deep Sleep, ESP32 sẽ chạy lại từ setup()
}