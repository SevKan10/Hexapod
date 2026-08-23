#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1(0x40);
Adafruit_PWMServoDriver pwm2(0x41);

#define SDA_PIN 17
#define SCL_PIN 16

#define SERVOMIN 102
#define SERVOMAX 512

void setServo(uint8_t id, int angle)
{
  angle = constrain(angle, 0, 180);

  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  if (id < 9)
  {
    pwm1.setPWM(id, 0, pulse);
  }
  else
  {
    pwm2.setPWM(id - 9, 0, pulse);
  }

  Serial.println("--------------------------------");
  Serial.print("Servo ID : ");
  Serial.println(id);

  Serial.print("Board    : ");
  Serial.println(id < 9 ? "0x40" : "0x41");

  Serial.print("Channel  : ");
  Serial.println(id < 9 ? id : id - 9);

  Serial.print("Angle    : ");
  Serial.println(angle);

  Serial.println("--------------------------------");
}

void setup()
{
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pwm1.begin();
  pwm2.begin();

  pwm1.setPWMFreq(50);
  pwm2.setPWMFreq(50);

  delay(500);

  Serial.println();
  Serial.println("======================================");
  Serial.println("      HEXAPOD SERVO CONTROLLER");
  Serial.println("======================================");
  Serial.println("Nhap:");
  Serial.println("id goc");
  Serial.println();
  Serial.println("Vi du:");
  Serial.println("0 90");
  Serial.println("5 95");
  Serial.println("17 110");
  Serial.println();
}

void loop()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0)
      return;

    int spaceIndex = cmd.indexOf(' ');

    if (spaceIndex == -1)
    {
      Serial.println("Sai cu phap!");
      Serial.println("Vi du: 5 90");
      return;
    }

    int id = cmd.substring(0, spaceIndex).toInt();
    int angle = cmd.substring(spaceIndex + 1).toInt();

    Serial.println();
    Serial.print("Lenh: ");
    Serial.println(cmd);

    if (id < 0 || id > 17)
    {
      Serial.println("Servo ID phai tu 0 den 17");
      return;
    }

    if (angle < 0 || angle > 180)
    {
      Serial.println("Goc phai tu 0 den 180");
      return;
    }

    setServo(id, angle);
  }
}