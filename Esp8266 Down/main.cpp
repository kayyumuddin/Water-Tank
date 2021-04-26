#include <Arduino.h>
#include <ESP8266WiFi.h>        // for ESP8266
#include <BlynkSimpleEsp8266.h> // for ESP8266

#include <ESP8266mDNS.h> // For OTA w/ ESP8266
#include <WiFiUdp.h>     // For OTA
#include <ArduinoOTA.h>  // For OTA

char auth[] = ""; // Auth
char ssid[] = "";
char pass[] = "";

BlynkTimer timer;

//Sinric//
#include "SinricPro.h"
#include "SinricProSwitch.h"

#define APP_KEY ""
#define APP_SECRET ""
#define SWITCH_ID ""

#define RELAY_PIN D5

//Ultrasonic//
#include <Ultrasonic.h>
Ultrasonic ultrasonic(D7, D6);

//Filter's//
#include <RunningMedian.h>
RunningMedian SonarInput = RunningMedian(5);

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter SonarFilter(3.0, 3.0, 0.1);

//Variable's//
class SonarData
{
public:
  float raw, litre, change, prev, filtered;

  void Get();
  void Filter();
  void Process();
  void Rate();
} sonar;

bool PowerState = false;

#define DivideRatio 1.98
#define MultiplyRatio 56.7
#define MaxHeight 215

void SonarData::Get()
{
  unsigned int ping_time = ultrasonic.timing();
  if (isnan(ping_time))
  {
    return;
  }
  sonar.raw = ping_time;
}

void SonarData::Filter()
{
  float sonar_dist = sonar.raw / 57.0;
  SonarInput.add(sonar_dist);
}

void SonarData::Process()
{
  sonar.filtered = SonarInput.getMedian();
  sonar.filtered = (MaxHeight - sonar.filtered) / DivideRatio;
  sonar.filtered = SonarFilter.updateEstimate(sonar.filtered);
  sonar.filtered = constrain(sonar.filtered, 0, 100);
  sonar.litre = sonar.filtered * MultiplyRatio;
}

void SonarData::Rate()
{
  sonar.change = sonar.litre - sonar.prev;
  sonar.prev = sonar.litre;
}

void SonarTimer()
{
  sonar.Get();
  sonar.Filter();
  sonar.Process();
}

void SendData()
{
  long RoundIndex = round(sonar.litre);
  Blynk.virtualWrite(V2, RoundIndex);

  String FormatIndex = String(sonar.change, 1);
  Blynk.virtualWrite(V3, FormatIndex);
}

void TimerStart()
{
  delay(20);
  timer.setInterval(100L, SonarTimer);
  delay(20);
  timer.setInterval(60000L, []() {
    sonar.Rate();
  });
  delay(20);
  timer.setTimeout(12000L, []() {
    timer.setInterval(1000L, SendData);
  });
}

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

void SwitchSetup()
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

bool onPowerState(const String &deviceId, bool &state)
{
  PowerState = state;
  Blynk.virtualWrite(V10, PowerState);
  digitalWrite(RELAY_PIN, PowerState ? HIGH : LOW);
  return true;
}

BLYNK_WRITE(V10)
{
  int pinValue = param.asInt();
  if(pinValue){
    PowerState = true;
  } else {
    PowerState = false;
  }

  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID];
  mySwitch.sendPowerStateEvent(PowerState);

  digitalWrite(RELAY_PIN, PowerState ? HIGH : LOW);
}

void setup()
{
  SwitchSetup();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID]; // create new switch device
  mySwitch.onPowerState(onPowerState);              // apply onPowerState callback
  SinricPro.begin(APP_KEY, APP_SECRET);             // start SinricPro

  Blynk.config(auth, "blynk-cloud.com", 80);
  Blynk.connect();

  ArduinoOTA.setHostname("Esp8266 Down"); // For OTA
  ArduinoOTA.begin();                     // For OTA

  TimerStart();
}

void loop()
{
  timer.run();
  Blynk.run();
  SinricPro.handle();
  ArduinoOTA.handle();
}