#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Ultrasonic.h>
#include <RunningMedian.h>
#include <SimpleKalmanFilter.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "MiniUpCredentials.h"

OneWire oneWire(D7);
DallasTemperature tempSensor(&oneWire);
DeviceAddress tempSensorAddr = {0x28, 0x3C, 0x58, 0x77, 0x91, 0x0B, 0x02, 0x17};

Ultrasonic pingSensor(D5, D6);

RunningMedian tempMedianFilter = RunningMedian(3);
RunningMedian pingMedianFilter = RunningMedian(5);

SimpleKalmanFilter kalmanFilter(3.0, 3.0, 0.1);

BlynkTimer timer;

float litre, flow_rate, temp;

double soundSpeed(double c)
{
  double s = 331.3 + 0.606 * c;
  return s / 20000.0;
}

void pingTimer()
{
  float rv = pingSensor.readTiming() * soundSpeed(temp);

  pingMedianFilter.add(rv);

  rv = (MAX_HEIGHT - pingMedianFilter.getMedian()) / DIVIDE_RATIO;
  litre = constrain(kalmanFilter.updateEstimate(rv), 0, 100) * MULTIPLY_RATIO;
}

void flowTimer()
{
  static float pl;
  flow_rate = litre - pl;
  pl = litre;
}

void tempTimer()
{
  tempSensor.requestTemperatures();
  float rv = tempSensor.getTempC(tempSensorAddr);

  if (!isnan(rv) && rv > -10 && rv < 70)
  {
    tempMedianFilter.add(rv);
  }

  temp = tempMedianFilter.getMedian();
  Blynk.virtualWrite(V4, temp);
}

void sendLitre()
{
  int v = round(litre);
  Blynk.virtualWrite(V2, v);
}

void sendRate()
{
  String v = String(flow_rate, 1);
  Blynk.virtualWrite(V3, v);
}

void timerSetup()
{
  timer.setInterval(10000L, tempTimer);
  delay(40);
  timer.setInterval(500L, pingTimer);
  delay(40);
  timer.setInterval(60000L, flowTimer);
  delay(40);
  timer.setInterval(30000L, sendRate);
  delay(40);
  timer.setTimeout(30000L, []()
                   { timer.setInterval(1000L, sendLitre); });
}

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

void switchSetup()
{
  digitalWrite(RELAY_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);
}

bool onPowerState(const String &deviceId, bool &state)
{
  bool relay_state = state;
  Blynk.virtualWrite(V10, relay_state);
  digitalWrite(RELAY_PIN, relay_state ? LOW : HIGH);
  return true;
}

BLYNK_WRITE(V10)
{
  bool relay_state = param.asInt();

  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID];
  mySwitch.sendPowerStateEvent(relay_state);

  digitalWrite(RELAY_PIN, relay_state ? LOW : HIGH);
}

void setup()
{
  switchSetup();
  tempSensor.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();

  Blynk.config(AUTH, SERVER_ADDRESS, 80);
  Blynk.connect();

  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID];
  mySwitch.onPowerState(onPowerState);
  SinricPro.begin(APP_KEY, APP_SECRET);

  timerSetup();
}

void loop()
{
  ArduinoOTA.handle();
  Blynk.run();
  timer.run();
  SinricPro.handle();
}