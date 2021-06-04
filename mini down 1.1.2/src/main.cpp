#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Ultrasonic.h>
#include <RunningMedian.h>
#include <SimpleKalmanFilter.h>

#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "MiniDownCredentials.h"

Ultrasonic pingSensor(D7, D6);

RunningMedian medianFilter = RunningMedian(5);
SimpleKalmanFilter kalmanFilter(3.0, 3.0, 0.1);

BlynkTimer timer;

float litre, flow_rate;

void pingTimer()
{
  float rv = pingSensor.readTiming() / 57.0;

  medianFilter.add(rv);

  rv = (MAX_HEIGHT - medianFilter.getMedian()) / DIVIDE_RATIO;
  litre = constrain(kalmanFilter.updateEstimate(rv), 0, 100) * MULTIPLY_RATIO;
}

void flowTimer()
{
  static float pl;
  flow_rate = litre - pl;
  pl = litre;
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
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

bool onPowerState(const String &deviceId, bool &state)
{
  bool relay_state = state;
  Blynk.virtualWrite(V10, relay_state);
  digitalWrite(RELAY_PIN, relay_state ? HIGH : LOW);
  return true;
}

BLYNK_WRITE(V10)
{
  bool relay_state = param.asInt();

  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID];
  mySwitch.sendPowerStateEvent(relay_state);

  digitalWrite(RELAY_PIN, relay_state ? HIGH : LOW);
}

void setup()
{
  switchSetup();

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