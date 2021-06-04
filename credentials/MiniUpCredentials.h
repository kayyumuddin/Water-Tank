#define AUTH "Blynk_Token"
#define SSID "WiFi_Ssid"
#define PASS "WiFi_Pass"

#define APP_KEY "SinricPro_App_key"
#define APP_SECRET "SinricPro_App_Secret"
#define SWITCH_ID "SinricPro_Switch_Id"

IPAddress SERVER_ADDRESS (188, 166, 206, 43);
#define HOSTNAME "Esp8266 Up"

#define RELAY_PIN D1

#define DIVIDE_RATIO 1.21    // (Depth of Tank that store water in cm) / 100
//My tank -> 121 cm

#define MULTIPLY_RATIO 20.0  // (Total tank volume in litre) / 100
//My tank -> 2000 litre

#define MAX_HEIGHT 132       // (Depth of Tank that store water in cm) + (Mounting height of sensor)
//My tank -> 121cm + 11cm = 132 cm
