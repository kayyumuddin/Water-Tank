#define AUTH "Blynk_Token"
#define SSID "WiFi_Ssid"
#define PASS "WiFi_Pass"

#define APP_KEY "SinricPro_App_key"
#define APP_SECRET "SinricPro_App_Secret"
#define SWITCH_ID "SinricPro_Switch_Id"

IPAddress SERVER_ADDRESS (188, 166, 206, 43);
#define HOSTNAME "Esp8266 Down"

#define RELAY_PIN D5

#define DIVIDE_RATIO 1.98    // (Depth of Tank that store water in cm) / 100
//My tank -> 198 cm

#define MULTIPLY_RATIO 56.7  // (Total tank volume in litre) / 100
//My tank -> 5670 litre

#define MAX_HEIGHT 214       // (Depth of Tank that store water in cm) + (Mounting height of sensor)
//My tank -> 198cm + 16cm = 214 cm
