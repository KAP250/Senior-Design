#include <WiFi.h>
#include <WebServer.h>
#include <esp32cam.h>

//Network login
const char* ssid = "DD-WRT";
const char* password = "bayberry7322";

//Web server
WebServer server(80);

static auto hiRes = esp32cam::Resolution::find(800, 600);

void sendJPG()
{
  auto frame = esp32cam::capture();
  if(frame == nullptr)
  {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }
  Serial.printf("CAPTURE DONE %d x %d %db\n", frame->getWidth(), frame->getHeight(), static_cast<int>(frame->size()));
  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

void jpgHi()
{
  if(!esp32cam::Camera.changeResolution(hiRes))
  {
    Serial.println("HI-RES FAILED");
  }
  sendJPG();
}

void setup() 
{
  Serial.begin(115200);
  Serial.println();

  //Camera setup in brackets below
  {
    using namespace esp32cam;
    Config config;
    config.setPins(pins::AiThinker);
    config.setResolution(hiRes);
    config.setBufferCount(1);
    config.setJpeg(80);

    if (psramFound()) 
    {
      Serial.println("PSRAM is available.");
    } 
    else 
    {
      Serial.println("PSRAM not found.");
    }

    bool ok = Camera.begin(config);
    Serial.println(ok ? "CAMERA STARTED" : "CAMERA FAILED");
  }

  //Bring WiFi up
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Trying to connect to WiFi");
    delay(500);
  }

  Serial.print("http://");
  Serial.println(WiFi.localIP());
  server.on("/cam-hi.jpg", jpgHi);
  server.begin();

  Serial.println("Setup done");
}

void loop() 
{
  server.handleClient();
}
