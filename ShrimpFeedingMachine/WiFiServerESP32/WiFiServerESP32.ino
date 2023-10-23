#include <WiFi.h>
#include <WebServer.h>
#include "Queue.h"
#include <LiquidCrystal_I2C.h>

// Use only  core 1
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 0;
#endif

LiquidCrystal_I2C lcd(0x27, 20, 4);
String ClientRequest, myresultat, DataBuffer;
WiFiServer server(80);
WiFiClient client;

TaskHandle_t task1Handle;

Queue <String> queue = Queue<String>();

void displayValue(String value)
{
  if (value[0] == 'e')
  {
    lcd.setCursor(0, 0);
    lcd.printstr("E:");
    lcd.printstr("    ");
    lcd.setCursor(2, 0);
    lcd.printstr(value.substring(1).c_str());
  }
  else if (value[0] == 's')
  {
    lcd.setCursor(0, 1);
    lcd.printstr("Sec:");
    lcd.printstr("    ");
    lcd.setCursor(4, 1);
    lcd.printstr(value.substring(1).c_str());
  }
  else if (value[0] == 'm')
  {
    lcd.setCursor(8, 1);
    lcd.printstr("Min:");
    lcd.printstr("    ");
    lcd.setCursor(12, 1);
    lcd.printstr(value.substring(1).c_str());
  }
  else if (value[0] == 'p')
  {
    lcd.setCursor(15, 1);
    lcd.printstr("Lv:");
    lcd.printstr("  ");
    lcd.setCursor(17, 1);
    lcd.printstr(value.substring(1).c_str());
  } 
  else if (value[0] == 'W')
  {
    lcd.setCursor(0, 2);
    lcd.printstr("Speed:");
    lcd.printstr("    ");
    lcd.setCursor(6, 2);
    lcd.printstr(value.substring(1).c_str());
  }
  else if (value[0] == 'o')
  {
    lcd.setCursor(0, 3);
    lcd.printstr("Food:");
    lcd.printstr("          ");
    lcd.setCursor(5, 3);
    if (value[1] == '1')
    {
      lcd.printstr(" available");
    }
    else
    {
      lcd.printstr(" empty");
    }
  }
}

String ReadIncomingRequest()
{
  while(client.available())
  {
    ClientRequest = (client.readStringUntil('\r'));
    
    if ((ClientRequest.indexOf("HTTP/1.1")>0)&&(ClientRequest.indexOf("/favicon.ico")<0))
    {
      myresultat = ClientRequest;
    }
  }
  return myresultat;
}

void task1(void *parameter)
{
  while (1)
  {  
    if (Serial.available() > 0)
    {
      String containValue;
      while (Serial.available() > 0 && containValue.length() < 5)
      {
        char temp = Serial.read(); // Read the input string character by character      
        containValue += temp;
      }
      queue.push(containValue);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void serialFlush()
{
  while(Serial.available() > 0)
  {
    char t = Serial.read();
  }
}

void setup()
{
  ClientRequest = "";

  Serial.begin(9600);
    // For LCD I2C
  lcd.begin();
  lcd.backlight();
  lcd.clear();

  WiFi.disconnect();
  delay(3000);
  Serial.println("START");
  WiFi.begin("Sussy","nguyenquang10");
  while ((!(WiFi.status() == WL_CONNECTED))){
    delay(300);
    Serial.print("..");
  }
  Serial.println("Connected");
  Serial.println("Your IP is");
  Serial.println((WiFi.localIP()));

  server.begin();
  xTaskCreate(task1, "Task 1", 2048, NULL, 1, &task1Handle);
}

void loop()
{      
    client = server.available();
    if (!client) { return; }
    while(!client.available()) {  delay(1); }
    ClientRequest = (ReadIncomingRequest());
    ClientRequest.remove(0, 5);
    ClientRequest.remove(ClientRequest.length()-9,9);

    if (ClientRequest == "request")
    {
      if (queue.count() > 0)
      {
        ClientRequest.clear();
        String temp = queue.pop();
        ClientRequest += temp[0];
        int i = 1;
        while (temp[i] == '0' && i < temp.length()-1) i++;
        while (i < temp.length()) 
        {
          ClientRequest += temp[i];
          i++;
        }                
        //Serial.print(ClientRequest);
        displayValue(ClientRequest);
        serialFlush();
      }
      else
      {
        ClientRequest.clear();
        ClientRequest += 'X';
      }
    }
    else
    {
      displayValue(ClientRequest);
      if (ClientRequest.length() < 5)
      {
        int i = 5 - ClientRequest.length();
        DataBuffer += ClientRequest[0];
        for (int zero = 0; zero < i; zero++)
        {
          DataBuffer += '0';
        }
        for (int j = 1; j < ClientRequest.length(); j++)
        {
          DataBuffer += ClientRequest[j];
        }
      }
      else
      {
        DataBuffer = ClientRequest;         
      }
      Serial.print(DataBuffer);
      DataBuffer.clear();      
	    serialFlush();      
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("");
    client.print(ClientRequest);
    client.stop();
    delay(1);
    client.flush();
}