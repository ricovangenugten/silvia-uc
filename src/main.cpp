#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <max6675.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "credentials.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3D
#define OLED_RESET -1

// pin definitions
int powerSw = D7; // GPIO13, shared with SPI MOSI
int pumpSw  = D3; // GPIO0, pulled up

// Declaration for an SSD1306 display connected to I2C
//  OLED SCL  D1  // GPIO5
//  OLED SDA  D2  // GPIO4

int pumpRly = D0; // GPIO16, HIGH at boot
int heatRly = D4; // GPIO2, HIGH at boot, pulled up, connected to onboard led

int ktcCLK  = D5; // GPIO14
int ktcSO   = D6; // GPIO12
int ktcCS   = D8; // GPIO15

class Relay
{
public:
  Relay(int pin, const char* name) :
    mPin(pin),
    mName(name),
    mState(false)
  {
  }

  void Init()
  {
    digitalWrite(mPin, HIGH);
    pinMode(mPin,      OUTPUT);
  }

  void SetState(bool state)
  {
    if (state != mState)
    {
      digitalWrite(mPin, state ? LOW : HIGH);
      Serial.print("Relay ");
      Serial.print(mName);
      if (state)
      {
        Serial.print(" on\n");
      }
      else
      {
        Serial.print(" off\n");
      }
      mState = state;
    }
  }
private:
  uint mPin;
  const char* mName;
  bool mState;
};

class Button
{
public:
  Button(int pin) :
    mPin(pin),
    mDebouncedState(false),
    mStateChangeCount(0),
    mStateChangeThreshold(5)
  {
  }

  void Init()
  {
    pinMode(mPin, INPUT_PULLUP);
  }

  bool GetDebouncedState()
  {
    bool curState = (digitalRead(mPin) == LOW);

    if (curState != mDebouncedState)
    {
      mStateChangeCount++;
    }
    else
    {
      mStateChangeCount = 0;
    }

    if (mStateChangeCount >= mStateChangeThreshold)
    {
      mDebouncedState = curState;
    }

    return mDebouncedState;
  }

  bool GetDebouncedRisingEdge()
  {
    bool prevDebouncedState = mDebouncedState;
    bool curDebouncedState = GetDebouncedState();
    return (curDebouncedState == true && prevDebouncedState == false);
  }

private:
  uint mPin;
  bool mDebouncedState;
  uint mStateChangeCount;
  uint mStateChangeThreshold;
};

class StateMachine
{
public:
  enum EState
  {
    EStateDisconnected = 0,
    EStatePowerOff,
    EStatePowerOn,
    EStatePumpOn,
  };

  StateMachine(PubSubClient& mqttClient) :
    mState(EStateDisconnected),
    mMqttClient(mqttClient)
  {}

  EState GetState()
  {
    return mState;
  }

  void SetState(EState state)
  {
    if (state != mState)
    {
      mState = state;
      const char* state_str = GetStateStr();
      mMqttClient.publish("silvia/state", state_str, true);
      Serial.print("State change to ");
      Serial.print(state_str);
      Serial.println();
    }
  }

  static const char* GetStateStr(EState state)
  {
    const char* state_str = nullptr;
    switch(state)
    {
      case EStateDisconnected:  state_str = "Disconnected";  break;
      case EStatePowerOff:      state_str = "PowerOff";      break;
      case EStatePowerOn:       state_str = "PowerOn";       break;
      case EStatePumpOn:        state_str = "PumpOn";        break;
    }
    return state_str;
  }

  const char* GetStateStr()
  {
    return GetStateStr(mState);
  }

  void Init()
  {
    SetState(EStatePowerOff);
  }

private:
  EState mState;
  PubSubClient& mMqttClient;
};

class TempSensor
{
public:
  TempSensor(MAX6675& ktc, PubSubClient& mqttClient) :
    mKtc(ktc),
    mMqttClient(mqttClient),
    mTemp(0.0)
  {}

  double& MeasureTemp()
  {
    double curTemp = mKtc.readCelsius();
    // TODO: remove this when thermocouple is connected
    curTemp = 91.12345;
    if (curTemp != mTemp)
    {
      mTemp = curTemp;
      static char temp_str[8];
      snprintf(temp_str, sizeof(temp_str), "%.2f", mTemp);
      mMqttClient.publish("silvia/temp", temp_str);
      Serial.print("Temp change to ");
      Serial.print(mTemp);
      Serial.println();
    }
    return mTemp;
  }

private:
  MAX6675& mKtc;
  PubSubClient& mMqttClient;
  double mTemp;
};


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

Button powerButton(powerSw);
Button pumpButton(pumpSw);

Relay heatRelay(heatRly, "Heat");
Relay pumpRelay(pumpRly, "Pump");

StateMachine stateMachine(mqttClient);

TempSensor tempSensor(ktc, mqttClient);

void print_start()
{
  display.setCursor(0,20);
}

template <class T>
void print_add (T arg)
{
   display.print(arg);
   Serial.print(arg);
}

void print_end()
{
  Serial.println();
  display.display();
}

void mqttCallbackProcessPayload(byte* payload, uint length, StateMachine::EState state)
{
  if (strncmp((char*)payload, StateMachine::GetStateStr(state), length) == 0)
  {
    stateMachine.SetState(state);
  }
}

void mqttCallback(char* topic, byte* payload, uint length)
{
  mqttCallbackProcessPayload(payload, length, StateMachine::EStatePowerOff);
  mqttCallbackProcessPayload(payload, length, StateMachine::EStatePowerOn);
}

void setup()
{
  // Set input and output pins
  heatRelay.Init();
  pumpRelay.Init();

  powerButton.Init();
  pumpButton.Init();

  // Initialize serial output
  Serial.begin(115200);

  // Start Screen
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }

  display.setFont(&FreeSans9pt7b);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (wifi_enabled)
  {
    WiFi.begin(wifi_ssid, wifi_pass);

    print_start();
    print_add("Connecting to WiFi");
    print_end();

    while (WiFi.status() != WL_CONNECTED)
    {
      delay(100);
      if (WiFi.status() == WL_CONNECT_FAILED)
      {
        print_start();
        print_add("Could not connect to WiFi");
        print_end();
        break;
      }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      print_start();
      print_add("IP address: ");
      print_add(WiFi.localIP());
      print_end();
    }
  }

  if (WiFi.status() == WL_CONNECTED && mqtt_enabled)
  {
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    print_start();
    print_add("Connecting to MQTT");
    print_end();

    if (mqttClient.connect(
          "SilviaESP8266", mqtt_user, mqtt_pass,
          "silvia/state", 0, true, StateMachine::GetStateStr(StateMachine::EStateDisconnected)))
    {
      print_start();
      print_add("Connected!");
      print_end();
      mqttClient.subscribe("silvia/set");
    }
    else
    {
      print_start();
      print_add("MQTT connect failed: ");
      print_add(mqttClient.state());
      print_end();
    }
  }

  //Init statemachine last so it can broadcast its state via mqtt
  stateMachine.Init();
}

void loop()
{
  const uint tick_ms = 10;
  const uint display_ticks    = 50;  // 500 ms
  const uint controller_ticks = 10; //  100 ms

  const double temp_set = 95.0;
  const double temp_hyst = 0.5;

  uint next_tick_ms = 0;
  uint tick = 0;

  double temp = 0.0;

  while (1)
  {
    if (millis() > next_tick_ms)
    {
      // Progress tick
      next_tick_ms += tick_ms;
      tick++;

      // Handle button pushes
      // Always react on power button pushes
      if (powerButton.GetDebouncedRisingEdge() == true)
      {
        // If state is off, turn on. If state is not off (heat or pump on), turn off
        stateMachine.SetState(
           (stateMachine.GetState() == StateMachine::EStatePowerOff) ?
             StateMachine::EStatePowerOn :
             StateMachine::EStatePowerOff);
      }

      // Only react on pump button pushes if power is not off
      if (stateMachine.GetState() != StateMachine::EStatePowerOff)
      {
        // Based on switch state, choose between heat on or pump on
        stateMachine.SetState(
          (pumpButton.GetDebouncedState() == true) ?
            StateMachine::EStatePumpOn :
            StateMachine::EStatePowerOn);
      }

      // Handle MQTT
      mqttClient.loop();

      // Handle display
      if (tick % display_ticks == 0)
      {
	// Read temp
        temp = tempSensor.MeasureTemp();

	// Update display
        display.setCursor(0,20);
        display.print("Temp: ");
        display.print(temp);
        display.print(" C ");
        display.print(stateMachine.GetStateStr());
        display.display();
      }

      // Handle temp controller
      if (tick % controller_ticks == 0)
      {
        switch (stateMachine.GetState())
        {
          case StateMachine::EStatePumpOn:
          {
            // pump on
            pumpRelay.SetState(true);

            // heater on (regardless of temp)
            heatRelay.SetState(true);

          }
          break;
          case StateMachine::EStatePowerOn:
          {
            // pump off
            pumpRelay.SetState(false);

            // do temp control
            if (isnan(temp) ||
                temp > (temp_set + temp_hyst))
            {
              // heater off
              heatRelay.SetState(false);
            }
            else if (temp < (temp_set - temp_hyst))
            {
              // heater on
              heatRelay.SetState(true);
            }
          }
          break;
          default: // EStatePowerOff and any invalid state
          {
            // heater off
            heatRelay.SetState(false);
            // pump off
            pumpRelay.SetState(false);
          }
          break;
        }
      }
    }
    delay(1);
  }
}
