
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <max6675.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "ArduinoOTA.h"

#include "credentials.h"

// Screen settings
static const int screen_width   = 128; // OLED display width, in pixels
static const int screen_height  = 64; // OLED display height, in pixels
static const int screen_address = 0x3c;
static const int screen_reset   = -1;

// Scheduling related numbers
static const uint tick_ms = 10;
static const uint display_ticks      = 100;   // 1 s
static const uint temp_meas_ticks    = 25;    // 250 ms
static const uint temp_send_ticks    = 1000;  // 10 s
static const uint mqtt_connect_ticks = 1000;  // 10 s

static const uint auto_off_time = 3600;  // 1 hour

static const uint temp_meas_history = 2;

// pin definitions
static const int ktcCLK  = D5; // GPIO14
static const int ktcSO   = D6; // GPIO12
static const int ktcCS   = D8; // GPIO15

#if BOARD_ID == 1

static const int sclPin  = D1; // GPIO5
static const int sdaPin  = D2; // GPIO4

static const int pumpRly = D7; // GPIO13
static const int heatRly = D0; // GPIO16, HIGH at boot

static const int pumpSw  = D3; // GPIO0, pulled up

#elif BOARD_ID == 2

static const int sclPin  = D3; // GPIO0
static const int sdaPin  = D4; // GPIO2

static const int pumpRly = D2; // GPIO4
static const int heatRly = D1; // GPIO5

static const int pumpSw  = D7; // GPIO13

#endif

class Relay
{
public:
  Relay(int pin, const char* name, unsigned long dead_time_interval) :
    mPin(pin),
    mName(name),
    mState(false),
    mDeadTime(0),
    mDeadTimeInterval(dead_time_interval)
  {
  }

  void Init()
  {
    digitalWrite(mPin, LOW);
    pinMode(mPin,      OUTPUT);
  }

  bool GetState()
  {
    return mState;
  }

  void SetState(bool state)
  {
    unsigned long curTime = millis();
    if (state != mState && curTime > mDeadTime)
    {
      // Only switch once every second max
      digitalWrite(mPin, state ? HIGH : LOW);
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
      mDeadTime = curTime + mDeadTimeInterval;
    }
  }

private:
  uint mPin;
  const char* mName;
  bool mState;
  unsigned long mDeadTime;
  unsigned long mDeadTimeInterval;
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
    mPrevState(EStateDisconnected),
    mMqttClient(mqttClient),
    mStateTransTime(0),
    mPrevStateDuration(0.0)
  {}

  EState GetState()
  {
    return mState;
  }

  void SetState(EState state)
  {
    if (state != mState)
    {
      // Store prev state details
      mPrevState = mState;
      mPrevStateDuration = GetTimeInState();

      // Do state transition
      mStateTransTime = millis();
      mState = state;

      PublishState();
    }
  }

  void PublishState()
  {
    const char* state_str = GetStateStr();
    mMqttClient.publish(mqtt_state_topic, state_str, true);
    Serial.print("State change to ");
    Serial.print(state_str);
    Serial.println();
  }

  float GetTimeInState()
  {
    unsigned long time_ms = millis() - mStateTransTime;
    return (time_ms / 1000.0);
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

  const char* GetDisplayStr(bool heater_on, float temp)
  {
    const char* state_str = nullptr;
    static char state_str_buf[16];

    if (mPrevState == EStatePumpOn && GetTimeInState() < 10.0)
    {
      snprintf(state_str_buf, sizeof(state_str_buf), "Shot: %.1f s", mPrevStateDuration);
      state_str = state_str_buf;
    }
    else
    {
      switch(mState)
      {
        case EStateDisconnected:
          state_str = "Disconnected";
          break;
        case EStatePowerOff:
          state_str = "Off";
          break;
        case EStatePowerOn:
          state_str = heater_on ? "Heating.." : "Ready";
          break;
        case EStatePumpOn:
          snprintf(state_str_buf, sizeof(state_str_buf), "Shot: %.0f s", GetTimeInState());
          state_str = state_str_buf;
          break;
      }
    }
    return state_str;
  }

  const char* GetStateStr()
  {
    return GetStateStr(mState);
  }

  void Init()
  {
    SetState(EStatePowerOn);
  }

private:
  EState mState;
  EState mPrevState;
  PubSubClient& mMqttClient;
  unsigned long mStateTransTime;
  float mPrevStateDuration;
};

class TempSensor
{
public:
  TempSensor(MAX6675& ktc, PubSubClient& mqttClient) :
    mKtc(ktc),
    mMqttClient(mqttClient),
    mTemp(0.0)
  {
    for (uint calc_id=0; calc_id<temp_meas_history;++calc_id)
    {
      mTempHistory[calc_id] = 0.0f;
    }
  }

  float& MeasureTemp()
  {
    static int store_id = 0;

    // store measurement in ringbuffer
    mTempHistory[store_id++] = mKtc.readCelsius();
    if (store_id == temp_meas_history)
    {
      store_id = 0;
    }

    // average over history
    mTemp = 0.0f;
    for (uint calc_id=0; calc_id<temp_meas_history;++calc_id)
    {
      mTemp += mTempHistory[calc_id];
    }
    mTemp /= temp_meas_history;

    // simple moving average
    //mTemp = 0.7 * mTemp + 0.3 * curTemp;

    return mTemp;
  }

  char* GetTempStr()
  {
    static char temp_str[8];
    //snprintf(temp_str, sizeof(temp_str), "%.1f", mTemp);
    snprintf(temp_str, sizeof(temp_str), "%.0f", roundf(mTemp));
    return temp_str;
  }

  void PublishTemp()
  {
    static char json_str[64];

    snprintf(json_str, sizeof(json_str), "{\"temperature\": %0.1f}", mTemp);
    mMqttClient.publish(mqtt_temp_topic, json_str);

    Serial.print("Temp change to ");
    Serial.print(mTemp);
    Serial.println();
  }

private:
  MAX6675& mKtc;
  PubSubClient& mMqttClient;
  float mTemp;
  float mTempHistory[temp_meas_history];
};


Adafruit_SSD1306 display(screen_width, screen_height, &Wire, screen_reset);

MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

Button pumpButton(pumpSw);

Relay heatRelay(heatRly, "Heat", 1000);
Relay pumpRelay(pumpRly, "Pump", 0);

StateMachine stateMachine(mqttClient);

TempSensor tempSensor(ktc, mqttClient);

void print_start()
{
  display.clearDisplay();
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

void mqttConnect()
{
  // try max 3 times
  for (uint i = 0; i < 4; i++)
  {
    if (mqttClient.connect(
          wifi_hostname, mqtt_user, mqtt_pass,
          mqtt_state_topic, 0, true,
          StateMachine::GetStateStr(StateMachine::EStateDisconnected)))
    {
      mqttClient.subscribe(mqtt_set_topic);
      stateMachine.PublishState();
      break;
    }
  }
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
  // Has to happen first
  WiFi.hostname(wifi_hostname);

  // Initialize i2c
  Wire.begin(sdaPin, sclPin);

  // Set input and output pins
  heatRelay.Init();
  pumpRelay.Init();

  pumpButton.Init();

  // Initialize serial output
  Serial.begin(115200);

  // Start Screen
  if(!display.begin(SSD1306_SWITCHCAPVCC, screen_address))
  {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }

  display.setFont(&FreeSans9pt7b);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (mqtt_enabled)
  {
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
  }

  if (wifi_enabled)
  {
    WiFi.begin(wifi_ssid, wifi_pass);

    print_start();
    print_add("Connecting to WiFi..");
    print_end();

    // Block untill connected, try for 5 seconds max
    uint tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < 50)
    {
      delay(100);
      if (WiFi.status() == WL_CONNECT_FAILED)
      {
        break;
      }
    }
  }

  if (WiFi.status() == WL_CONNECTED && mqtt_enabled)
  {
    print_start();
    print_add("Connecting to MQTT..");
    print_end();

    mqttConnect();
  }


  //Init statemachine last so it can broadcast its state via mqtt
  stateMachine.Init();

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}

float temp_set = 91.0;

uint next_tick_ms = 0;
uint tick = 0;

float temp = 0.0;

void loop()
{
    ArduinoOTA.handle();

    auto time = millis();

    if (time > next_tick_ms)
    {
      // Progress tick
      next_tick_ms += tick_ms;
      tick++;

      // Handle button pushes
      // Only react on pump button pushes if power is not off
      if (stateMachine.GetState() != StateMachine::EStateDisconnected)
      {
        // Based on switch state, choose between heat on or pump on
        stateMachine.SetState(
          (pumpButton.GetDebouncedState() == true) ?
            StateMachine::EStatePumpOn :
            StateMachine::EStatePowerOn);
      }

      // Handle MQTT messages
      mqttClient.loop();

      // Handle temp controller
      if (tick % temp_meas_ticks == 0)
      {
        temp = tempSensor.MeasureTemp();

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
                temp > temp_set)
            {
              // heater off
              heatRelay.SetState(false);
            }
            else
            {
              // heater on
              heatRelay.SetState(true);
            }

            if (stateMachine.GetTimeInState() > auto_off_time)
            {
              stateMachine.SetState(StateMachine::EStatePowerOff);
            }

          }
          break;
          default: // EStateDisconnected and any invalid state
          {
            // heater off
            heatRelay.SetState(false);
            // pump off
            pumpRelay.SetState(false);
          }
          break;
        }
      }

      // Handle display
      if (tick % display_ticks == 0)
      {
	      // Update display
        display.clearDisplay();
        display.setCursor(0,20);
        display.setFont(&FreeSansBold12pt7b);
        display.print(tempSensor.GetTempStr());
        display.print(" °C");
        display.setCursor(0,50);
        display.setFont(&FreeSans9pt7b);
        display.print(stateMachine.GetDisplayStr(heatRelay.GetState(), temp));
        if (WiFi.status() != WL_CONNECTED)
          display.print(" no WiFi");
        else if (not mqttClient.connected())
          display.print(" no MQTT");
        display.display();
      }

      if (tick % temp_send_ticks == 0)
      {
        tempSensor.PublishTemp();
      }

      // Handle MQTT reconnect
      if (tick % mqtt_connect_ticks == 0)
      {
        if (mqtt_enabled &&                       // If mqtt is enabled
            heatRelay.GetState() == false &&      // and heat relay is powered off
            WiFi.status() == WL_CONNECTED &&      // and wifi is connected
            !mqttClient.connected())              // but mqtt is disconnected
        {
          // Reconnect MQTT (blocking)
          mqttConnect();
        }
      }
    }
    delay(1);
}
