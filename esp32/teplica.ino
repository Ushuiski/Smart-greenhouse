/*********
  Руи Сантос
  Более подробно о проекте на: https://randomnerdtutorials.com
*********/

#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// впишите в двух строчках ниже SSID и пароль для своей WiFi-сети,
// чтобы ваша ESP32 могла подключиться к WiFi-роутеру:
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

//Определяем тип датчика
#define DHTTYPE DHT11   // DHT 11

// вставьте в переменную «MQTT_HOST» IP-адрес своей Raspberry Pi,
// чтобы ESP32 могла подключиться к MQTT-брокеру Mosquitto:
#define MQTT_HOST IPAddress(192, 168, 1, 54)
#define MQTT_PORT 1883

// создаем объекты для управления MQTT-клиентом:
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// задаем количество столбцов и рядов для LCD-дисплея:
const int lcdColumns = 16;
const int lcdRows = 2;

// создаем объект LCD-дисплея,
// присваивая ему адрес, а также количество столбцов и рядов;
// (если вам неизвестен адрес дисплея,
// запустите скетч для сканирования I2C-устройств):
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// иконка uhflecf:
byte thermometerIcon[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B01110,
  B00000,
  B00000,
  B00000
};

unsigned long previousMillis = 0;  // время, когда в последний раз была опубликована температура
const long interval = 10000;       // интервал, с которым будут публиковаться данные от датчика

const int ledPin = 17;             // GPIO-контакт,к которому подключен свет
int ledState = LOW;                // текущее состояние выходного контакта

const int fan1Pin = 5;             // GPIO-контакт,к которому подключен вентилятор №1
int fan1State = LOW;               // текущее состояние выходного контакта

const int fan2Pin = 18;             // GPIO-контакт,к которому подключен вентилятор №2
int fan2State = LOW;               // текущее состояние выходного контакта

const int reservePin = 19;         // Резервный GPIO-контакт
int reserveState = LOW;            // текущее состояние выходного контакта

const int buttonLedPin = 35;         // задаем GPIO-контакт, к которому подключена кнопка №1
int buttonLedState = LOW;;                  // текущее состояние входного контакта (кнопка №1)
int lastButtonLedState = LOW;        // предыдущее состояние входного контакта (кнопка №1)

const int buttonFan1Pin = 32;         // задаем GPIO-контакт,к которому подключена кнопка №2
int buttonFan1State = LOW;;                 // текущее состояние входного контакта (кнопка №2)
int lastButtonFan1State = LOW;       // предыдущее состояние входного контакта (кнопка №2)

const int buttonFan2Pin = 33;         // задаем GPIO-контакт,к которому подключена кнопка №3
int buttonFan2State = LOW;;                 // текущее состояние входного контакта (кнопка №3)
int lastButtonFan2State = LOW;       // предыдущее состояние входного контакта (кнопка №3)

const int buttonReservePin = 25;         // задаем GPIO-контакт,к которому подключена кнопка №4
int buttonReserveState = LOW;;                  // текущее состояние входного контакта (кнопка №4)
int lastButtonReserveState = LOW;        // предыдущее состояние входного контакта (кнопка №4)

unsigned long lastDebounceTime = 0;  // время, когда в последний раз был переключен выходной контакт
unsigned long debounceDelay = 300;    // время антидребезга (увеличьте это значение,если выходной сигнал по-прежнему «прыгает»)

// DHT Sensor
uint8_t DHTPin = 4;

// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);

float Temperature;
float Humidity;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  //  "Подключаемся к WiFi..."
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  //  "Подключаемся к MQTT..."
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");  //  "Подключились к WiFi"
      Serial.println("IP address: ");  //  "IP-адрес: "
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      //  "WiFi-связь потеряна"
      // делаем так, чтобы
      // ESP32 не переподключалась к MQTT
      // во время переподключения к WiFi:
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// в этой функции можно добавить новые топики для подписки:
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");  //  "Подключились к MQTT."
  Serial.print("Session present: ");  //  "Текущая сессия: "
  Serial.println(sessionPresent);

  // подписываем ESP32 на топик «esp32/led»:
  uint16_t packetIdSub1 = mqttClient.subscribe("esp32/led", 0);
  Serial.print("Subscribing at QoS 0, packetId: "); //  "Подписываемся при QoS 0, ID пакета: "
  Serial.println(packetIdSub1);

  // подписываем ESP32 на топик «esp32/fan1»:
  uint16_t packetIdSub2 = mqttClient.subscribe("esp32/fan1", 0);
  Serial.print("Subscribing at QoS 0, packetId: "); //  "Подписываемся при QoS 0, ID пакета: "
  Serial.println(packetIdSub2);

  // подписываем ESP32 на топик «esp32/fan2»:
  uint16_t packetIdSub3 = mqttClient.subscribe("esp32/fan2", 0);
  Serial.print("Subscribing at QoS 0, packetId: "); //  "Подписываемся при QoS 0, ID пакета: "
  Serial.println(packetIdSub3);

  // подписываем ESP32 на топик «esp32/reserve»:
  uint16_t packetIdSub4 = mqttClient.subscribe("esp32/reserve", 0);
  Serial.print("Subscribing at QoS 0, packetId: "); //  "Подписываемся при QoS 0, ID пакета: "
  Serial.println(packetIdSub4);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT."); //  "Отключились от MQTT."
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged."); //  "Подписка подтверждена."
  Serial.print("  packetId: ");  //  "  ID пакета: "
  Serial.println(packetId);
  Serial.print("  qos: ");  //  "  уровень качества обслуживания: "
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged."); //  "Отписка подтверждена."
  Serial.print("  packetId: ");  //  "  ID пакета: "
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged."); //  "Публикация подтверждена."
  Serial.print("  packetId: ");  //  "  ID пакета: "
  Serial.println(packetId);
}

// в этой функции задается, что произойдет,
// когда ESP32 получит то или иное сообщение»
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  // проверяем, пришло ли MQTT-сообщение в топик «esp32/led»:
  if (strcmp(topic, "esp32/led") == 0) {
    // если светодиод выключен, включаем его (и наоборот):
    if (messageTemp == "true") {
      digitalWrite(ledPin, LOW);
    }
    else if (messageTemp == "false") {
      digitalWrite(ledPin, HIGH);
    }
  }
  
  // проверяем, пришло ли MQTT-сообщение в топик «esp32/fan1»:
  if (strcmp(topic, "esp32/fan1") == 0) {
    // если светодиод выключен, включаем его (и наоборот):
    if (messageTemp == "true") {
      digitalWrite(fan1Pin, LOW);
    }
    else if (messageTemp == "false") {
      digitalWrite(fan1Pin, HIGH);
    }
  }

  // проверяем, пришло ли MQTT-сообщение в топик «esp32/fan2»:
  if (strcmp(topic, "esp32/fan2") == 0) {
    // если светодиод выключен, включаем его (и наоборот):
    if (messageTemp == "true") {
      digitalWrite(fan2Pin, LOW);
    }
    else if (messageTemp == "false") {
      digitalWrite(fan2Pin, HIGH);
    }
  }

  // проверяем, пришло ли MQTT-сообщение в топик «esp32/reserve»:
  if (strcmp(topic, "esp32/reserve") == 0) {
    // если светодиод выключен, включаем его (и наоборот):
    if (messageTemp == "true") {
      digitalWrite(reservePin, LOW);
    }
    else if (messageTemp == "false") {
      digitalWrite(reservePin, HIGH);
    }
  }
  
  Serial.println("Publish received.");
  //  "Опубликованные данные получены."
  Serial.print("  message: ");  //  "  сообщение: "
  Serial.println(messageTemp);
  Serial.print("  topic: ");  //  "  топик: "
  Serial.println(topic);
  Serial.print("  qos: ");  //  "  уровень качества обслуживания: "
  Serial.println(properties.qos);
  Serial.print("  dup: ");  //  "  дублирование сообщения: "
  Serial.println(properties.dup);
  Serial.print("  retain: ");  //  "  сохраненные сообщения: "
  Serial.println(properties.retain);
  Serial.print("  len: ");  //  "  размер: "
  Serial.println(len);
  Serial.print("  index: ");  //  "  индекс: "
  Serial.println(index);
  Serial.print("  total: ");  //  "  суммарно: "
  Serial.println(total);
}

void setup() {
  Serial.begin(115200);

  // инициализируем LCD-дисплей:
  lcd.init();
  // включаем подсветку LCD-дисплея:
  lcd.backlight();
  // создаем иконку термометра:
  lcd.createChar(0, thermometerIcon);
  
  // делаем контакт светодиода выходным (OUTPUT)
  // и задаем ему значение «LOW»:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // делаем контакт fan №1 выходным (OUTPUT)
  // и задаем ему значение «LOW»:
  pinMode(fan1Pin, OUTPUT);
  digitalWrite(fan1Pin, LOW);

  // делаем контакт fan №2 выходным (OUTPUT)
  // и задаем ему значение «LOW»:
  pinMode(fan2Pin, OUTPUT);
  digitalWrite(fan2Pin, LOW);

  // делаем контакт reserve выходным (OUTPUT)
  // и задаем ему значение «LOW»:
  pinMode(reservePin, OUTPUT);
  digitalWrite(reservePin, LOW);

  // делаем контакты для кнопок входными (INPUT)
  pinMode(buttonLedPin, INPUT);
  pinMode(buttonFan1Pin, INPUT);
  pinMode(buttonFan2Pin, INPUT);
  pinMode(buttonReservePin, INPUT);

  //настраиваем контакт датчика температуры и влажности
  pinMode(DHTPin, INPUT);
  dht.begin();


  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T = ");  //  "Температура"
  lcd.setCursor(0, 1);
  lcd.print("H = "); //  "Влажность"
}

void loop() {
  unsigned long currentMillis = millis();
  // каждые Х секунд
  // ESP32 будет публиковать новое MQTT-сообщение
  // в топик «esp32/temperature»:
  if (currentMillis - previousMillis >= interval) {
    // сохраняем время, когда в последний раз
    // были опубликованы новые данные от датчика:
    previousMillis = currentMillis;
    Temperature = dht.readTemperature();
    Humidity = dht.readHumidity();
    lcd.setCursor(5, 0);
    lcd.print(Temperature);
    lcd.print(" ");
    lcd.write(0);
    lcd.print("C"); 
    lcd.setCursor(5, 1);
    lcd.print(Humidity);
    lcd.print(" %");

  // публикуем в топик «esp32/temperature»
  // MQTT-сообщение с температурой в градусах Цельсия:
  uint16_t packetIdPub1 = mqttClient.publish("esp32/temperature", 2,
                          true,  String(Temperature).c_str());

  Serial.print("Publishing on topic esp32/temperature at QoS 2, packetId: ");
  //  "Публикуем в топик «esp32/temperature»
  //   при QoS 2, ID пакета: "
  Serial.println(packetIdPub1);

  // публикуем в топик «esp32/Humidity»
  // MQTT-сообщение с  влажностью:
  uint16_t packetIdPub2 = mqttClient.publish("esp32/Humidity", 2,
                          true,  String(Humidity).c_str());

  Serial.print("Publishing on topic esp32/Humidity at QoS 2, packetId: ");
  //  "Публикуем в топик «esp32/Humidity»
  //   при QoS 2, ID пакета: "
  Serial.println(packetIdPub2);

  }
// считываем состояния кнопок
// и сохраняем их в локальные переменные:
int reading1 = digitalRead(buttonLedPin);
// если состояние кнопки изменилось (из-за шума или нажатия),
// сбрасываем таймер:
if (reading1 != lastButtonLedState) {
  // сбрасываем таймер «антидребезга»:
  lastDebounceTime = millis();
}

int reading2 = digitalRead(buttonFan1Pin);
// если состояние кнопки изменилось (из-за шума или нажатия),
// сбрасываем таймер:
if (reading2 != lastButtonFan1State) {
  // сбрасываем таймер «антидребезга»:
  lastDebounceTime = millis();
}

int reading3 = digitalRead(buttonFan2Pin);
// если состояние кнопки изменилось (из-за шума или нажатия),
// сбрасываем таймер:
if (reading3 != lastButtonFan2State) {
  // сбрасываем таймер «антидребезга»:
  lastDebounceTime = millis();
}

int reading4 = digitalRead(buttonReservePin);
// если состояние кнопки изменилось (из-за шума или нажатия),
// сбрасываем таймер:
if (reading4 != lastButtonReserveState) {
  // сбрасываем таймер «антидребезга»:
  lastDebounceTime = millis();
}

// если состояние кнопки изменилось после периода «антидребезга»:
if ((millis() - lastDebounceTime) > debounceDelay) {
  // и если новое значение отличается от того,
  // что хранится сейчас в переменной «buttonLedState»:
  if (reading1 != buttonLedState) {
    buttonLedState = reading1;
    // публикуем MQTT-сообщение в топик «esp32/led/toggle»
    // чтобы переключить состояние светодиода
    // (т.е. чтобы включить или выключить его):
    if ((buttonLedState == HIGH)) {
      if (!digitalRead(ledPin)) {
        mqttClient.publish("esp32/led/toggle", 0, true, "false");
        Serial.println("Publishing on topic esp32/led/toggle topic at QoS 0"); //  "Публикуем в топик «esp32/led/toggle» при QoS 0"
      }
      else if (digitalRead(ledPin)) {
        mqttClient.publish("esp32/led/toggle", 0, true, "true");
        Serial.println("Publishing on topic esp32/led/toggle topic at QoS 0"); // "Публикуем в топик «esp32/led/toggle» при QoS 0"
      }
    }
  }
  // и если новое значение отличается от того,
  // что хранится сейчас в переменной «buttonFan1State»:
  if (reading2 != buttonFan1State) {
    buttonFan1State = reading2;
    // публикуем MQTT-сообщение в топик «esp32/fan1/toggle»
    // чтобы переключить состояние светодиода
    // (т.е. чтобы включить или выключить его):
    if ((buttonFan1State == HIGH)) {
      if (!digitalRead(fan1Pin)) {
        mqttClient.publish("esp32/fan1/toggle", 0, true, "false");
        Serial.println("Publishing on topic esp32/fan1/toggle topic at QoS 0");
        //  "Публикуем в топик «esp32/fan1/toggle»
        //   при QoS 0"
      }
      else if (digitalRead(fan1Pin)) {
        mqttClient.publish("esp32/fan1/toggle", 0, true, "true");
        Serial.println("Publishing on topic esp32/fan1/toggle topic at QoS 0");
        //  "Публикуем в топик «esp32/fan1/toggle»
        //   при QoS 0"
      }
    }
  }

  // и если новое значение отличается от того,
  // что хранится сейчас в переменной «buttonReserveState»:
  if (reading3 != buttonFan2State) {
    buttonFan2State = reading3;
    // публикуем MQTT-сообщение в топик «esp32/fan2/toggle»
    // чтобы переключить состояние светодиода
    // (т.е. чтобы включить или выключить его):
    if ((buttonFan2State == HIGH)) {
      if (!digitalRead(fan2Pin)) {
        mqttClient.publish("esp32/fan2/toggle", 0, true, "false");
        Serial.println("Publishing on topic esp32/fan2/toggle topic at QoS 0");
        //  "Публикуем в топик «esp32/fan2/toggle»
        //   при QoS 0"
      }
      else if (digitalRead(fan2Pin)) {
        mqttClient.publish("esp32/fan2/toggle", 0, true, "true");
        Serial.println("Publishing on topic esp32/fan2/toggle topic at QoS 0");
        //  "Публикуем в топик «esp32/fan2/toggle»
        //   при QoS 0"
      }
    }
  }

  // и если новое значение отличается от того,
  // что хранится сейчас в переменной «buttonState»:
  if (reading4 != buttonReserveState) {
    buttonReserveState = reading4;
    // публикуем MQTT-сообщение в топик «esp32/reserve/toggle»
    // чтобы переключить состояние светодиода
    // (т.е. чтобы включить или выключить его):
    if ((buttonReserveState == HIGH)) {
      if (!digitalRead(reservePin)) {
        mqttClient.publish("esp32/reserve/toggle", 0, true, "false");
        Serial.println("Publishing on topic esp32/reserve/toggle topic at QoS 0");
        //  "Публикуем в топик «esp32/reserve/toggle»
        //   при QoS 0"
      }
      else if (digitalRead(reservePin)) {
        mqttClient.publish("esp32/reserve/toggle", 0, true, "true");
        Serial.println("Publishing on topic esp32/reserve/toggle topic at QoS 0");
        //  "Публикуем в топик «esp32/reserve/toggle»
        //   при QoS 0"
      }
    }
  }
}
// сохраняем новое значение кнопки в переменную «lastButtonState»;
// в результате при следующем проходе через loop()
// новое значение кнопки будет считаться ее предыдущим значением:
lastButtonLedState = reading1;
lastButtonFan1State = reading2;
lastButtonFan2State = reading3;
lastButtonReserveState = reading4;
}
