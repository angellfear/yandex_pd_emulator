/*
yandex pd controller emulator
autor: angellfear <angell@angellfear.ru>

*/

// закоментировать если надо для дуо
#define max2

#include <Wire.h>

// Адрес I2C для эмуляции HUSB238
#define HUSB238_ADDRESS 0x08

// Адреса регистров
#define REG_PD_STATUS0 0x00
#define REG_PD_STATUS1 0x01
#define REG_GO_COMMAND 0x09
#define REG_PDO_SELECT 0x08 // Новый регистр для выбора PDO

// Маски и параметры
#define PD_SRC_VOLTAGE_MASK 0xF0 // Биты [7:4] для напряжения
#define PD_SRC_CURRENT_MASK 0x0F // Биты [3:0] для тока

// Поддерживаемые уровни мощности
#ifdef max2
#define SUPPORTED_VOLTAGE 0b0100  // 15 В
#define SUPPORTED_CURRENT 0b1010  // 3 А
#else
#define SUPPORTED_VOLTAGE 0b0110  // 20 В
#define SUPPORTED_CURRENT 0b1101  // 3.25 А
#endif

#define SDA_0 4
#define SCL_0 5

// Регистр состояния
uint8_t pd_status0 = 0;
uint8_t pd_status1 = 0;
uint8_t selected_pdo = 0x00; // Регистр для выбора PDO

void setup() {
  Serial.begin(115200);
  Serial.println("HUSB238 эмулятор запущен");

  // Настройка I2C в режиме slave
//  Wire.begin(SDA_0,SCL_0);
  Wire.begin(HUSB238_ADDRESS);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);

  // Инициализация регистров
  pd_status0 = (SUPPORTED_VOLTAGE << 4) | SUPPORTED_CURRENT; // Установка напряжения и тока
  pd_status1 = 0b10000000; // Устройство подключено (бит 7)

  Serial.println("Готово к работе");
}

void loop() {
  // Основной цикл пустой, всё обрабатывается в I2C прерываниях
}

// Обработчик запросов от мастера
void onRequest() {
  uint8_t regAddress = Wire.read();
  Serial.print("I2C запрос: регистр 0x");
  Serial.println(regAddress, HEX);

  switch (regAddress) {
    case REG_PD_STATUS0:
      Serial.print("Отправка REG_PD_STATUS0: 0b");
      Serial.println(pd_status0, BIN);
      Wire.write(pd_status0);
      break;
    case REG_PD_STATUS1:
      Serial.print("Отправка REG_PD_STATUS1: 0b");
      Serial.println(pd_status1, BIN);
      Wire.write(pd_status1);
      break;
    case REG_PDO_SELECT:
      Serial.print("Отправка REG_PDO_SELECT: 0x");
      Serial.println(selected_pdo, HEX);
      Wire.write(selected_pdo);
      break;
    default:
      Serial.print("Неизвестный регистр 0x");
      Serial.println(regAddress, HEX);
      Wire.write(0x00);
      break;
  }
}

// Обработчик данных от мастера
void onReceive(int numBytes) {
  if (numBytes < 2) return; // Ожидаем как минимум регистр и данные

  uint8_t regAddress = Wire.read();
  uint8_t data = Wire.read();

  Serial.print("I2C запись: регистр 0x");
  Serial.print(regAddress, HEX);
  Serial.print(", данные 0b");
  Serial.println(data, BIN);

  switch (regAddress) {
    case REG_GO_COMMAND:
      handleGoCommand(data);
      break;
    case REG_PDO_SELECT:
      handlePdoSelect(data);
      break;
    default:
      Serial.print("Запись в неизвестный регистр 0x");
      Serial.println(regAddress, HEX);
      break;
  }
}

void handleGoCommand(uint8_t data) {
  if (data == 0b10000) {
    // Сброс устройства
    Serial.println("Сброс устройства...");
    pd_status0 = (SUPPORTED_VOLTAGE << 4) | SUPPORTED_CURRENT; // Восстановление параметров
    pd_status1 = 0b10000000; // Подключено
  } else if (data == 0b00001) {
    // Запрос PDO
    Serial.println("Запрос PDO выполнен");
    pd_status1 |= 0b00000001; // Подтверждение запроса
  } else if (data == 0b00100) {
    // Запрос возможностей источника
    Serial.println("Запрос возможностей источника");
  } else {
    Serial.println("Неизвестная команда в REG_GO_COMMAND");
  }
}

void handlePdoSelect(uint8_t data) {
  uint8_t voltage = (data & PD_SRC_VOLTAGE_MASK) >> 4;
  uint8_t current = data & PD_SRC_CURRENT_MASK;

  Serial.print("Запрошенный профиль: напряжение = ");
  Serial.print(voltage * 5); // Напряжение в вольтах (шаг 5 В)
  Serial.print(" В, ток = ");
  Serial.print(current * 0.5); // Ток в амперах (шаг 0.5 А)
  Serial.println(" А");

  if (voltage == SUPPORTED_VOLTAGE && current == SUPPORTED_CURRENT) {
    Serial.println("Установлен поддерживаемый профиль");
    pd_status0 = data; // Обновляем регистр состояния
    selected_pdo = data; // Сохраняем выбранный профиль
  } else {
    Serial.println("Ошибка: запрошенный профиль мощности не поддерживается");
  }
}
