#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"

#define ADVERTISING_INTERVAL 3000
#define DEBOUNCE_SENSOR_MS 600
#define PIN_LED_BLE PIN_LED1
#define PIN_LED_SENSOR PIN_LED2
#define PIN_SW_SENSOR PIN_BUTTON1
#define SR_LEN 15
#define VBAT_MAX_IN_MV 3000
#define TX_POWER 4

BLEPeripheral blePeripheral = BLEPeripheral();

BLEService mainBleService("c83e0fa6-f5b7-473a-a2a2-3957117a2f58");
BLEUnsignedIntCharacteristic sensorCharacteristic("8afc0cf0-19bf-4bd2-9413-6e7259765edf", BLERead | BLEWrite);
BLEService batteryService("180F");
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("2A19", BLERead);

volatile bool g_sensorValueChanged = false;
volatile uint32_t g_lastSensorValueChanged = 0;
volatile uint32_t g_counterValue = 0;
const char g_localName[] = "ProtoSensor";

static void gpio_init(void);
void bleConnectedCallback(BLECentral &);
void sensorCharacteristicWrittenCallback(BLECentral &, BLECharacteristic &);
void log(String s);
void digitalWriteLog(uint32_t ulPin, uint32_t ulVal);
void updateAdvertisingScanData();
void sensorValueChanged();
unsigned char getBatteryLevel(void);

static void gpio_init(void)
{
#ifdef ENABLE_LOG
  pinMode(PIN_LED_BLE, OUTPUT);
  pinMode(PIN_LED_SENSOR, OUTPUT);
#endif
  pinMode(PIN_SW_SENSOR, INPUT_PULLUP);

  attachInterruptLowAccuracy(digitalPinToInterrupt(PIN_SW_SENSOR), sensorValueChanged, FALLING);
}

void setup()
{
#ifdef ENABLE_LOG
  Serial.begin(115200);
#endif

  gpio_init();

  blePeripheral.setDeviceName(g_localName);
  blePeripheral.setLocalName(g_localName);

  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(batteryLevelCharacteristic);

  blePeripheral.setAdvertisedServiceUuid(mainBleService.uuid());
  blePeripheral.addAttribute(mainBleService);
  blePeripheral.addAttribute(sensorCharacteristic);

  blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL);
  blePeripheral.setEventHandler(BLEConnected, bleConnectedCallback);

  sensorCharacteristic.setValue(g_counterValue);
  sensorCharacteristic.setEventHandler(BLEWritten, sensorCharacteristicWrittenCallback);

  blePeripheral.begin();

  blePeripheral.setTxPower(TX_POWER);

  // Manualy updates advertising scan data
  updateAdvertisingScanData();

  // enable low power mode
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
}

void loop()
{
  log("Loop");

  log("Sleep");
  digitalWriteLog(PIN_LED_BLE, LOW);

  // Enter Low power mode
  sd_app_evt_wait();
  // Exit Low power mode

  log("Wake-up");
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);

  digitalWriteLog(PIN_LED_BLE, HIGH);

  if (g_sensorValueChanged)
  {
    g_sensorValueChanged = false;

    log("Sensor value changed");
    digitalWriteLog(PIN_LED_SENSOR, HIGH);
    delay(20);
    // Sets characteristic value
    sensorCharacteristic.setValue(g_counterValue);

    // Update advertising scan data with counter value
    updateAdvertisingScanData();

    digitalWriteLog(PIN_LED_SENSOR, LOW);
  }

  // poll peripheral
  blePeripheral.poll();
}

void bleConnectedCallback(BLECentral &bleCentral)
{
  unsigned char batteryLevel = getBatteryLevel();
  if (batteryLevel > 100)
  {
    batteryLevel = 100;
  }
  batteryLevelCharacteristic.setValue(batteryLevel);
}

void sensorValueChanged()
{
  uint32_t diff = millis() - g_lastSensorValueChanged;
  if (diff > DEBOUNCE_SENSOR_MS || diff < 0)
  {
    g_sensorValueChanged = true;
    g_counterValue = g_counterValue + 1;
  }
  g_lastSensorValueChanged = millis();
}

void sensorCharacteristicWrittenCallback(BLECentral &central, BLECharacteristic &characteristic)
{
  // The new value has already been written on characteristic
  // We still save it and flag the value as changed to allow the advertising packet to be updated
  g_sensorValueChanged = true;
  g_counterValue = *characteristic.value();
}

// https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
void updateAdvertisingScanData()
{
  unsigned char srData[31];
  unsigned char srDataLen = 0;
  int scanDataSize = 3;
  BLEEirData scanData[scanDataSize];

  // - Local name
  scanData[0].length = strlen(g_localName);
  scanData[0].type = 0x09;
  memcpy(scanData[0].data, g_localName, scanData[0].length);

  // - Tx Power
  scanData[1].length = 1;
  scanData[1].type = 0x0A;
  scanData[1].data[0] = TX_POWER;

  // - Manufacturer Data
  scanData[2].length = 2 + 4;
  scanData[2].type = 0xFF;
  // Manufacturer ID
  scanData[2].data[0] = 0xFF;
  scanData[2].data[1] = 0xFF;
  // Manufacturer data content
  scanData[2].data[2] = g_counterValue & 0xFF;
  scanData[2].data[3] = (g_counterValue >> 8) & 0xFF;
  scanData[2].data[4] = (g_counterValue >> 16) & 0xFF;
  scanData[2].data[5] = (g_counterValue >> 24) & 0xFF;

  if (scanDataSize && scanData)
  {
    for (int i = 0; i < scanDataSize; i++)
    {
      srData[srDataLen + 0] = scanData[i].length + 1;
      srData[srDataLen + 1] = scanData[i].type;
      srDataLen += 2;

      memcpy(&srData[srDataLen], scanData[i].data, scanData[i].length);

      srDataLen += scanData[i].length;
    }
  }

  // - Sets only avertising scan data
  sd_ble_gap_adv_data_set(NULL, 0, srData, srDataLen);
}

unsigned char getBatteryLevel(void)
{
  // Configure ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;

  NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
  NRF_ADC->TASKS_START = 1;

  while (!NRF_ADC->EVENTS_END)
  {
  }

  uint16_t vbg_in_mv = 1200;
  uint8_t adc_max = 255;
  uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;

  return (unsigned char)((vbat_current_in_mv * 100) / VBAT_MAX_IN_MV);
}

void log(String s)
{
#ifdef ENABLE_LOG
  Serial.println(s);
#endif
}

void digitalWriteLog(uint32_t ulPin, uint32_t ulVal)
{
#ifdef ENABLE_LOG
  digitalWrite(ulPin, ulVal);
#endif
}