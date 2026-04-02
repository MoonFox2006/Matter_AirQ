#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_matter.h>
#include <app/server/Server.h>

#undef INADDR_NONE
#include <Arduino.h>
#if ! CONFIG_ENABLE_CHIPOBLE
#include <WiFi.h>
#include "secrets.h"
#endif

#define UPDATE_INTERVAL   60000 // 1 min.

#define SED

#ifdef SED
enum { COMMISSIONED, COMMISSION_FAIL, NETWORK_CONNECTED, SESSION_ESTABLISHED, TEMP_ATTR_UPDATED, HUM_ATTR_UPDATED, PRESS_ATTR_UPDATED, CO2_ATTR_UPDATED };
#endif

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::identification;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

const uint8_t BTN_PIN = BOOT_PIN;
const uint8_t BTN_LEVEL = LOW;

const uint8_t LED_PIN = 22;
const uint8_t LED_LEVEL = LOW;

#ifdef SED
static EventGroupHandle_t matter_flags = NULL;
#endif
static volatile uint32_t buttonPressed = 0;
static float co2 = 450; // 450 PPM
static int16_t temp = 2000; // 20 C
static uint16_t hum = 2500; // 25%
static int16_t press = 1000; // 1000 hPa
static uint16_t endpoint_id = 0;

static void ARDUINO_ISR_ATTR btnISR() {
  static uint32_t lastTime = 0;

  if (digitalRead(BTN_PIN) != BTN_LEVEL) { // Button released
    uint32_t duration = millis() - lastTime;

    if (duration >= 50)
      buttonPressed = duration;
  }
  lastTime = millis();
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
  esp_err_t err = ESP_OK;

  log_v("Attribute update callback: type: %u, endpoint: %u, cluster: %u, attribute: %u, val: %u", type, endpoint_id, cluster_id, attribute_id, val->val.u32);
  switch (type) {
    case PRE_UPDATE:  // Callback before updating the value in the database
      log_v("Attribute update callback: PRE_UPDATE");
      break;
    case POST_UPDATE:  // Callback after updating the value in the database
      log_v("Attribute update callback: POST_UPDATE");
#ifdef SED
      if (endpoint_id == ::endpoint_id) {
        if ((cluster_id == TemperatureMeasurement::Id) && (attribute_id == TemperatureMeasurement::Attributes::MeasuredValue::Id)) {
          log_i("+TEMP_ATTR_UPDATED");
          xEventGroupSetBits(matter_flags, 1 << TEMP_ATTR_UPDATED);
        } else if ((cluster_id == RelativeHumidityMeasurement::Id) && (attribute_id == RelativeHumidityMeasurement::Attributes::MeasuredValue::Id)) {
          log_i("+HUM_ATTR_UPDATED");
          xEventGroupSetBits(matter_flags, 1 << HUM_ATTR_UPDATED);
        } else if ((cluster_id == PressureMeasurement::Id) && (attribute_id == PressureMeasurement::Attributes::MeasuredValue::Id)) {
          log_i("+PRESS_ATTR_UPDATED");
          xEventGroupSetBits(matter_flags, 1 << PRESS_ATTR_UPDATED);
        } else if ((cluster_id == CarbonDioxideConcentrationMeasurement::Id) && (attribute_id == CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id)) {
          log_i("+CO2_ATTR_UPDATED");
          xEventGroupSetBits(matter_flags, 1 << CO2_ATTR_UPDATED);
        }
      }
#endif
      break;
    case READ:  // Callback for reading the attribute value. This is used when the `ATTRIBUTE_FLAG_OVERRIDE` is set.
      log_v("Attribute update callback: READ");
      break;
    case WRITE:  // Callback for writing the attribute value. This is used when the `ATTRIBUTE_FLAG_OVERRIDE` is set.
      log_v("Attribute update callback: WRITE");
      break;
    default:
      log_v("Attribute update callback: Unknown type %d", type);
      break;
  }
  return err;
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant, void *priv_data) {
  esp_err_t err = ESP_OK;

  log_v("Identification callback to endpoint %d: type: %u, effect: %u, variant: %u", endpoint_id, type, effect_id, effect_variant);
  if (type == identification::callback_type_t::START) {
    log_v("Identification callback: START");
  } else if (type == identification::callback_type_t::EFFECT) {
    log_v("Identification callback: EFFECT");
  } else if (type == identification::callback_type_t::STOP) {
    log_v("Identification callback: STOP");
  }
  return err;
}

// This callback is invoked for all Matter events. The application can handle the events as required.
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
  switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
      log_v("Interface %s Address changed", event->InterfaceIpAddressChanged.Type == chip::DeviceLayer::InterfaceIpChangeType::kIpV4_Assigned ? "IPv4" : "IPV6");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
      log_v("Commissioning complete");
#ifdef SED
      log_v("+COMMISSIONED");
      xEventGroupSetBits(matter_flags, (1 << COMMISSIONED));
#endif
      break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
      log_v("Commissioning failed, fail safe timer expired");
#ifdef SED
      log_i("+COMMISSION_FAIL");
      xEventGroupSetBits(matter_flags, (1 << COMMISSION_FAIL));
#endif
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
      log_v("Commissioning session started");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
      log_v("Commissioning session stopped");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
      log_v("Commissioning window opened");
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
      log_v("Commissioning window closed");
      break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
      log_v("Fabric removed successfully");
      if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
        log_v("No fabric left, opening commissioning window");

        chip::CommissioningWindowManager &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
        constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);

        if (! commissionMgr.IsCommissioningWindowOpen()) {
          // After removing last fabric, it does not remove the Wi-Fi credentials and still has IP connectivity so, only advertising on DNS-SD.
          CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds, chip::CommissioningWindowAdvertisement::kDnssdOnly);

          if (err != CHIP_NO_ERROR) {
            log_e("Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
          }
        }
      }
      break;
    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
      log_v("Fabric will be removed");
      break;
    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
      log_v("Fabric is updated");
      break;
    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
      log_v("Fabric is committed");
      break;
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
      log_v("BLE deinitialized and memory reclaimed");
      break;

    case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
      log_v("WiFiConnectivityChange");
      break;
    case chip::DeviceLayer::DeviceEventType::kInternetConnectivityChange:
      log_v("InternetConnectivityChange");
#ifdef SED
      log_i("+NETWORK_CONNECTED");
      xEventGroupSetBits(matter_flags, (1 << NETWORK_CONNECTED));
#endif
      break;
    case chip::DeviceLayer::DeviceEventType::kServiceConnectivityChange:
      log_v("ServiceConnectivityChange");
      break;
    case chip::DeviceLayer::DeviceEventType::kTimeSyncChange:
      log_v("TimeSyncChange");
      break;
    case chip::DeviceLayer::DeviceEventType::kWiFiDeviceAvailable:
      log_v("WiFiDeviceAvailable");
      break;
    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkStarted:
      log_v("OperationalNetworkStarted");
      break;
    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkEnabled:
      log_v("OperationalNetworkEnabled");
      break;
    case chip::DeviceLayer::DeviceEventType::kServerReady:
      log_v("ServerReady");
      break;
    case chip::DeviceLayer::DeviceEventType::kSecureSessionEstablished:
      log_v("SecureSessionEstablished");
#ifdef SED
      if (xEventGroupGetBits(matter_flags) & (1 << NETWORK_CONNECTED)) {
        log_i("+SESSION_ESTABLISHED");
        xEventGroupSetBits(matter_flags, (1 << SESSION_ESTABLISHED));
      }
#endif
      break;
    default:
      break;
  }
}

static esp_matter::node_t *matter_init() {
  node::config_t node_config;
  node_t *result = nullptr;

  // Create a Matter node and add the mandatory Root Node device type on endpoint 0
  // node handle can be used to add/modify other endpoints.
  result = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
  if (! result) {
    log_e("Failed to create Matter node!");
  }
  return result;
}

static bool matter_begin() {
  /* Matter start */
  esp_err_t err = esp_matter::start(app_event_cb);

  if (err != ESP_OK) {
    log_e("Failed to start Matter, err: %d!", err);
    return false;
  }
  return true;
}

static bool matter_isDeviceCommissioned() {
  return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

static bool matter_isWiFiConnected() {
  return chip::DeviceLayer::ConnectivityMgr().IsWiFiStationConnected();
}

static bool matter_isThreadConnected() {
  return chip::DeviceLayer::ConnectivityMgr().IsThreadAttached();
}

static bool matter_isDeviceConnected() {
  return matter_isWiFiConnected() || matter_isThreadConnected();
}

static void matter_decommission() {
  esp_matter::factory_reset();
}

static uint16_t air_sensor_init() {
  air_quality_sensor::config_t air_quality_sensor_config;
  endpoint_t *endpoint;

  endpoint = air_quality_sensor::create(node::get(), &air_quality_sensor_config, ENDPOINT_FLAG_NONE, nullptr);
  if (endpoint) {
    return endpoint::get_id(endpoint);
  } else {
    log_e("Failed to create Air Quality Sensor endpoint!");
  }
  return 0xFFFF;
}

static bool temp_sensor_init(uint16_t endpoint_id, int16_t temp) {
  temperature_sensor::config_t temperature_sensor_config;

  temperature_sensor_config.temperature_measurement.measured_value = temp;
//  temperature_sensor_config.temperature_measurement.min_measured_value = nullptr;
//  temperature_sensor_config.temperature_measurement.max_measured_value = nullptr;

  if (temperature_sensor::add(endpoint::get(endpoint_id), &temperature_sensor_config) == ESP_OK) {
    return true;
  } else {
    log_e("Failed to add Temperature Measurement cluster!");
  }
  return false;
}

// Application cluster specification, 7.18.2.11. Temperature
// represents a temperature on the Celsius scale with a resolution of 0.01°C.
// temp = (temperature in °C) x 100
static bool temp_sensor_update(uint16_t endpoint_id, int16_t temp) {
  esp_matter_attr_val_t val = esp_matter_invalid(NULL);
  attribute_t *attribute = attribute::get(endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id);

  if (attribute) {
    attribute::get_val(attribute, &val);
    val.val.i16 = temp;
    return attribute::update(endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val) == ESP_OK;
  }
  return false;
}

static bool hum_sensor_init(uint16_t endpoint_id, uint16_t hum) {
  humidity_sensor::config_t humidity_sensor_config;

  humidity_sensor_config.relative_humidity_measurement.measured_value = hum;
//  humidity_sensor_config.relative_humidity_measurement.min_measured_value = nullptr;
//  humidity_sensor_config.relative_humidity_measurement.max_measured_value = nullptr;

  if (humidity_sensor::add(endpoint::get(endpoint_id), &humidity_sensor_config) == ESP_OK) {
    return true;
  } else {
    log_e("Failed to add Humidity Measurement cluster!");
  }
  return false;
}

static bool hum_sensor_update(uint16_t endpoint_id, uint16_t hum) {
  esp_matter_attr_val_t val = esp_matter_invalid(NULL);
  attribute_t *attribute = attribute::get(endpoint_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id);

  if (attribute) {
    attribute::get_val(attribute, &val);
    val.val.u16 = hum;
    return attribute::update(endpoint_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val) == ESP_OK;
  }
  return false;
}

static bool press_sensor_init(uint16_t endpoint_id, int16_t press) {
  pressure_sensor::config_t pressure_sensor_config;

  pressure_sensor_config.pressure_measurement.pressure_measured_value = press;
//  pressure_sensor_config.pressure_measurement.pressure_min_measured_value = nullptr;
//  pressure_sensor_config.pressure_measurement.pressure_max_measured_value = nullptr;

  if (pressure_sensor::add(endpoint::get(endpoint_id), &pressure_sensor_config) == ESP_OK) {
    return true;
  } else {
    log_e("Failed to add Pressure Measurement cluster!");
  }
  return false;
}

static bool press_sensor_update(uint16_t endpoint_id, int16_t press) {
  esp_matter_attr_val_t val = esp_matter_invalid(NULL);
  attribute_t *attribute = attribute::get(endpoint_id, PressureMeasurement::Id, PressureMeasurement::Attributes::MeasuredValue::Id);

  if (attribute) {
    attribute::get_val(attribute, &val);
    val.val.i16 = press;
    return attribute::update(endpoint_id, PressureMeasurement::Id, PressureMeasurement::Attributes::MeasuredValue::Id, &val) == ESP_OK;
  }
  return false;
}

static bool co2_sensor_init(uint16_t endpoint_id, float co2) {
  cluster::carbon_dioxide_concentration_measurement::config_t co2_sensor_config;
  cluster_t *cluster = cluster::carbon_dioxide_concentration_measurement::create(endpoint::get(endpoint_id), &co2_sensor_config, CLUSTER_FLAG_SERVER);

  if (cluster) {
    cluster::carbon_dioxide_concentration_measurement::attribute::create_measured_value(cluster, co2);
    cluster::carbon_dioxide_concentration_measurement::attribute::create_measurement_unit(cluster, chip::to_underlying(CarbonDioxideConcentrationMeasurement::MeasurementUnitEnum::kPpm));
    return true;
  } else {
    log_e("Failed to add CO2 Concentration Measurement cluster!");
  }
  return false;
}

static bool co2_sensor_update(uint16_t endpoint_id, float co2) {
  esp_matter_attr_val_t val = esp_matter_invalid(NULL);
  attribute_t *attribute = attribute::get(endpoint_id, CarbonDioxideConcentrationMeasurement::Id, CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id);

  if (attribute) {
    attribute::get_val(attribute, &val);
    val.val.f = co2;
    return attribute::update(endpoint_id, CarbonDioxideConcentrationMeasurement::Id, CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id, &val) == ESP_OK;
  }
  return false;
}

static bool update_sensors(uint16_t endpoint_id) {
  bool result;

  digitalWrite(LED_PIN, LED_LEVEL);
  if (random(100) < 50)
    temp -= random(100);
  else
    temp += random(100);
  if (random(100) < 50)
    hum -= random(100);
  else
    hum += random(100);
  if (random(100) < 50)
    press -= random(10);
  else
    press += random(10);
  if (random(100) < 50)
    co2 -= random(100) / 10.0;
  else
    co2 += random(100) / 10.0;
  Serial.printf("Temperature %.2f C, humidity %.2f%%, pressure %d hPa, CO2 %.0f PPM\r\n", temp / 100.0, hum / 100.0, press, co2);
#ifdef SED
  xEventGroupClearBits(matter_flags, (1 << TEMP_ATTR_UPDATED) | (1 << HUM_ATTR_UPDATED) | (1 << PRESS_ATTR_UPDATED) | (1 << CO2_ATTR_UPDATED));
#endif
  result = temp_sensor_update(endpoint_id, temp);
  result = hum_sensor_update(endpoint_id, hum) && result;
  result = press_sensor_update(endpoint_id, press) && result;
  result = co2_sensor_update(endpoint_id, co2) && result;
#ifdef SED
  result = ((xEventGroupWaitBits(matter_flags, (1 << TEMP_ATTR_UPDATED) | (1 << HUM_ATTR_UPDATED) | (1 << PRESS_ATTR_UPDATED) | (1 << CO2_ATTR_UPDATED), pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)) &
    ((1 << TEMP_ATTR_UPDATED) | (1 << HUM_ATTR_UPDATED) | (1 << PRESS_ATTR_UPDATED) | (1 << CO2_ATTR_UPDATED))) == ((1 << TEMP_ATTR_UPDATED) | (1 << HUM_ATTR_UPDATED) | (1 << PRESS_ATTR_UPDATED) | (1 << CO2_ATTR_UPDATED))) && result;
#endif
  digitalWrite(LED_PIN, ! LED_LEVEL);
  return result;
}

void setup() {
  delay(1000);
  Serial.begin(115200);

#ifdef SED
  matter_flags = xEventGroupCreate();
  assert(matter_flags);
#endif

  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(BTN_PIN, btnISR, CHANGE);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ! LED_LEVEL);

#if ! CONFIG_ENABLE_CHIPOBLE
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  Serial.printf("Connecting to \"%s\"...\r\n", WIFI_SSID);
#endif

  assert(matter_init());

  endpoint_id = air_sensor_init();
  assert(endpoint_id != 0xFFFF);
  assert(temp_sensor_init(endpoint_id, temp));
  assert(hum_sensor_init(endpoint_id, hum));
  assert(press_sensor_init(endpoint_id, press));
  assert(co2_sensor_init(endpoint_id, co2));

  assert(matter_begin());

#ifdef SED
  // Check Matter Accessory Commissioning state, which may change during execution of loop()
  if (! matter_isDeviceCommissioned()) {
    Serial.println();
    Serial.println("Matter Node is not commissioned yet.");
    Serial.println("Initiate the device discovery in your Matter environment.");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\r\n", "34970112332");
    Serial.printf("QR code URL: %s\r\n", "https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT:Y.K9042C00KA0648G00");

    while (! (xEventGroupWaitBits(matter_flags, (1 << COMMISSIONED), pdTRUE, pdTRUE, pdMS_TO_TICKS(100)) & (1 << COMMISSIONED))) {
      digitalWrite(LED_PIN, ! digitalRead(LED_PIN));
    }
    digitalWrite(LED_PIN, ! LED_LEVEL);
    Serial.println("Matter Node is commissioned.");
  }

  if (! matter_isDeviceConnected()) {
    Serial.print("Waiting for network");
    while (! matter_isDeviceConnected()) {
      digitalWrite(LED_PIN, ! digitalRead(LED_PIN));
      Serial.print('.');
      delay(250);
    }
    digitalWrite(LED_PIN, ! LED_LEVEL);
    Serial.println(" OK");
  }

  if (xEventGroupWaitBits(matter_flags, (1 << SESSION_ESTABLISHED), pdTRUE, pdTRUE, pdMS_TO_TICKS(30000))) {
    if (update_sensors(endpoint_id)) {
      Serial.println("Matter attributes updated");
      delay(esp_reset_reason() == ESP_RST_DEEPSLEEP ? 1000 : 2000);
    }
  }

  Serial.println("Entering deep sleep...");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(UPDATE_INTERVAL * 1000UL);
  esp_deep_sleep_disable_rom_logging();
  esp_deep_sleep_start();
#endif
}

void loop() {
#ifndef SED
  static uint32_t lastUpdate = 0;

  if (buttonPressed) {
    if (buttonPressed >= 2000) { // 2 sec. to decommission
      Serial.println("Decommissioning Temperature Sensor Matter Accessory. It shall be commissioned again.");
      matter_decommission();
    } else {
      update_sensors(endpoint_id);
      lastUpdate = millis();
    }
    buttonPressed = 0;
  }

  // Check Matter Accessory Commissioning state, which may change during execution of loop()
  if (! matter_isDeviceCommissioned()) {
    Serial.println();
    Serial.println("Matter Node is not commissioned yet.");
    Serial.println("Initiate the device discovery in your Matter environment.");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\r\n", "34970112332");
    Serial.printf("QR code URL: %s\r\n", "https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT:Y.K9042C00KA0648G00");

    uint32_t timeCount = 0;

    // waits for Matter Temperature Sensor Commissioning.
    while (! matter_isDeviceCommissioned()) {
      if (buttonPressed >= 2000)
        break;
      delay(100);
      digitalWrite(LED_PIN, ! digitalRead(LED_PIN));
      if ((timeCount++ % 50) == 0) { // 50*100ms = 5 sec
        Serial.println("Matter Node not commissioned yet. Waiting for commissioning.");
      }
    }
    digitalWrite(LED_PIN, ! LED_LEVEL);
    if (matter_isDeviceCommissioned())
      Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
  }

  if (! matter_isDeviceConnected()) {
    digitalWrite(LED_PIN, (millis() % 250 < 50) == LED_LEVEL);
  } else {
    digitalWrite(LED_PIN, ! LED_LEVEL);
  }

  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    update_sensors(endpoint_id);
    lastUpdate = millis();
  }
#endif
}
