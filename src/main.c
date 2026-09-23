/*
 * ============================================================================
 *      ESP32-H2 — Контроллер моторизованной шторы, чистый ESP-IDF
 * ============================================================================
 *
 * Полный перенос с framework=arduino на framework=espidf. Причина и вся
 * методология — см. esp32h2_light_sleep_test/roadmap.md (Этап 8-13) и
 * esp32h2_light_sleep_test/src/main.c (тот же паттерн сна, оттуда перенесён
 * практически без изменений).
 *
 * ГЛАВНОЕ АРХИТЕКТУРНОЕ ОТЛИЧИЕ ОТ Arduino-ВЕРСИИ
 * -------------------------------------------------
 * В Arduino-версии (см. git: `git show 4187ddb8:esp32h2_light_sleep/src/main.cpp`)
 * сон был на ручном таймере (`esp_light_sleep_start()` в `loop()` каждые
 * 1.5с) ОДНОВРЕМЕННО с `esp_zb_sleep_enable(true)` — два независимых
 * механизма сна, конфликтовавших друг с другом и приводивших к рассинхрону
 * `keep_alive`-опроса координатора и отвалу устройства от сети за ночь.
 * Здесь сном управляет ИСКЛЮЧИТЕЛЬНО сигнал `ESP_ZB_COMMON_SIGNAL_CAN_SLEEP`
 * в `esp_zb_app_signal_handler()` — никакого собственного таймера сна и
 * никакого `loop()` с поллингом в этом файле нет вообще.
 *
 * Реальный light sleep (0.07-0.17мА, подтверждено амперметром на тестовом
 * стенде) требует вызова `esp_pm_configure(light_sleep_enable=true)` — без
 * него автоматический tickless-idle путь никогда не выбирает
 * PM_MODE_LIGHT_SLEEP, независимо от sdkconfig-флагов (см. app_main()).
 *
 * ПЕРЕНОС LED-АВТОМАТА — НЕ ПРОВЕРЕН НА ЖЕЛЕЗЕ ДО ЭТОГО ПОРТА
 * -------------------------------------------------------------
 * Форма конечного автомата (LEDPattern/ledRefresh/ledComputeDesired) взята
 * из незакоммиченной рабочей копии Arduino-версии (не из последнего коммита
 * 4187ddb8, где ещё был старый поллинговый blinkLED()) — пользователь
 * подтвердил, что это не проверено на реальном железе, но такая форма всё
 * равно нужна: поллинг с фиксированной длительностью (delay-based blink)
 * не совместим с архитектурой сна по CAN_SLEEP (см. правило проекта про
 * поллинг в CLAUDE.md).
 *
 * НОВЫЕ ЗАВИСИМОСТИ (idf_component.yml), не угаданы — версии/API проверены
 * по официальным заголовкам/исходникам перед использованием:
 *   - espressif/button (iot_button)  — 3 физические кнопки, поддержка
 *     power-save для GPIO-кнопок.
 *   - espressif/led_strip (RMT)      — WS2812, замена Adafruit_NeoPixel.
 *
 * OTA-ОБНОВЛЕНИЕ ПО ZIGBEE (через Z2M) — как выпустить новую версию
 * -------------------------------------------------------------------
 * Устройство стоит в корпусе без доступа к USB — после первой прошивки по
 * кабелю (см. ниже) все следующие обновления идут по воздуху. Подробности,
 * найденные грабли и разбор реального обновления — roadmap.md, Этап 21,
 * и SKILL.md (раздел "Zigbee OTA").
 *
 *   1. Поднять FW_FILE_VERSION ниже (== код версии; Z2M ставит образ, только
 *      если его версия строго больше текущей на устройстве).
 *   2. `pio run` — после сборки `tools/post_build_ota.py` (extra_scripts в
 *      platformio.ini) сам пересобирает tools/ota_out/ota/<файл>.ota и
 *      tools/ota_out/my_index.json из свежего firmware.bin (номер версии,
 *      OTA_MANUFACTURER_CODE, OTA_IMAGE_TYPE читает прямо из этого файла —
 *      отдельно вызывать make_ota.py вручную не нужно).
 *   3. Скопировать оба файла (my_index.json и папку ota/) в каталог данных
 *      Z2M — туда же, где лежит его configuration.yaml (в HA-аддоне это
 *      /config/zigbee2mqtt/); "url" в индексе уже указывает на "ota/<файл>".
 *      Если ota.zigbee_ota_override_index_location в конфиге ещё не задан —
 *      задать один раз через интерфейс Z2M (Настройки -> OTA), не правкой
 *      файла на лету: работающий Z2M перезаписывает configuration.yaml из
 *      памяти и правка потеряется.
 *   4. В Z2M на карточке устройства: "Проверить обновление" -> "Обновить".
 *      Батарея должна быть заряжена (защита в ota_upgrade_handler() не
 *      начнёт приём ниже ~70%, OTA_MIN_BATTERY_VOLTAGE) — передача ~620КБ
 *      идёт по поллингу заметное время (по факту первого прогона — около
 *      двух часов) с постоянно включённым радио.
 *
 * Первая OTA-совместимая прошивка (с CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
 * заливается по кабелю один раз — это опция загрузчика, по воздуху себя не
 * обновляет. Неудачная OTA-прошивка (не вышла в сеть) откатывается сама
 * (esp_ota_mark_app_invalid_rollback_and_reboot() по таймеру, см.
 * OTA_VALIDATE_TIMEOUT_MS) — кабель для этого случая не нужен.
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "led_strip.h"

static const char *TAG = "blinds";

// ========== НАСТРОЙКИ ОТЛАДКИ ==========
#define DEBUG_ENCODER true // true = выводить отладку энкодера

// ========== ПИНЫ ДРАЙВЕРА DRV8871 ==========
#define MOTOR_IN1_PIN GPIO_NUM_10 // ШИМ канал A (управление скоростью/направлением)
#define MOTOR_IN2_PIN GPIO_NUM_11 // ШИМ канал B (управление скоростью/направлением)

// ========== ПИН ЭНКОДЕРА ==========
#define ENCODER_PIN GPIO_NUM_5 // GPIO05 для энкодера

// ========== ПАРАМЕТРЫ ШИМ ==========
#define PWM_FREQUENCY 20000                             // 20 кГц (выше слышимого диапазона, не гудит)
#define PWM_RESOLUTION LEDC_TIMER_10_BIT                 // 10 бит (0-1023)
#define PWM_MAX_DUTY 1023        // (1<<10)-1 — аппаратный потолок скважности ШИМ

// ========== ПАРАМЕТРЫ МОТОРА ==========
// MAX/MIN_PWM_DUTY настраиваются из Z2M на лету (см. CUSTOM_SETTINGS_CLUSTER_ID) — правка на реальном
// изделии в корпусе иначе требует перезаливки по OTA (~2 часа на образ). Значения ниже — только дефолты
// на первую загрузку/если в NVS ничего не сохранено; runtime-значения — s_min_pwm_duty/s_max_pwm_duty.
#define MAX_PWM_DUTY_DEFAULT 900 // Максимальная командная скорость
#define MIN_PWM_DUTY_DEFAULT 750 // Минимальная скорость
#define TRASHHOLD_PWM_PILSE 800  // Кол-во импульсов за которые снижаем скорость

// ========== ПИНЫ КНОПОК ==========
#define BUTTON_UP_PIN GPIO_NUM_1   // Кнопка UP (замыкает на GND)
#define BUTTON_DOWN_PIN GPIO_NUM_2 // Кнопка DOWN (замыкает на GND)
#define BUTTON_SET_PIN GPIO_NUM_3  // Кнопка SET (замыкает на GND)

// ========== ПАРАМЕТРЫ КНОПОК ==========
#define LONG_PRESS_MS 3000 // Долгое нажатие 3 секунды

// ========== ПАРАМЕТРЫ ИМПУЛЬСНОГО ДАТЧИКА ==========
#define TOTAL_PULSES_LIMIT 500000 // Абсолютно максимальное количество импульсов (предел) или дефолтный предел

// ========== ПАРАМЕТРЫ СВЕТОДИОДА WS2812 ==========
#define LED_PIN GPIO_NUM_14 // Пин для WS2812
#define NUM_LEDS 1          // Количество светодиодов
#define LED_BRIGHTNESS_DIV 85 // Яркость ~3/255 из Arduino-версии — здесь делим итоговый цвет на этот коэффициент

// ========== КОНФИГУРАЦИЯ ZIGBEE ==========
#define ZIGBEE_ENDPOINT 10 // Zigbee endpoint

// ========== НАСТРОЙКИ МОТОРА, ИЗМЕНЯЕМЫЕ ИЗ Z2M (кастомный кластер) ==========
// 0xFC00 — начало приватного (manufacturer-specific range) диапазона ID кластеров по Zigbee-спеке,
// стандартный выбор для собственных кластеров у самодельных устройств (device+converter под своим
// контролем, wildcard-manufacturer-code — тот же паттерн уже видели в issues esp-zigbee-sdk). Читаются/
// пишутся как обычный ACCESS_READ_WRITE атрибут — постоянного отчёта (ACCESS_REPORTING) им не нужно.
#define CUSTOM_SETTINGS_CLUSTER_ID 0xFC00
#define ATTR_MIN_PWM_DUTY_ID 0x0000
#define ATTR_MAX_PWM_DUTY_ID 0x0001
#define ATTR_MOTOR_INVERT_ID 0x0002

// ========== OTA-ОБНОВЛЕНИЕ ПО ZIGBEE (Z2M) ==========
// Версия прошивки в формате OTA file_version — ПОВЫШАТЬ НА КАЖДЫЙ РЕЛИЗ: Z2M не ставит образ с версией <= текущей
// без force. Те же значения (FW_FILE_VERSION/OTA_MANUFACTURER_CODE/OTA_IMAGE_TYPE) нужны в tools/make_ota.py
// и в записи индекса Z2M — иначе Z2M не сопоставит образ с устройством.
#define FW_FILE_VERSION 0x00000004
#define OTA_MANUFACTURER_CODE 0x131B // = ESP_ZB_OTA_UPGRADE_MANUFACTURER_CODE_DEF_VALUE (Espressif)
#define OTA_IMAGE_TYPE 0x0001        // свой тип образа (0x0000-0xffbf — manufacturer specific)
#define OTA_HW_VERSION 0x0001
#define OTA_MAX_DATA_SIZE 64                     // размер блока данных OTA (дефолт примера esp-zigbee-sdk)
#define OTA_MIN_BATTERY_VOLTAGE 7.6f             // ~70% по batteryVoltageToPercent(); Z2M советует 70%+ для OTA батарейных устройств
#define OTA_VALIDATE_TIMEOUT_MS (10 * 60 * 1000) // не подтвердили новую прошивку за это время после загрузки — откат

#define NVS_NAMESPACE "storage"

// ========== ПЕРЕМЕННЫЕ АЦП ==========
#define ADC_BAT_PIN GPIO_NUM_4  // пин входа АЦП с делителя батареи
#define ADC_BAT_CHANNEL ADC_CHANNEL_3 // GPIO4 = ADC1_GPIO4_CHANNEL (soc/adc_channel.h, esp32h2)
#define BUZZER_PIN GPIO_NUM_12  // пин пищалки (буззера)
#define MOSFET_PIN GPIO_NUM_13  // ON_OFF_PWR по схеме: через VT3->VT2 включает линию 3V3_ON_OFF, от которой запитаны WS2812 (LED) и разъём ENCODER — не мотор (тот питается отдельно, напрямую от PWR_MOTOR)

// ========== ВРЕМЯ (замена millis()) ==========
static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Печать аптайма в читаемом виде ч:мм:сс — для сверки момента события
// (нажатие кнопки, команда из Z2M) с многочасовым/ночным логом стабильности.
static void logUptime(const char *prefix)
{
    int64_t uptime_s = esp_timer_get_time() / 1000000;
    ESP_LOGI(TAG, "[%s] Uptime: %lld:%02lld:%02lld (h:m:s)", prefix, uptime_s / 3600, (uptime_s / 60) % 60, uptime_s % 60);
}

// ========== НАПРАВЛЕНИЕ МОТОРА / СОСТОЯНИЯ СИСТЕМЫ ==========

typedef enum {
    DIR_NONE = 0,
    DIR_OPEN = 1,
    DIR_CLOSE = 2,
} motor_direction_t;

typedef enum {
    STATE_NORMAL = 0,
    STATE_CALIBRATION = 1,
    STATE_CALIBRATION_UP = 2,
    STATE_CALIBRATION_DOWN = 3,
    STATE_PAIRING = 4,
} system_state_t;

// ========== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

static motor_direction_t motor_direction = DIR_NONE;
static volatile bool motor_invert = false; // false — не инвертировано; true — инвертировано

// Runtime-настройки мотора, изменяемые из Z2M (см. CUSTOM_SETTINGS_CLUSTER_ID) без перезаливки прошивки.
// Дефолты — из #define выше, реальные значения после первого сохранения читаются из NVS в
// loadMotorSettings(). motor_invert выше персистится тем же способом, но остаётся отдельной volatile —
// её меняют ещё и из ISR-смежного контекста кнопок, тип менять не стали.
static uint16_t s_min_pwm_duty = MIN_PWM_DUTY_DEFAULT;
static uint16_t s_max_pwm_duty = MAX_PWM_DUTY_DEFAULT;

static int32_t current_pulse_position = 0;     // текущее кол-во импульсов с датчика оборотов (положение шторы)
static int32_t current_max_pulse_position = 0; // максимальное кол-во импульсов (лимит перемещения)
static int32_t target_pulse_position = 0;      // к какому кол-ву импульсов нужно двигать мотор

static int32_t calibration_pulse_position = 0; // счётчик импульсов в режиме калибровки

static volatile bool pulse_detected = false; // флаг, что в ISR обновили счётчик импульсов

static bool zigbee_connected = false;
static bool zigbee_connected_old = false;

// Длительность льготного периода после джойна — см. s_stay_awake_until_ms ниже.
// Было 20000 — на практике интервью Z2M иногда не укладывалось в это время.
#define JOIN_GRACE_PERIOD_MS 60000

// Льготный период после успешного джойна, в течение которого CAN_SLEEP
// перехватывается так же, как при !zigbee_connected — см. комментарий у
// esp_zb_app_signal_handler()/STEERING про причину (Z2M ZDO Active
// Endpoints Request проваливался с "can not get active endpoints", если
// чип успевал уйти в реальный сон раньше, чем координатор успевал
// опросить endpoint'ы/кластеры сразу после джойна).
static uint32_t s_stay_awake_until_ms = 0;

// Идёт приём OTA-образа: не даём чипу спать (перехват CAN_SLEEP, как у мотора) и не двигаем мотор.
static volatile bool s_ota_in_progress = false;

// Текущая прошивка — только что установленная по OTA и ещё не подтверждённая (ESP_OTA_IMG_PENDING_VERIFY,
// возможно только при CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y). Пока не подтверждена — при любом перезапуске
// загрузчик откатится на предыдущую; см. ota_confirm_new_image_if_needed().
static bool s_ota_pending_verify = false;
static esp_timer_handle_t s_ota_validate_timer = NULL;

// Не даём стеку спать, пока не подтверждён первый успешный join к сети —
// см. предупреждение про issue #787 в esp32h2_light_sleep_test/src/main.c.
static bool s_steering_done = false;

static bool motor_is_running = false;

static system_state_t system_state = STATE_NORMAL;
static float battery_voltage = 0; // напряжение на АКБ, вольт

// Таймер "быстрого поллинга" (лимиты мотора/энкодер) — тикает ТОЛЬКО пока
// мотор крутится или мы в режиме калибровки/пейринга, не постоянно. Создан
// после того, как на реальном железе подтвердилось: безусловный периодический
// esp_timer (100мс) не даёт стеку Zigbee вообще ни разу просигналить
// CAN_SLEEP — тот же класс проблемы, что и ручной poll в Arduino-loop(),
// просто через другой механизм. См. poll_timer_sync().
static esp_timer_handle_t s_motor_poll_timer = NULL;
static esp_timer_handle_t s_led_blink_timer = NULL;

// esp_zb_zcl_report_attr_cmd_req() валил стек в "Zigbee stack assertion
// failed zcl/zcl_general_commands.c:612" при каждом вызове, независимо от
// потока (Zigbee-стек, кнопка, даже esp_timer) — реальная причина оказалась
// не в контексте вызова, а в незаполненном поле .direction (см.
// report_defer_timer_cb()). Отправка всё равно оставлена отложенной через
// этот одноразовый таймер — единая точка для будущей отладки/логирования,
// лишним не мешает.
static esp_timer_handle_t s_report_defer_timer = NULL;
static volatile bool s_report_pending = false;

// Не даёт чипу входить в light sleep (ни в наш явный esp_zb_sleep_now(), ни
// в автоматический tickless idle) на время работы мотора — см. комментарий
// у initPWM()/motorSetDirectionAndSpeed(). Держится строго между
// motorStart() и motorStop(), рекурсивный (acquire/release должны быть
// сбалансированы).
static esp_pm_lock_handle_t s_motor_pm_lock = NULL;

// ========== БУЗЗЕР ==========

static void shortBeep(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 0);
}

static void longBeep(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BUZZER_PIN, 0);
}

static void doubleBeep(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 0);
}

// ========== ФУНКЦИИ РАБОТЫ С ПАМЯТЬЮ (NVS) ==========

static void savePosition(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        err = nvs_set_i32(nvs_handle, "pulse_position", current_pulse_position);
        ESP_LOGI(TAG, "[NVS] set pulse_position = %s", esp_err_to_name(err));
        err = nvs_set_i32(nvs_handle, "pulse_max_count", current_max_pulse_position);
        ESP_LOGI(TAG, "[NVS] set pulse_max_count = %s", esp_err_to_name(err));
        err = nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "[NVS] commit = %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Position saved: %d / %d", (int)current_pulse_position, (int)current_max_pulse_position);
    } else {
        ESP_LOGW(TAG, "Position save error: %d / %d", (int)current_pulse_position, (int)current_max_pulse_position);
    }
}

static void loadPosition(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    int32_t saved_pulse_pos = 0;
    int32_t saved_max_pulses = 0;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle) == ESP_OK) {
        err = nvs_get_i32(nvs_handle, "pulse_position", &saved_pulse_pos);
        ESP_LOGI(TAG, "[NVS] get pulse_position = %s", esp_err_to_name(err));
        err = nvs_get_i32(nvs_handle, "pulse_max_count", &saved_max_pulses);
        ESP_LOGI(TAG, "[NVS] get pulse_max_count = %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
    }
    ESP_LOGI(TAG, "[NVS] loaded pulse_position = %d, pulse_max_count = %d", (int)saved_pulse_pos, (int)saved_max_pulses);

    current_max_pulse_position = saved_max_pulses;
    if (current_max_pulse_position == 0) current_max_pulse_position = TOTAL_PULSES_LIMIT;
    if (current_max_pulse_position > TOTAL_PULSES_LIMIT) current_max_pulse_position = TOTAL_PULSES_LIMIT;

    current_pulse_position = saved_pulse_pos;
    if (current_pulse_position < 0) current_pulse_position = 0;
    if (current_pulse_position > current_max_pulse_position) current_pulse_position = current_max_pulse_position;

    target_pulse_position = current_pulse_position;

    ESP_LOGI(TAG, "Loaded: pulse_pos=%d, max_pulse_pos=%d", (int)current_pulse_position, (int)current_max_pulse_position);
}

/**
 * @brief Настройки мотора (min/max PWM duty, инверсия), меняемые из Z2M — см. CUSTOM_SETTINGS_CLUSTER_ID и
 * settings_attr_write_handler(). Отдельно от savePosition()/loadPosition(): другая частота изменения (редко,
 * по решению пользователя) и другой набор ключей NVS.
 */
static void saveMotorSettings(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        nvs_set_u16(nvs_handle, "min_pwm_duty", s_min_pwm_duty);
        nvs_set_u16(nvs_handle, "max_pwm_duty", s_max_pwm_duty);
        nvs_set_u8(nvs_handle, "motor_invert", motor_invert ? 1 : 0);
        esp_err_t err = nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "[NVS] Motor settings saved: min=%u max=%u invert=%d (commit=%s)", s_min_pwm_duty, s_max_pwm_duty,
                 (int)motor_invert, esp_err_to_name(err));
    }
}

static void loadMotorSettings(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle) == ESP_OK) {
        uint16_t v16;
        if (nvs_get_u16(nvs_handle, "min_pwm_duty", &v16) == ESP_OK) s_min_pwm_duty = v16;
        if (nvs_get_u16(nvs_handle, "max_pwm_duty", &v16) == ESP_OK) s_max_pwm_duty = v16;
        uint8_t v8;
        if (nvs_get_u8(nvs_handle, "motor_invert", &v8) == ESP_OK) motor_invert = (v8 != 0);
        nvs_close(nvs_handle);
    }
    // Аппаратный потолок (PWM_MAX_DUTY, 10 бит) уже клампится в motorSetDirectionAndSpeed(); здесь только
    // защита от совсем бессмысленных значений (min > max), если кто-то запишет из Z2M криво.
    if (s_min_pwm_duty > s_max_pwm_duty) s_min_pwm_duty = s_max_pwm_duty;
    ESP_LOGI(TAG, "Motor settings: min_pwm_duty=%u max_pwm_duty=%u motor_invert=%d", s_min_pwm_duty, s_max_pwm_duty,
             (int)motor_invert);
}

// ========== ОБРАБОТЧИК ПРЕРЫВАНИЯ ЭНКОДЕРА ==========

// IRAM_ATTR: обработчик может выполняться, пока обычный код во флеше временно
// недоступен (например, во время операций с флеш-памятью) — как и в
// Arduino-версии.
static void IRAM_ATTR encoderISR(void *arg)
{
    pulse_detected = true;

    if (system_state == STATE_NORMAL) {
        int32_t val = current_pulse_position;
        if (motor_direction == DIR_OPEN) {
            val++;
        } else if (motor_direction == DIR_CLOSE) {
            val--;
        }
        current_pulse_position = val;
    } else if (system_state == STATE_CALIBRATION_DOWN) {
        int32_t val = calibration_pulse_position;
        if (motor_direction == DIR_OPEN) {
            val++;
        } else if (motor_direction == DIR_CLOSE) {
            val--;
        }
        calibration_pulse_position = val;
    }
}

static void initEncoder(void)
{
    gpio_config_t encoder_cfg = {
        .pin_bit_mask = 1ULL << ENCODER_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&encoder_cfg);
    // На ESP32-H2 при включённом esp_pm (light sleep) CONFIG_PM_SLP_DISABLE_GPIO
    // включён по умолчанию: на КАЖДЫЙ вход/выход из light sleep (в т.ч. из
    // автоматического tickless idle, не только по нашему CAN_SLEEP) все GPIO
    // переключаются между рабочей конфигурацией (pull-up, input, interrupt) и
    // "спящей" (floating, direction disabled) для экономии ~200-300мкА. Именно
    // эти переключения pull-up на GPIO5 давали ложные срабатывания ISR при
    // подключённом к 3.3В/GND проводе — подтверждено сравнением с Arduino-
    // версией (там esp_pm_configure() не работал вовсе, поэтому переключений
    // не было и ложных импульсов тоже). gpio_sleep_sel_dis() исключает этот
    // пин из автопереключения — конфигурация остаётся одинаковой всегда.
    gpio_sleep_sel_dis(ENCODER_PIN);
    // gpio_install_isr_service() вызывается один раз в app_main() (общий ISR-
    // сервис на все пины, включая кнопки — см. ниже), здесь только
    // регистрируем обработчик конкретно для этого пина.
    gpio_isr_handler_add(ENCODER_PIN, encoderISR, NULL);
    ESP_LOGI(TAG, "[Encoder] Interrupt on GPIO%d, FALLING", ENCODER_PIN);
}

// ========== АЦП БАТАРЕИ ==========

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;

static void initBatteryADC(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12, // полный диапазон ~0-3.3В (ADC_ATTEN_DB_11 — то же самое, но deprecated)
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_BAT_CHANNEL, &chan_cfg));

    // Curve Fitting — единственная схема калибровки, поддерживаемая ESP32-H2
    // (SOC_ADC_CALIBRATION_V1_SUPPORTED=1 в soc_caps.h, Line Fitting не
    // поддерживается на этом чипе).
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_BAT_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t cali_err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handle);
    if (cali_err != ESP_OK) {
        ESP_LOGW(TAG, "[ADC] Calibration scheme unavailable: %s — battery readings uncalibrated", esp_err_to_name(cali_err));
        s_adc_cali_handle = NULL;
    }
    ESP_LOGI(TAG, "[ADC] Battery ADC initialized");
}

/**
 * @brief Измеряет напряжение аккумуляторной батареи 2S Li-Ion.
 *
 * Несколько измерений встроенным АЦП ESP32-H2, усреднение, пересчёт в
 * напряжение батареи с учётом резистивного делителя. Формула и калибровка
 * делителя — без изменений из Arduino-версии.
 *
 * @return Напряжение батареи в формате Zigbee (0.1В на единицу), например
 *         8.40В -> 84.
 */
static uint8_t readBatteryVoltage(void)
{
    int mv;

    // Первое измерение отбрасываем (может быть некорректным сразу после
    // подключения делителя) — как и в Arduino-версии.
    if (s_adc_cali_handle) {
        adc_oneshot_get_calibrated_result(s_adc_handle, s_adc_cali_handle, ADC_BAT_CHANNEL, &mv);
    } else {
        int raw;
        adc_oneshot_read(s_adc_handle, ADC_BAT_CHANNEL, &raw);
    }

    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        if (s_adc_cali_handle) {
            adc_oneshot_get_calibrated_result(s_adc_handle, s_adc_cali_handle, ADC_BAT_CHANNEL, &mv);
        } else {
            int raw;
            adc_oneshot_read(s_adc_handle, ADC_BAT_CHANNEL, &raw);
            mv = raw; // без калибровки — грубая оценка, лучше, чем ничего
        }
        sum += mv;
        esp_rom_delay_us(150);
    }

    float adcVoltage = (sum / 8.0f) / 1000.0f;

    const float BATTERY_DIVIDER = (1000.0f + 47.0f) / 47.0f;
    const float BATTERY_CAL = 0.991f;
    float batteryVoltage = adcVoltage * BATTERY_DIVIDER * BATTERY_CAL;
    battery_voltage = batteryVoltage;

    ESP_LOGI(TAG, "Battery voltage: %.2f V", batteryVoltage);

    uint8_t batteryVoltage_res = (uint8_t)(batteryVoltage * 10.0f + 0.5f);
    return batteryVoltage_res;
}

static uint8_t batteryVoltageToPercent(uint8_t zigbeeVoltage)
{
    typedef struct {
        uint8_t voltage; // Напряжение в формате Zigbee (0.1В)
        uint8_t percent;
    } point_t;

    // Растянуто с исходной таблицы Arduino-версии (7.2-8.4В, форма кривой
    // сохранена) на реальный диапазон разряда 2S Li-ion 6.0-8.4В (2.8-3.0В
    // на банку) — прежний порог 7.2В=0% отсекал заметную часть ёмкости.
    static const point_t table[] = {
        {84, 100}, {82, 95}, {80, 88}, {78, 80}, {76, 70}, {74, 58},
        {72, 45}, {70, 32}, {68, 20}, {66, 10}, {64, 5}, {62, 2}, {60, 0},
    };
    const size_t table_len = sizeof(table) / sizeof(table[0]);

    if (zigbeeVoltage >= table[0].voltage) {
        ESP_LOGI(TAG, "Battery: max percent");
        return 100;
    }
    if (zigbeeVoltage <= table[table_len - 1].voltage) {
        ESP_LOGI(TAG, "Battery: min percent");
        return 0;
    }

    for (size_t i = 0; i < table_len - 1; i++) {
        if (zigbeeVoltage <= table[i].voltage && zigbeeVoltage >= table[i + 1].voltage) {
            float k = (float)(zigbeeVoltage - table[i + 1].voltage) / (float)(table[i].voltage - table[i + 1].voltage);
            uint8_t result = (uint8_t)(table[i + 1].percent + k * (table[i].percent - table[i + 1].percent) + 0.5f);
            ESP_LOGI(TAG, "Battery: %d.%dV -> %d%%", zigbeeVoltage / 10, zigbeeVoltage % 10, result);
            return result;
        }
    }
    ESP_LOGI(TAG, "Battery: zero return");
    return 0;
}

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ ПРЕОБРАЗОВАНИЯ ==========

static int32_t pulsesToPercent(int32_t pulses)
{
    if (pulses < 0) pulses = 0;
    if (pulses > current_max_pulse_position) pulses = current_max_pulse_position;
    return (pulses * 100) / current_max_pulse_position;
}

static int32_t percentToPulses(int32_t percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return (percent * current_max_pulse_position) / 100;
}

// ========== ОТПРАВКА СТАТУСА В ZIGBEE ==========

/**
 * @brief Колбэк одноразового таймера — единственное место, где реально
 * вызывается esp_zb_zcl_report_attr_cmd_req(). См. reportStatusToZ2M() и
 * комментарий у s_report_defer_timer про причину отложенной отправки.
 */
static void report_defer_timer_cb(void *arg)
{
    if (!s_report_pending) {
        return;
    }
    s_report_pending = false;

    // На отключённой от сети сети отчёт отправлять некуда — пропускаем.
    if (!zigbee_connected) {
        return;
    }

    // Найдена реальная причина ассерта "zcl/zcl_general_commands.c:612":
    // известный баг/недокументированное требование esp-zigbee-sdk (см.
    // github.com/espressif/esp-idf issue #15962 — тот же ассерт на ESP32-H2)
    // — не отсутствие поля .direction. Официальный пример Espressif для
    // esp_zb_zcl_report_attr_cmd_req() явно выставляет
    // .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI (0x01), у нас же оно
    // оставалось 0 (ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV) через "= {0}" —
    // рассинхронизация направления команды с ролью кластера (сервер),
    // видимо, и валит внутреннюю проверку стека.
    //
    // Структура несёт только ОДИН attributeID/clusterID за вызов — репорт
    // только процента батареи (как было раньше) не обновлял ни напряжение,
    // ни позицию шторы в Z2M вообще (там оставались дефолты интервью:
    // Voltage 0mV, Position/Battery не менялись). Шлём все три атрибута
    // отдельными вызовами.
    struct {
        uint16_t cluster_id;
        uint16_t attribute_id;
    } attrs_to_report[] = {
        {ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID},
        {ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID},
        {ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID},
    };

    // Адресация ENDP_NOT_PRESENT рассчитана на таблицу биндинга (получателя
    // выбирает стек по Bind-записям для этого кластера/атрибута). На практике
    // Z2M не всегда успевает создать биндинг во время интервью (см. вкладку
    // "Привязать" в Z2M — пусто даже после успешного интервью), и такие
    // отчёты уходят в никуда. Шлём напрямую координатору (0x0000, endpoint 1
    // — стандартный endpoint zigbee-herdsman/Z2M для приёма APS-фреймов),
    // это не зависит от биндинга вообще.
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
        for (size_t i = 0; i < sizeof(attrs_to_report) / sizeof(attrs_to_report[0]); i++) {
            esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
            report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
            report_attr_cmd.clusterID = attrs_to_report[i].cluster_id;
            report_attr_cmd.attributeID = attrs_to_report[i].attribute_id;
            report_attr_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
            report_attr_cmd.zcl_basic_cmd.dst_endpoint = 1;
            report_attr_cmd.zcl_basic_cmd.src_endpoint = ZIGBEE_ENDPOINT;
            report_attr_cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
            esp_err_t report_err = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
            ESP_LOGI(TAG, "[Report] cluster=0x%04x attr=0x%04x -> %s", attrs_to_report[i].cluster_id,
                     attrs_to_report[i].attribute_id, esp_err_to_name(report_err));
        }
        esp_zb_lock_release();
    }
}

/**
 * @brief Отправка текущего статуса (позиция шторы, батарея) в Zigbee2MQTT.
 *
 * esp_zb_zcl_set_attribute_val()/esp_zb_zcl_report_attr_cmd_req() — та же
 * последовательность вызовов, что использует ZigbeeEP.cpp/ZigbeeWindowCovering.cpp
 * (Arduino-библиотека поверх того же esp-zigbee-sdk) внутри
 * setLiftPercentage()/setBatteryPercentage()/setBatteryVoltage()/
 * reportBatteryPercentage() — имена и формат (BatteryPercentageRemaining в
 * единицах по 0.5%, отсюда "* 2") сверены с исходником ZigbeeEP.cpp, не
 * угаданы.
 */
static void reportStatusToZ2M(void)
{
    uint8_t lift_percentage = 100 - pulsesToPercent(current_pulse_position);

    uint8_t zb_voltage = readBatteryVoltage();
    uint8_t percent = batteryVoltageToPercent(zb_voltage);
    uint8_t zb_percent_x2 = (percent > 100 ? 100 : percent) * 2; // BatteryPercentageRemaining — единицы по 0.5%

    // esp_zb_zcl_set_attribute_val() на атрибуте с ACCESS_REPORTING внутри
    // задевает служебную логику zboss для его СОБСТВЕННОГО механизма
    // автоотчётности (zb_zcl_mark_attr_for_reporting_manuf() и далее) — это
    // требует esp_zb_lock, как и любой другой вызов в стек не из его
    // собственного колбэка (задокументировано в issues проекта esp-zigbee-sdk,
    // напр. #752, #537, #491 — паттерн acquire/set_attribute_val/release).
    // Без лока эта функция, вызванная из задачи esp_timer (hourly_report_cb()/
    // motor_poll_timer_cb()->motorStop()), стабильно валила
    // "assert failed: vPortExitCritical ... port_uxCriticalNesting[0] > 0"
    // на реальном железе — подтверждено декодированием дампа стека через
    // riscv32-esp-elf-addr2line (см. roadmap.md, Этап 19). Увеличение стека
    // задачи esp_timer (CONFIG_ESP_TIMER_TASK_STACK_SIZE) не помогло — краш
    // повторялся по той же цепочке вызовов с тем же остатком стека, значит
    // причина не в нехватке памяти, а в гонке за структуры репортинга.
    if (esp_zb_lock_acquire(portMAX_DELAY)) {
        esp_zb_zcl_set_attribute_val(ZIGBEE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                      ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &lift_percentage, false);
        esp_zb_zcl_set_attribute_val(ZIGBEE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                      ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &zb_percent_x2, false);
        esp_zb_zcl_set_attribute_val(ZIGBEE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                      ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &zb_voltage, false);
        esp_zb_lock_release();
    }

    // esp_zb_zcl_report_attr_cmd_req() падает со "Zigbee stack assertion
    // failed zcl/zcl_general_commands.c:612", если вызвана из потока самого
    // Zigbee-стека — подтверждено на реальном железе. Позже выяснилось, что
    // задача кнопки (iot_button) ТОЖЕ небезопасна для прямого вызова (тот же
    // краш после второго клика UP/DOWN, вызывающего motorStop()), хотя это
    // не задача Zigbee-стека — то есть безопасность зависит не от "какая
    // задача", а от чего-то ещё (вероятно, размера стека вызывающей задачи).
    // Единственный контекст, подтверждённо безопасный на железе — колбэки
    // esp_timer (часовой отчёт, updatePosition() из мотор-поллинг-таймера).
    // Поэтому сам вызов report_attr_cmd_req() всегда откладывается в
    // report_defer_timer_cb(), независимо от того, откуда вызвана эта
    // функция.
    s_report_pending = true;
    esp_timer_start_once(s_report_defer_timer, 10 * 1000);

    // Диагностика краша "vPortExitCritical" (см. sdkconfig.defaults,
    // CONFIG_ESP_TIMER_TASK_STACK_SIZE) — краш происходил при вызове этой
    // функции из задачи esp_timer (hourly_report_cb()/motor_poll_timer_cb()
    // ->motorStop()), но она же вызывается и из других задач (кнопки, Z2M
    // ZCL-колбэки) — печатаем имя задачи вместе с остатком, чтобы не путать
    // контексты. uxTaskGetStackHighWaterMark() — это МИНИМАЛЬНЫЙ когда-либо
    // остававшийся запас стека с момента старта задачи (монотонно убывает),
    // а не текущее использование — после нескольких срабатываний с разных
    // путей значение "осядет" на реальном худшем случае для каждой задачи.
    ESP_LOGI(TAG, "[Stack] task=%s high water mark: %u bytes", pcTaskGetName(NULL),
             (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
}

// ========== СВЕТОДИОД (WS2812 через led_strip, конечный автомат) ==========

static led_strip_handle_t s_led_strip;

/**
 * @brief (Пере)создаёт RMT-устройство led_strip. Вынесено из setupLED() в
 * отдельную функцию, чтобы её же можно было вызвать повторно из
 * mosfet_sync() при каждом включении питания WS2812 — см. комментарий там.
 */
static void createLEDStrip(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_PIN,
        .max_leds = NUM_LEDS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz — дефолт, указан явно
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led_strip));
}

static void ledWrite(uint8_t r, uint8_t g, uint8_t b)
{
    // Кэша "уже выставлен этот цвет — не шлём повторно" здесь нарочно нет:
    // WS2812 не хранит цвет между отключениями питания, а MOSFET_PIN
    // регулярно снимает с него питание (mosfet_sync()). Кэш по одному лишь
    // логическому значению цвета не знает, было ли оно реально запитано в
    // момент прошлой отправки — приводило к тому, что повторный запрос того
    // же цвета после цикла питания тихо пропускался, и светодиод оставался
    // тёмным (баг, воспроизведённый на реальном железе: одни направления/
    // команды зажигали LED, другие — нет, в зависимости от истории цветов).
    // Делим яркость (LED_BRIGHTNESS_DIV), т.к. led_strip не имеет отдельного
    // "brightness"-параметра как Adafruit_NeoPixel::setBrightness() — вместо
    // этого масштабируем сами компоненты цвета.
    led_strip_set_pixel(s_led_strip, 0, r / LED_BRIGHTNESS_DIV, g / LED_BRIGHTNESS_DIV, b / LED_BRIGHTNESS_DIV);
    led_strip_refresh(s_led_strip);
}

static inline void setLEDColor(uint8_t r, uint8_t g, uint8_t b) { ledWrite(r, g, b); }
static inline void setLEDOff(void) { ledWrite(0, 0, 0); }

typedef enum {
    LED_OFF,
    LED_GREEN,
    LED_RED,
    LED_BLUE_BLINK,
    LED_GREEN_BLINK,
} led_pattern_t;

static led_pattern_t led_pattern = LED_OFF;

static uint32_t blink_blue_lastChange = 0;
static bool blink_blue_on = false;
static uint32_t blink_green_lastChange = 0;
static bool blink_green_on = false;

static bool blinkPhase(uint16_t onTime, uint16_t offTime, uint32_t *lastChange, bool *state)
{
    uint32_t now = now_ms();
    uint32_t interval = *state ? onTime : offTime;
    if (now - *lastChange >= interval) {
        *lastChange = now;
        *state = !*state;
    }
    return *state;
}

/**
 * @brief Функция переходов: какой паттерн должен гореть сейчас. Приоритет
 * (нет связи проверяется РАНЬШЕ калибровки — по решению пользователя: иначе
 * пока не откалибровано, индикатор "нет связи" никогда не покажется, даже
 * если оба условия верны одновременно):
 *   1. мотор крутится  -> зелёный/красный (сплошной)
 *   2. нет связи       -> синий блик (не составной цвет — решение
 *                          пользователя вместо жёлтого; синий — единственный
 *                          чистый RGB-канал, ещё не занятый другим паттерном)
 *   3. нет лимитов     -> зелёный блик (не составной цвет — решение
 *                          пользователя вместо белого; совпадение с solid-
 *                          зелёным (OPEN) не мешает — отличаются миганием)
 *   4. иначе           -> выключено
 */
static led_pattern_t ledComputeDesired(void)
{
    if (motor_is_running) {
        return (motor_direction == DIR_OPEN) ? LED_GREEN : LED_RED;
    }
    if (!zigbee_connected) {
        return LED_BLUE_BLINK;
    }
    if (current_max_pulse_position == TOTAL_PULSES_LIMIT) {
        return LED_GREEN_BLINK;
    }
    return LED_OFF;
}

static void ledApply(void)
{
    switch (led_pattern) {
    case LED_GREEN:
        setLEDColor(0, 255, 0);
        break;
    case LED_RED:
        setLEDColor(255, 0, 0);
        break;
    case LED_OFF:
        setLEDOff();
        break;
    case LED_BLUE_BLINK:
        if (blinkPhase(250, 250, &blink_blue_lastChange, &blink_blue_on)) {
            setLEDColor(0, 0, 255);
        } else {
            setLEDOff();
        }
        break;
    case LED_GREEN_BLINK:
        if (blinkPhase(1000, 1000, &blink_green_lastChange, &blink_green_on)) {
            setLEDColor(0, 255, 0);
        } else {
            setLEDOff();
        }
        break;
    }
}

static void led_blink_timer_cb(void *arg)
{
    ledApply();
}

/**
 * @brief Включает/выключает periodic-таймер мигания LED (аналог
 * poll_timer_sync() для мотора). Мигающие паттерны (синий — нет связи,
 * зелёный — нет калибровки) требуют периодического вызова ledApply(), иначе
 * blinkPhase() не продвигается и светодиод замирает в одной фазе (баг,
 * найденный на реальном железе — светился постоянно вместо мигания).
 * Таймер тикает ТОЛЬКО пока паттерн мигающий; в обычном рабочем состоянии
 * (подключено + откалибровано -> LED_OFF) не запускается вовсе и не мешает
 * CAN_SLEEP — то же самое разделение, что и poll_timer_sync() для мотора.
 */
static void led_blink_timer_sync(void)
{
    bool should_run = (led_pattern == LED_BLUE_BLINK || led_pattern == LED_GREEN_BLINK);
    if (should_run) {
        esp_timer_start_periodic(s_led_blink_timer, 100 * 1000);
    } else {
        esp_timer_stop(s_led_blink_timer);
    }
}

/**
 * @brief Включает/выключает MOSFET_PIN ("периферия") — по подтверждению
 * пользователя, через этот MOSFET запитан сам WS2812, а не только
 * мотор/энкодер. Раньше включался/выключался только в motorStart()/
 * motorStop(), без учёта нужд LED — из-за этого адресный светодиод реально
 * обновлялся только один раз после сброса (пока MOSFET ещё был включён с
 * загрузки), а затем терял питание при первом же motorStop() и переставал
 * обновляться вообще, независимо от того, что показывает led_pattern.
 * Включаем, если мотор крутится ИЛИ LED должен что-то показывать
 * (led_pattern != LED_OFF). Важен порядок вызова внутри ledRefresh(): ДО
 * ledApply(), иначе новый цвет уходит на ещё обесточенный WS2812 и не
 * запоминается (см. комментарий в ledRefresh()).
 *
 * На переходе OFF->ON RMT-устройство led_strip дополнительно ПЕРЕСОЗДАЁТСЯ
 * (led_strip_del + createLEDStrip), а не просто перевыставляется цвет —
 * подтверждено на реальном железе: после одного успешного показа цвета
 * между циклами light sleep все последующие обновления (уже с любым
 * значением цвета, не только определённым каналом) переставали доходить
 * до физического светодиода, хотя led_strip_refresh() не возвращал ошибку.
 * Тот же класс проблемы, что и с LEDC ШИМ мотора (см. SKILL.md,
 * "a continuously-driven peripheral needs an esp_pm_lock") — автоматические
 * микро-сны tickless idle между обращениями к RMT портят его внутреннее
 * состояние; в отличие от мотора, WS2812 используется слишком редко и
 * коротко, чтобы держать esp_pm_lock постоянно, поэтому вместо запрета сна
 * просто пересоздаём периферию заново перед каждым использованием.
 */
static void mosfet_sync(void)
{
    static bool s_mosfet_was_on = false;
    bool should_be_on = motor_is_running || (led_pattern != LED_OFF);
    gpio_set_level(MOSFET_PIN, should_be_on ? 1 : 0);
    if (should_be_on && !s_mosfet_was_on) {
        led_strip_del(s_led_strip);
        createLEDStrip();
        ledApply();
    }
    s_mosfet_was_on = should_be_on;
}

/**
 * @brief Обновление автомата. Вызывается из motorStart/motorStop и из
 * esp_zb_app_signal_handler() при смене состояния подключения к Zigbee —
 * НЕ из поллингового loop(), которого в этом файле нет.
 */
static void ledRefresh(void)
{
    if (zigbee_connected != zigbee_connected_old) {
        ESP_LOGI(TAG, "Zigbee connected state changed: %s", zigbee_connected ? "CONNECTED" : "DISCONNECTED");
        zigbee_connected_old = zigbee_connected;
        // НЕ вызываем reportStatusToZ2M() здесь: ledRefresh() в этой ветке
        // вызывается прямо из esp_zb_app_signal_handler() (контекст самого
        // Zigbee-стека) — синхронная отправка ZCL report-команды отсюда
        // валила стек в "Zigbee stack assertion failed
        // zcl/zcl_general_commands.c:612" на каждой загрузке (проверено на
        // реальном железе). Из motorStart/motorStop/updatePosition тот же
        // reportStatusToZ2M() вызывается безопасно — те работают из другого
        // контекста (задачи кнопок/таймера), как и в Arduino-версии
        // (ZigbeeEP.cpp::reportClusterAttribute() берёт esp_zb_lock именно
        // для кросс-тасковых вызовов).
    }

    led_pattern_t desired = ledComputeDesired();
    if (desired != led_pattern) {
        led_pattern = desired;
    }
    // mosfet_sync() ДО ledApply() — WS2812 должен получить питание раньше,
    // чем на него уйдут данные о цвете, иначе первая команда после LED_OFF
    // (MOSFET выключен) уходит в обесточенный светодиод и не запоминается:
    // WS2812 не имеет памяти цвета между включениями питания. Мигающие
    // паттерны самозалечивались на следующем тике led_blink_timer (MOSFET
    // уже включён к тому моменту), но сплошные (зелёный/красный при
    // движении мотора) применяются один раз при смене паттерна и оставались
    // тёмными на реальном железе — баг с прошлым порядком вызовов.
    mosfet_sync();
    ledApply();
    led_blink_timer_sync();
}

static void setupLED(void)
{
    createLEDStrip();
    led_strip_clear(s_led_strip);

    // См. gpio_sleep_sel_dis(ENCODER_PIN)/(MOTOR_IN1/2_PIN) и SKILL.md —
    // тот же класс проблемы: LED_PIN (RMT) тоже мог страдать от
    // автоматического переключения GPIO-конфигурации на микро-снах tickless
    // idle, из-за чего только первое обновление цвета после старта реально
    // доходило до физического светодиода, а все последующие "терялись".
    gpio_sleep_sel_dis(LED_PIN);

    ESP_LOGI(TAG, "[LED] WS2812 initialized");
}

// ========== УПРАВЛЕНИЕ МОТОРОМ ==========

// Диагностика на реальном железе показала: даже с gpio_sleep_sel_dis() на
// пинах мотора (см. ниже) при работе через LEDC оставались периодические
// колебания скорости — временная замена на простой цифровой GPIO-выход без
// ШИМ убрала их полностью. Значит, дело не в конфигурации самого пина, а в
// том, что автоматические микро-сны tickless idle задевают саму периферию
// LEDC (её таймер/тактирование), а не только GPIO pad — gpio_sleep_sel_dis()
// это не покрывает. Реальный фикс — s_motor_pm_lock (ESP_PM_NO_LIGHT_SLEEP,
// см. motorStart()/motorStop()) — вообще не даёт чипу входить в light sleep
// (ни в наш явный, ни в автоматический) на время работы мотора.
static void initPWM(void)
{
    gpio_reset_pin(MOTOR_IN1_PIN);
    gpio_reset_pin(MOTOR_IN2_PIN);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = MOTOR_IN1_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = MOTOR_IN2_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

    // GPIO pad сам по себе тоже переключается между рабочей/спящей
    // конфигурацией (CONFIG_PM_SLP_DISABLE_GPIO, см. gpio_sleep_sel_dis()
    // в initEncoder() и SKILL.md) — не покрывает саму периферию LEDC (см.
    // s_motor_pm_lock выше), но не помешает исключить и pad на всякий случай.
    gpio_sleep_sel_dis(MOTOR_IN1_PIN);
    gpio_sleep_sel_dis(MOTOR_IN2_PIN);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "motor_pwm", &s_motor_pm_lock));

    ESP_LOGI(TAG, "[PWM] Motor PWM initialized");
}

static void motorSetDirectionAndSpeed(motor_direction_t dir, uint16_t duty)
{
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY; // ограничение — аппаратный потолок, как в Arduino-версии
    motor_direction_t dir_t = dir;
    if (motor_invert) {
        ESP_LOGI(TAG, "[PWM] Inverting direction");
        if (dir == DIR_OPEN) dir_t = DIR_CLOSE;
        else if (dir == DIR_CLOSE) dir_t = DIR_OPEN;
    }

    switch (dir_t) {
    case DIR_OPEN:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        break;
    case DIR_CLOSE:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        break;
    default:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        break;
    }
}

/**
 * @brief Включает/выключает таймер быстрого поллинга (100мс) в зависимости
 * от того, нужен ли он прямо сейчас (мотор крутится или режим не
 * STATE_NORMAL — как раз то, что в Arduino-loop() было условием "не спать").
 * Вызывается из motorStart/motorStop и из точек смены system_state.
 * esp_timer_start_periodic/esp_timer_stop на уже-в-нужном-состоянии таймере
 * просто вернут ошибку, которую здесь намеренно игнорируем — не критично.
 */
static void poll_timer_sync(void)
{
    bool should_run = motor_is_running || (system_state != STATE_NORMAL);
    if (should_run) {
        esp_timer_start_periodic(s_motor_poll_timer, 100 * 1000);
    } else {
        esp_timer_stop(s_motor_poll_timer);
    }
}

static void motorStop(void)
{
    if (motor_is_running) {
        motor_is_running = false;
        motor_direction = DIR_NONE;
        motorSetDirectionAndSpeed(DIR_NONE, 0);
        esp_pm_lock_release(s_motor_pm_lock);

        ESP_LOGI(TAG, "[Motor] STOP at pulse_pos=%d", (int)current_pulse_position);
        if (system_state == STATE_NORMAL) savePosition();
    }
    reportStatusToZ2M();
    ledRefresh();
    poll_timer_sync();
}

static void motorStart(motor_direction_t dir)
{
    // Во время приёма OTA-образа мотор двигать МОЖНО — образ пишется в другой слот флеша (app0/app1), само
    // движение его не трогает. Единственный побочный эффект — savePosition() (NVS) тоже пишет во флеш, а любая
    // запись/стирание флеша на ESP32-H2 (одно ядро) кратковременно отключает прерывания, теоретически рискуя
    // пропустить импульс энкодера — но это тот же порядок величины, что уже существующий некритичный проскок
    // -1..-4 на лимитах (см. checkMotorStopConditions()), а параллельный доступ к флешу сам ESP-IDF сериализует
    // (порчи данных не будет). Блокируем не движение, а то, что реально обрывает саму передачу с концами —
    // переход в спаривание/сброс к заводским (см. startPairingMode()/forceFactoryReset()).
    if (motor_is_running) {
        motorStop();
    }

    if (battery_voltage <= 6.5f) {
        ESP_LOGW(TAG, "[Motor] Battery too low, can't start motor !!!");
    }

    motor_direction = dir;
    motor_is_running = true;
    mosfet_sync(); // включает периферию (питание мотора и WS2812) — раньше, чем ШИМ на мотор
    esp_pm_lock_acquire(s_motor_pm_lock);
    motorSetDirectionAndSpeed(dir, s_max_pwm_duty);

    ESP_LOGI(TAG, "[Motor] Starting %s, pulses=%d, target_pulses=%d", (dir == DIR_OPEN) ? "OPEN" : "CLOSE",
             (int)current_pulse_position, (int)target_pulse_position);
    ledRefresh();
    poll_timer_sync();
}

static void checkMotorStopConditions(void)
{
    if (!motor_is_running) return;
    if (system_state != STATE_NORMAL) return; // в режиме калибровки лимиты не проверяем

    static bool limit_trashhold_reached = false;
    bool limit_reached = false;

    if (motor_direction == DIR_OPEN && current_pulse_position >= current_max_pulse_position) {
        limit_reached = true;
        ESP_LOGI(TAG, "[Limit] OPEN limit reached: %d >= %d", (int)current_pulse_position, (int)current_max_pulse_position);
    } else if (motor_direction == DIR_CLOSE && current_pulse_position <= 0) {
        limit_reached = true;
        ESP_LOGI(TAG, "[Limit] CLOSE limit reached: %d <= 0", (int)current_pulse_position);
    }

    if (limit_reached) {
        motorStop();
        limit_trashhold_reached = false;
        return;
    }

    if ((motor_direction == DIR_OPEN) && (current_pulse_position >= (current_max_pulse_position - TRASHHOLD_PWM_PILSE)) &&
        (!limit_trashhold_reached)) {
        limit_trashhold_reached = true;
        ESP_LOGI(TAG, "[Limit] OPEN approached the threshold: %d >= %d", (int)current_pulse_position,
                 (int)current_max_pulse_position);
        motorSetDirectionAndSpeed(motor_direction, s_min_pwm_duty);
    } else if ((motor_direction == DIR_CLOSE) && (current_pulse_position <= (0 + TRASHHOLD_PWM_PILSE)) && (!limit_trashhold_reached)) {
        limit_trashhold_reached = true;
        ESP_LOGI(TAG, "[Limit] CLOSE approached the threshold: %d <= 0", (int)current_pulse_position);
        motorSetDirectionAndSpeed(motor_direction, s_min_pwm_duty);
    }

    bool target_reached = false;
    if (motor_direction == DIR_OPEN && current_pulse_position >= target_pulse_position) {
        target_reached = true;
    } else if (motor_direction == DIR_CLOSE && current_pulse_position <= target_pulse_position) {
        target_reached = true;
    }

    if (target_reached) {
        ESP_LOGI(TAG, "[Target] Target reached: %d pulses, stopping", (int)target_pulse_position);
        motorStop();
        limit_trashhold_reached = false;
    }
}

// ========== ОБНОВЛЕНИЕ ПОЗИЦИИ ==========

/**
 * @brief Почасовой отчёт в Z2M — отдельный редкий таймер (см. app_main),
 * не связан с "быстрым" поллингом мотора: часовой период на много порядков
 * больше sleep-порога (2с), сну не мешает.
 */
static void hourly_report_cb(void *arg)
{
    ESP_LOGI(TAG, "[Z2M] Reporting Status");
    reportStatusToZ2M();
}

/**
 * @brief Проверка позиции/отладка энкодера. Вызывается из motor_poll_timer_cb
 * — то есть ТОЛЬКО пока мотор крутится или мы не в STATE_NORMAL (см.
 * poll_timer_sync()), не постоянно, как в Arduino-loop().
 */
static void updatePosition(void)
{
    static uint32_t last_encoder_debug = 0;

    if (system_state != STATE_NORMAL) {
#if DEBUG_ENCODER
        if (pulse_detected && now_ms() - last_encoder_debug > 5000) {
            last_encoder_debug = now_ms();
            pulse_detected = false;
            ESP_LOGI(TAG, "[Encoder] Calibrate Pulses: %d", (int)calibration_pulse_position);
        }
#endif
        return;
    }

#if DEBUG_ENCODER
    if (pulse_detected && now_ms() - last_encoder_debug > 1000) {
        last_encoder_debug = now_ms();
        pulse_detected = false;
        ESP_LOGI(TAG, "[Encoder] Pulses: %d, target_pulses=%d", (int)current_pulse_position, (int)target_pulse_position);
    }
#endif

    if (motor_is_running) {
        int32_t new_pos = current_pulse_position;
        if (new_pos < 0) new_pos = 0;
        if (new_pos > current_max_pulse_position) new_pos = current_max_pulse_position;

        if (new_pos != current_pulse_position) {
            current_pulse_position = new_pos;
            reportStatusToZ2M();
            savePosition();
        }
    }
}

// ========== ZIGBEE КОЛБЭКИ (команды из Z2M) ==========

static void fullOpen(void)
{
    ESP_LOGI(TAG, "[Z2M] OPEN");
    if (current_pulse_position < current_max_pulse_position) {
        target_pulse_position = current_max_pulse_position;
        motorStart(DIR_OPEN);
    }
}

static void fullClose(void)
{
    ESP_LOGI(TAG, "[Z2M] CLOSE");
    if (current_pulse_position > 0) {
        target_pulse_position = 0;
        motorStart(DIR_CLOSE);
    }
}

static void stopMotorCommand(void)
{
    ESP_LOGI(TAG, "[Z2M] STOP");
    motorStop();
    target_pulse_position = current_pulse_position;
}

static void goToLiftPercentage(uint8_t liftPercentage)
{
    ESP_LOGI(TAG, "[Z2M] POSITION %u%%", liftPercentage);

    motorStop();
    int32_t our_position_pulses = percentToPulses(100 - liftPercentage);
    if (our_position_pulses < 0) our_position_pulses = 0;
    if (our_position_pulses > current_max_pulse_position) our_position_pulses = current_max_pulse_position;

    if (our_position_pulses > current_pulse_position) {
        target_pulse_position = our_position_pulses;
        motorStart(DIR_OPEN);
    } else if (our_position_pulses < current_pulse_position) {
        target_pulse_position = our_position_pulses;
        motorStart(DIR_CLOSE);
    }
}

// ========== РЕЖИМЫ СПАРИВАНИЯ / СБРОСА ==========

/**
 * @brief esp_zb_factory_reset() стирает NVRAM Zigbee-стека и (по документации
 * esp_zigbee_core.h: "please refer esp_zb_factory_reset() for erase NVRAM
 * and other action") сама перезагружает чип — ровно то же, что делал
 * Zigbee.factoryReset(true) в Arduino-версии (тот вызывал именно эту же
 * функцию напрямую, см. ZigbeeCore.cpp). Бесконечный цикл после вызова —
 * подстраховка на случай, если перезагрузка не мгновенная, как и было в
 * Arduino-версии.
 */
static void startPairingMode(void)
{
    // esp_zb_factory_reset() ниже перезагружает чип почти сразу — если идёт приём OTA-образа, это оборвёт
    // передачу (не испортит ничего — как обрыв питания, но потеряет уже принятые часы). В отличие от движения
    // мотора, это реально стоит отложить: не физический процесс, а осознанное действие пользователя, повторить
    // которое можно секундой позже.
    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "[Pairing] Ignored: OTA in progress");
        return;
    }
    ESP_LOGW(TAG, "PAIRING MODE");
    iot_button_stop();
    gpio_intr_disable(ENCODER_PIN);

    esp_zb_factory_reset();
    ESP_LOGW(TAG, "Waiting PAIRING");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGW(TAG, "Waiting PAIRING");
    }
}

static void forceFactoryReset(void)
{
    // См. комментарий в startPairingMode() — та же причина.
    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "[Factory reset] Ignored: OTA in progress");
        return;
    }
    ESP_LOGW(TAG, "FACTORY RESET");
    motorStop();
    nvs_flash_erase();
    iot_button_stop();
    gpio_intr_disable(ENCODER_PIN);

    esp_zb_factory_reset();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGW(TAG, "Waiting FACTORY RESET");
    }
}

// ========== ОБРАБОТЧИК СОБЫТИЙ КНОПОК ==========

/**
 * @brief Общий колбэк для всех трёх кнопок (espressif/button, iot_button).
 * usr_data — номер GPIO-пина кнопки, передан при регистрации. Вся логика
 * комбинаций перенесена 1:1 из Arduino buttonEventHandler().
 */
static void buttonEventHandler(void *button_handle, void *usr_data)
{
    gpio_num_t gpio = (gpio_num_t)(intptr_t)usr_data;
    button_event_t event_id = iot_button_get_event((button_handle_t)button_handle);

    // Аптайм на каждый клик/долгое нажатие (не на PRESS_DOWN/PRESS_UP —
    // это просто отслеживание удержания для комбинаций, не само событие) —
    // до дебаунса ниже, чтобы не терять его для сверки с логом.
    if (event_id == BUTTON_SINGLE_CLICK || event_id == BUTTON_LONG_PRESS_START) {
        logUptime("Button");
    }

    static bool up_pressed = false;
    static bool down_pressed = false;
    static bool set_pressed = false;
    static uint32_t last_set_press_time = 0;
    const uint32_t SET_DEBOUNCE_MS = 2000;

    if (gpio == BUTTON_UP_PIN) {
        if (event_id == BUTTON_PRESS_DOWN) up_pressed = true;
        else if (event_id == BUTTON_PRESS_UP) up_pressed = false;
    }
    if (gpio == BUTTON_DOWN_PIN) {
        if (event_id == BUTTON_PRESS_DOWN) down_pressed = true;
        else if (event_id == BUTTON_PRESS_UP) down_pressed = false;
    }
    if (gpio == BUTTON_SET_PIN) {
        if (event_id == BUTTON_PRESS_DOWN) set_pressed = true;
        else if (event_id == BUTTON_PRESS_UP) set_pressed = false;
    }

    if ((last_set_press_time > 0) && ((now_ms() - last_set_press_time) < SET_DEBOUNCE_MS)) {
        return;
    }

    // ===== UP+DOWN долгое нажатие: инверсия направления =====
    // !set_pressed — иначе при удержании всех трёх кнопок это условие
    // перехватывает событие раньше, чем дойдёт очередь до проверки
    // SET+UP+DOWN ниже (баг, унаследованный из Arduino-версии, подтверждено
    // на реальном железе).
    if (event_id == BUTTON_LONG_PRESS_START && system_state == STATE_NORMAL) {
        if ((gpio == BUTTON_UP_PIN && down_pressed && !set_pressed) || (gpio == BUTTON_DOWN_PIN && up_pressed && !set_pressed)) {
            if (!motor_is_running) {
                last_set_press_time = now_ms();
                if (motor_invert) {
                    motor_invert = false;
                    shortBeep();
                    ESP_LOGW(TAG, "Motor direction is inverted to FALSE");
                } else {
                    motor_invert = true;
                    doubleBeep();
                    ESP_LOGW(TAG, "Motor direction is inverted to TRUE");
                }
                saveMotorSettings(); // раньше не сохранялось вообще — терялось при каждой перезагрузке
                return;
            } else {
                ESP_LOGW(TAG, "First stop motor");
            }
        }
    }

    // ===== SET+UP+DOWN долгое нажатие: полный сброс к заводским =====
    if (event_id == BUTTON_LONG_PRESS_START && system_state == STATE_NORMAL) {
        if (down_pressed && up_pressed && set_pressed) {
            if (!motor_is_running) {
                last_set_press_time = now_ms();
                longBeep();
                forceFactoryReset();
                return;
            } else {
                ESP_LOGW(TAG, "First stop motor");
            }
        }
    }

    // ===== SET долгое нажатие: режим Zigbee Pairing =====
    if ((event_id == BUTTON_LONG_PRESS_START) && (system_state == STATE_NORMAL) && (gpio == BUTTON_SET_PIN)) {
        if (!motor_is_running) {
            last_set_press_time = now_ms();
            shortBeep();
            startPairingMode();
            return;
        } else {
            ESP_LOGW(TAG, "First stop motor");
        }
    }

    // ===== SET одиночный клик: вход/выход из калибровки =====
    if ((event_id == BUTTON_SINGLE_CLICK) && (system_state == STATE_NORMAL) && (gpio == BUTTON_SET_PIN)) {
        if (!motor_is_running) {
            system_state = STATE_CALIBRATION_UP;
            poll_timer_sync(); // не даём чипу спать в режиме калибровки — как в Arduino-loop()
            ESP_LOGI(TAG, "Entered CALIBRATION MODE UP");
            last_set_press_time = now_ms();
            shortBeep();
            return;
        } else {
            ESP_LOGW(TAG, "First stop motor");
        }
    }

    // ===== Одиночные клики =====
    if (event_id == BUTTON_SINGLE_CLICK) {
        if (gpio == BUTTON_UP_PIN) {
            ESP_LOGI(TAG, "[UP] Click");
            if (motor_is_running) {
                motorStop();
                target_pulse_position = current_pulse_position;
            } else {
                if ((system_state == STATE_NORMAL) && (current_pulse_position < current_max_pulse_position)) {
                    target_pulse_position = current_max_pulse_position;
                    motorStart(DIR_OPEN);
                } else if (system_state != STATE_NORMAL) {
                    motorStart(DIR_OPEN);
                }
            }
        }

        if (gpio == BUTTON_DOWN_PIN) {
            ESP_LOGI(TAG, "[DOWN] Click");
            if (motor_is_running) {
                motorStop();
                target_pulse_position = current_pulse_position;
            } else {
                if ((system_state == STATE_NORMAL) && (current_pulse_position > 0)) {
                    target_pulse_position = 0;
                    motorStart(DIR_CLOSE);
                } else if (system_state != STATE_NORMAL) {
                    motorStart(DIR_CLOSE);
                }
            }
        }

        if (gpio == BUTTON_SET_PIN) {
            if (system_state == STATE_CALIBRATION || system_state == STATE_CALIBRATION_UP || system_state == STATE_CALIBRATION_DOWN) {
                if (system_state == STATE_CALIBRATION_UP) {
                    system_state = STATE_CALIBRATION_DOWN;
                    shortBeep();
                    calibration_pulse_position = 0;
                    ESP_LOGI(TAG, "JUMP to CALIBRATION DOWN");
                } else if (system_state == STATE_CALIBRATION_DOWN) {
                    system_state = STATE_NORMAL;
                    poll_timer_sync(); // калибровка окончена — снова можно спать
                    doubleBeep();
                    ESP_LOGI(TAG, "Exiting CALIBRATION MODE");
                    if (calibration_pulse_position != 0) {
                        current_max_pulse_position = abs((int)calibration_pulse_position);
                        calibration_pulse_position = 0;
                        current_pulse_position = 0;
                    }
                    ESP_LOGI(TAG, "current calibration pulse=%d, current_pos=%d, max pos=%d", (int)calibration_pulse_position,
                             (int)current_pulse_position, (int)current_max_pulse_position);
                    savePosition();
                    // Выход из калибровки может поменять current_max_pulse_position
                    // (а значит и ledComputeDesired()), но сам по себе не вызывает
                    // ledRefresh() — без явного вызова LED застревал в прежнем
                    // паттерне (зелёный блик) до следующего постороннего события
                    // (например, команды из Z2M), которое случайно дёргало
                    // ledRefresh() через motorStart/motorStop.
                    ledRefresh();
                }
                last_set_press_time = now_ms();
                return;
            }
            if (motor_is_running) {
                ESP_LOGI(TAG, "[SET] Moving - instant stop");
                motorStop();
                target_pulse_position = current_pulse_position;
            }
        }
    }
}

static button_handle_t s_up_button, s_down_button, s_set_button;

/**
 * @brief Регистрация одной GPIO-кнопки через espressif/button. enable_power_save
 * — реальное поле button_gpio_config_t (button_gpio.h), задокументировано как
 * "enable power save mode". Точная схема взаимодействия с
 * esp_sleep_enable_gpio_wakeup()/gpio_wakeup_enable() (которые остаются
 * основным источником пробуждения, см. app_main) не задокументирована
 * примерами библиотеки — проверяется эмпирически на первом реальном прогоне
 * (см. план миграции/roadmap).
 */
static button_handle_t create_button(gpio_num_t gpio)
{
    button_config_t btn_cfg = {
        .long_press_time = LONG_PRESS_MS,
        .short_press_time = 0, // 0 = использовать дефолт библиотеки
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = gpio,
        .active_level = 0, // кнопка замыкает на GND
        .enable_power_save = true,
        .disable_pull = false, // внутренний pull-up
    };
    button_handle_t handle = NULL;
    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &handle));

    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, buttonEventHandler, (void *)(intptr_t)gpio);
    iot_button_register_cb(handle, BUTTON_PRESS_UP, NULL, buttonEventHandler, (void *)(intptr_t)gpio);
    iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, NULL, buttonEventHandler, (void *)(intptr_t)gpio);
    iot_button_register_cb(handle, BUTTON_LONG_PRESS_START, NULL, buttonEventHandler, (void *)(intptr_t)gpio);
    return handle;
}

static void setupButtons(void)
{
    s_up_button = create_button(BUTTON_UP_PIN);
    s_down_button = create_button(BUTTON_DOWN_PIN);
    s_set_button = create_button(BUTTON_SET_PIN);
    ESP_LOGI(TAG, "[Buttons] Initialized");
}

// ========== ПРОБУЖДЕНИЕ ПО КНОПКАМ (сон) ==========

/**
 * @brief Источник пробуждения по кнопкам — тот же механизм, что подтверждён
 * рабочим на тестовом стенде (esp32h2_light_sleep_test/src/main.c):
 * gpio_wakeup_enable() для каждого пина, потом один esp_sleep_enable_gpio_wakeup().
 * Настраивается ПОСЛЕ первого успешного джойна — раньше (до
 * esp_pm_configure()/старта Zigbee-стека) это давало полный сброс чипа на
 * тестовом стенде при использовании EXT1 (см. roadmap.md, Этап 13); здесь
 * используется обычный gpio_wakeup, но порядок вызовов сохранён на всякий
 * случай — не переносился отдельно для проверки на этом железе.
 */
static void enable_button_wakeup(void)
{
    ESP_ERROR_CHECK(gpio_wakeup_enable(BUTTON_UP_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(BUTTON_DOWN_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(BUTTON_SET_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
}

// ========== ZIGBEE: OTA-ОБНОВЛЕНИЕ ПРОШИВКИ ==========

/**
 * @brief Обработчик ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID — приём образа от Z2M блок за блоком в соседний
 * OTA-слот (app0/app1, см. zigbee.csv). Паттерн START->esp_ota_begin, RECEIVE->esp_ota_write,
 * FINISH->esp_ota_end+esp_ota_set_boot_partition+esp_restart — как в официальном примере
 * esp-zigbee-sdk/examples/esp_zigbee_ota/ota_client. Возвращаемое значение != ESP_OK стек трактует как отказ
 * (esp_zb_core_action_callback_t) — так отклоняется старт обновления.
 */
static esp_err_t ota_upgrade_handler(const esp_zb_zcl_ota_upgrade_value_message_t *msg)
{
    static esp_ota_handle_t ota_handle = 0;
    static const esp_partition_t *ota_partition = NULL;
    static uint32_t received = 0;   // байт САМОГО образа приложения, записанных в слот (без sub-element заголовка)
    static uint32_t image_len = 0;  // длина образа из sub-element заголовка (тег 0x0000)
    static bool element_header_done = false;
    static uint32_t last_logged_kb = 0;
    esp_err_t err;

    switch (msg->upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        // Мотор двигать нельзя ровно так же, как во время OTA (см. motorStart()): не начинаем приём, пока он
        // крутится или идёт калибровка/спаривание, и не начинаем на севшей батарее — передача долгая и
        // энергоёмкая (радио включено всё время).
        // battery_voltage обновляется только при отчёте (остановка мотора/часовой таймер) — сразу после загрузки
        // там 0.0 В и защита отказала бы на исправной батарее. Мерим заново прямо перед проверкой.
        readBatteryVoltage();
        if (motor_is_running || system_state != STATE_NORMAL || battery_voltage < OTA_MIN_BATTERY_VOLTAGE) {
            ESP_LOGW(TAG, "[OTA] Refused: motor_running=%d state=%d battery=%.2fV (need >= %.1fV)", (int)motor_is_running,
                     (int)system_state, battery_voltage, OTA_MIN_BATTERY_VOLTAGE);
            return ESP_FAIL;
        }
        ota_partition = esp_ota_get_next_update_partition(NULL);
        if (ota_partition == NULL) {
            ESP_LOGE(TAG, "[OTA] No OTA update partition found");
            return ESP_FAIL;
        }
        err = esp_ota_begin(ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[OTA] esp_ota_begin failed: %s", esp_err_to_name(err));
            return err;
        }
        received = 0;
        image_len = 0;
        element_header_done = false;
        last_logged_kb = 0;
        s_ota_in_progress = true;
        // ZCL-заголовок файла на START стеком ещё не разобран (поля ota_header нулевые) — версию/размер печатаем
        // с первым блоком данных.
        ESP_LOGI(TAG, "[OTA] Start -> partition %s (running fw 0x%08x)", ota_partition->label, (unsigned)FW_FILE_VERSION);
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
        if (!s_ota_in_progress) {
            return ESP_FAIL;
        }
        const uint8_t *data = msg->payload;
        uint32_t len = msg->payload_size;
        if (!element_header_done) {
            // Стек 1.6.x (проверено на железе) снимает ZCL-заголовок файла, но отдаёт данные вместе с 6-байтным
            // заголовком sub-element'а (Zigbee OTA, 11.3.2): тег uint16 (0x0000 = Upgrade Image) + длина uint32,
            // и только затем идёт сам образ приложения. Заголовок целиком лежит в первом блоке.
            if (len < 6 + 1 || (data[0] | (data[1] << 8)) != 0x0000) {
                ESP_LOGE(TAG, "[OTA] Bad sub-element header (size=%u tag=0x%02x%02x) — abort", (unsigned)len, len > 1 ? data[1] : 0,
                         len > 0 ? data[0] : 0);
                esp_ota_abort(ota_handle);
                s_ota_in_progress = false;
                return ESP_FAIL;
            }
            image_len = (uint32_t)data[2] | ((uint32_t)data[3] << 8) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 24);
            data += 6;
            len -= 6;
            // Сам образ обязан начинаться с магического байта ESP (0xE9) — иначе прислали не тот файл: не портим слот.
            if (data[0] != 0xE9) {
                ESP_LOGE(TAG, "[OTA] Image does not start with ESP magic 0xE9 (got 0x%02x) — abort", data[0]);
                esp_ota_abort(ota_handle);
                s_ota_in_progress = false;
                return ESP_FAIL;
            }
            element_header_done = true;
            ESP_LOGI(TAG, "[OTA] Receiving: file_version=0x%08x, app image %u bytes", (unsigned)msg->ota_header.file_version,
                     (unsigned)image_len);
        }
        err = esp_ota_write(ota_handle, data, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[OTA] esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            return err;
        }
        received += len;
        if ((received / 1024) >= last_logged_kb + 32) { // прогресс раз в 32 КБ, а не на каждый 64-байтный блок
            last_logged_kb = received / 1024;
            ESP_LOGI(TAG, "[OTA] Received %u of %u bytes (%u%%)", (unsigned)received, (unsigned)image_len,
                     (unsigned)(image_len ? (received * 100u / image_len) : 0));
        }
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        // Записанных байт образа должно быть ровно столько, сколько заявлено в sub-element заголовке; дальше
        // целостность проверит esp_ota_end() (магия, размер, контрольная сумма и SHA256 образа) на FINISH.
        ESP_LOGI(TAG, "[OTA] Download complete: wrote %u of %u bytes", (unsigned)received, (unsigned)image_len);
        if (received != image_len) {
            ESP_LOGE(TAG, "[OTA] Size mismatch — abort");
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            return ESP_FAIL;
        }
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
        if (!s_ota_in_progress) {
            return ESP_FAIL;
        }
        err = esp_ota_end(ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[OTA] esp_ota_end failed (image invalid): %s — staying on current firmware", esp_err_to_name(err));
            s_ota_in_progress = false;
            return err;
        }
        err = esp_ota_set_boot_partition(ota_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[OTA] esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
            s_ota_in_progress = false;
            return err;
        }
        ESP_LOGI(TAG, "[OTA] Done, rebooting into %s", ota_partition->label);
        vTaskDelay(pdMS_TO_TICKS(500)); // даём логу и стеку дослать финальный ответ серверу
        esp_restart();
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        if (s_ota_in_progress) {
            ESP_LOGW(TAG, "[OTA] Aborted by server/stack after %u bytes", (unsigned)received);
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
        }
        break;

    default:
        break;
    }

    return ESP_OK;
}

/**
 * @brief Обработчик записи атрибутов кастомного кластера настроек (CUSTOM_SETTINGS_CLUSTER_ID) из Z2M —
 * min/max PWM duty мотора и инверсия направления, без перезаливки прошивки. esp_zb_zcl_set_attr_value_message_t
 * приходит УЖЕ после того, как стек применил запись к своему собственному хранилищу атрибута — новое значение
 * читаем из msg->attribute.data.value (не полагаемся на то, что это тот же указатель, что дали при
 * esp_zb_cluster_add_attr(), это внутренняя деталь реализации стека, которую мы не проверяли).
 */
static esp_err_t settings_attr_write_handler(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->info.cluster != CUSTOM_SETTINGS_CLUSTER_ID) {
        return ESP_OK;
    }
    switch (msg->attribute.id) {
    case ATTR_MIN_PWM_DUTY_ID: {
        uint16_t v = *(uint16_t *)msg->attribute.data.value;
        if (v > PWM_MAX_DUTY) v = PWM_MAX_DUTY;
        s_min_pwm_duty = v;
        ESP_LOGI(TAG, "[Settings] min_pwm_duty = %u (from Z2M)", s_min_pwm_duty);
        saveMotorSettings();
        break;
    }
    case ATTR_MAX_PWM_DUTY_ID: {
        uint16_t v = *(uint16_t *)msg->attribute.data.value;
        if (v > PWM_MAX_DUTY) v = PWM_MAX_DUTY;
        s_max_pwm_duty = v;
        ESP_LOGI(TAG, "[Settings] max_pwm_duty = %u (from Z2M)", s_max_pwm_duty);
        saveMotorSettings();
        break;
    }
    case ATTR_MOTOR_INVERT_ID: {
        motor_invert = (*(uint8_t *)msg->attribute.data.value != 0);
        ESP_LOGI(TAG, "[Settings] motor_invert = %d (from Z2M)", (int)motor_invert);
        saveMotorSettings();
        break;
    }
    default:
        break;
    }
    return ESP_OK;
}

// ========== ZIGBEE: ОБРАБОТЧИК ZCL-КОМАНД ==========

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID) {
        return ota_upgrade_handler((const esp_zb_zcl_ota_upgrade_value_message_t *)message);
    }
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        return settings_attr_write_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    }

    if (callback_id != ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID) {
        return ESP_OK;
    }

    const esp_zb_zcl_window_covering_movement_message_t *msg = message;

    // По любой команде из Z2M дополнительно печатаем аптайм — см. logUptime().
    logUptime("Z2M");

    switch (msg->command) {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        fullOpen();
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        fullClose();
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        stopMotorCommand();
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
        goToLiftPercentage((uint8_t)msg->payload.percentage_lift_value);
        break;
    default:
        ESP_LOGI(TAG, "[Z2M] window covering command 0x%02x", (unsigned)msg->command);
        break;
    }
    return ESP_OK;
}

// ========== ZIGBEE: ПОДТВЕРЖДЕНИЕ / ОТКАТ ПРОШИВКИ ПОСЛЕ OTA ==========

// Таймер-предохранитель: прошивка, установленная по OTA, не подтвердила себя за OTA_VALIDATE_TIMEOUT_MS после
// загрузки (не вышла в сеть, но и не упала — иначе откат сделал бы загрузчик сам) — откатываемся вручную.
static void ota_validate_timeout_cb(void *arg)
{
    ESP_LOGE(TAG, "[OTA] New firmware was not confirmed within %d ms — rolling back", OTA_VALIDATE_TIMEOUT_MS);
    esp_ota_mark_app_invalid_rollback_and_reboot();
}

/**
 * @brief Если бегущая прошивка — свежая, после OTA, и ещё не подтверждена (ESP_OTA_IMG_PENDING_VERIFY), а
 * устройство уже в сети после льготного периода джойна — подтверждаем её (esp_ota_mark_app_valid_cancel_rollback)
 * и снимаем таймер-предохранитель. До подтверждения любой перезапуск = откат на прошлую прошивку.
 * Вызывается из перехвата CAN_SLEEP — то есть регулярно и только когда стек уже отработал вход в сеть.
 */
static void ota_confirm_new_image_if_needed(void)
{
    if (!s_ota_pending_verify || !zigbee_connected) {
        return;
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[OTA] New firmware confirmed (joined network), rollback cancelled");
        s_ota_pending_verify = false;
        if (s_ota_validate_timer != NULL) {
            esp_timer_stop(s_ota_validate_timer);
        }
    } else {
        ESP_LOGE(TAG, "[OTA] esp_ota_mark_app_valid_cancel_rollback failed: %s", esp_err_to_name(err));
    }
}

// ========== ZIGBEE: СИГНАЛЬНЫЙ ОБРАБОТЧИК (СЕРДЦЕ АРХИТЕКТУРЫ СНА) ==========

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

/**
 * @brief 1:1 перенос из esp32h2_light_sleep_test/src/main.c — уже проверено
 * на реальном железе (стабильный сон 0.07-0.17мА, кнопки, Z2M-команды).
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    esp_zb_app_signal_type_t sig_type = *signal_struct->p_app_signal;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized, starting network steering");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    // ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT — rejoin по данным из NVRAM (обычный
    // случай для уже подключённого устройства). В этом случае
    // ESP_ZB_BDB_SIGNAL_STEERING не генерируется вовсе (документировано в
    // zboss_api_zdo.h) — оба сигнала обрабатываются одинаково.
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (signal_struct->esp_err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network (signal 0x%x), enabling sleep participation", sig_type);
            s_steering_done = true;
            zigbee_connected = true;
            // 20с льготного периода — даём координатору/Z2M время опросить
            // endpoint'ы и кластеры (интервью) сразу после джойна, пока чип
            // гарантированно не спит. См. s_stay_awake_until_ms.
            s_stay_awake_until_ms = now_ms() + JOIN_GRACE_PERIOD_MS;
            esp_zb_sleep_enable(true);
            static bool s_button_wakeup_enabled = false;
            if (!s_button_wakeup_enabled) {
                enable_button_wakeup();
                s_button_wakeup_enabled = true;
            }
            ledRefresh();
        } else {
            ESP_LOGW(TAG, "Join/steering failed (signal 0x%x, status=%d), retrying", sig_type, signal_struct->esp_err_status);
            esp_zb_scheduler_alarm(bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        // esp_zb_sleep_enable(false) здесь раньше "выключало сон" — на
        // самом деле оно не блокирует сам сигнал CAN_SLEEP, а превращает
        // esp_zb_sleep_now() в мгновенный no-op, что на реальном железе
        // дало бешеный цикл "Going to sleep"/"Woke up" много раз в секунду
        // (тот же класс бага, что и Баг 4 из roadmap Этап 14, только
        // триггер — LEAVE, а не движение мотора). Правильный способ не
        // спать, пока не переподключились — тот же vTaskDelay-перехват в
        // CAN_SLEEP ниже, через условие !zigbee_connected.
        ESP_LOGW(TAG, "Device left the network — not sleeping until re-joined");
        s_steering_done = false;
        zigbee_connected = false;
        ledRefresh();
        break;

    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        // Пока мотор крутится (или калибровка/пейринг, или нет связи —
        // !zigbee_connected, чтобы не проспать переподключение, или ещё не
        // прошёл льготный период после джойна — s_stay_awake_until_ms,
        // иначе Z2M ZDO Active Endpoints Request может провалиться, если
        // чип уснёт раньше, чем координатор успеет опросить endpoint'ы)
        // — НЕ уходим
        // в реальный light sleep: энкодер (GPIO5) не настроен как источник
        // пробуждения, поэтому во время сна его импульсы просто теряются —
        // мотор мог бы проскочить мимо цели/лимита незамеченным. Короткий
        // vTaskDelay (не эквивалент bare "break;") — GPIO-прерывания при
        // этом продолжают работать как обычно (в отличие от light sleep),
        // а сама задержка даёт стеку время на внутреннюю работу. Просто
        // ничего не делать в ответ на CAN_SLEEP (bare break, без всякой
        // блокирующей паузы) пробовали в тестовом стенде — вызывало
        // зависание на 15+ секунд без прогресса стека (см.
        // esp32h2_light_sleep_test/roadmap.md, Этап 13) — vTaskDelay не то
        // же самое, что bare break, реального зависания не дал на практике.
        if (now_ms() < s_stay_awake_until_ms) {
            static uint32_t s_last_grace_log_ms = 0;
            if (now_ms() - s_last_grace_log_ms > 5000) {
                s_last_grace_log_ms = now_ms();
                ESP_LOGI(TAG, "[Join] Waiting for Z2M interview, %lu ms left", (unsigned long)(s_stay_awake_until_ms - now_ms()));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }
        // Дошли сюда — льготный период после джойна прошёл (иначе выше был break): если прошивка только что
        // установлена по OTA и мы всё ещё в сети — подтверждаем её (отмена отката).
        ota_confirm_new_image_if_needed();
        // s_ota_in_progress — идёт приём OTA-образа, не спим (передача идёт по поллингу, сон её сорвёт).
        if (motor_is_running || system_state != STATE_NORMAL || !zigbee_connected || s_ota_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }
        // Единственное место входа в РЕАЛЬНЫЙ сон во всём проекте — см.
        // заголовок файла и esp32h2_light_sleep_test/roadmap.md, Этап 8-13.
        ESP_LOGI(TAG, "Going to sleep");
        esp_zb_sleep_now();
        ESP_LOGI(TAG, "Woke up, cause=%d", (int)esp_sleep_get_wakeup_cause());
        break;

    default:
        ESP_LOGD(TAG, "Unhandled signal: %s (0x%x), status=%d", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 signal_struct->esp_err_status);
        break;
    }
}

// ========== ZIGBEE: ЗАДАЧА СТЕКА ==========

static void esp_zb_task(void *pvParameters)
{
    esp_zb_platform_config_t platform_config = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg =
            {
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
                .keep_alive = 3000, // см. esp32h2_light_sleep_test/roadmap.md, Этап 7 — не полагаемся на дефолт
            },
    };
    esp_zb_init(&zb_nwk_cfg);

    // Дефолтный sleep threshold стека — 20мс (esp_zigbee_core.h) — даёт
    // микро-сны без этой правки. Проверено на тестовом стенде.
    ESP_ERROR_CHECK(esp_zb_sleep_set_threshold(2000));

    // Эндпоинт собран вручную — та же причина и тот же проверенный паттерн,
    // что и в esp32h2_light_sleep_test/src/main.c: у esp_zb_window_covering_ep_create()
    // нет способа получить cluster_list обратно, чтобы добавить
    // Manufacturer/Model и Power Configuration (батарея). Последовательность
    // вызовов сверена с реализацией Arduino Zigbee-библиотеки
    // (ZigbeeWindowCovering.cpp/ZigbeeEP.cpp) поверх того же esp-zigbee-sdk.
    esp_zb_window_covering_cfg_t window_cfg = ESP_ZB_DEFAULT_WINDOW_COVERING_CONFIG();
    // power_source — часть esp_zb_basic_cluster_cfg_t, создаётся сразу
    // esp_zb_basic_cluster_create() (в отличие от manufacturer/model, которых
    // в этой структуре нет вовсе) — просто выставляем перед созданием.
    window_cfg.basic_cfg.power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;

    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&window_cfg.basic_cfg);
    char manufacturer[] = {8, 'E', 'S', 'P', '3', '2', '-', 'H', '2', '\0'};
    char model_id[] = {14, 'W', 'i', 'n', 'd', 'o', 'w', 'C', 'o', 'v', 'e', 'r', 'i', 'n', 'g', '\0'};
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id));

    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&window_cfg.identify_cfg);
    esp_zb_attribute_list_t *groups_cluster = esp_zb_groups_cluster_create(&window_cfg.groups_cfg);
    esp_zb_attribute_list_t *scenes_cluster = esp_zb_scenes_cluster_create(&window_cfg.scenes_cfg);
    esp_zb_attribute_list_t *window_covering_cluster = esp_zb_window_covering_cluster_create(&window_cfg.window_cfg);
    // CurrentPositionLiftPercentage — НЕ входит в esp_zb_window_covering_cluster_cfg_t
    // (там только covering_type/covering_status/covering_mode, сверено по
    // esp_zigbee_type.h) и не создаётся esp_zb_window_covering_cluster_create()
    // по умолчанию — это опциональный атрибут, добавляется отдельно, как и
    // manufacturer/model на Basic-кластере выше. Без этого Z2M отвечал
    // 'UNSUPPORTED_ATTRIBUTE' на попытку прочитать позицию (подтверждено на
    // реальном железе), а наш push-отчёт для него либо не уходил, либо
    // отвергался координатором как отчёт по незарегистрированному атрибуту.
    uint8_t initial_lift_percentage = 0;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                             ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_U8,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                             &initial_lift_percentage));

    // Power Configuration cluster — батарея. esp_zb_zcl_attr_list_create()
    // создаёт ПУСТОЙ список атрибутов (в отличие от *_cluster_create()
    // хелперов у стандартных кластеров) — оба атрибута добавляются явно, как
    // в ZigbeeEP.cpp::setPowerSource(). Начальные значения — заглушки,
    // реальные выставляются в reportStatusToZ2M() после первого измерения.
    uint8_t initial_battery_percent_x2 = 100; // 50% * 2
    uint8_t initial_battery_voltage = 0;
    esp_zb_attribute_list_t *power_config_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                                          &initial_battery_percent_x2));
    // esp_zb_power_config_cluster_add_attr() добавляет BatteryVoltage без
    // флага ACCESS_REPORTING (внутренний дефолт SDK для этого ID) —
    // esp_zb_zcl_report_attr_cmd_req() на нём возвращал ESP_ERR_NOT_SUPPORTED
    // на реальном железе, хотя BatteryPercentageRemaining через тот же
    // хелпер репортился нормально. Регистрируем явно через generic
    // add_attr с ACCESS_REPORTING, как и CurrentPositionLiftPercentage выше.
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                             ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                             &initial_battery_voltage));

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_groups_cluster(cluster_list, groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_scenes_cluster(cluster_list, scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_window_covering_cluster(cluster_list, window_covering_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(cluster_list, power_config_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // OTA-кластер (клиент) — обновление прошивки из Z2M. Значения по умолчанию для полей без своего смысла —
    // макросы из zcl/esp_zigbee_zcl_ota.h. Приём самого образа — ota_upgrade_handler(). Порядок и набор
    // служебных атрибутов (client data, server addr/endpoint) — как в официальном примере
    // esp-zigbee-sdk/examples/esp_zigbee_ota/ota_client.
    esp_zb_ota_cluster_cfg_t ota_cfg = {
        .ota_upgrade_file_version = FW_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_MANUFACTURER_CODE,
        .ota_upgrade_image_type = OTA_IMAGE_TYPE,
        .ota_min_block_reque = ESP_ZB_OTA_UPGRADE_MIN_BLOCK_PERIOD_DEF_VALUE,
        .ota_upgrade_file_offset = ESP_ZB_ZCL_OTA_UPGRADE_FILE_OFFSET_DEF_VALUE,
        .ota_upgrade_downloaded_file_ver = ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_FILE_VERSION_DEF_VALUE,
        .ota_upgrade_server_id = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_DEF_VALUE,
        .ota_image_upgrade_status = ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE,
    };
    esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_cfg);
    esp_zb_zcl_ota_upgrade_client_variable_t ota_client_variable = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_HW_VERSION,
        .max_data_size = OTA_MAX_DATA_SIZE,
    };
    uint16_t ota_server_addr = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ADDR_DEF_VALUE;
    uint8_t ota_server_endpoint = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ENDPOINT_DEF_VALUE;
    // Служебные атрибуты OTA-клиента — ТОЛЬКО через esp_zb_ota_cluster_add_attr() ("Add an attribute and
    // variables in OTA client cluster"), а не универсальным esp_zb_cluster_add_attr(): с типом TYPE_NULL
    // универсальный не сохраняет структуру client data, и стек при первой же OTA-команде от Z2M ("Check for
    // updates" -> Image Notify) читал NULL в zb_zcl_process_ota_upgrade_specific_commands_cli -> Load access
    // fault (проверено на железе).
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&ota_client_variable));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_server_addr));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_server_endpoint));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    // Кастомный кластер настроек мотора (см. define'ы CUSTOM_SETTINGS_CLUSTER_ID выше) — начальные значения
    // из NVS/дефолтов (loadMotorSettings() уже отработала к этому моменту, до esp_zb_task()). Запись из Z2M
    // ловится в settings_attr_write_handler() через ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID.
    uint8_t motor_invert_u8 = motor_invert ? 1 : 0;
    esp_zb_attribute_list_t *settings_cluster = esp_zb_zcl_attr_list_create(CUSTOM_SETTINGS_CLUSTER_ID);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(settings_cluster, CUSTOM_SETTINGS_CLUSTER_ID, ATTR_MIN_PWM_DUTY_ID, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_min_pwm_duty));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(settings_cluster, CUSTOM_SETTINGS_CLUSTER_ID, ATTR_MAX_PWM_DUTY_ID, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_max_pwm_duty));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(settings_cluster, CUSTOM_SETTINGS_CLUSTER_ID, ATTR_MOTOR_INVERT_ID, ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &motor_invert_u8));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, settings_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = ZIGBEE_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config));
    ESP_ERROR_CHECK(esp_zb_device_register(ep_list));

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_rx_on_when_idle(false); // sleepy end device

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

// ========== БЫСТРЫЙ ПОЛЛИНГ МОТОРА (замена loop(), только пока не спим) ==========

/**
 * @brief updatePosition()/checkMotorStopConditions() вызывались в Arduino
 * каждую итерацию loop(). Здесь — раз в 100мс, но ТОЛЬКО пока таймер реально
 * запущен (см. poll_timer_sync() — включается на время движения мотора/
 * калибровки, выключается в остальное время). Без этого условия таймер сам
 * по себе не давал стеку Zigbee ни разу просигналить CAN_SLEEP — проверено
 * на реальном железе (0 пробуждений за минуты работы с безусловным
 * 100мс-таймером).
 */
static void motor_poll_timer_cb(void *arg)
{
    updatePosition();
    checkMotorStopConditions();
}

// ========== НАСТРОЙКА GPIO/ПЕРИФЕРИИ ==========

static void configure_gpio(void)
{
    gpio_config_t mosfet_cfg = {
        .pin_bit_mask = 1ULL << MOSFET_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&mosfet_cfg);
    // См. gpio_sleep_sel_dis(ENCODER_PIN)/(MOTOR_IN1/2_PIN)/(LED_PIN) и
    // SKILL.md — без этого сам уровень MOSFET_PIN мог сбрасываться при входе
    // в light sleep (CONFIG_PM_SLP_DISABLE_GPIO), обесточивая LED/энкодер
    // прямо посреди мигающего паттерна вместо простого "замирания" на
    // последнем цвете.
    gpio_sleep_sel_dis(MOSFET_PIN);
    gpio_set_level(MOSFET_PIN, 1); // включаем периферию на старте

    gpio_config_t buzzer_cfg = {
        .pin_bit_mask = 1ULL << BUZZER_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&buzzer_cfg);

    // Общий ISR-сервис на все пины с прерываниями (энкодер + кнопки —
    // кнопки регистрирует сама библиотека espressif/button внутри себя).
    gpio_install_isr_service(0);
}

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    esp_log_level_set("zigbee", ESP_LOG_VERBOSE);
    esp_log_level_set("zboss", ESP_LOG_VERBOSE);
    esp_log_level_set("zdo", ESP_LOG_VERBOSE);

    // Версия прошивки и состояние OTA-образа. PENDING_VERIFY возможен только для образа, только что
    // установленного по OTA при CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y — тогда ставим таймер-предохранитель
    // (откат, если не подтвердим прошивку за OTA_VALIDATE_TIMEOUT_MS), а подтверждает её
    // ota_confirm_new_image_if_needed() после входа в сеть. Делаем ДО запуска Zigbee-задачи.
    ESP_LOGI(TAG, "Firmware file_version=0x%08x", (unsigned)FW_FILE_VERSION);
    {
        esp_ota_img_states_t ota_state;
        const esp_partition_t *running = esp_ota_get_running_partition();
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            s_ota_pending_verify = true;
            ESP_LOGW(TAG, "[OTA] Running a NEW firmware (pending verify) — must join the network within %d ms or roll back",
                     OTA_VALIDATE_TIMEOUT_MS);
            const esp_timer_create_args_t validate_args = {
                .callback = &ota_validate_timeout_cb,
                .name = "ota_validate",
            };
            ESP_ERROR_CHECK(esp_timer_create(&validate_args, &s_ota_validate_timer));
            ESP_ERROR_CHECK(esp_timer_start_once(s_ota_validate_timer, (uint64_t)OTA_VALIDATE_TIMEOUT_MS * 1000));
        }
    }

    configure_gpio();
    setupLED();
    setLEDColor(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(800)); // ВРЕМЕННО увеличено (было 100) для диагностики пропадающего красного
    setLEDColor(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    setLEDColor(0, 0, 255);
    vTaskDelay(pdMS_TO_TICKS(800));
    setLEDOff();

    initPWM();
    motorSetDirectionAndSpeed(DIR_NONE, 0);

    setupButtons();
    initEncoder();
    initBatteryADC();

    loadPosition();
    loadMotorSettings();

    // Найдено чтением pm_impl.c (esp32h2_light_sleep_test/roadmap.md, Этап
    // 13): без этого вызова автоматический tickless-idle путь никогда не
    // выбирает PM_MODE_LIGHT_SLEEP, независимо от sdkconfig-флагов.
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    // Быстрый поллинг мотора — только СОЗДАЁМ таймер здесь, не запускаем.
    // Старт/стоп — через poll_timer_sync() из motorStart/motorStop и точек
    // смены system_state (см. их определения выше). Пока мотор не крутится
    // и мы в STATE_NORMAL, этот таймер не тикает вообще — не мешает CAN_SLEEP.
    const esp_timer_create_args_t poll_timer_args = {
        .callback = &motor_poll_timer_cb,
        .name = "motor_poll",
    };
    ESP_ERROR_CHECK(esp_timer_create(&poll_timer_args, &s_motor_poll_timer));

    // Мигание LED — та же схема: таймер только создаётся здесь, запускает/
    // останавливает его led_blink_timer_sync() из ledRefresh() в зависимости
    // от того, мигающий ли сейчас паттерн (см. её определение выше).
    const esp_timer_create_args_t led_blink_timer_args = {
        .callback = &led_blink_timer_cb,
        .name = "led_blink",
    };
    ESP_ERROR_CHECK(esp_timer_create(&led_blink_timer_args, &s_led_blink_timer));

    // Одноразовый таймер отложенной ZCL-отправки — см. report_defer_timer_cb()
    // и комментарий у s_report_defer_timer.
    const esp_timer_create_args_t report_defer_timer_args = {
        .callback = &report_defer_timer_cb,
        .name = "report_defer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&report_defer_timer_args, &s_report_defer_timer));

    // Почасовой отчёт — период (1ч) на много порядков больше sleep-порога
    // (2с), сну не мешает, можно держать всегда включённым.
    const esp_timer_create_args_t hourly_timer_args = {
        .callback = &hourly_report_cb,
        .name = "hourly_report",
    };
    esp_timer_handle_t hourly_report_timer;
    ESP_ERROR_CHECK(esp_timer_create(&hourly_timer_args, &hourly_report_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(hourly_report_timer, 3600ULL * 1000 * 1000));

    // ledRefresh() до этого места вызывается только реактивно — на смену
    // motor_is_running или на смену zigbee_connected (событие "смены" внутри
    // esp_zb_app_signal_handler()). Если стык JOIN/STEERING ни разу не
    // случился с момента загрузки (например, сеть недоступна после
    // factory reset), это событие "смены" никогда не произойдёт, и вся
    // LED-логика (паттерн, таймер мигания, MOSFET питания LED/энкодера)
    // ни разу не инициализируется — светодиод остаётся тёмным бессрочно.
    // Явный вызов здесь даёт корректное начальное состояние сразу при
    // старте, не дожидаясь первого реактивного триггера.
    ledRefresh();

    ESP_LOGI(TAG, "System Ready. Current position: %d pulses (%d%%), max=%d", (int)current_pulse_position,
             (int)pulsesToPercent(current_pulse_position), (int)current_max_pulse_position);
}
