import * as m from 'zigbee-herdsman-converters/lib/modernExtend';
import {Zcl} from 'zigbee-herdsman';

// Кастомный кластер настроек мотора (CUSTOM_SETTINGS_CLUSTER_ID в main.c), значения читаются/пишутся
// напрямую как raw-атрибуты — гердсман не знает про этот кластер, поэтому cluster/attribute задаются
// числами, а не именами (см. ClusterWithAttribute/{ID,type} в zigbee-herdsman-converters/lib/modernExtend.ts,
// пример с manufacturer-specific кластером 0xfc06 из открытых конвертеров с тем же паттерном).
const SETTINGS_CLUSTER = 0xfc00;

export default {
    zigbeeModel: ['WindowCovering'],
    model: 'WindowCovering',
    vendor: 'ESP32-H2',
    description: 'Window Covering with battery support',
    // OTA-обновление прошивки из Z2M (zigbee-herdsman-converters 26.x: Definition.ota — boolean или объект-фильтр
    // {modelId, manufacturerName, otaHeaderString, hardwareVersionMin/Max}). Образ выбирается по
    // manufacturerCode + imageType из запроса самого устройства (OTA_MANUFACTURER_CODE/OTA_IMAGE_TYPE в main.c) —
    // индекс лежит в my_index.json, см. tools/make_ota.py.
    ota: true,
    extend: [
        m.windowCovering({controls: ["lift"]}),
        m.battery({percentage: true, voltage: true}),
        // Настройки мотора из кастомного кластера 0xFC00 (ATTR_MIN_PWM_DUTY_ID/ATTR_MAX_PWM_DUTY_ID/
        // ATTR_MOTOR_INVERT_ID в main.c) — без этих exposes сам факт, что устройство отдаёт такой
        // кластер при интервью, ничего в UI Z2M не показывает.
        m.numeric({
            name: 'min_pwm_duty',
            cluster: SETTINGS_CLUSTER,
            attribute: {ID: 0x0000, type: Zcl.DataType.UINT16},
            description: 'Минимальная скважность ШИМ мотора (медленная зона у краёв хода)',
            access: 'ALL',
            valueMin: 0,
            valueMax: 1023,
            entityCategory: 'config',
        }),
        m.numeric({
            name: 'max_pwm_duty',
            cluster: SETTINGS_CLUSTER,
            attribute: {ID: 0x0001, type: Zcl.DataType.UINT16},
            description: 'Максимальная скважность ШИМ мотора (полный ход)',
            access: 'ALL',
            valueMin: 0,
            valueMax: 1023,
            entityCategory: 'config',
        }),
        m.binary({
            name: 'motor_invert',
            cluster: SETTINGS_CLUSTER,
            attribute: {ID: 0x0002, type: Zcl.DataType.BOOLEAN},
            description: 'Инверсия направления вращения мотора',
            access: 'ALL',
            valueOn: [true, true],
            valueOff: [false, false],
            entityCategory: 'config',
        }),
    ],
    // Без configure - modernExtend сам все настроит
};