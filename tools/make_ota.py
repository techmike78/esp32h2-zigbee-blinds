#!/usr/bin/env python3
"""Упаковка прошивки ESP32-H2 в Zigbee OTA-образ (.ota) для Zigbee2MQTT + локальный индекс Z2M.

Формат файла — стандартный Zigbee OTA (ZCL, little-endian): заголовок (0x0BEEF11E, версия 0x0100, длина,
field control, manufacturer, image type, file version, zigbee stack version, header string[32],
total image size), затем один sub-element: tag 0x0000 (Upgrade Image), длина uint32, данные = app .bin.

Версия прошивки, код производителя и тип образа берутся из src/main.c (FW_FILE_VERSION, OTA_MANUFACTURER_CODE,
OTA_IMAGE_TYPE) — единый источник, чтобы .ota, индекс Z2M и прошивка не расходились.

Раскладка файлов — как в документации/практике Z2M: индекс лежит рядом с configuration.yaml, образы — в
подкаталоге ota/, а "url" в индексе задаётся относительно каталога данных Z2M ("ota/<файл>.ota").

Использование:
    python tools/make_ota.py                       # собрать tools/ota_out/ota/<...>.ota + tools/ota_out/my_index.json
    python tools/make_ota.py --deploy "\\\\ha\\config\\zigbee2mqtt"   # + положить в каталог данных Z2M (рядом с configuration.yaml)
"""
import argparse
import json
import os
import re
import struct
import sys

OTA_FILE_ID = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_LENGTH = 56          # без необязательных полей
OTA_FIELD_CONTROL = 0x0000
ZIGBEE_STACK_VERSION = 0x0002   # Zigbee PRO
TAG_UPGRADE_IMAGE = 0x0000
ESP_IMAGE_MAGIC = 0xE9
OTA_SLOT_SIZE = 0x140000        # размер app0/app1 в zigbee.csv


def read_define(main_c_text, name):
    m = re.search(r"^\s*#define\s+" + re.escape(name) + r"\s+(0x[0-9A-Fa-f]+|\d+)\b", main_c_text, re.M)
    if not m:
        sys.exit(f"Не нашёл #define {name} в main.c")
    return int(m.group(1), 0)


def build_ota(app_bin, manufacturer, image_type, file_version, header_string):
    sub_element = struct.pack("<HI", TAG_UPGRADE_IMAGE, len(app_bin)) + app_bin
    total_size = OTA_HEADER_LENGTH + len(sub_element)
    hs = header_string.encode("ascii")[:32].ljust(32, b"\x00")
    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_FILE_ID, OTA_HEADER_VERSION, OTA_HEADER_LENGTH, OTA_FIELD_CONTROL,
        manufacturer, image_type, file_version, ZIGBEE_STACK_VERSION, hs, total_size,
    )
    assert len(header) == OTA_HEADER_LENGTH, len(header)
    return header + sub_element


def verify_ota(data, manufacturer, image_type, file_version, app_bin):
    (fid, hver, hlen, fc, manuf, itype, fver, stack, _hs, total) = struct.unpack_from("<IHHHHHIH32sI", data, 0)
    assert fid == OTA_FILE_ID and hver == OTA_HEADER_VERSION and hlen == OTA_HEADER_LENGTH and fc == 0
    assert (manuf, itype, fver) == (manufacturer, image_type, file_version)
    assert total == len(data), (total, len(data))
    tag, length = struct.unpack_from("<HI", data, hlen)
    assert tag == TAG_UPGRADE_IMAGE and length == len(app_bin) and hlen + 6 + length == total
    assert data[hlen + 6:hlen + 6 + length] == app_bin


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", default=os.path.join(root, ".pio", "build", "esp32-h2-devkitm-1", "firmware.bin"))
    ap.add_argument("--main-c", default=os.path.join(root, "src", "main.c"))
    ap.add_argument("--out-dir", default=os.path.join(root, "tools", "ota_out"))
    ap.add_argument("--header-string", default="ESP32-H2 blinds")
    ap.add_argument("--index-name", default="my_index.json",
                    help="имя индекса; то же значение — в configuration.yaml: ota.zigbee_ota_override_index_location")
    ap.add_argument("--deploy", help="каталог данных Z2M (рядом с configuration.yaml; например, смонтированная Samba-шара HA "
                                     "\\\\ha\\config\\zigbee2mqtt), куда положить ota/<файл>.ota и обновить индекс")
    args = ap.parse_args()

    main_c = open(args.main_c, encoding="utf-8").read()
    file_version = read_define(main_c, "FW_FILE_VERSION")
    manufacturer = read_define(main_c, "OTA_MANUFACTURER_CODE")
    image_type = read_define(main_c, "OTA_IMAGE_TYPE")

    app_bin = open(args.bin, "rb").read()
    if not app_bin or app_bin[0] != ESP_IMAGE_MAGIC:
        sys.exit(f"{args.bin}: не образ приложения ESP (первый байт {app_bin[:1].hex() or 'пусто'}, ожидался 0xE9)")
    if len(app_bin) > OTA_SLOT_SIZE:
        sys.exit(f"Образ {len(app_bin)} байт не влезает в OTA-слот {OTA_SLOT_SIZE}")

    ota = build_ota(app_bin, manufacturer, image_type, file_version, args.header_string)
    verify_ota(ota, manufacturer, image_type, file_version, app_bin)

    name = f"{manufacturer:04X}-{image_type:04X}-{file_version:08X}.ota"
    rel_url = "ota/" + name  # относительно каталога данных Z2M (рядом с configuration.yaml)
    entry = {
        "url": rel_url,
        "fileVersion": file_version,
        "fileSize": len(ota),
        "manufacturerCode": manufacturer,
        "imageType": image_type,
    }

    def write_tree(base_dir):
        """Кладёт <base>/ota/<name>.ota и обновляет <base>/<index-name>, не трогая чужие записи индекса."""
        os.makedirs(os.path.join(base_dir, "ota"), exist_ok=True)
        with open(os.path.join(base_dir, "ota", name), "wb") as f:
            f.write(ota)
        index_path = os.path.join(base_dir, args.index_name)
        index = []
        if os.path.exists(index_path):
            index = [e for e in json.load(open(index_path, encoding="utf-8"))
                     if not (e.get("manufacturerCode") == manufacturer and e.get("imageType") == image_type)]
        index.append(entry)
        with open(index_path, "w", encoding="utf-8") as f:
            json.dump(index, f, indent=2)
        return index_path

    local_index = write_tree(args.out_dir)
    print(f"OTA image : {os.path.join(args.out_dir, 'ota', name)} ({len(ota)} bytes, app {len(app_bin)} bytes)")
    print(f"  version=0x{file_version:08X} manufacturer=0x{manufacturer:04X} image_type=0x{image_type:04X}")
    print(f"Z2M index : {local_index}  (url={rel_url})")

    if args.deploy:
        deployed_index = write_tree(args.deploy)
        print(f"Deployed  : {os.path.join(args.deploy, 'ota', name)}")
        print(f"Deployed  : {deployed_index}")
        print(f"В configuration.yaml Z2M должно быть: ota: {{ zigbee_ota_override_index_location: {args.index_name} }}")


if __name__ == "__main__":
    main()
