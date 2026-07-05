import argparse
import math
import serial
import serial.tools.list_ports
import time
from typing import Optional, List
from module_rtsi import rtsi
import const

RTSI_FREQ = 125
SERIAL_BAUDRATE = 115200
DEFAULT_HOST = const.eliteIP

# Имена регистров для подписки
OUTPUT_VARIABLES = 'actual_TCP_pose,output_int_register0'
INPUT_REGISTERS = [
    'input_double_register0',
    'input_double_register1',
    'input_double_register2',
    'input_double_register3',
    'input_double_register4',
    'input_double_register5'
]
INPUT_VARIABLES = ','.join(INPUT_REGISTERS)


def parse_arguments() -> argparse.Namespace:
    """Разбор аргументов командной строки."""
    parser = argparse.ArgumentParser(
        description='Управление роботом через RTSI и COM-порт.'
    )
    parser.add_argument(
        '--host',
        default=DEFAULT_HOST,
        help=f'IP-адрес робота (по умолчанию: {DEFAULT_HOST})'
    )
    return parser.parse_args()


def connect_serial(baudrate: int = SERIAL_BAUDRATE) -> Optional[serial.Serial]:
    """
    Подключение к COM-порту с интерактивным выбором.
    """
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[ERR] Нет доступных COM-портов.")
        return None

    print("[INF] Доступные COM-порты:")
    for port in ports:
        print(f"  → {port.device} — {port.description}")

    port_name = input("\n[INP] Введите имя порта (например, COM3): ").strip()
    if not port_name:
        print("[ERR] Имя порта не указано.")
        return None

    try:
        ser = serial.Serial(port_name, baudrate, timeout=1)
        print(f"[OK] Подключено к {port_name} [{baudrate} бод]")
        return ser
    except Exception as e:
        print(f"[ERR] Ошибка подключения к порту: {e}")
        return None


def parse_servoj_command(raw_line: str) -> Optional[List[float]]:
    """
    Преобразование строки вида "D|val1,val2,...,val6" в список углов (радианы).

    move_point = [v0, v1-90, v2, v3-90, v4+90, v5]
    Затем переводим в радианы.
    """

    if not raw_line or not raw_line.startswith('D|'):
        return None

    try:
        data_part = raw_line[2:].strip()
        values = [float(x) for x in data_part.split(',')]
        if len(values) != 6:
            print(f"[WARN] Ожидалось 6 значений, получено {len(values)}")
            return None

        # Преобразование согласно спецификации
        move_point = [
            values[0],
            values[1] - 90.0,
            values[2],
            values[3] - 90.0,
            values[4] + 90.0,
            values[5]
        ]

        radians_point = [math.radians(angle) for angle in move_point]
        return radians_point

    except ValueError as e:
        print(f"[ERR] Ошибка преобразования чисел: {e}")
        return None
    except Exception as e:
        print(f"[ERR] Непредвиденная ошибка при разборе строки: {e}")
        return None


def update_input_registers(input_data, angles: List[float]) -> None:
    """
    Запись шести углов в соответствующие input_double_register поля.
    """

    for i, value in enumerate(angles):
        setattr(input_data, INPUT_REGISTERS[i], value)


def main() -> None:
    args = parse_arguments()

    # Подключение к RTSI
    print(f"[INF] Подключение к роботу по адресу {args.host} ...")
    rt = rtsi(args.host)
    try:
        rt.connect()
        rt.version_check()
        version = rt.controller_version()
        print(f"[OK] Подключено к контроллеру версии: {version}")
    except Exception as e:
        print(f"[ERR] Не удалось подключиться к RTSI: {e}")
        return

    # Подписка на выходные данные
    output_data = rt.output_subscribe(OUTPUT_VARIABLES, RTSI_FREQ)
    # Подписка на входные регистры
    input_data = rt.input_subscribe(INPUT_VARIABLES)

    rt.start()
    print("[INF] RTSI запущен")

    ser = connect_serial()
    if ser is None:
        rt.disconnect()
        return

    first_iteration = True
    start_pose = None
    run = True

    print("[WAIT] Ожидание сигнала от робота (output_int_register0 == 1) ...")

    try:
        while run:
            recv = rt.get_output_data()
            if recv is None:
                time.sleep(0.001)
                continue

            if first_iteration:
                first_iteration = False
                for reg in INPUT_REGISTERS:
                    setattr(input_data, reg, 0.0)
                rt.set_input(input_data)
                print("[INIT] Входные регистры обнулены")

            if recv.output_int_register0 == 1:
                if ser.in_waiting > 0:
                    line = ser.readline()
                    try:
                        decoded = line.decode('utf-8').strip()
                        print(f"[RX] Получено: {decoded}")

                        angles = parse_servoj_command(decoded)
                        if angles is None:
                            print("[ERR] Некорректная команда, пропускаем")
                            continue

                        update_input_registers(input_data, angles)
                        rt.set_input(input_data)
                        print(f"[CMD] Отправлено в RTSI: {angles}")

                        if start_pose is None:
                            start_pose = recv.actual_TCP_pose
                            print(f"[POS] Начальная поза TCP: {start_pose}")

                    except UnicodeDecodeError:
                        hex_data = line.hex()
                        print(f"[HEX] {hex_data}")
                    except Exception as e:
                        print(f"[ERR] Ошибка обработки данных: {e}")
                        run = False
            else:
                pass

    except KeyboardInterrupt:
        print("\n[STOP] Остановка по запросу пользователя")
    except Exception as e:
        print(f"[ERROR] Критическая ошибка: {e}")
    finally:
        print("[SHUTDOWN] Завершение работы...")
        rt.pause()
        rt.disconnect()
        ser.close()
        print("[OK] Программа завершена")


if __name__ == "__main__":
    main()
