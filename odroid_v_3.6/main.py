import can
import time
import struct
import logging

# --- Конфигурация ---
CAN_INTERFACE = 'can_rear'
# ВАЖНО: Убедитесь, что эта скорость совпадает с настройкой в ODrive
# и в команде `ip link set`
BITRATE = 250000

# --- Определения CAN для axis0 (node_id = 0) ---
# ID оси, которую мы будем контролировать.
# Для axis0 должен быть 0 в соответствии с вашим DBC
AXIS_ID = 0
# CAN ID формируется как: (ID_оси << 5) | ID_команды
# Для axis0 с AXIS_ID = 0, CAN ID равен просто ID_команды.
CAN_ID_SET_AXIS_STATE = 0x007
CAN_ID_SET_INPUT_VEL = 0x00D

# --- Определения состояний ODrive ---
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


def set_axis_state(bus, axis_id, state):
    """Отправляет команду на установку состояния оси (IDLE, CLOSED_LOOP и т.д.)."""
    can_id = (axis_id << 5) | CAN_ID_SET_AXIS_STATE
    # Данные - это 32-битное целое число
    data = state.to_bytes(4, 'little')
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        data_hex = ' '.join(f'{b:02X}' for b in data)
        data_hex_compact = ''.join(f'{b:02X}' for b in data)
        logging.info(f"КОМАНДА: Установить состояние {state} для оси {axis_id}")
        print(f"  → CAN ID: 0x{can_id:03X} (axis_id={axis_id}, cmd=0x{CAN_ID_SET_AXIS_STATE:02X})")
        print(f"  → Данные: {data_hex} (state={state})")
        print(f"  → cansend: cansend {CAN_INTERFACE} {can_id:03X}#{data_hex_compact}")
    except can.CanError:
        logging.error("ОШИБКА: Сообщение не было отправлено")

def set_velocity(bus, axis_id, velocity, torque_ff=0.0):
    """Отправляет команду на установку скорости вращения."""
    can_id = (axis_id << 5) | CAN_ID_SET_INPUT_VEL
    # Данные: 4 байта для скорости (float) и 4 байта для опережающего момента (float)
    # Используем struct для упаковки float в 4 байта в формате little-endian ('<f')
    data = struct.pack('<f', velocity) + struct.pack('<f', torque_ff)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        data_hex = ' '.join(f'{b:02X}' for b in data)
        data_hex_compact = ''.join(f'{b:02X}' for b in data)
        logging.info(f"КОМАНДА: Установить скорость {velocity:.2f} об/с для оси {axis_id}")
        print(f"  → CAN ID: 0x{can_id:03X} (axis_id={axis_id}, cmd=0x{CAN_ID_SET_INPUT_VEL:02X})")
        print(f"  → Данные: {data_hex} (velocity={velocity:.2f}, torque_ff={torque_ff:.2f})")
        print(f"  → cansend: cansend {CAN_INTERFACE} {can_id:03X}#{data_hex_compact}")
    except can.CanError:
        logging.error("ОШИБКА: Сообщение не было отправлено")


def main():
    """
    Запускает тестовую последовательность для управления мотором ODrive по CAN.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
    bus = None
    try:
        # Инициализация CAN-шины
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=BITRATE)
        logging.info(f"Успешное подключение к CAN-интерфейсу {CAN_INTERFACE}")

        # --- Тестовая последовательность ---
        print("\n--- Демо-скрипт управления ODrive по CAN ---")
        print(f"Целевая ось: axis0 (node_id = {AXIS_ID})")
        input("Нажмите Enter, чтобы начать...")

        # 1. Включить мотор (перевести в режим управления с обратной связью)
        print("\nШаг 1: Включение мотора (CLOSED_LOOP_CONTROL)...")
        set_axis_state(bus, AXIS_ID, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(2)  # Пауза, чтобы мотор успел включиться

        # 2. Задать целевую скорость
        test_velocity = 5.0
        print(f"\nШаг 2: Вращение со скоростью {test_velocity} об/с...")
        set_velocity(bus, AXIS_ID, test_velocity, 0.0)
        time.sleep(5)  # Вращаем мотор 5 секунд

        # 3. Остановить мотор (установить скорость 0)
        print("\nШаг 3: Остановка мотора (скорость 0.0 об/с)...")
        set_velocity(bus, AXIS_ID, 0.0, 0.0)
        time.sleep(3)  # Пауза для полной остановки

        # 4. Выключить мотор (перевести в режим ожидания)
        print("\nШаг 4: Выключение мотора (IDLE)...")
        set_axis_state(bus, AXIS_ID, AXIS_STATE_IDLE)
        time.sleep(1)

        print("\n--- Тестовая последовательность успешно завершена! ---")

    except Exception as e:
        logging.error(f"Произошла ошибка: {e}")
        logging.error(f"Убедитесь, что интерфейс '{CAN_INTERFACE}' поднят и ODrive включен.")

    finally:
        if bus:
            # В любом случае (даже при ошибке) пытаемся выключить мотор и закрыть соединение
            logging.info("Финальная проверка: отправка команды IDLE для безопасности.")
            set_axis_state(bus, AXIS_ID, AXIS_STATE_IDLE)
            bus.shutdown()
            logging.info("Соединение с CAN-шиной закрыто.")


if __name__ == "__main__":
    main()