import serial  # to communicate with the hoverboard
import threading
import time


# Класс для взаимодействия с ховербордом по Serial-порту
# Этот класс оставлен без изменений, так как он корректно реализует протокол
class Hoverboard_serial:

    def __init__(self, adresse, baud):
        try:
            self.uart = serial.Serial(adresse, baud, timeout=1)
        except serial.SerialException as e:
            print(f"Ошибка: не удалось открыть порт {adresse}. Убедитесь, что он не занят другой программой.")
            print(f"Системная ошибка: {e}")
            exit()

        self.startBytes = bytes.fromhex('ABCD')[::-1]  # lower byte first
        self.incomingBytesPrev = bytes()

    def send_command(self, steer, speed):
        '''
        Отправляет байтовую посылку для управления ховербордом

        :param steer: -1000...1000 (управление поворотом)
        :param speed: -1000...1000 (управление скоростью)

        Структура посылки:
        Start frame(unsigned int16) : 0xABCD
        Steer(signed int16) : -1000 to 1000
        Speed(signed int16) : -1000 to 1000
        Checksum(unsigned int16) : XOR checksum
        '''
        steerBytes = steer.to_bytes(2, byteorder="little", signed=True)
        speedBytes = speed.to_bytes(2, byteorder="little", signed=True)
        checksumBytes = bytes(a ^ b ^ c for (a, b, c) in zip(self.startBytes, steerBytes, speedBytes))

        command = self.startBytes + steerBytes + speedBytes + checksumBytes

        self.uart.write(command)

    def receive_feedback(self):
        '''
        Получает и парсит данные обратной связи от ховерборда.
        ВЕРСИЯ БЕЗ ПРОВЕРКИ КОНТРОЛЬНОЙ СУММЫ (CHECKSUM).
        '''
        # Ищем стартовый фрейм 0xABCD
        incomingByte = self.uart.read()
        if not incomingByte:
            return None

        bufStartFrame = self.incomingBytesPrev + incomingByte

        if bufStartFrame != self.startBytes:
            self.incomingBytesPrev = incomingByte
            return None

        # Если стартовый фрейм найден, читаем остальную часть пакета (14 байт)
        data_packet = self.uart.read(14)
        if len(data_packet) < 14:
            self.incomingBytesPrev = bytes()
            return None

        # Сразу парсим данные, ИГНОРИРУЯ проверку контрольной суммы
        cmd1 = int.from_bytes(data_packet[0:2], 'little', signed=True)
        cmd2 = int.from_bytes(data_packet[2:4], 'little', signed=True)
        speedR_meas = int.from_bytes(data_packet[4:6], 'little', signed=True)
        speedL_meas = int.from_bytes(data_packet[6:8], 'little', signed=True)
        batVoltage = int.from_bytes(data_packet[8:10], 'little', signed=True)
        boardTemp = int.from_bytes(data_packet[10:12], 'little', signed=True)
        # Последние 2 байта (checksum) мы просто игнорируем

        feedback = {
            "cmd1": cmd1,
            "cmd2": cmd2,
            "speedR_meas": speedR_meas,
            "speedL_meas": speedL_meas,
            "batVoltage": batVoltage / 100.0,  # Приводим к Вольтам
            "boardTemp": boardTemp / 10.0,  # Приводим к градусам Цельсия
        }

        self.incomingBytesPrev = bytes()  # Сбрасываем для поиска следующего пакета
        return feedback

    def close(self):
        # Перед закрытием отправляем команду остановки
        print("Отправка команды остановки...")
        self.send_command(0, 0)
        time.sleep(0.1)
        self.send_command(0, 0)
        time.sleep(0.1)
        self.uart.close()
        print("Порт закрыт.")


# =======================================================================
# =================== ИЗМЕНЕННАЯ ФУНКЦИЯ ================================
# =======================================================================

# Поток для отправки команд
def thread_send_command(hover_serial):
    # --- ЗАДАЙТЕ ВАШИ ЗНАЧЕНИЯ ЗДЕСЬ ---
    # Установите желаемую скорость (от -1000 до 1000)
    # Положительное значение - вперед, отрицательное - назад.
    FIXED_SPEED = 500

    # Установите желаемое значение поворота (от -1000 до 1000)
    # Положительное - в одну сторону, отрицательное - в другую.
    STEER_VALUE = 500
    # -------------------------------------

    TIME_SEND = 0.02  # Интервал отправки команд (20 мс = 50 Гц)

    print("--- Поток отправки команд запущен ---")
    print(f'--> Отправка команды: Скорость={FIXED_SPEED}, Поворот={STEER_VALUE}')

    while not stop_threads:
        # Отправляем одну и ту же команду постоянно
        hover_serial.send_command(STEER_VALUE, FIXED_SPEED)

        # Пауза перед следующей отправкой
        time.sleep(TIME_SEND)

    print("--- Поток отправки команд остановлен ---")


# Поток для получения обратной связи
def thread_receive_feedback(hover_serial):
    print("--- Поток получения данных запущен ---")
    while not stop_threads:
        feedback = hover_serial.receive_feedback()
        if feedback:
            # Уменьшим частоту вывода, чтобы не засорять консоль
            print(f'Receiving:\t {feedback}')
        # Небольшая пауза, чтобы не загружать процессор
        time.sleep(0.02) # Можно увеличить паузу до интервала отправки
    print("--- Поток получения данных остановлен ---")


if __name__ == "__main__":
    # --- НАСТРОЙКИ ---
    # Укажите ваш порт
    SERIAL_PORT = 'COM10'
    # Скорость по умолчанию из документации
    SERIAL_BAUD = 115200

    print(f"Подключение к {SERIAL_PORT} на скорости {SERIAL_BAUD}...")
    hover_serial = Hoverboard_serial(SERIAL_PORT, SERIAL_BAUD)

    stop_threads = False  # Флаг для остановки потоков

    try:
        thread1 = threading.Thread(target=thread_send_command, args=(hover_serial,))
        thread2 = threading.Thread(target=thread_receive_feedback, args=(hover_serial,))

        thread1.start()
        thread2.start()

        # Ждем, пока пользователь не нажмет Ctrl+C
        while thread1.is_alive() and thread2.is_alive():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nПолучено прерывание с клавиатуры (Ctrl+C)...")
        stop_threads = True  # Сигнализируем потокам о необходимости завершения
        thread1.join()
        thread2.join()

    except Exception as e:
        print(f"Произошла ошибка: {e}")
        stop_threads = True

    finally:
        hover_serial.close()