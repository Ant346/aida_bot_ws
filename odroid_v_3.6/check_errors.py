import can
import struct
import time

INTERFACES = ['can_front', 'can_rear']
BITRATE = 250000

AXIS_MAP = {
    0x001: "Axis 0 (Node 0)",
    0x021: "Axis 1 (Node 1)"
}

def check_errors():
    buses = {}
    for iface in INTERFACES:
        try:
            buses[iface] = can.interface.Bus(channel=iface, bustype='socketcan', bitrate=BITRATE)
            print(f"📡 Подключено к {iface}...")
        except Exception as e:
            print(f"⚠️ Не удалось открыть {iface}: {e}")

    if not buses:
        print("❌ Нет доступных CAN интерфейсов. Убедитесь, что setup_can.sh запущен и адаптеры подключены.")
        return

    print("\nОжидание heartbeat сообщений от моторов...\n(Для остановки нажмите Ctrl+C)")

    # Чтобы не засорять терминал, будем выводить сообщение только если состояние изменилось
    last_printed_state = {}

    try:
        while True:
            for iface, bus in buses.items():
                msg = bus.recv(0.1) # Ждем сообщение 100мс
                if msg is None:
                    continue
                    
                if msg.arbitration_id in AXIS_MAP:
                    axis_error = struct.unpack('<I', msg.data[0:4])[0]
                    axis_state = msg.data[4]
                    axis_name = AXIS_MAP[msg.arbitration_id]
                    
                    state_key = f"{iface}_{axis_name}"
                    
                    if axis_error != 0:
                        current_status = f"❌ {iface:<9} | {axis_name:<15} | Состояние: {axis_state} | ОШИБКА: 0x{axis_error:X}"
                        if last_printed_state.get(state_key) != current_status:
                            print(current_status)
                            last_printed_state[state_key] = current_status
                    else:
                        current_status = f"✅ {iface:<9} | {axis_name:<15} | Состояние: {axis_state} | Ошибок нет"
                        if last_printed_state.get(state_key) != current_status:
                            print(current_status)
                            last_printed_state[state_key] = current_status
    except KeyboardInterrupt:
        print("\nСкрипт остановлен.")
    finally:
        for bus in buses.values():
            bus.shutdown()

if __name__ == "__main__":
    check_errors()
