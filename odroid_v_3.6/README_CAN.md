# CAN Interface Setup Guide

## Быстрый старт

1. **Установите udev правила** (если еще не сделано):
   ```bash
   sudo ./setup_rules.sh
   ```

2. **Запустите CAN интерфейсы**:
   ```bash
   sudo ./start_can.sh
   ```

3. **Проверьте статус**:
   ```bash
   ip link show can0
   ip link show can1
   ```

4. **Тестирование**:
   ```bash
   # Мониторинг сообщений
   candump can0
   candump can1
   
   # Отправка тестового сообщения
   cansend can0 123#DEADBEEF
   ```

5. **Остановка**:
   ```bash
   sudo ./stop_can.sh
   ```

## Конфигурация

- **Битрейт по умолчанию**: 250000 bps (совпадает с main.py)
- **Front CAN**: `/dev/can_front` -> `can0`
- **Rear CAN**: `/dev/can_rear` -> `can1`

## Изменение битрейта

Если нужно использовать другой битрейт:
```bash
sudo CAN_BITRATE=500000 ./start_can.sh
```

## Устранение проблем

### "No such device" при использовании candump
- Убедитесь, что запустили `sudo ./start_can.sh`
- Проверьте, что устройства существуют: `ls -la /dev/can_*`
- Проверьте статус интерфейсов: `ip link show | grep can`

### Устройства не найдены
- Проверьте подключение USB адаптеров
- Убедитесь, что udev правила установлены: `sudo ./setup_rules.sh`
- Перезагрузите udev: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### Ошибка при запуске slcand
- Убедитесь, что устройство не используется другим процессом
- Попробуйте остановить интерфейсы: `sudo ./stop_can.sh`
- Проверьте права доступа: `ls -la /dev/can_*`

## Использование с Python

В `main.py` используется библиотека `python-can`:
```python
import can
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=250000)
```

Убедитесь, что битрейт в Python коде совпадает с настройкой в `start_can.sh`.

