"""Пульт и RealSense делят hidraw. Запрос MAC (отчёт 0x81) на этом ядре
возвращает EPIPE, пока устройство держит драйвер playstation, и ds4drv
из-за этого бросает весь перебор. Входные отчёты при этом идут. MAC для
телеопа не нужен.
"""
try:
    from ds4drv.backends.hidraw import HidrawDS4Device, HidrawUSBDS4Device
except Exception:
    HidrawUSBDS4Device = None

if HidrawUSBDS4Device is not None:
    def set_operational(self):
        try:
            addr = self.read_feature_report(0x81, 6)[1:]
            addr = ["{0:02x}".format(c) for c in bytearray(addr)]
            addr = ":".join(reversed(addr)).upper()
            self.device_name = "{0} {1}".format(addr, self.device_name)
            self.device_addr = addr
        except OSError:
            pass

    def write_report(self, report_id, data):
        try:
            HidrawDS4Device.write_report(self, report_id, data)
        except OSError:
            pass

    HidrawUSBDS4Device.set_operational = set_operational
    HidrawUSBDS4Device.write_report = write_report
