# Brake controller status reported by the STM32 over CAN (ID 0x131).
float32 current_ma       # Brake current measured by the INA240 sensor, in milliamps
bool relay_active        # State of the G2R-24 relay (true = ON / engaged, false = OFF)
uint8 watchdog_status    # 0 = Normal, 1 = Triggered (E-Stop or open-load fault)
float32 servo_angle_deg  # Brake angle the STM32 reports holding (CAN 0x133): flash value at boot, live command after (0–180°)
