# Brake controller status reported by the STM32 over CAN (ID 0x131).
float32 current_ma       # Brake current measured by the INA240 sensor, in milliamps
bool relay_active        # State of the G2R-24 relay (true = ON / engaged, false = OFF)
uint8 watchdog_status    # 0 = Normal, 1 = Triggered (E-Stop or open-load fault)
float32 servo_start_deg  # Servo angle for brake OFF/released, reported by STM32 (CAN 0x133): flash value at boot, live command after (0–180°)
float32 servo_stop_deg   # Servo angle for brake ON/engaged,  reported by STM32 (CAN 0x133): flash value at boot, live command after (0–180°)
