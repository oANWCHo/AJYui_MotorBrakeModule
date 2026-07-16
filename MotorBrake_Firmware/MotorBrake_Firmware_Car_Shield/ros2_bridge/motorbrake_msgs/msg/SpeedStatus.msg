# Closed-loop speed controller status reported by the STM32 over CAN (ID 0x122).
float32 measured_speed_mps   # Measured wheel speed (m/s); sign follows commanded direction
float32 target_speed_mps     # Commanded target speed (m/s); sign = direction
bool controller_enabled      # Speed PID loop running (mode relay ON)      -- status_flags bit0
bool forward_cmd             # Forward output line commanded                -- status_flags bit1
bool reverse_cmd             # Reverse output line commanded                -- status_flags bit2
bool pedal_output_active     # Pedal output line active                     -- status_flags bit3
bool speed_sensor_valid      # Speed sensor producing a valid reading       -- status_flags bit4
bool timeout_active          # No /cmd_vel (speed_cmd) within the timeout    -- status_flags bit5
bool e_stop_active           # PC13 E_Stop active                           -- status_flags bit6
bool fault_active            # Speed fault latched                          -- status_flags bit7
uint8 fault_code             # 0 = none (reserved for future fault codes)
uint16 sequence              # Rolling counter so the PC can detect dropped frames
