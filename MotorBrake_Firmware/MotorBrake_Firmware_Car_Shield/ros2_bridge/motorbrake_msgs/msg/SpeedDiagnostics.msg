# Raw Curtis 1510 I/O snapshot reported by the STM32 over CAN (ID 0x123).
bool fwd_input           # FWD signal read from the Curtis input side       -- input_flags bit0
bool rev_input           # REV signal read from the Curtis input side       -- input_flags bit1
bool pedal_input         # Pedal signal read from the input side            -- input_flags bit2
bool fwd_output          # FWD output driven by this board to the Curtis    -- output_flags bit0
bool rev_output          # REV output driven by this board to the Curtis    -- output_flags bit1
bool pedal_output        # Pedal output driven by this board                -- output_flags bit2
bool mode_relay          # Mode relay energised (outputs routed to Curtis)  -- output_flags bit3
float32 mcor_input_v     # MCOR feedback/input voltage read from the wiper (V)
float32 mcor_output_v    # MCOR output voltage this board drives to the Curtis (V)
float32 speed_sensor_hz  # Pulse frequency from the speed sensor (Hz)
