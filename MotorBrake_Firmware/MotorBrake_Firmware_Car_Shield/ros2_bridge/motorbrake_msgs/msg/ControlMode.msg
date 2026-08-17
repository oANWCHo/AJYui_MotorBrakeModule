# Which side currently drives the Curtis 1510 throttle/direction lines.
#
# Derived by the bridge from the mode relay (Relay_Mode, PA10) reported in
# /speed_diagnostics (CAN 0x123, output_flags bit3). That is a hardware readback
# of the relay contacts, so `mode` reflects what is actually wired through -- not
# merely what was asked for on /speed_enable.

uint8 MODE_PASSTHROUGH = 0   # Manual: pedal/direction wired straight to the Curtis
uint8 MODE_SYSTEM      = 1   # MCU outputs routed to the Curtis (this board drives)

uint8 mode                   # MODE_PASSTHROUGH or MODE_SYSTEM, from the mode relay
bool system_control          # Convenience: mode == MODE_SYSTEM
bool controller_enabled      # Speed PID loop commanded on (/speed_enable intent).
                             # May lead `mode` by a few ms while the relay settles;
                             # a lasting disagreement means the relay did not switch.
