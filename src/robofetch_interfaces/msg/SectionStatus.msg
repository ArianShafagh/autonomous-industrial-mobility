# Live status of one production section, published on /factory/<section_id>/status.
# Durations are SIMULATION seconds (the robot's clock); rates are units per FACTORY hour.
# factory seconds = simulation seconds * time_scale.

uint8 RUNNING=0
uint8 DEGRADED=1     # worn machine, producing slower
uint8 BLOCKED=2      # output buffer full, line stopped, production being lost
uint8 FAULT=3        # machine broken, under repair, nothing produced

builtin_interfaces/Time stamp
string section_id
string scenario
uint8 status
string status_text
float64 time_scale
float64 factory_time_s

uint32 produced_total        # units finished since the shift started
uint32 picked_total          # units the robot has collected
uint32 buffer_units          # finished units waiting for the robot
uint32 buffer_capacity
float32 buffer_fill          # buffer_units / buffer_capacity, 0..1
float32 unit_mass_kg
float32 buffer_mass_kg

float32 rate_nominal_per_hour
float32 rate_actual_per_hour # 0 while BLOCKED or FAULT
float32 health_percent
float32 time_to_full_s       # expected time until BLOCKED; inf while FAULT, 0 when full
float32 fault_remaining_s    # repair time left while FAULT

uint32 faults_total
float32 lost_units           # production lost while BLOCKED
float32 blocked_time_s
