# One robot action as executed by the mission executor, published on /mission/events when the
# action starts and again when it ends. PREDICTED values come from robot_model + path_matrix
# before the action; MEASURED values come from the live robot (telemetry and ground-truth pose).

builtin_interfaces/Time stamp
string run_id
uint32 seq               # action number within the run
string action            # PICKUP | DELIVER | CHARGE | WAIT
string target            # A | B | C | delivery | charger | (empty for WAIT)
string phase             # STARTED | SUCCEEDED | FAILED
string detail

float64 sim_time_s
float32 duration_s             # measured, sim seconds (0 on STARTED)
float32 predicted_duration_s
float32 distance_m             # measured from the ground-truth pose
float32 predicted_distance_m   # from path_matrix
float32 energy_wh              # measured battery energy drawn during the action
float32 predicted_energy_wh
float32 charged_wh             # energy put back while charging
float32 battery_start_percent
float32 battery_end_percent

float32 payload_kg             # carried after the action
uint32 units                   # units picked up (PICKUP) or delivered (DELIVER)
