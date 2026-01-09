std_msgs/Header header

# CAN health
bool   can_valid               # invalid counter/checksums
bool   can_timeout             # CAN bus dropped out
uint32 can_error_counter

# car speed
float32 v_ego                  # best estimate of speed
float32 a_ego                  # best estimate of acceleration
float32 v_ego_raw              # unfiltered speed from wheel speed sensors
float32 v_ego_cluster          # best estimate of speed shown on instrument cluster, used for UI

float32 v_cruise               # actual set speed
float32 v_cruise_cluster       # set speed to display in the UI

float32 yaw_rate               # best estimate of yaw rate
bool    standstill
WheelSpeeds wheel_speeds

# gas pedal, 0.0-1.0
float32 gas                    # this is user pedal only
bool    gas_pressed            # this is user pedal only

float32 engine_rpm

# brake pedal, 0.0-1.0
float32 brake                  # this is user pedal only
bool    brake_pressed          # this is user pedal only
bool    regen_braking          # this is user pedal only
bool    parking_brake
bool    brake_hold_active

# steering wheel
float32 steering_angle_deg
float32 steering_angle_offset_deg   # Offset between sensors if multiple
float32 steering_rate_deg
float32 steering_torque             # TODO: standardize units
float32 steering_torque_eps         # TODO: standardize units
bool    steering_pressed            # if the user is using the steering wheel
bool    steering_disengage          # more force than steering_pressed, disengages for applicable brands
bool    steer_fault_temporary       # temporary EPS fault
bool    steer_fault_permanent       # permanent EPS fault
bool    invalid_lkas_setting        # stock LKAS incorrectly configured (on/off)
bool    stock_aeb
bool    stock_lkas
bool    stock_fcw
bool    esp_disabled
bool    acc_faulted
bool    car_faulted_non_critical    # some ECU is faulted, but car remains controllable
bool    esp_active
bool    vehicle_sensors_invalid     # invalid steering angle readings, etc.
bool    low_speed_alert             # lost steering control due to dynamic min steering speed
bool    block_pcm_enable            # whether to allow PCM to enable this frame

# cruise state
CruiseState cruise_state

# gear
GearShifter gear_shifter

# button presses
ButtonEvent[] button_events
bool          button_enable          # user is requesting enable, usually one frame. set if pcmCruise=False
bool          left_blinker
bool          right_blinker
bool          generic_toggle

# lock info
bool door_open
bool seatbelt_unlatched

# clutch (manual transmission only)
bool clutch_pressed

# blindspot sensors
bool left_blindspot                  # Is there something blocking the left lane change
bool right_blindspot                 # Is there something blocking the right lane change

float32 fuel_gauge                   # battery or fuel tank level from 0.0 to 1.0
bool    charging

# process meta
float32 cum_lag_ms

# deprecated (simplified representations)
uint16[] errors_deprecated           # minimal: numeric IDs from deprecated EventName enum
bool     brake_lights_deprecated
bool     steering_rate_limited_deprecated
uint64[] can_mono_times_deprecated   # originally List(UInt64)
bool     can_rcv_timeout_deprecated
OnroadEventDeprecated[] events_deprecated


