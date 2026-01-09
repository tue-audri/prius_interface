from panda_msgs.msg import CarState as PandaCarState


def fill_panda_car_state_from_capnp(car_state, msg: PandaCarState) -> PandaCarState:
    """
    Populate a PandaCarState ROS message from an openpilot capnp carState object.
    Header should be set by the caller.
    """
    # CAN health (if available in capnp; use getattr with defaults for safety)
    msg.can_valid = getattr(car_state, "canValid", False)
    msg.can_timeout = getattr(car_state, "canTimeout", False)
    msg.can_error_counter = getattr(car_state, "canErrorCounter", 0)

    # Speed and dynamics
    msg.v_ego = getattr(car_state, "vEgo", 0.0)
    msg.a_ego = getattr(car_state, "aEgo", 0.0)
    msg.v_ego_raw = getattr(car_state, "vEgoRaw", 0.0)
    msg.v_ego_cluster = getattr(car_state, "vEgoCluster", 0.0)
    msg.v_cruise = getattr(car_state, "vCruise", 0.0)
    msg.v_cruise_cluster = getattr(car_state, "vCruiseCluster", 0.0)
    msg.yaw_rate = getattr(car_state, "yawRate", 0.0)
    msg.standstill = getattr(car_state, "standstill", False)

    # Wheel speeds (if present)
    if hasattr(car_state, "wheelSpeeds"):
        try:
            msg.wheel_speeds.fl = car_state.wheelSpeeds.fl
            msg.wheel_speeds.fr = car_state.wheelSpeeds.fr
            msg.wheel_speeds.rl = car_state.wheelSpeeds.rl
            msg.wheel_speeds.rr = car_state.wheelSpeeds.rr
        except Exception:
            # If anything goes wrong, leave wheel speeds at their default values
            pass

    # Pedals
    msg.gas = getattr(car_state, "gas", 0.0)
    msg.gas_pressed = getattr(car_state, "gasPressed", False)
    msg.engine_rpm = getattr(car_state, "engineRpm", 0.0)
    msg.brake = getattr(car_state, "brake", 0.0)
    msg.brake_pressed = getattr(car_state, "brakePressed", False)
    msg.regen_braking = getattr(car_state, "regenBraking", False)
    msg.parking_brake = getattr(car_state, "parkingBrake", False)
    msg.brake_hold_active = getattr(car_state, "brakeHoldActive", False)

    # Steering
    msg.steering_angle_deg = getattr(car_state, "steeringAngleDeg", 0.0)
    msg.steering_angle_offset_deg = getattr(car_state, "steeringAngleOffsetDeg", 0.0)
    msg.steering_rate_deg = getattr(car_state, "steeringRateDeg", 0.0)
    msg.steering_torque = getattr(car_state, "steeringTorque", 0.0)
    msg.steering_torque_eps = getattr(car_state, "steeringTorqueEps", 0.0)
    msg.steering_pressed = getattr(car_state, "steeringPressed", False)
    msg.steering_disengage = getattr(car_state, "steeringDisengage", False)
    msg.steer_fault_temporary = getattr(car_state, "steerFaultTemporary", False)
    msg.steer_fault_permanent = getattr(car_state, "steerFaultPermanent", False)
    msg.invalid_lkas_setting = getattr(car_state, "invalidLkasSetting", False)

    # Stock systems / faults
    msg.stock_aeb = getattr(car_state, "stockAeb", False)
    msg.stock_lkas = getattr(car_state, "stockLkas", False)
    msg.stock_fcw = getattr(car_state, "stockFcw", False)
    msg.esp_disabled = getattr(car_state, "espDisabled", False)
    msg.acc_faulted = getattr(car_state, "accFaulted", False)
    msg.car_faulted_non_critical = getattr(car_state, "carFaultedNonCritical", False)
    msg.esp_active = getattr(car_state, "espActive", False)
    msg.vehicle_sensors_invalid = getattr(car_state, "vehicleSensorsInvalid", False)
    msg.low_speed_alert = getattr(car_state, "lowSpeedAlert", False)
    msg.block_pcm_enable = getattr(car_state, "blockPcmEnable", False)

    # Cruise state (basic mapping)
    try:
        cs = car_state.cruiseState
        msg.cruise_state.enabled = getattr(cs, "enabled", False)
        msg.cruise_state.speed = getattr(cs, "speed", 0.0)
        msg.cruise_state.speed_cluster = getattr(cs, "speedCluster", 0.0)
        msg.cruise_state.available = getattr(cs, "available", False)
        msg.cruise_state.speed_offset = getattr(cs, "speedOffset", 0.0)
        msg.cruise_state.standstill = getattr(cs, "standstill", False)
        msg.cruise_state.non_adaptive = getattr(cs, "nonAdaptive", False)
    except Exception:
        # If cruise state isn't present or has unexpected structure, leave defaults
        pass

    # Gear (encode openpilot gear string into uint8 value where possible)
    gear_str = getattr(car_state, "gearShifter", "unknown")
    gear_enum_map = {
        "unknown": 0,
        "park": 1,
        "drive": 2,
        "neutral": 3,
        "reverse": 4,
        "sport": 5,
        "low": 6,
        "brake": 7,
        "eco": 8,
        "manumatic": 9,
    }
    msg.gear_shifter.value = gear_enum_map.get(gear_str, 0)

    # Blinkers and generic toggle
    msg.button_enable = getattr(car_state, "buttonEnable", False)
    msg.left_blinker = getattr(car_state, "leftBlinker", False)
    msg.right_blinker = getattr(car_state, "rightBlinker", False)
    msg.generic_toggle = getattr(car_state, "genericToggle", False)

    # Doors / seatbelt / clutch
    msg.door_open = getattr(car_state, "doorOpen", False)
    msg.seatbelt_unlatched = getattr(car_state, "seatbeltUnlatched", False)
    msg.clutch_pressed = getattr(car_state, "clutchPressed", False)

    # Blindspot
    msg.left_blindspot = getattr(car_state, "leftBlindspot", False)
    msg.right_blindspot = getattr(car_state, "rightBlindspot", False)

    # Fuel / charging
    msg.fuel_gauge = getattr(car_state, "fuelGauge", 0.0)
    msg.charging = getattr(car_state, "charging", False)

    # Process meta
    msg.cum_lag_ms = getattr(car_state, "cumLagMs", 0.0)

    return msg


