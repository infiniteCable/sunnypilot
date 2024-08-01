from cereal import car
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan, mebcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags
from openpilot.selfdrive.controls.lib.drive_helpers import VOLKSWAGEN_V_CRUISE_MIN
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
ButtonType = car.CarState.ButtonEvent.Type

#def apply_meb_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, CCP):
#  if v_ego_raw > 1: # we don't enforce checks for this in panda at the moment, but keep it near measured current curv to project user input
#    apply_curvature = clip(apply_curvature, current_curvature - CCP.CURVATURE_ERROR, current_curvature + CCP.CURVATURE_ERROR)
#  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CCP)
#  
#  return clip(apply_curvature, -CCP.CURVATURE_MAX, CCP.CURVATURE_MAX)


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.last_button_frame = 0

    self.sm = messaging.SubMaster(['longitudinalPlanSP'])
    self.param_s = Params()
    self.is_metric = self.param_s.get_bool("IsMetric")
    self.speed_limit_control_enabled = False
    self.last_speed_limit_sign_tap = False
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_set_dis_prev = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.last_cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    self.acc_type = -1
    self.send_count = 0

    #self.apply_curvature_last = 0
    self.apply_angle_last = 0
    self.lat_active_prev = False
    self.steering_power = 0
    self.long_heartbeat = 0
    self.long_active_prev = False

  def update(self, CC, CS, now_nanos):
    if not self.CP.pcmCruiseSpeed:
      self.sm.update(0)

      if self.sm.updated['longitudinalPlanSP']:
        self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
        self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
        self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
        self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
        self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
        self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
        self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

      if self.frame % 200 == 0:
        self.speed_limit_control_enabled = self.param_s.get_bool("EnableSlc")
        self.is_metric = self.param_s.get_bool("IsMetric")
      self.last_speed_limit_sign_tap = self.param_s.get_bool("LastSpeedLimitSignTap")
      self.v_cruise_min = VOLKSWAGEN_V_CRUISE_MIN[self.is_metric] * (CV.KPH_TO_MPH if not self.is_metric else 1)
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if not self.CP.pcmCruiseSpeed:
      if not self.last_speed_limit_sign_tap_prev and self.last_speed_limit_sign_tap:
        self.sl_force_active_timer = self.frame
        self.param_s.put_bool_nonblocking("LastSpeedLimitSignTap", False)
      self.last_speed_limit_sign_tap_prev = self.last_speed_limit_sign_tap

      sl_force_active = self.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
      sl_inactive = not sl_force_active and (not self.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
      sl_temp_inactive = not sl_force_active and (self.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
      slc_active = not sl_inactive and not sl_temp_inactive

      self.slc_active_stock = slc_active

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state
        #   * steering power as counter near zero before standstill OP lane assist deactivation
        # MEB rack can be used continously without found time limits yet
        # Steering power counter is used to:
        #   * prevent sudden fluctuations at low speeds
        #   * avoid HCA refused
        #   * easy user intervention
        #   * keep it near maximum regarding speed to get full steering power in shortest time

        if CC.latActive:
          hca_enabled          = True
          self.lat_active_prev = True
          #current_curvature    = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1) # TODO verify sign (clockwise is negative)
          #apply_curvature      = apply_meb_curvature_limits(actuators.curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw, self.CCP)
          apply_angle          = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, self.CCP)
          apply_angle          = clip(apply_angle, CS.out.steeringAngleDeg - self.CCP.ANGLE_ERROR, CS.out.steeringAngleDeg + self.CCP.ANGLE_ERROR)

          # steering power as lazy counter
          steering_power_min_by_speed = interp(CS.out.vEgoRaw, [0, self.CCP.STEERING_POWER_MAX_BY_SPEED], [self.CCP.STEERING_POWER_MIN, self.CCP.STEERING_POWER_MAX])
          #steering_power_by_curvature = self.CCP.STEERING_POWER_MAX * abs(apply_curvature) / self.CCP.STEERING_POWER_MAX_BY_CURVATURE # maximum steering power is reached with 10 degrees
          steering_power_by_angle = self.CCP.STEERING_POWER_MAX * abs(apply_angle) / self.CCP.STEERING_POWER_MAX_BY_ANGLE # maximum steering power is reached with 10 degrees
          #steering_power_target       = clip(steering_power_by_curvature, steering_power_min_by_speed, self.CCP.STEERING_POWER_MAX)
          steering_power_target       = clip(steering_power_by_angle, steering_power_min_by_speed, self.CCP.STEERING_POWER_MAX)

          if self.steering_power < self.CCP.STEERING_POWER_MIN:  # OP lane assist just activated
            self.steering_power = min(self.steering_power + self.CCP.STEERING_POWER_NORMAL_STEPS, self.CCP.STEERING_POWER_MIN)

          elif CS.out.steeringPressed and self.steering_power > self.CCP.STEERING_POWER_USER: # user action results in decreasing the steering power
            self.steering_power = max(self.steering_power - self.CCP.STEERING_POWER_CRITICAL_STEPS, self.CCP.STEERING_POWER_USER)

          elif self.steering_power < self.CCP.STEERING_POWER_MAX: # following desired target
            if self.steering_power < steering_power_target:
              self.steering_power = min(self.steering_power + self.CCP.STEERING_POWER_CRITICAL_STEPS, steering_power_target)
            elif self.steering_power > steering_power_target:
              self.steering_power = max(self.steering_power - self.CCP.STEERING_POWER_NORMAL_STEPS, steering_power_target)

          #if abs(apply_angle) > 45:
          #  new_steer = self.CCP.STEER_MAX -
          #  apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)

        else:
          if self.lat_active_prev and self.steering_power > 0: # decrement power to zero before disabling lane assist to prevent EPS fault
            hca_enabled            = True
            #current_curvature      = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
            #apply_curvature        = current_curvature
            apply_angle            = CS.out.steeringAngleDeg
            self.steering_power    = max(self.steering_power - self.CCP.STEERING_POWER_NORMAL_STEPS, 0)
          else:
            hca_enabled           = False
            self.lat_active_prev  = False
            self.steering_power   = 0
            #apply_curvature       = 0.
            apply_angle           = 0

        #self.apply_curvature_last = apply_curvature
        self.apply_angle_last = clip(apply_angle, -self.CCP.ANGLE_MAX, self.CCP.ANGLE_MAX)
        can_sends.append(self.CCS.create_steering_control_curvature(self.packer_pt, CANBUS.pt, apply_angle, hca_enabled, self.steering_power))

      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.

        if CC.latActive:
          new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
          apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_steer_last == apply_steer:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_steer -= (1, -1)[apply_steer < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
            hca_enabled = abs(apply_steer) > 0
        else:
          hca_enabled = False
          apply_steer = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_steer_last = apply_steer
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

      if self.CP.flags & VolkswagenFlags.MEB:
        enable_long = True if CS.out.buttonEvents in [car.CarState.ButtonEvent.Type.setCruise, car.CarState.ButtonEvent.Type.resumeCruise] and not CC.longActive else False
        just_disabled = True if self.long_active_prev and not CC.longActive else False
        self.long_active_prev = CC.longActive
        current_speed = CS.out.vEgo * CV.MS_TO_KPH
        reversing = CS.out.gearShifter in [car.CarState.GearShifter.reverse]
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, just_disabled, enable_long)
        acc_hold_type = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, just_disabled, starting,
                                               stopping, CS.esp_hold_confirmation)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel, acc_control,
                                                           acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
                                                           current_speed, reversing, CS.meb_acc_02_values))
        
      else:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                         acc_control, stopping, starting, CS.esp_hold_confirmation))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if self.CP.flags & VolkswagenFlags.MEB:
        sound_alert = 0
        if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
          hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
          sound_alert = 1
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control, sound_alert))

      else:
        if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
          hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        if self.long_heartbeat != 221:
          self.long_heartbeat = 221
        elif self.long_heartbeat == 221:
          self.long_heartbeat = 360

        t_follow = get_T_FOLLOW(hud_control.leadDistanceBars - 1)
        t_gap = t_follow * CS.out.vEgo # in m, we have to scale our dest signal soon with suitable factor and regard maybe different factors of our zeitluecke signals
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, CC.cruiseControl.override)
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed, t_gap,
                                                         lead_distance, hud_control.leadDistanceBars, self.long_heartbeat,
                                                         CS.esp_hold_confirmation, CS.meb_acc_01_values, CS.distance_stock_values))

      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))
    if not (CC.cruiseControl.cancel or CC.cruiseControl.resume) and CS.out.cruiseState.enabled:
      if not self.CP.pcmCruiseSpeed:
        self.cruise_button = self.get_cruise_buttons(CS, CC.vCruise)
        if self.cruise_button is not None:
          if self.acc_type == -1:
            if self.button_count >= 2 and self.v_set_dis_prev != self.v_set_dis:
              self.acc_type = 1 if abs(self.v_set_dis - self.v_set_dis_prev) >= 10 and self.last_cruise_button in (1, 2) else \
                              0 if abs(self.v_set_dis - self.v_set_dis_prev) < 10 and self.last_cruise_button not in (1, 2) else 1
            if self.send_count >= 10 and self.v_set_dis_prev == self.v_set_dis:
              self.cruise_button = 3 if self.cruise_button == 1 else 4
          if self.acc_type == 0:
            self.cruise_button = 1 if self.cruise_button == 1 else 2  # accel, decel
          elif self.acc_type == 1:
            self.cruise_button = 3 if self.cruise_button == 1 else 4  # resume, set
          if self.frame % self.CCP.BTN_STEP == 0:
            can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, CS.gra_stock_values, frame=(self.frame // self.CCP.BTN_STEP),
                                                                 buttons=self.cruise_button, custom_stock_long=True))
            self.send_count += 1
        else:
          self.send_count = 0
        self.last_cruise_button = self.cruise_button

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    #new_actuators.curvature = self.apply_curvature_last
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.v_set_dis_prev = self.v_set_dis
    self.frame += 1
    return new_actuators, can_sends

  # multikyd methods, sunnyhaibin logic
  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled:
      for be in CS.out.buttonEvents:
        if be.type in (ButtonType.accelCruise, ButtonType.resumeCruise,
                       ButtonType.decelCruise, ButtonType.setCruise) and be.pressed:
          self.timer = 40
        elif be.type == ButtonType.gapAdjustCruise and be.pressed:
          self.timer = 300
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda: "default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    self.speed_diff = self.target_speed - self.v_set_dis
    if self.target_speed > self.v_set_dis:
      self.button_type = 1
    elif self.target_speed < self.v_set_dis and self.v_set_dis > self.v_cruise_min:
      self.button_type = 2
    return None

  def type_1(self):
    cruise_button = 1
    self.button_count += 1
    if self.target_speed <= self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = 2
    self.button_count += 1
    if self.target_speed >= self.v_set_dis or self.v_set_dis <= self.v_cruise_min:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = None
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)

  def get_button_control(self, CS, final_speed, v_cruise_kph_prev):
    self.init_speed = round(min(final_speed, v_cruise_kph_prev) * (CV.KPH_TO_MPH if not self.is_metric else 1))
    self.v_set_dis = round(CS.out.cruiseState.speed * (CV.MS_TO_MPH if not self.is_metric else CV.MS_TO_KPH))
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def curve_speed_hysteresis(self, cur_speed: float, hyst=(0.75 * CV.MPH_TO_KPH)):
    if cur_speed > self.steady_speed:
      self.steady_speed = cur_speed
    elif cur_speed < self.steady_speed - hyst:
      self.steady_speed = cur_speed
    return self.steady_speed

  def get_cruise_buttons(self, CS, v_cruise_kph_prev):
    cruise_button = None
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.out.cruiseState.enabled:
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      if self.slc_state > 1:
        target_speed_kph = set_speed_kph
      else:
        target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)
      if self.v_tsc_state != 0 or self.m_tsc_state > 1:
        self.final_speed_kph = self.get_curve_speed(target_speed_kph, v_cruise_kph_prev)
      else:
        self.final_speed_kph = target_speed_kph

      cruise_button = self.get_button_control(CS, self.final_speed_kph, v_cruise_kph_prev)  # MPH/KPH based button presses
    return cruise_button
