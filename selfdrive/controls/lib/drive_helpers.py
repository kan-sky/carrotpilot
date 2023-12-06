import math

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_MDL, DT_CTRL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params
import numpy as np
from common.filter_simple import StreamingMovingAverage

EventName = car.CarEvent.EventName

## 국가법령정보센터: 도로설계기준
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]
MIN_CURVE_SPEED = 20. * CV.KPH_TO_MS


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 30 #40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = 1.6  # should be CV.MPH_TO_KPH, but this causes rounding errors

MIN_DIST = 0.001
MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

MAX_VEL_ERR = 5.0

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}

class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # ajouatom
    self.brake_pressed_count = 0
    self.gas_pressed_count = 0
    self.softHoldActive = 0
    self.button_cnt = 0
    self.long_pressed = False
    self.button_prev = ButtonType.unknown
    self.cruiseActivate = 0
    self.params = Params()
    self.v_cruise_kph_set = V_CRUISE_UNSET
    self.cruiseSpeedTarget = 0
    self.roadSpeed = 30
    self.xState = 0
    self.trafficState = 0
    self.sendEvent_frame = 0
    self.turnSpeed_prev = 300
    self.curvatureFilter = StreamingMovingAverage(20)
    self.softHold_count = 0
    self.cruiseActiveReady = 0
    self.apilotEventWait = 0
    self.apilotEventPrev = 0

    
    #ajouatom: params
    self.params_count = 0
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = 2
    self.autoResumeFromGasSpeed = Params().get_int("AutoResumeFromGasSpeed")
    self.autoCancelFromGasMode = Params().get_int("AutoCancelFromGasMode")
    self.steerRatioApply = float(self.params.get_int("SteerRatioApply")) * 0.1
    self.liveSteerRatioApply = float(self.params.get_int("LiveSteerRatioApply")) * 0.01
    self.autoCurveSpeedCtrlUse = int(Params().get("AutoCurveSpeedCtrlUse"))
    self.autoCurveSpeedFactor = float(int(Params().get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
    self.autoCurveSpeedFactorIn = float(int(Params().get("AutoCurveSpeedFactorIn", encoding="utf8")))*0.01
    self.cruiseOnDist = float(int(Params().get("CruiseOnDist", encoding="utf8"))) / 100.
    self.softHoldMode = Params().get_int("SoftHoldMode")

  def _params_update():
    self.params_count += 1
    if self.params_count == 10:
      self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
      self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
      self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
      self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
      self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
      self.autoNaviSpeedCtrl = 2
    elif self.params_count == 20:
      self.autoResumeFromGasSpeed = Params().get_int("AutoResumeFromGasSpeed")
      self.autoCancelFromGasMode = Params().get_int("AutoCancelFromGasMode")
      self.cruiseOnDist = float(int(Params().get("CruiseOnDist", encoding="utf8"))) / 100.
      self.softHoldMode = Params().get_int("SoftHoldMode")
    elif self.params_count == 30:
      self.steerRatioApply = float(self.params.get_int("SteerRatioApply")) * 0.1
      self.liveSteerRatioApply = float(self.params.get_int("LiveSteerRatioApply")) * 0.01
    elif self.params_count >= 100:
      self.autoCurveSpeedCtrlUse = int(Params().get("AutoCurveSpeedCtrlUse"))
      self.autoCurveSpeedFactor = float(int(Params().get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
      self.autoCurveSpeedFactorIn = float(int(Params().get("AutoCurveSpeedFactorIn", encoding="utf8")))*0.01
      self.params_count = 0
    
  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, reverse_cruise_increase, controls):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        #self._update_v_cruise_non_pcm(CS, enabled, is_metric, reverse_cruise_increase)
        self._update_v_cruise_apilot(CS, controls)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self._update_event_apilot(CS, controls)
        #self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET
      self.v_cruise_kph_set = V_CRUISE_UNSET
      self.cruiseActivate = 0

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric, reverse_cruise_increase):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = reverse_cruise_increase
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = not reverse_cruise_increase
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool, conditional_experimental_mode) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode and not conditional_experimental_mode else V_CRUISE_INITIAL

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_set = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = self.v_cruise_kph_set = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph

  def _make_event(self, controls, event_name, waiting = 20):
    elapsed_time = (controls.sm.frame - self.sendEvent_frame)*DT_CTRL
    if elapsed_time > self.apilotEventWait:
      controls.events.add(event_name)
      self.sendEvent_frame = controls.sm.frame
      self.apilotEventPrev = event_name
      self.apilotEventWait = waiting

  def _update_event_apilot(self, CS, controls):
    lp = controls.sm['longitudinalPlan']
    xState = lp.xState
    trafficState = lp.trafficState

    if xState != self.xState and controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0: #0:lead, 1:cruise, 2:e2eCruise, 3:e2eStop, 4:e2ePrepare
      if xState == 3:
        self._make_event(controls, EventName.trafficStopping)  # stopping
      elif xState == 4 and self.softHoldActive == 0:
        self._make_event(controls, EventName.trafficSignGreen) # starting
    self.xState = xState

    if trafficState != self.trafficState: #0: off, 1:red, 2:green
      if self.softHoldActive == 2 and trafficState == 2:
        self._make_event(controls, EventName.trafficSignChanged)
    self.trafficState = trafficState
  def _update_lead(self, controls):
    leadOne = controls.sm['radarState'].leadOne
    if leadOne.status:
      self.lead_dRel = leadOne.dRel
      self.lead_vRel = leadOne.vRel
    else:
      self.lead_dRel = 0
      self.lead_vRel = 0

  def _update_v_cruise_apilot(self, CS, controls):
    self._update_lead(controls)
    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    if self.v_cruise_kph_set == V_CRUISE_UNSET:
      self.v_cruise_kph_set = V_CRUISE_INITIAL
    v_cruise_kph = self.v_cruise_kph_set    
    v_cruise_kph = self._update_cruise_buttons(CS, v_cruise_kph, controls)
    #v_cruise_kph = self.update_apilot_cmd(controls, v_cruise_kph)
    v_cruise_kph_apply = self.cruise_control_speed(v_cruise_kph)
    apn_limit_kph = self.update_speed_apilot(CS, controls, self.v_cruise_kph)
    v_cruise_kph_apply = min(v_cruise_kph_apply, apn_limit_kph)
    self.curveSpeed = self.apilot_curve(CS, controls)
    if self.autoCurveSpeedCtrlUse > 0:
      v_cruise_kph_apply = min(v_cruise_kph_apply, self.curveSpeed)
    self.v_cruise_kph_set = v_cruise_kph
    self.v_cruise_kph = v_cruise_kph_apply

  def apilot_curve(self, CS, controls):
    if len(controls.sm['modelV2'].orientationRate.z) != 33:
      return 300
    # 회전속도를 선속도 나누면 : 곡률이 됨. [20]은 약 4초앞의 곡률을 보고 커브를 계산함.
    #curvature = abs(controls.sm['modelV2'].orientationRate.z[20] / clip(CS.vEgo, 0.1, 100.0))
    orientationRates = np.array(controls.sm['modelV2'].orientationRate.z, dtype=np.float32)
    # 계산된 결과로, oritetationRates를 나누어 조금더 curvature값이 커지도록 함.
    speed = min(self.turnSpeed_prev / 3.6, clip(CS.vEgo, 0.5, 100.0))    
    #curvature = np.max(np.abs(orientationRates[12:])) / speed  # 12: 약1.4초 미래의 curvature를 계산함.
    curvature = np.max(np.abs(orientationRates[12:20])) / speed  # 12: 약1.4~3.5초 미래의 curvature를 계산함.
    curvature = self.curvatureFilter.process(curvature) * self.autoCurveSpeedFactor
    turnSpeed = 300
    if abs(curvature) > 0.0001:
      turnSpeed = interp(curvature, V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
      turnSpeed = clip(turnSpeed, MIN_CURVE_SPEED, 255)
    else:
      turnSpeed = 300

    self.turnSpeed_prev = turnSpeed
    speed_diff = max(0, CS.vEgo*3.6 - turnSpeed)
    turnSpeed = turnSpeed - speed_diff * self.autoCurveSpeedFactorIn
    #controls.debugText2 = 'CURVE={:5.1f},curvature={:5.4f},mode={:3.1f}'.format(self.turnSpeed_prev, curvature, self.drivingModeIndex)
    return turnSpeed

  def update_apilot_cmd(self, controls, v_cruise_kph, longActiveUser):
    msg = controls.sm['roadLimitSpeed']
    #print(msg.xCmd, msg.xArg, msg.xIndex)

    if msg.xIndex > 0 and msg.xIndex != self.xIndex:
      self.xIndex = msg.xIndex
      if msg.xCmd == "SPEED":
        if msg.xArg == "UP":
          v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph, self.roadSpeed)
        elif msg.xArg == "DOWN":
          if self.v_ego_kph_set < v_cruise_kph:
            v_cruise_kph = self.v_ego_kph_set
          elif v_cruise_kph > 30:
            v_cruise_kph -= 10
        else:
          v_cruise_kph = clip(int(msg.xArg), V_CRUISE_MIN, V_CRUISE_MAX)
      #elif msg.xCmd == "CRUISE":
      #  if msg.xArg == "ON":
      #    longActiveUser = 1
      #  elif msg.xArg == "OFF":
      #    self.userCruisePaused = True
      #    longActiveUser = -1
      #  elif msg.xArg == "GO":
      #    if longActiveUser <= 0:
      #      longActiveUser = 1
      #    elif self.xState in [XState.softHold, XState.e2eStop]:
      #      controls.cruiseButtonCounter += 1
      #    else:
      #      v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph, self.roadSpeed)
      #  elif msg.xArg == "STOP":
      #    if self.xState in [XState.e2eStop, XState.e2eCruisePrepare]:
      #      controls.cruiseButtonCounter -= 1
      #    else:
      #      v_cruise_kph = 20
      elif msg.xCmd == "LANECHANGE":
        pass
    return v_cruise_kph, longActiveUser

  def _update_cruise_buttons(self, CS, v_cruise_kph, controls):

    self.cruiseButtonMode = 2    

    if v_cruise_kph > 200:
      v_cruise_kph = V_CRUISE_INITIAL

    if CS.brakePressed:
      self.brake_pressed_count = 1 if self.brake_pressed_count < 0 else self.brake_pressed_count + 1
      self.softHold_count = self.softHold_count + 1 if self.softHoldMode > 0 and CS.vEgo < 0.1 else 0
      self.softHoldActive = 1 if self.softHold_count > 60 else 0        
    else:
      self.softHold_count = 0
      self.brake_pressed_count = -1 if self.brake_pressed_count > 0 else self.brake_pressed_count - 1

    gas_tok = False
    if CS.gasPressed:
      self.gas_pressed_count = 1 if self.gas_pressed_count < 0 else self.gas_pressed_count + 1
    else:
      gas_tok = True if 0 < self.gas_pressed_count < 60 else False
      self.gas_pressed_count = -1 if self.gas_pressed_count > 0 else self.gas_pressed_count - 1

    if controls.enabled or CS.brakePressed or CS.gasPressed:
      self.cruiseActiveReady = 0

    ## ButtonEvent process
    button_kph = v_cruise_kph
    buttonEvents = CS.buttonEvents
    button_speed_up_diff = 1
    button_speed_dn_diff = 10 if self.cruiseButtonMode in [3, 4] else 1

    button_type = 0
    if self.button_cnt > 0:
      self.button_cnt += 1
    for b in buttonEvents:
      if b.pressed and self.button_cnt==0 and b.type in [ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.gapAdjustCruise, ButtonType.cancel]:
        self.button_cnt = 1
        self.button_prev = b.type
      elif not b.pressed and self.button_cnt > 0:
        if b.type == ButtonType.cancel:
          button_type = ButtonType.cancel
        elif not self.long_pressed and b.type == ButtonType.accelCruise:
          button_kph += button_speed_up_diff if controls.is_metric else button_speed_up_diff * CV.MPH_TO_KPH
          button_type = ButtonType.accelCruise
        elif not self.long_pressed and b.type == ButtonType.decelCruise:
          button_kph -= button_speed_dn_diff if controls.is_metric else button_speed_dn_diff * CV.MPH_TO_KPH
          button_type = ButtonType.decelCruise
        elif not self.long_pressed and b.type == ButtonType.gapAdjustCruise:
          button_type = ButtonType.gapAdjustCruise

        self.long_pressed = False
        self.button_cnt = 0
    if self.button_cnt > 40:
      self.long_pressed = True
      V_CRUISE_DELTA = 10
      if self.button_prev == ButtonType.cancel:
        button_type = ButtonType.cancel
        self.button_cnt = 0          
      elif self.button_prev == ButtonType.accelCruise:
        button_kph += V_CRUISE_DELTA - button_kph % V_CRUISE_DELTA
        button_type = ButtonType.accelCruise
        self.button_cnt %= 40
      elif self.button_prev == ButtonType.decelCruise:
        button_kph -= V_CRUISE_DELTA - -button_kph % V_CRUISE_DELTA
        button_type = ButtonType.decelCruise
        self.button_cnt %= 40
      elif self.button_prev == ButtonType.gapAdjustCruise:
        button_type = ButtonType.gapAdjustCruise
        self.button_cnt = 0

    button_kph = clip(button_kph, V_CRUISE_MIN, V_CRUISE_MAX)
    #return button_type, self.long_pressed, v_cruise_kph

    if button_type != 0 and controls.enabled:
      if self.long_pressed:
        if button_type in [ButtonType.accelCruise, ButtonType.decelCruise]:
          v_cruise_kph = button_kph
      else:
        if button_type == ButtonType.accelCruise:
          if self.softHoldActive > 0:
            self.softHoldActive = 0
          else:
            v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
        elif button_type == ButtonType.decelCruise:
          if v_cruise_kph > self.v_ego_kph_set - 2:
            v_cruise_kph = self.v_ego_kph_set
          else:
            #v_cruise_kph = button_kph
            self.cruiseActiveReady = 1
            self.cruiseActivate = -1

    if self.gas_pressed_count > 0 or button_type in [ButtonType.cancel, ButtonType.accelCruise, ButtonType.decelCruise]:
    #  self.softHoldActive = 0
      self.cruiseActivate = 0

    ## Auto Engage/Disengage via Gas/Brake
    if gas_tok:
      if controls.enabled:
        v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
      elif self.autoResumeFromGasSpeed > 0:
        print("Cruise Activate from GasTok")
        self.cruiseActivate = 1
    elif self.gas_pressed_count == -1:
      if controls.enabled:
        if 0 < self.lead_dRel < CS.vEgo * 0.8 and self.autoCancelFromGasMode > 0:
          self.cruiseActivate = -1
          print("Cruise Deactivate from gas.. too close leadCar!")
      elif self.v_ego_kph_set > self.autoResumeFromGasSpeed > 0:
        if self.cruiseActivate <= 0:
          print("Cruise Activate from Speed")
        self.cruiseActivate = 1
    elif self.brake_pressed_count == -1 and self.softHoldActive == 0:
      if not controls.enabled and self.v_ego_kph_set < 70.0 and controls.experimental_mode and Params().get_bool("AutoResumeFromBrakeReleaseTrafficSign"):
        print("Cruise Activate from TrafficSign")
        self.cruiseActivate = 1

    if self.gas_pressed_count == -1 and not gas_tok:
      if self.autoCancelFromGasMode > 0:
        if self.v_ego_kph_set < self.autoResumeFromGasSpeed:
          print("Cruise Deactivate from gas pressed");
          self.cruiseActivate = -1
        if controls.experimental_mode and self.autoCancelFromGasMode == 2:
          print("Cruise Deactivate from gas pressed: experimental mode");
          self.cruiseActivate = -1

    if self.gas_pressed_count > 0 and self.v_ego_kph_set > v_cruise_kph:
      v_cruise_kph = self.v_ego_kph_set
    elif self.brake_pressed_count == -1 and self.softHoldActive == 1 and self.softHoldMode > 0:
      print("Cruise Activete from SoftHold")
      self.softHoldActive = 2
      self.cruiseActivate = 1
    elif self.cruiseActiveReady > 0:
      if 0 < self.lead_dRel or self.xState == 3:
        print("Cruise Activate from Lead or Traffic sign stop")
        self.cruiseActivate = 1
    elif not controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0:
      cruiseOnDist = abs(self.cruiseOnDist)
      if cruiseOnDist > 0 and CS.vEgo > 0.2 and self.lead_vRel < 0 and 0 < self.lead_dRel < cruiseOnDist:
        self._make_event(controls, EventName.stopstop)
        if cruiseOnDist > 0:
          self.cruiseActivate = 1

    v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX)
    return v_cruise_kph

  def v_cruise_speed_up(self, v_cruise_kph):
    if v_cruise_kph < self.roadSpeed:
      v_cruise_kph = self.roadSpeed
    else:
      for speed in range (40, V_CRUISE_MAX, Params().get_int("CruiseSpeedUnit")):
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
    return clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX)

  def decelerate_for_speed_camera(self, safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):
    if left_dist <= safe_dist:
      return safe_speed
    temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
    dV = (-safe_speed + math.sqrt(temp)) * decel_rate
    apply_speed = min(250 , safe_speed + dV)
    min_speed = prev_apply_speed - (decel_rate * 1.2) * 2 * DT_CTRL
    apply_speed = max(apply_speed, min_speed)
    return apply_speed

  def update_speed_apilot(self, CS, controls, v_cruise_kph_prev):
    v_ego = CS.vEgoCluster
    msg = self.roadLimitSpeed = controls.sm['roadLimitSpeed']

    self.roadSpeed = clip(30, msg.roadLimitSpeed, 150.0)
    camType = int(msg.camType)
    xSignType = msg.xSignType

    isSpeedBump = False
    isSectionLimit = False
    safeSpeed = 0
    leftDist = 0
    speedLimitType = 0
    safeDist = 0
  
    
    if camType == 22 or xSignType == 22:
      safeSpeed = self.autoNaviSpeedBumpSpeed
      isSpeedBump = True

    if msg.xSpdLimit > 0 and msg.xSpdDist > 0:
      safeSpeed = msg.xSpdLimit if safeSpeed <= 0 else safeSpeed
      leftDist = msg.xSpdDist
      isSectionLimit = True if xSignType==165 or leftDist > 3000 or camType == 4 else False
      isSectionLimit = False if leftDist < 50 else isSectionLimit
      speedLimitType = 2 if not isSectionLimit else 3
    elif msg.camLimitSpeed > 0 and msg.camLimitSpeedLeftDist>0:
      safeSpeed = msg.camLimitSpeed
      leftDist = msg.camLimitSpeedLeftDist
      isSectionLimit = True if leftDist > 3000 or camType == 4 else False
      isSectionLimit = False if leftDist < 50 else isSectionLimit
      speedLimitType = 2 if not isSectionLimit else 3
    elif CS.speedLimit > 0 and CS.speedLimitDistance > 0 and self.autoNaviSpeedCtrl >= 2:
      safeSpeed = CS.speedLimit
      leftDist = CS.speedLimitDistance
      speedLimitType = 2 if leftDist > 1 else 3

    if isSpeedBump:
      speedLimitType = 1 
      safeDist = self.autoNaviSpeedBumpTime * v_ego
    elif safeSpeed>0 and leftDist>0:
      safeDist = self.autoNaviSpeedCtrlEnd * v_ego

    safeSpeed *= self.autoNaviSpeedSafetyFactor

    log = ""
    if isSectionLimit:
      applySpeed = safeSpeed
    elif leftDist > 0 and safeSpeed > 0 and safeDist > 0:
      applySpeed = self.decelerate_for_speed_camera(safeSpeed/3.6, safeDist, v_cruise_kph_prev * CV.KPH_TO_MS, self.autoNaviSpeedDecelRate, leftDist) * CV.MS_TO_KPH
      if applySpeed < CS.vEgoCluster * CV.MS_TO_KPH:
        self._make_event(controls, EventName.speedDown, 60.0)
    else:
      applySpeed = 255

    apTbtSpeed = apTbtDistance = 0
    log = "{:.1f}<{:.1f}/{:.1f},{:.1f} B{} A{:.1f}/{:.1f} N{:.1f}/{:.1f} C{:.1f}/{:.1f} V{:.1f}/{:.1f} ".format(
                  applySpeed, safeSpeed, leftDist, safeDist,
                  1 if isSpeedBump else 0, 
                  msg.xSpdLimit, msg.xSpdDist,
                  msg.camLimitSpeed, msg.camLimitSpeedLeftDist,
                  CS.speedLimit, CS.speedLimitDistance,
                  apTbtSpeed, apTbtDistance)
    if applySpeed < 200:
      print(log)
    #controls.debugText1 = log
    return applySpeed #, roadSpeed, leftDist, speedLimitType

  def cruise_control_speed(self, v_cruise_kph):
    v_cruise_kph_apply = v_cruise_kph    
    cruise_eco_control = self.params.get_int("CruiseEcoControl")
    if cruise_eco_control > 0:
      if self.cruiseSpeedTarget > 0:
        if self.cruiseSpeedTarget < v_cruise_kph:
          self.cruiseSpeedTarget = v_cruise_kph
        elif self.cruiseSpeedTarget > v_cruise_kph:
          self.cruiseSpeedTarget = 0
      elif self.cruiseSpeedTarget == 0 and self.v_ego_kph_set + 3 < v_cruise_kph and v_cruise_kph > 20.0:  # 주행중 속도가 떨어지면 다시 크루즈연비제어 시작.
        self.cruiseSpeedTarget = v_cruise_kph

      if self.cruiseSpeedTarget != 0:  ## 크루즈 연비 제어모드 작동중일때: 연비제어 종료지점
        if self.v_ego_kph_set > self.cruiseSpeedTarget: # 설정속도를 초과하면..
          self.cruiseSpeedTarget = 0
        else:
          v_cruise_kph_apply = self.cruiseSpeedTarget + cruise_eco_control  # + 설정 속도로 설정함.
    else:
      self.cruiseSpeedTarget = 0

    return v_cruise_kph_apply


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates, distances, average_desired_curvature):
  if len(psis) != CONTROL_N or len(distances) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
    distances = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = CP.steerActuatorDelay + .2

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  # Pfeiferj's #28118 PR - https://github.com/commaai/openpilot/pull/28118
  distance = interp(delay, ModelConstants.T_IDXS[:CONTROL_N], distances)
  distance = max(MIN_DIST, distance)
  average_curvature_desired = psi / distance if average_desired_curvature else psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  desired_curvature_rate = curvature_rates[0]
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                     -max_curvature_rate,
                                     max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: car.CarParams.LateralTorqueTuning, friction_compensation: bool) -> float:
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  friction = float(friction_interp) if friction_compensation else 0.0
  return friction


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0
