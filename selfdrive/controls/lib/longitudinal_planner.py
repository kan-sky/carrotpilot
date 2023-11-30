#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from cereal import log

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.controls.speed_limit_controller import SpeedLimitController
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.system.swaglog import cloudlog

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Acceleration profiles - Credit goes to the DragonPilot team!
                 # MPH = [0.,  35,   35,  40,    40,  45,    45,  67,    67,   67, 123]
A_CRUISE_MIN_BP_CUSTOM = [0., 2.0, 2.01, 11., 11.01, 18., 18.01, 28., 28.01,  33., 55.]
                 # MPH = [0., 6.71, 13.4, 17.9, 24.6, 33.6, 44.7, 55.9, 67.1, 123]
A_CRUISE_MAX_BP_CUSTOM = [0.,    3,   6.,   8.,  11.,  15.,  20.,  25.,  30., 55.]

A_CRUISE_MIN_VALS_ECO_TUNE = [-0.480, -0.480, -0.40, -0.40, -0.40, -0.36, -0.32, -0.28, -0.28, -0.25, -0.25]
A_CRUISE_MAX_VALS_ECO_TUNE = [3.5, 3.3, 1.7, 1.1, .76, .62, .47, .36, .28, .09]

A_CRUISE_MIN_VALS_SPORT_TUNE = [-0.500, -0.500, -0.42, -0.42, -0.42, -0.42, -0.40, -0.35, -0.35, -0.30, -0.30]
A_CRUISE_MAX_VALS_SPORT_TUNE = [3.5, 3.5, 3.0, 2.6, 1.4, 1.0, 0.7, 0.6, .38, .2]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# VTSC variables
TARGET_LAT_A = 1.9  # m/s^2
MIN_TARGET_V = 5    # m/s


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_min_accel_eco_tune(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP_CUSTOM, A_CRUISE_MIN_VALS_ECO_TUNE)

def get_max_accel_eco_tune(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO_TUNE)

def get_min_accel_sport_tune(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP_CUSTOM, A_CRUISE_MIN_VALS_SPORT_TUNE)

def get_max_accel_sport_tune(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT_TUNE)

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)
    self.v_model_error = 0.0

    self.x_desired_trajectory = np.zeros(CONTROL_N)
    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.param_read_counter = 0
    self.read_param()
    self.personality = log.LongitudinalPersonality.standard

    # FrogPilot variables
    self.is_metric = self.params.get_bool("IsMetric")

    self.green_light = False
    self.override_slc = False
    self.previously_driving = False
    self.stopped_for_light_previously = False

    self.green_light_count = 0
    self.overridden_speed = 0
    self.slc_target = 0
    self.v_offset = 0
    self.v_target = MIN_TARGET_V

  def read_param(self):
    try:
      self.personality = int(self.params.get('LongitudinalPersonality'))
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard

  @staticmethod
  def parse_model(model_msg, model_error):
    if (len(model_msg.position.x) == 33 and
       len(model_msg.velocity.x) == 33 and
       len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm):
    if self.param_read_counter % 50 == 0:
      self.read_param()
    self.param_read_counter += 1
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_lead = sm['radarState'].leadOne.vLead
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # neokii
    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      self.vCluRatio = vCluRatio
      v_cruise *= vCluRatio
      #v_cruise = int(v_cruise * CV.MS_TO_KPH + 0.25) * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      if self.acceleration_profile == 1:
        accel_limits = [get_min_accel_eco_tune(v_ego), get_max_accel_eco_tune(v_ego)]
      elif self.acceleration_profile == 2:
        accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      elif self.acceleration_profile == 3:
        accel_limits = [get_min_accel_sport_tune(v_ego), get_max_accel_sport_tune(v_ego)]
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      
      #ajouatom
      self.mpc.prev_a = np.full(N+1, self.a_desired) ## mpc에서는 prev_a를 참고하여 constraint작동함.... pid off -> on시에는 현재 constraint가 작동하지 않아서 집어넣어봄...
      accel_limits_turns[0] = accel_limits_turns[0] = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    carcontrol, carstate, modeldata, radarstate = sm['carControl'], sm['carState'], sm['modelV2'], sm['radarState']
    enabled = sm['controlsState'].enabled
    have_lead = ConditionalExperimentalMode.detect_lead(radarstate)

    # Conditional Experimental Mode
    if self.conditional_experimental_mode and enabled:
      ConditionalExperimentalMode.update(carstate, modeldata, radarstate, v_ego, v_lead, self.v_offset)

    # Green light alert
    if self.green_light_alert:
      standstill = carstate.standstill

      self.previously_driving |= not standstill
      self.previously_driving &= carcontrol.drivingGear

      stopped_for_light = ConditionalExperimentalMode.stop_sign_and_light(carstate, have_lead, radarstate.leadOne.dRel, modeldata, v_ego, v_lead) and standstill

      self.green_light_count += 1 if not stopped_for_light and self.stopped_for_light_previously else -1
      self.green_light_count = np.clip(self.green_light_count, 0, 10)

      # Only trigger the alert if the green light is detected for 0.5 seconds
      self.green_light = self.green_light_count >= 10 and self.previously_driving and not have_lead
      # Reset the counter when the green light alert is triggered
      self.green_light_count *= not self.green_light

      self.stopped_for_light_previously |= stopped_for_light
      self.stopped_for_light_previously &= not self.green_light

    # Pfeiferj's Speed Limit Controller
    if self.speed_limit_controller:
      speed_limit = min(carstate.cruiseState.speedLimit, self.update_speed_apilot(sm, v_cruise))
      SpeedLimitController.update_current_max_velocity(speed_limit, v_cruise)
      #SpeedLimitController.update_current_max_velocity(carstate.cruiseState.speedLimit, v_cruise)
      desired_speed_limit = SpeedLimitController.desired_speed_limit

      # Override SLC upon gas pedal press and reset upon brake/cancel button
      self.override_slc |= carstate.gasPressed
      self.override_slc &= enabled
      self.override_slc &= v_ego > desired_speed_limit > 0

      # Set the max speed to the manual set speed
      if carstate.gasPressed:
        self.overridden_speed = np.clip(v_ego, desired_speed_limit, v_cruise)
      self.overridden_speed *= enabled

      # Use the speed limit if its not being overridden
      if not self.override_slc:
        if v_cruise > desired_speed_limit > 0:
          self.slc_target = round(desired_speed_limit)
          v_cruise = self.slc_target
      else:
        self.slc_target = self.overridden_speed

    # Pfeiferj's Vision Turn Controller
    if self.vision_turn_controller and prev_accel_constraint:
      # Set the curve sensitivity
      orientation_rate = np.array(np.abs(modeldata.orientationRate.z)) * self.curve_sensitivity
      velocity = np.array(modeldata.velocity.x)

      # Get the maximum lat accel from the model
      self.max_pred_lat_acc = np.amax(orientation_rate * velocity)

      # Get the maximum curve based on the current velocity
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Set the target lateral acceleration
      adjusted_target_lat_a = TARGET_LAT_A * self.turn_aggressiveness

      # Get the target velocity for the maximum curve
      self.v_target = (adjusted_target_lat_a / max_curve) ** 0.5
      self.v_target = np.nanmax([self.v_target, MIN_TARGET_V])

      # Configure the offset value for the UI
      self.v_offset = max(0, int(v_cruise - self.v_target))

      if v_cruise > self.v_target:
        v_cruise = max(5, self.v_target)
    else:
      self.v_offset = 0

    self.mpc.set_weights(prev_accel_constraint, self.custom_personalities, self.aggressive_jerk, self.standard_jerk, self.relaxed_jerk, personality=self.personality)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, have_lead, self.aggressive_acceleration, self.increased_stopping_distance, self.smoother_braking,
                    self.custom_personalities, self.aggressive_follow, self.standard_follow, self.relaxed_follow, personality=self.personality)

    self.x_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.x_solution)
    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.x_desired_trajectory = self.x_desired_trajectory_full[:CONTROL_N]
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.distances = self.x_desired_trajectory.tolist()
    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time
    longitudinalPlan.personality = self.personality

    # FrogPilot longitudinalPlan variables
    longitudinalPlan.conditionalExperimental = ConditionalExperimentalMode.experimental_mode
    longitudinalPlan.greenLight = bool(self.green_light)
    longitudinalPlan.slcOverridden = self.override_slc
    longitudinalPlan.slcSpeedLimit = SpeedLimitController.desired_speed_limit
    longitudinalPlan.slcSpeedLimitOffset = SpeedLimitController.offset
    longitudinalPlan.vtscOffset = self.v_offset
    # LongitudinalPlan variables for onroad driving insights
    longitudinalPlan.safeObstacleDistance = self.mpc.safe_obstacle_distance
    longitudinalPlan.stoppedEquivalenceFactor = self.mpc.stopped_equivalence_factor
    longitudinalPlan.desiredFollowDistance = self.mpc.safe_obstacle_distance - self.mpc.stopped_equivalence_factor
    longitudinalPlan.safeObstacleDistanceStock = self.mpc.safe_obstacle_distance_stock
    longitudinalPlan.stoppedEquivalenceFactorStock = self.mpc.stopped_equivalence_factor_stock

    pm.send('longitudinalPlan', plan_send)
    
  def update_frogpilot_params(self):
    self.longitudinal_tune = self.params.get_bool("LongitudinalTune")
    self.acceleration_profile = self.params.get_int("AccelerationProfile") if self.longitudinal_tune else 2
    self.aggressive_acceleration = self.params.get_bool("AggressiveAcceleration") and self.longitudinal_tune
    self.increased_stopping_distance = self.params.get_int("StoppingDistance") * (1 if self.is_metric else 0.3048) if self.longitudinal_tune else 0
    self.smoother_braking = self.params.get_bool("SmoothBraking") and self.longitudinal_tune

    self.conditional_experimental_mode = self.params.get_bool("ConditionalExperimental")
    if self.conditional_experimental_mode:
      self.params.put_bool("ExperimentalMode", True)

    self.custom_personalities = self.params.get_bool("CustomPersonalities")
    self.aggressive_follow = self.params.get_int("AggressiveFollow") / 10
    self.standard_follow = self.params.get_int("StandardFollow") / 10
    self.relaxed_follow = self.params.get_int("RelaxedFollow") / 10
    self.aggressive_jerk = self.params.get_int("AggressiveJerk") / 10
    self.standard_jerk = self.params.get_int("StandardJerk") / 10
    self.relaxed_jerk = self.params.get_int("RelaxedJerk") / 10

    self.green_light_alert = self.params.get_bool("GreenLightAlert")
    self.speed_limit_controller = self.params.get_bool("SpeedLimitController")

    self.vision_turn_controller = self.params.get_bool("VisionTurnControl")
    if self.vision_turn_controller:
      self.curve_sensitivity = self.params.get_int("CurveSensitivity") / 100
      self.turn_aggressiveness = self.params.get_int("TurnAggressiveness") / 100



## ajouatom
  def decelerate_for_speed_camera(self, safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):

    if left_dist <= safe_dist:
      return safe_speed
    temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
    dV = (-safe_speed + math.sqrt(temp)) * decel_rate
    apply_speed = min(250 , safe_speed + dV)
    #min_speed = prev_apply_speed - (decel_rate * 1.2) * 2 * DT_MDL
    #apply_speed = max(apply_speed, min_speed)
    return apply_speed

  def update_speed_apilot(self, sm, v_cruise):
    CS = sm['carState']
    v_ego = CS.vEgoCluster
    msg = self.roadLimitSpeed = sm['roadLimitSpeed']

    active = msg.active
    self.ndaActive = 1 if active > 0 else 0
    roadSpeed = clip(30, msg.roadLimitSpeed, 150.0)
    camType = int(msg.camType)
    xSignType = msg.xSignType

    isSpeedBump = False
    isSectionLimit = False
    safeSpeed = 0
    leftDist = 0
    speedLimitType = 0
    safeDist = 0
    
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = 2
    
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
      #HW: v_cruise값을 넣으면 안됨... 이전 적용값을 넣어야하는데... 최소감속량을 말함..
      applySpeed = self.decelerate_for_speed_camera(safeSpeed/3.6, safeDist, v_cruise, self.autoNaviSpeedDecelRate, leftDist) * 3.6
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
    return applySpeed * CV.KPH_TO_MS #, roadSpeed, leftDist, speedLimitType
