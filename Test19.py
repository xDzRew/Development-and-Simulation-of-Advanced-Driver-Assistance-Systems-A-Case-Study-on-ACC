#!/usr/bin/env python
import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'
        ))[0]
    )
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

if sys.version_info >= (3, 0):
    from configparser import ConfigParser
else:
    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import (
        KMOD_CTRL, KMOD_SHIFT, K_0, K_9, K_BACKQUOTE, K_BACKSPACE,
        K_COMMA, K_DOWN, K_ESCAPE, K_F1, K_LEFT, K_PERIOD, K_RIGHT,
        K_SLASH, K_SPACE, K_TAB, K_UP, K_a, K_c, K_d, K_h, K_m, K_p,
        K_q, K_r, K_s, K_w, K_l, K_i, K_z, K_x,
        K_b, K_n
    )
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame package is installed")

try:
    import numpy as np
except ImportError:
    raise RuntimeError("cannot import numpy, make sure numpy package is installed")


def find_weather_presets():
    """
    ฟังก์ชันสำหรับดึงรายการ weather presets ในโลกของ CARLA
    เพื่อใช้ในการเปลี่ยนสภาพอากาศ
    """
    rgx = re.compile(r'.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    """
    ฟังก์ชันช่วยในการดึง "ชื่อ" ของ actor (เช่น ยี่ห้อ/รุ่นรถ)
    เพื่อแสดงผลใน HUD
    """
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
class World(object):
    """
    คลาส World จัดการสภาพแวดล้อมหลักของ CARLA (world), HUD, และการ spawn/destroy รถหลัก (player)
    รวมถึงติดตั้งเซนเซอร์ต่าง ๆ (CollisionSensor, LaneInvasionSensor, GnssSensor, CameraManager)
    """
    def __init__(self, carla_world, hud, actor_filter):
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        """
        ฟังก์ชัน restart สำหรับสลับรถ หรือ reset ตำแหน่งการ spawn รถใหม่
        รวมถึงการสร้างหรือติดตั้ง sensor ใหม่
        """
        cam_index = self.camera_manager.index if self.camera_manager else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager else 0

        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if self.player:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

        while not self.player:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

        # Attach sensors
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)

        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        """
        ฟังก์ชันเปลี่ยนสภาพอากาศ (Weather Preset)
        """
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        """
        เรียกในทุกๆ frame (world tick) เพื่ออัปเดต HUD
        """
        self.hud.tick(self, clock)

    def render(self, display):
        """
        วาดภาพจากกล้อง (camera_manager) และ HUD ลงใน pygame display
        """
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        """
        ฟังก์ชันทำลาย (destroy) รถและเซนเซอร์ต่าง ๆ ก่อนเปลี่ยนหรือลบโลก
        """
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor
        ]
        for s in sensors:
            if s:
                s.stop()
                s.destroy()
        if self.player:
            self.player.destroy()


# ==============================================================================
# -- DualControl (ACC in km/h) -------------------------------------------------
# ==============================================================================
class DualControl(object):
    """
    คลาส DualControl จัดการ Input ทั้งหมดของรถหลัก (player) ไม่ว่าจะเป็น
    - Keyboard
    - Joystick/Wheel
    - ตลอดจน logic ของระบบ ACC (Adaptive Cruise Control)
    """
    def __init__(self, world):
        # ตรวจสอบว่า actor เป็น vehicle หรือ walker
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")

        self._steer_cache = 0.0
        world.hud.notification(
            "Press 'p' to toggle ACC; 'b/n' to adjust ACC speed; 'h' or '?' for help.", 4.0)

        # ---------------------------
        # พารามิเตอร์การทำงานของระบบ ACC
        # ---------------------------
        self.acc_enabled = False
        self.acc_target_speed_kmh = 20.0

        self.acc_kp = 1.0
        self.front_detect_range = 120
        self.front_detect_angle = 50
        self.acc_follow_distance = 20

        # (แก้ไข) แยกตัวแปร limit คันเร่งเป็นสองตัว
        self.acc_throttle_limit_no_car = 1.0      # เร่งได้เต็ม เมื่อไม่เจอรถคันหน้า
        self.acc_throttle_limit_with_car = 0.5    # เร่งได้แค่ครึ่ง เมื่อเจอรถคันหน้า

        # state สำหรับจับการกดคันเร่งซ้ำสองครั้ง (เพื่อ OFF ACC)
        self.acc_pedal_state = 0
        self.acc_throttle_prev = False

        # ข้อมูลรถข้างหน้า + เลนของเรา (อัปเดตใน parse_events)
        self.front_vehicles_info = []
        self.ego_lane_id = None

        # Init joystick/wheel
        pygame.joystick.init()
        if pygame.joystick.get_count() > 1:
            print("Warning: multiple joysticks found, using the first one!")
        try:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
        except:
            self._joystick = None
            print("No joystick found. Using keyboard only?")

        # อ่าน config ของ wheel (ถ้ามี)
        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(self._parser.get('G29 Racing Wheel', 'steering_wheel', fallback=0))
        self._throttle_idx = int(self._parser.get('G29 Racing Wheel', 'throttle', fallback=1))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake', fallback=2))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse', fallback=10))
        self._handbrake_idx = int(self._parser.get('G29 Racing Wheel', 'handbrake', fallback=11))

        self._world = world

    def _get_speed_kmh(self):
        """
        ฟังก์ชันช่วยคำนวณความเร็ว (km/h) ของรถเรา (ego vehicle)
        """
        vel = self._world.player.get_velocity()
        speed_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        return speed_ms * 3.6

    def parse_events(self, clock):
        """
        ฟังก์ชันหลักในการอ่าน event จาก pygame (ทั้ง keyboard/joystick)
        และนำไปคุมพฤติกรรมของรถ รวมถึง logic ของระบบ ACC
        """
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights

        # --- Handle pygame Events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

            elif event.type == pygame.KEYDOWN:
                # Toggle ACC เมื่อกดปุ่ม 'p'
                if event.key == K_p:
                    if not self.acc_enabled:
                        speed_kmh = self._get_speed_kmh()
                        if speed_kmh >= 10.0:
                            self.acc_enabled = True
                            self.acc_target_speed_kmh = speed_kmh
                            self._world.hud.notification("ACC ON")
                            self.acc_pedal_state = 0
                            self.acc_throttle_prev = False
                        else:
                            self._world.hud.notification("Speed <10 km/h, cannot enable ACC")
                    else:
                        self.acc_enabled = False
                        self._world.hud.notification("ACC OFF")

                elif event.key == K_b:
                    if self.acc_enabled:
                        self.acc_target_speed_kmh += 1
                        self._world.hud.notification("ACC speed +1")

                elif event.key == K_n:
                    if self.acc_enabled:
                        self.acc_target_speed_kmh -= 1
                        if self.acc_target_speed_kmh < 0:
                            self.acc_target_speed_kmh = 0
                        self._world.hud.notification("ACC speed -1")

                # คีย์อื่น ๆ
                if event.key == K_BACKSPACE:
                    self._world.restart()
                elif event.key == K_F1:
                    self._world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    self._world.hud.help.toggle()
                elif event.key == K_TAB:
                    self._world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    self._world.next_weather(reverse=True)
                elif event.key == K_c:
                    self._world.next_weather()
                elif event.key == K_BACKQUOTE:
                    self._world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    self._world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    self._world.camera_manager.toggle_recording()

                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = self._world.player.get_control().gear
                        self._world.hud.notification('%s Transmission' % (
                            'Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1

                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        if not self._lights & carla.VehicleLightState.Position:
                            self._world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            self._world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            self._world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            self._world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

            elif event.type == pygame.JOYBUTTONDOWN:
                # จัดการปุ่มบนจอยสติ๊ก/พวงมาลัย ถ้ามี
                if event.button == 0:
                    self._world.restart()
                elif event.button == 1:
                    self._world.hud.toggle_info()
                elif event.button == 2:
                    self._world.camera_manager.toggle_camera()
                elif event.button == 3:
                    self._world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 23:
                    self._world.camera_manager.next_sensor()

            elif event.type == pygame.QUIT:
                return True

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

        # ถ้า actor เป็น Walker จะใช้โหมดควบคุมอีกแบบ
        if not isinstance(self._control, carla.VehicleControl):
            self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            self._world.player.apply_control(self._control)
            return False

        # parse keyboard สำหรับการบังคับเลี้ยว/คันเร่งพื้นฐาน
        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
        # parse wheel (ถ้ามี)
        self._parse_vehicle_wheel()

        # ถ้าแบตหมด (<= 0%) บังคับเบรคเต็ม
        if self._world.hud.battery_level_kwh <= 0.0:
            self._control.throttle = 0.0
            self._control.brake = 1.0
        else:
            # OFF ACC ถ้ากดเบรคเอง > 0.1 หรือเข้าเกียร์ถอยหลัง
            if self.acc_enabled:
                if self._control.brake > 0.1:
                    self.acc_enabled = False
                    self._world.hud.notification("ACC OFF (pressed brake)")
                if self._control.reverse:
                    self.acc_enabled = False
                    self._world.hud.notification("ACC OFF (reverse gear)")

            # อัปเดตข้อมูลรถข้างหน้า + เลน
            self.front_vehicles_info, self.ego_lane_id = self._find_front_vehicles()

            # ถ้า ACC เปิดอยู่ => ประมวลผล ACC
            if self.acc_enabled:
                self._apply_acc()

        # อัปเดตไฟสัญญาณรถ (brake light, reverse light)
        self._control.reverse = self._control.gear < 0
        if self._control.brake:
            current_lights |= carla.VehicleLightState.Brake
        else:
            current_lights &= ~carla.VehicleLightState.Brake
        if self._control.reverse:
            current_lights |= carla.VehicleLightState.Reverse
        else:
            current_lights &= ~carla.VehicleLightState.Reverse
        if current_lights != self._lights:
            self._lights = current_lights
            self._world.player.set_light_state(carla.VehicleLightState(self._lights))

        # สั่งให้รถเคลื่อนตามค่า control
        self._world.player.apply_control(self._control)
        return False

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        อ่านสภาวะปุ่มคีย์บอร์ด (WASD/ลูกศร) เพื่อบังคับรถพื้นฐาน
        """
        self._control.throttle = 1.0 if (keys[K_UP] or keys[K_w]) else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = max(-0.7, min(self._steer_cache, 0.7))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if (keys[K_DOWN] or keys[K_s]) else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        """
        อ่านสภาวะปุ่ม/แกนของ Wheel/Joystick เพื่อบังคับรถ (ถ้ามี)
        """
        if not self._joystick:
            return

        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]

        # steering
        K1 = 1.0
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        # throttle
        K2 = 1.6
        throttleCmd = K2 + (2.05*math.log10(-0.7*jsInputs[self._throttle_idx]+1.4) - 1.2)/0.92
        throttleCmd = max(0.0, min(throttleCmd, 1.0))

        # brake
        brakeCmd = 1.6 + (2.05*math.log10(-0.7*jsInputs[self._brake_idx]+1.4) - 1.2)/0.92
        brakeCmd = max(0.0, min(brakeCmd, 1.0))
        if brakeCmd < 0.05:
            brakeCmd = 0.0

        pedal_pressed = (throttleCmd > 0.05)
        # ถ้า ACC เปิดอยู่ ให้ตรวจว่าผู้ใช้กดคันเร่งซ้ำเพื่อ OFF ACC หรือไม่
        if self.acc_enabled:
            if pedal_pressed:
                if self.acc_pedal_state == 0:
                    self.acc_pedal_state = 1
                    self._world.hud.notification("ACC pedal pressed first time => still ON")
                elif self.acc_pedal_state == 1:
                    if not self.acc_throttle_prev:
                        self.acc_pedal_state = 2
                        self.acc_enabled = False
                        self._world.hud.notification("ACC OFF (second pedal press)")
        self.acc_throttle_prev = pedal_pressed

        self._control.steer = steerCmd
        self._control.throttle = throttleCmd
        self._control.brake = brakeCmd
        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _same_lane_check(self, ego_wp, v_wp):
        """
        ตรวจสอบว่า waypoint ของรถคันหน้า (v_wp) อยู่เลนเดียวกันกับ ego_wp (รถเรา) หรือไม่
        โดยเพิ่มการตรวจสอบ heading ให้แม่นยำขึ้น
        """
        if not ego_wp or not v_wp:
            return False
        if v_wp.lane_type != carla.LaneType.Driving:
            return False

        # ตรวจสอบ lane_id มีทิศทางเดียวกัน (lane_id มีสัญลักษณ์ + เหมือนกัน)
        if (ego_wp.lane_id * v_wp.lane_id) <= 0:
            return False

        # lane_id ตรงกันหรือไม่ (ในแง่ absolute)
        lane_diff = abs(abs(ego_wp.lane_id) - abs(v_wp.lane_id))
        if lane_diff != 0:
            return False

        # ตรวจสอบ heading ว่าใกล้เคียงกันหรือไม่ (ป้องกันรถมาอีกทิศ)
        heading_threshold = 60  # องศา
        ego_heading = ego_wp.transform.rotation.yaw
        veh_heading = v_wp.transform.rotation.yaw

        def normalize_angle(deg):
            while deg > 180.0:
                deg -= 360.0
            while deg < -180.0:
                deg += 360.0
            return deg

        ego_heading = normalize_angle(ego_heading)
        veh_heading = normalize_angle(veh_heading)
        heading_diff = abs(ego_heading - veh_heading)
        if heading_diff > heading_threshold:
            return False

        return True

    def _find_front_vehicles(self):
        """
        หา "รถที่อยู่ด้านหน้า" ในระยะ front_detect_range และมุม front_detect_angle
        ส่งคืนรายการรถ (เรียงจากระยะใกล้->ไกล) พร้อมระบุว่า same_lane หรือไม่
        """
        ego_transform = self._world.player.get_transform()
        ego_location = ego_transform.location
        ego_forward = ego_transform.get_forward_vector()

        ego_wp = self._world.world.get_map().get_waypoint(
            ego_location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        ego_lane_id = ego_wp.lane_id if ego_wp else None

        vehicles = self._world.world.get_actors().filter('vehicle.*')
        front_vehicles_info = []

        for v in vehicles:
            if v.id == self._world.player.id:
                continue
            loc = v.get_transform().location
            diff = loc - ego_location
            dist = diff.length()
            if dist > self.front_detect_range:
                continue

            dot = (diff.x * ego_forward.x + diff.y * ego_forward.y + diff.z * ego_forward.z)
            lengthF = math.sqrt(ego_forward.x**2 + ego_forward.y**2 + ego_forward.z**2)
            cosA = dot / (dist * lengthF + 1e-6)
            cosA = max(min(cosA, 1.0), -1.0)
            angle_deg = math.degrees(math.acos(cosA))

            if angle_deg < self.front_detect_angle:
                v_wp = self._world.world.get_map().get_waypoint(
                    v.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
                )
                same_lane = self._same_lane_check(ego_wp, v_wp)

                vel = v.get_velocity()
                speed_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                speed_kmh = speed_ms * 3.6

                front_vehicles_info.append({
                    'id': v.id,
                    'speed': speed_kmh,
                    'distance': dist,
                    'same_lane': same_lane
                })

        front_vehicles_info.sort(key=lambda x: x['distance'])
        return front_vehicles_info, ego_lane_id

    def _apply_acc(self):
        """
        ฟังก์ชันคำนวณคันเร่ง/เบรค ของระบบ ACC
        เพิ่มเงื่อนไข:
          - ระยะ < 25 ม. => เบรคแรง
          - ระยะ < 30 ม. => เบรคเบา
          - (ใหม่) ระยะ <= 40 ม. => ยกคันเร่ง (ไม่เบรค)
        """
        front_same_lane = [f for f in self.front_vehicles_info if f['same_lane']]

        if len(front_same_lane) > 0:
            lead = front_same_lane[0]
            dist_lead = lead['distance']

            # 1) เมื่อระยะ <= 22 ม. => เบรคแรง
            if dist_lead <= 22.00:
                self._control.throttle = 0.0
                self._control.brake = 0.6
                return

            # 2) เมื่อระยะ <= 25 ม. => เบรคเบา
            elif dist_lead <= 25.00:
                self._control.throttle = 0.0
                self._control.brake = 0.3
                return

            # 3) เมื่อระยะ <= 28 ม. => ยกคันเร่ง (ไม่เบรค)
            elif dist_lead <= 28.00:
                self._control.throttle = 0.0
                self._control.brake = 0.0
                return

            else:
                # ถ้าเกิน 28 ม. => ทำงานตาม logic ACC ปกติ
                if dist_lead <= self.acc_follow_distance:
                    target_kmh = min(lead['speed'], self.acc_target_speed_kmh)
                else:
                    target_kmh = self.acc_target_speed_kmh
        else:
            # ถ้าไม่มีรถคันหน้าที่เลนเดียวกัน => วิ่งตาม acc_target_speed_kmh
            target_kmh = self.acc_target_speed_kmh

        # ส่วนคำนวณ throttle/brake จากส่วนต่างระหว่างความเร็วเป้าหมายกับปัจจุบัน
        current_ms = self._get_speed_kmh() / 3.6
        target_ms = target_kmh / 3.6
        error = target_ms - current_ms

        acc_cmd = self.acc_kp * error

        if acc_cmd > 0:
            new_throttle = min(acc_cmd, 1.0)
            new_brake = 0.0
        else:
            new_throttle = 0.0
            new_brake = min(abs(acc_cmd), 1.0)

        # (แก้ไข) เลือก throttle limit ตามว่ามีรถคันหน้าหรือไม่
        if len(front_same_lane) > 0:
            new_throttle = min(new_throttle, self.acc_throttle_limit_with_car)
        else:
            new_throttle = min(new_throttle, self.acc_throttle_limit_no_car)

        self._control.throttle = new_throttle
        self._control.brake = new_brake

    def _parse_walker_keys(self, keys, milliseconds):
        """
        ถ้า actor เป็น Walker (คนเดิน) จะใช้ฟังก์ชันนี้ในการขยับ
        """
        self._control.speed = 0.0
        if keys[pygame.locals.K_DOWN] or keys[pygame.locals.K_s]:
            self._control.speed = 0.0
        if keys[pygame.locals.K_LEFT] or keys[pygame.locals.K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[pygame.locals.K_RIGHT] or keys[pygame.locals.K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[pygame.locals.K_UP] or keys[pygame.locals.K_w]:
            self._control.speed = 5.556 if (pygame.key.get_mods() & KMOD_SHIFT) else 2.778
        self._control.jump = keys[pygame.locals.K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        """
        ช่วยเช็คว่าผู้ใช้กดปิดโปรแกรมด้วยปุ่ม ESC หรือ Ctrl+Q หรือไม่
        """
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD, etc. -----------------------------------------------------------------
# ==============================================================================
class HUD(object):
    """
    จัดการการแสดงผลข้อมูลต่าง ๆ ของเกม เช่น FPS, Speed, ตำแหน่ง, จำนวนรถ,
    สถานะแบตเตอรี่, ข้อความแจ้งเตือน ฯลฯ
    """
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        # แบตเตอรี่ (สมมุติ) เริ่มต้น
        self.battery_capacity_kwh = 60.0
        self.battery_level_kwh = self.battery_capacity_kwh

        # ข้อมูลรถข้างหน้าและเลน (สำหรับแสดงใน HUD)
        self.front_vehicles_info = []
        self.ego_lane_id = None

    def on_world_tick(self, timestamp):
        """
        เรียกทุกครั้งเมื่อมี world tick เพื่อ update FPS หรือค่าสภาพโลกอื่น ๆ
        """
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        """
        เรียกในทุกๆ frame เพื่อ update ข้อมูลใน HUD
        """
        self._notifications.tick(world, clock)
        if not self._show_info:
            return

        dt = clock.get_time() * 1e-3
        player = world.player
        if player:
            vel = player.get_velocity()
            speed_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

            # สมการคำนวนใช้พลังงานแบบง่าย
            P_base = 5.0   # kW
            k = 0.0002     # แรงต้านตาม v^3
            load_kW = P_base + k * (speed_ms ** 3)

            # Energy (kWh) = (kW / 3600) * dt
            energy_used_kWh = (load_kW / 3600.0) * dt
            self.battery_level_kwh -= energy_used_kWh
            if self.battery_level_kwh < 0:
                self.battery_level_kwh = 0

        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()

        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if (179.5 > t.rotation.yaw > 0.5) else ''
        heading += 'W' if (-0.5 > t.rotation.yaw > -179.5) else ''

        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]

        vehicles = world.world.get_actors().filter('vehicle.*')
        speed_current = 3.6*math.sqrt(v.x**2 + v.y**2 + v.z**2)

        battery_percentage = (self.battery_level_kwh / self.battery_capacity_kwh) * 100

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.world.get_map().name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.1f km/h' % speed_current,
            f'Heading: {t.rotation.yaw:.1f} deg {heading}',
            'Location:% 20s' % ('(%5.1f,%5.1f)' % (t.location.x, t.location.y)),
            f'GNSS:    ({world.gnss_sensor.lat:.6f},{world.gnss_sensor.lon:.6f})',
            'Height:  %5.1f m' % t.location.z,
            ''
        ]
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear: %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)
            ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)
            ]

        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles),
            ''
        ]

        # เตือนเมื่อแบต < 10%
        if battery_percentage < 10.0 and self.battery_level_kwh > 0.0:
            self.notification("WARNING: Low Battery (<10%)", 1.0)

        # แสดงสถานะแบต
        self._info_text.append(
            f'Battery: {self.battery_level_kwh:.2f} kWh ({battery_percentage:.1f}%)'
        )

        # ข้อมูลเลนและรถคันหน้า (นำมาจาก controller)
        self._info_text += [
            '',
            f'Ego lane_id: {self.ego_lane_id}',
            '',
            f'Front Vehicles in range: {len(self.front_vehicles_info)}'
        ]
        if len(self.front_vehicles_info) == 0:
            self._info_text.append('  - None -')
        else:
            for fv in self.front_vehicles_info:
                self._info_text.append(
                    f'  ID:{fv["id"]} Dist:{fv["distance"]:.1f}m '
                    f'Spd:{fv["speed"]:.1f}km/h same_lane={fv["same_lane"]}'
                )

    def toggle_info(self):
        """
        แสดง/ซ่อนข้อมูล HUD
        """
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """
        แสดงข้อความ notification ชั่วคราว
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """
        แสดงข้อความ error (เปลี่ยนสีเป็นแดง)
        """
        self._notifications.set_text(f'Error: {text}', (255, 0, 0))

    def render(self, display):
        """
        ฟังก์ชันวาด text หรือกราฟใน HUD ลงบนหน้าจอ pygame
        """
        if self._show_info:
            info_surface = pygame.Surface((280, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 120
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x+8, v_offset+8 + (1.0-y)*30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset+8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset+8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2])/(item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + f*(bar_width-6), v_offset+8), (6, 6))
                        else:
                            rect = pygame.Rect(
                                (bar_h_offset, v_offset+8), (f*bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18

        self._notifications.render(display)
        self.help.render(display)


class FadingText(object):
    """
    จัดการข้อความ notification แบบค่อย ๆ จางหายไป (fade-out)
    """
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3*clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0*self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


class HelpText(object):
    """
    แสดง help text หรือ docstring ของสคริปต์ เมื่อกดปุ่ม 'h' หรือ '?'
    """
    def __init__(self, font, width, height):
        doc_text = __doc__ or "No docstring"
        lines = doc_text.split('\n')
        self.font = font
        self.dim = (680, len(lines)*22 + 12)
        self.pos = (0.5*width - 0.5*self.dim[0], 0.5*height - 0.5*self.dim[1])
        self._render = False
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n*22))
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


class CollisionSensor(object):
    """
    เซนเซอร์ตรวจจับการชน (Collision) บันทึกและส่งค่าแรงกระแทก
    ไปแสดงใน HUD
    """
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification(f"Collision with {actor_type}")
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


class LaneInvasionSensor(object):
    """
    เซนเซอร์ตรวจจับการข้ามเส้นแบ่งเลน
    """
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = [f"{str(x).split()[-1]}" for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


class GnssSensor(object):
    """
    เซนเซอร์ GNSS สำหรับเก็บค่าพิกัด latitude/longitude
    """
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(carla.Location(x=1.0, z=2.8)),
            attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


class CameraManager(object):
    """
    จัดการกล้องหลัก (sensor.camera.* หรือ sensor.lidar.*)
    สามารถเปลี่ยนโหมดหรือสลับตำแหน่งกล้องได้
    """
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))
        ]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        """
        สลับตำแหน่งกล้อง (เช่น มุมมองหลังรถ, ในรถ)
        """
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        if self.sensor:
            self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        """
        เปลี่ยนชนิด sensor (camera หรือ lidar) ตามที่กำหนด
        """
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (
            self.sensors[index][0] != self.sensors[self.index][0])
        if needs_respawn:
            if self.sensor:
                self.sensor.stop()
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent
            )
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        """
        เลือก sensor ถัดไป (วนลูป)
        """
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        """
        toggle บันทึกภาพจาก sensor ลงดิสก์
        """
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        """
        วาดภาพจากกล้องลงใน pygame display
        """
        if self.surface:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        """
        ฟังก์ชัน callback สำหรับ sensor.camera / sensor.lidar
        แปลงเป็นภาพแล้วนำไปแสดงใน pygame
        """
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim)/100.0
            lidar_data += (0.5*self.hud.dim[0], 0.5*self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
def game_loop(args):
    """
    ฟังก์ชันหลักที่รัน loop ของ pygame:
      - สร้าง HUD, World
      - สร้างตัวควบคุมรถ (DualControl)
      - วน loop รับ event, อัปเดต HUD, วาดภาพ
    """
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.FULLSCREEN | pygame.DOUBLEBUF
        )
        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter)
        controller = DualControl(world)

        clock = pygame.time.Clock()
        while True:
            # ปรับเป็น 120 FPS เพื่อให้ตอบสนองได้ไวขึ้น
            clock.tick_busy_loop(120)
            if controller.parse_events(clock):
                return

            # HUD อ่านข้อมูลรถหน้า + lane (อัปเดตจาก controller) ทุกเฟรม
            hud.front_vehicles_info = controller.front_vehicles_info
            hud.ego_lane_id = controller.ego_lane_id

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world:
            world.destroy()
        pygame.quit()


def main():
    """
    จุดเริ่มต้นของสคริปต์: อ่าน argument และเรียก game_loop
    """
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client with ACC in km/h (enhanced ACC logic)'
    )
    argparser.add_argument('--host', metavar='H', default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--res', metavar='WIDTHxHEIGHT', default='1280x720',
                           help='window resolution (default: 1280x720)')
    argparser.add_argument('--filter', metavar='PATTERN', default='vehicle.*',
                           help='actor filter (default: "vehicle.*")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    print(__doc__)

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
