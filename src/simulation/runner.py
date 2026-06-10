"""Orkiestracja symulacji CARLA + NURD.

Spina warstwy: planowanie trasy, spawn aktorów, percepcję (NURD) i sterowanie.
Sama nie liczy geometrii ani ryzyka — deleguje do wyspecjalizowanych modułów.
"""

import os
import queue

import carla
import cv2

from nurd_app import NURDApp
from src.nurd_system.control import VehicleControlModule
from src.nurd_system.risk import RiskLevel
from src.simulation.carla_utils import carla_image_to_bgr

from .config import (
    CAMERA_PRESETS,
    FOV,
    FREE_WALK_SPEED_MPS,
    IMG_H,
    IMG_W,
    ROUTE_MIN_M,
    ROUTE_STEP_M,
)
from .crossing import crossing_at_route_distance
from .geometry import planar_distance
from .hud import draw_status_overlay
from .props import spawn_road_sign
from .route_planner import find_vehicle_spawn, load_map_and_route
from .weather import apply_weather
from .scenario import (
    build_route_crossings,
    next_scene,
    previous_scene_active,
    scene_trigger_lead,
)
from .vru_spawner import spawn_free_sidewalk_pedestrians, spawn_vru


class CarlaSimulation:
    """Pojedynczy przebieg symulacji sterowany argumentami CLI."""

    def __init__(self, args):
        self.args = args
        self.brake_enabled = not args.no_brake

        self.nurd = self._build_perception(args)
        self.control = VehicleControlModule(cruise_speed_kmh=args.cruise_speed)

        self.client = None
        self.world = None
        self.carla_map = None
        self.blueprint_library = None
        self.route = None
        self.spawn_transform = None
        self.original_settings = None

        self.vehicle = None
        self.camera = None
        self.spectator = None
        self.image_queue = queue.Queue()
        self.actors = []

        self.planned_crossings = []
        self.sidewalk_patrols = []
        self.free_walkers = []

        self.actual_route_length = 0.0
        self.route_start = None

    @staticmethod
    def _build_perception(args) -> NURDApp:
        focal_length = VehicleControlModule.focal_length_from_fov(IMG_W, FOV)
        return NURDApp(
            args.model,
            focal_length_px=focal_length,
            image_width=IMG_W,
            image_height=IMG_H,
            base_speed=args.cruise_speed,
            fuse_closing_speed=args.fuse_closing_speed,
        )

    def connect_and_load(self):
        args = self.args
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(30.0)
        print(f"[*] Łączenie z CARLA: {args.host}:{args.port}")

        loaded_map, world, spawn_transform, route = load_map_and_route(
            self.client, args.map, args.route_length, ROUTE_MIN_M,
        )
        if spawn_transform is None or not route:
            raise RuntimeError("Nie udało się zaplanować prostej trasy")

        self.world = world
        self.spawn_transform = spawn_transform
        self.route = route
        self.carla_map = world.get_map()
        self.blueprint_library = world.get_blueprint_library()
        print(f"[*] Mapa: {loaded_map}")

        self.actual_route_length = min(args.route_length, len(route) * ROUTE_STEP_M)
        self.planned_crossings = build_route_crossings(self.actual_route_length)
        self._print_plan()

    def _print_plan(self):
        args = self.args
        print(f"[*] Trasa: {self.actual_route_length:.0f} m prosto, {args.cruise_speed:.0f} km/h")
        sequence = " → ".join(pc["label"] for pc in self.planned_crossings)
        print(
            f"[*] Sekwencja {len(self.planned_crossings)} scen "
            f"({args.approach_seconds:.0f}s dojście + {args.pause_on_lane:.0f}s postój): {sequence}"
        )
        for pc in self.planned_crossings:
            kind = "rowerzysta" if pc["vru_type"] == "cyclist" else "pieszy"
            print(f"    - {kind} ({pc['label']}): przejście przy ~{pc['distance_m']:.0f} m trasy")

    def configure_world(self):
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.args.sim_delta
        self.world.apply_settings(settings)
        print(f"[*] Krok symulacji: {self.args.sim_delta}s")

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        applied = apply_weather(self.world, self.args.weather)
        if applied:
            print(f"[*] Pogoda: {applied}")

    def spawn_vehicle_and_camera(self):
        self.vehicle = find_vehicle_spawn(
            self.world, self.carla_map, self.route, self.spawn_transform,
        )
        if self.vehicle is None:
            raise RuntimeError("Nie udało się zespawnować auta na trasie")

        self.actors.append(self.vehicle)
        self.vehicle.set_autopilot(False)
        self.route_start = self.vehicle.get_location()
        print(
            "[*] Jazda prosto + sterowanie NURD (hamowanie)"
            if self.brake_enabled
            else "[*] Jazda prosto + detekcja (bez hamowania)"
        )

        camera_bp = self.blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(IMG_W))
        camera_bp.set_attribute("image_size_y", str(IMG_H))
        camera_bp.set_attribute("fov", str(FOV))

        self.camera = self.world.spawn_actor(
            camera_bp, CAMERA_PRESETS[self.args.camera_view], attach_to=self.vehicle,
        )
        self.actors.append(self.camera)
        self.camera.listen(self.image_queue.put)
        print(f"[*] Kamera na aucie: widok '{self.args.camera_view}'")

        if not self.args.no_sync_spectator:
            self.spectator = self.world.get_spectator()
            self.spectator.set_transform(self.camera.get_transform())
            print(f"[*] Okno CARLA = kamera auta (widok '{self.args.camera_view}')")

    def spawn_environment(self):
        args = self.args
        self.free_walkers, self.sidewalk_patrols = spawn_free_sidewalk_pedestrians(
            self.world, self.blueprint_library, self.route, args.free_pedestrians, FREE_WALK_SPEED_MPS,
        )
        self.actors.extend(self.free_walkers)
        if self.free_walkers:
            print(f"[*] {len(self.free_walkers)} pieszych spaceruje po chodnikach")
        else:
            print("[*] Piesi na chodnikach: wyłączeni")

        if args.end_sign:
            sign = spawn_road_sign(
                self.world, self.blueprint_library, self.route, self.actual_route_length * 0.95,
            )
            if sign is not None:
                self.actors.append(sign)
                print("[*] Znak drogowy w pasie (~95% trasy) — obiekt spoza klas YOLO")
            else:
                print("[!] Nie udało się postawić znaku drogowego")

        lead = scene_trigger_lead(args.cruise_speed, args.approach_seconds)
        print(
            f"[*] {len(self.planned_crossings)} scen VRU | auto {args.cruise_speed:.0f} km/h | "
            f"rower {args.cyclist_speed:.0f} km/h | trigger ~{lead:.0f} m przed punktem"
        )

    def _maybe_spawn_next_vru(self, distance_driven: float, lead_m: float):
        planned = next_scene(self.planned_crossings)
        if planned is None or previous_scene_active(self.planned_crossings):
            return
        if distance_driven < planned["distance_m"] - lead_m:
            return

        args = self.args
        is_vehicle = planned["vru_type"] in ("cyclist", "motorcycle")
        ride_speed_mps = args.cyclist_speed / 3.6
        speed = ride_speed_mps if is_vehicle else args.pedestrian_speed

        start, lane_stop, end = crossing_at_route_distance(
            self.route, planned["distance_m"], planned["from_side"],
            speed, args.approach_seconds, is_bike=is_vehicle,
        )
        actor, manual = spawn_vru(
            self.world, self.blueprint_library, planned["vru_type"],
            start, lane_stop, end,
            args.pedestrian_speed, ride_speed_mps, args.pause_on_lane,
        )
        manual.begin()
        planned["actor"] = actor
        planned["manual_crossing"] = manual
        planned["triggered"] = True
        self.actors.append(actor)

        kind = {"cyclist": "Rowerzysta", "motorcycle": "Motocykl"}.get(planned["vru_type"], "Pieszy")
        print(
            f"[!] {kind} ({planned['label']}): start ~{speed * args.approach_seconds:.0f} m "
            f"na jezdni → {args.approach_seconds:.0f}s dojście → "
            f"{args.pause_on_lane:.0f}s na pasie (trasa ~{planned['distance_m']:.0f} m)",
            flush=True,
        )

    def _advance_vru(self, dt: float):
        for planned in self.planned_crossings:
            mc = planned.get("manual_crossing")
            actor = planned.get("actor")
            if mc is not None and mc.active and actor is not None:
                mc.update(actor, dt)
        for patrol, walker in zip(self.sidewalk_patrols, self.free_walkers):
            if patrol.active and walker.is_alive:
                patrol.update(walker, dt)

    def _latest_image(self):
        image = self.image_queue.get(timeout=2.0)
        while not self.image_queue.empty():
            image = self.image_queue.get_nowait()
        return image

    def _apply_control(self, speed_ms: float, risks, distance_driven: float, route_finished: bool):
        steer = self.control.compute_route_steer(
            self.vehicle, self.route, distance_driven, ROUTE_STEP_M,
        )
        if route_finished:
            throttle, brake = 0.0, 0.6
        elif self.brake_enabled:
            throttle, brake = self.control.compute_throttle_brake(speed_ms, risks)
            worst = self.control.select_worst_risk(risks) if risks else None
            if worst and worst["risk_level"] in (RiskLevel.HIGH, RiskLevel.CRITICAL):
                print(
                    f"[!] {worst['risk_level'].name} "
                    f"TTC={worst['ttc_value']:.1f}s brake={brake:.2f}"
                )
        else:
            throttle, brake = self.control.compute_throttle_brake(speed_ms, [])

        self.vehicle.apply_control(
            carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
        )

    def run_loop(self):
        args = self.args
        lead_m = scene_trigger_lead(args.cruise_speed, args.approach_seconds)
        headless = self._init_window()

        # Krok symulacji (czas „świata”). Używamy go do kinematyki i ruchu VRU,
        # by percepcja i scena były spójne oraz odporne na czas inferencji YOLO.
        sim_dt = max(self.args.sim_delta, 1e-3)
        frame_count = 0
        route_finished = False

        while True:
            self.world.tick()
            self._sync_spectator()

            velocity = self.vehicle.get_velocity()
            speed_ms = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
            speed_kmh = speed_ms * 3.6
            distance_driven = planar_distance(self.route_start, self.vehicle.get_location())

            self._maybe_spawn_next_vru(distance_driven, lead_m)

            if distance_driven >= self.actual_route_length and not route_finished:
                route_finished = True
                print(f"[*] Koniec trasy ({distance_driven:.0f} m)", flush=True)

            frame = carla_image_to_bgr(self._latest_image())

            self._advance_vru(sim_dt)

            processed_frame, risks = self.nurd.process_frame(frame.copy(), sim_dt, ego_speed_ms=speed_ms)
            draw_status_overlay(processed_frame, risks, self.brake_enabled, speed_kmh)

            self._apply_control(speed_ms, risks, distance_driven, route_finished)
            frame_count += 1

            if headless:
                self._log_headless(frame_count, speed_kmh, distance_driven, risks)
            else:
                cv2.imshow("NURD + CARLA", processed_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if args.frames > 0 and frame_count >= args.frames:
                print(f"[*] Osiągnięto limit {args.frames} klatek")
                break
            if route_finished and speed_kmh < 1.0 and frame_count > 20:
                print("[*] Auto zatrzymane po trasie")
                break

    def _init_window(self) -> bool:
        if self.args.headless:
            print("[*] Start pętli (headless). Ctrl+C = wyjście")
            return True
        try:
            cv2.namedWindow("NURD + CARLA", cv2.WINDOW_NORMAL)
            print("[*] Start pętli. 'q' = wyjście")
            return False
        except cv2.error:
            print("[!] Brak GUI (Qt/xcb) — przełączam na headless. Ctrl+C = wyjście")
            self.args.headless = True
            return True

    def _sync_spectator(self):
        if self.spectator is not None and self.camera.is_alive:
            try:
                self.spectator.set_transform(self.camera.get_transform())
            except RuntimeError:
                pass

    def _log_headless(self, frame_count, speed_kmh, distance_driven, risks):
        if frame_count % 5 != 0:
            return
        worst = self.control.select_worst_risk(risks) if risks else None
        status = f"{distance_driven:.0f}/{self.actual_route_length:.0f}m | VRU: {len(risks)}"
        if worst:
            status += f" | {worst['risk_level'].name} | cel {worst['target_speed']:.0f} km/h"
        print(f"[{frame_count}] {speed_kmh:.1f} km/h | {status}")

    def cleanup(self):
        if self.camera is not None:
            try:
                if self.camera.is_alive:
                    self.camera.stop()
            except RuntimeError:
                pass
        try:
            if self.world is not None and self.original_settings is not None:
                self.world.apply_settings(self.original_settings)
        except RuntimeError:
            pass

    def run(self):
        try:
            self.connect_and_load()
            self.configure_world()
            self.spawn_vehicle_and_camera()
            self.spawn_environment()
            self.run_loop()
        except KeyboardInterrupt:
            print("[*] Przerwano przez użytkownika")
        except Exception as exc:
            print("[!] Błąd:", exc)
            raise
        finally:
            if not self.args.headless:
                try:
                    cv2.destroyAllWindows()
                except cv2.error:
                    pass
            try:
                self.cleanup()
                print("[*] Cleanup zakończony", flush=True)
            except Exception as exc:
                print(f"[*] Cleanup pominięty: {exc}", flush=True)
            os._exit(0)


def run(argv=None):
    from .cli import parse_args, resolve_model_path

    args = parse_args(argv)
    args.model = resolve_model_path(args.model)
    CarlaSimulation(args).run()
