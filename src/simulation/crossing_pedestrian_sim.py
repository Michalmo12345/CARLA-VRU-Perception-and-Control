import carla
import random
import time
import queue
import numpy as np
import cv2

HOST = "localhost"
PORT = 2000
MAP_NAME = "Town02"

NUM_PEDESTRIANS = 300

IMG_W = 1280
IMG_H = 720

VEHICLE_TRANSFORM = carla.Transform(
    carla.Location(x=-7.53, y=251.36, z=0.25),
    carla.Rotation(pitch=0.33, yaw=90.0)
)

SPECTATOR_TRANSFORM = carla.Transform(
    carla.Location(x=-6.38, y=238.73, z=7.32),
    carla.Rotation(pitch=-21.24, yaw=87.22)
)


def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)

    world = client.load_world(MAP_NAME)
    blueprint_library = world.get_blueprint_library()

    print("World loaded")

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    actors = []
    image_queue = queue.Queue()

    try:
        vehicle_bp = blueprint_library.find("vehicle.tesla.model3")

        vehicle = world.spawn_actor(vehicle_bp, VEHICLE_TRANSFORM)
        actors.append(vehicle)

        vehicle.set_autopilot(True, traffic_manager.get_port())
        traffic_manager.global_percentage_speed_difference(-20.0)

        print("Vehicle autopilot ON")


        spectator = world.get_spectator()
        spectator.set_transform(SPECTATOR_TRANSFORM)

        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(IMG_W))
        camera_bp.set_attribute("image_size_y", str(IMG_H))

        camera = world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            attach_to=vehicle
        )

        actors.append(camera)
        camera.listen(lambda image: image_queue.put(image))

        print("Camera set")

        walker_bps = blueprint_library.filter("walker.pedestrian.*")

        spawn_points = []
        for _ in range(NUM_PEDESTRIANS):
            loc = world.get_random_location_from_navigation()
            if loc:
                spawn_points.append(carla.Transform(loc))

        walkers = []
        controllers = []

        for spawn_point in spawn_points:
            walker_bp = random.choice(walker_bps)

            walker = world.try_spawn_actor(walker_bp, spawn_point)

            if walker is not None:
                walkers.append(walker)
                actors.append(walker)

        world.tick()

        controller_bp = blueprint_library.find("controller.ai.walker")

        for walker in walkers:
            controller = world.spawn_actor(controller_bp, carla.Transform(), walker)
            controllers.append(controller)
            actors.append(controller)


        world.tick()

        for controller in controllers:
            controller.start()
            controller.go_to_location(world.get_random_location_from_navigation())
            controller.set_max_speed(1.2)

        print(f"Spawned pedestrians: {len(walkers)}")

        last_event_time = time.time()
        event_delay = random.uniform(1, 3)

        cv2.namedWindow("CARLA Camera", cv2.WINDOW_NORMAL)

        print("START LOOP")

        while True:
            world.tick()

            vehicle_tf = vehicle.get_transform()
            forward = vehicle_tf.get_forward_vector()
            right = carla.Vector3D(x=forward.y, y=-forward.x, z=0)


            if time.time() - last_event_time > event_delay:

                crossing_distance = random.uniform(10, 18)
                base = vehicle_tf.location + forward * crossing_distance

                start = base - right * random.uniform(2, 5)
                end = base + right * random.uniform(2, 5)

                walker = random.choice(walkers)
                controller = controllers[walkers.index(walker)]

                walker.set_transform(carla.Transform(start))

                controller.go_to_location(end)
                controller.set_max_speed(3.5)

                print("Pedestrian crossing!")

                last_event_time = time.time()
                event_delay = random.uniform(1, 3)

            image = image_queue.get(timeout=2.0)
            frame = np.frombuffer(image.raw_data, dtype=np.uint8)
            frame = frame.reshape((image.height, image.width, 4))
            frame = frame[:, :, :3]

            cv2.imshow("CARLA Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print("ERROR:", e)

    finally:
        cv2.destroyAllWindows()

        for actor in reversed(actors):
            try:
                actor.destroy()
            except:
                pass

        world.apply_settings(original_settings)
        print("Cleanup done")


if __name__ == "__main__":
    main()