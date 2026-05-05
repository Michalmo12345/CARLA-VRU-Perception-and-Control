import carla
import random
import time
import queue
import numpy as np
import cv2

HOST = "localhost"
PORT = 2000
MAP_NAME = "Town02"
NUM_PEDESTRIANS = 500

IMG_W = 1280
IMG_H = 720


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

    actors = []
    image_queue = queue.Queue()

    try:
        vehicle_bp = blueprint_library.find("vehicle.tesla.model3")

        fixed_transform = carla.Transform(
            carla.Location(x=-7.53, y=251.36, z=0.25),
            carla.Rotation(pitch=0.33, yaw=90.0, roll=0.0)
        )

        vehicle = world.try_spawn_actor(vehicle_bp, fixed_transform)

        if vehicle is None:
            raise RuntimeError("Nie udało się zespawnować pojazdu")

        actors.append(vehicle)

        print("Vehicle spawned")

        spectator = world.get_spectator()

        spectator.set_transform(carla.Transform(
            carla.Location(x=-6.38, y=238.73, z=7.32),
            carla.Rotation(pitch=-21.24, yaw=87.22, roll=0.0)
        ))

        print("Spectator set (fixed position)")

        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(IMG_W))
        camera_bp.set_attribute("image_size_y", str(IMG_H))
        camera_bp.set_attribute("fov", "90")

        camera_transform = carla.Transform(
            carla.Location(x=1.6, z=1.7)
        )

        camera = world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=vehicle
        )

        actors.append(camera)

        camera.listen(lambda image: image_queue.put(image))

        print("Camera attached")

        walker_bps = blueprint_library.filter("walker.pedestrian.*")
        walkers = []

        for i in range(NUM_PEDESTRIANS):
            loc = world.get_random_location_from_navigation()
            if loc is None:
                continue

            walker = world.try_spawn_actor(
                random.choice(walker_bps),
                carla.Transform(loc)
            )

            if walker:
                walker.set_simulate_physics(False)
                walkers.append(walker)
                actors.append(walker)

                world.debug.draw_point(
                    loc,
                    size=0.25,
                    color=carla.Color(255, 0, 0),
                    life_time=20.0
                )

        print(f"Spawned pedestrians: {len(walkers)}")

        cv2.namedWindow("CARLA Camera", cv2.WINDOW_NORMAL)

        print("START LOOP")

        while True:
            world.tick()

            image = image_queue.get(timeout=2.0)

            frame = np.frombuffer(image.raw_data, dtype=np.uint8)
            frame = frame.reshape((image.height, image.width, 4))
            frame = frame[:, :, :3]

            cv2.imshow("CARLA Camera", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping...")

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