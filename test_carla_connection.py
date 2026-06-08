"""Szybki test połączenia z serwerem CARLA."""

import argparse
import sys

try:
    import carla
except ImportError:
    print("Brak modułu carla. Uruchom: pip install carla==0.9.16")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()

    print(f"Łączenie z CARLA {args.host}:{args.port} ...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        version = client.get_server_version()
        world = client.get_world()
        print(f"OK — CARLA {version}, mapa: {world.get_map().name}")
    except RuntimeError as exc:
        print(f"BŁĄD: {exc}")
        print()
        print("Jeśli CARLA działa na Windowsie, a testujesz z WSL2:")
        print("  1. Włącz mirrored networking w C:\\Users\\<user>\\.wslconfig:")
        print("       [wsl2]")
        print("       networkingMode=mirrored")
        print("  2. Uruchom: wsl --shutdown  (w PowerShell)")
        print("  3. Spróbuj ponownie z --host 127.0.0.1")
        sys.exit(1)


if __name__ == "__main__":
    main()
