"""Presety pogody dla symulacji.

Pozwalają empirycznie sprawdzić odporność detekcji (YOLO) na warunki inne niż
jasny dzień — deszcz, mgłę, noc. Mapa nazwa→``carla.WeatherParameters`` jest
jawna, więc łatwo dodać kolejny wariant.
"""

import carla

WEATHER_CHOICES = ("default", "clear", "cloudy", "wet", "rain", "fog", "night", "rainnight")


def _fog() -> carla.WeatherParameters:
    return carla.WeatherParameters(
        cloudiness=90.0,
        precipitation=0.0,
        fog_density=100.0,
        fog_distance=8.0,
        sun_altitude_angle=45.0,
    )


def _presets():
    return {
        "clear": carla.WeatherParameters.ClearNoon,
        "cloudy": carla.WeatherParameters.CloudyNoon,
        "wet": carla.WeatherParameters.WetNoon,
        "rain": carla.WeatherParameters.HardRainNoon,
        "fog": _fog(),
        "night": carla.WeatherParameters.ClearNight,
        "rainnight": carla.WeatherParameters.HardRainNight,
    }


def apply_weather(world, name: str):
    """Ustawia pogodę i zwraca użytą nazwę (lub None, gdy bez zmian)."""
    if not name or name == "default":
        return None
    preset = _presets().get(name)
    if preset is None:
        return None
    world.set_weather(preset)
    return name
