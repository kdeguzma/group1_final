class ZoneManager:
    def __init__(
        self,
        zones: list[dict[float, tuple[str, float, float, float]]],
        base_station: dict[float, tuple[str, float, float, float]],
    ) -> None:
        """Initialize with zone list and base station pose.

        Args:
            zones (list[dict[float, tuple[str, float, float, float]]]): The list of zones.
            base_station (dict[float, tuple[str, float, float, float]]): The robot's pose at the base station.
        """
        self.zones = zones
        self.index = 0
        self.base = base_station
        self.survivor_count = 0

    def current_zone(self) -> dict[float, tuple[str, float, float, float]]:
        """Return the current zone dict (id, x, y, yaw).

        Returns:
            dict[float, tuple[str, float, float, float]]: A dictionary of the current zone.
        """
        return self.zones[self.index]

    def has_remaining(self) -> bool:
        """True if there are unvisited zones.

        Return:
            bool: True if there are unvisited zones, False otherwise.
        """
        return self.index < len(self.zones)

    def advance(self) -> None:
        """Move to the next zone."""
        self.index += 1

    def base_pose(self) -> dict[float, tuple[str, float, float, float]]:
        """Return the base station pose dict (x, y, yaw).

        Returns:
            dict[float, tuple[str, float, float, float]]: The base station pose dict.
        """
        return self.base

    def next_survivor_id(self) -> str:
        """Return a unique ID like 'survivor_1', 'survivor_2', etc.

        Returns:
            str: A unique ID for each survivor.
        """
        self.survivor_count += 1
        return f"survivor_{self.survivor_count}"
