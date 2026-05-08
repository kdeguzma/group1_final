class ZoneManager:
    def __init__(
        self,
        zones: list[dict[str, float | str]],
        base_station: dict[str, float],
    ) -> None:
        """Initialize with zone list and base station pose.

        Args:
            zones (list[dict[]]): The list of zones.
            base_station (dict[]]): The robot's pose at the base station.
        """
        # The list of zone dictionaries. Contains the keys id, x, y, and yaw.
        self._zones: list[dict[str, float | str]] = zones
        # The base position with keys x, y, and yaw.
        self._base_station: dict[str, float] = base_station
        # The index of the current zone in _zones.
        self._index: int = 0
        # The total number of survivors.
        self._survivor_count: int = 0

    def advance(self) -> None:
        """Move to the next zone."""
        # If there's remaining zones, increment the index by 1.
        if self.has_remaining():
            self._index += 1

    def base_pose(self) -> dict[str, float]:
        """Return the base station pose dict (x, y, yaw).

        Returns:
            dict[str, float]: A copy of the base station pose dict {x, y, yaw}.
        """
        # A copy is needed, otherwise BT nodes can corrupt the shared mission state.
        return self._base_station.copy()

    def current_index(self) -> int:
        """Return the index of the current zone.

        Returns:
            int: The index of the current zone.
        """
        return self._index

    def current_zone(self) -> dict[str, float | str]:
        """Return the current zone dict (id, x, y, yaw).

        Returns:
            dict[str, float | str]: A dictionary of the current zone {id, x, y, yaw}.
        """
        # Raise an error if there are no zones left.
        if not self.has_remaining():
            raise IndexError("No remaining zones")
        return self._zones[self._index]

    def has_remaining(self) -> bool:
        """True if there are unvisited zones.

        Return:
            bool: True if there are unvisited zones, False otherwise.
        """
        return self._index < len(self._zones)

    def next_survivor_id(self) -> str:
        """Return a unique ID like 'survivor_1', 'survivor_2', etc.

        Returns:
            str: A unique ID for each survivor.
        """
        self._survivor_count += 1
        return f"survivor_{self._survivor_count}"

    def total_zones(self) -> int:
        """Return the total number of zones in the patrol list.

        Returns:
            int: The total number of zones from the list length.
        """
        return len(self._zones)
