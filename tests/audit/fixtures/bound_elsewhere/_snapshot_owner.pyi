# Audit contract fixture (#232): binds Snapshot, which another module's header declares.
class Snapshot:
    joints: dict[str, float]
    def values(self, names: list[str]) -> list[float]: ...
