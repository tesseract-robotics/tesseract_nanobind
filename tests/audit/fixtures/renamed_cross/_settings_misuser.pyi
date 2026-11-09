# Audit contract fixture: `check(const ns::Settings&)` with its parameter typed by a class
# that is not `ns::Settings`: the rename elsewhere does not cover it.
import _settings_owner

def check(settings: _settings_owner.Unrelated) -> None: ...
