# Audit contract fixture: `check(const ns::Settings&)` with its parameter typed by the class
# `_settings_owner` binds under another name (tesseract_motion_planners' `checkPIQPSettings`).
import _settings_owner

def check(settings: _settings_owner.RenamedSettings) -> None: ...
