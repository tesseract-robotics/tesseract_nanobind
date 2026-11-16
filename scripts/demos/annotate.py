"""A missing demo file is a warning, not a failed docs build (#446). Standard library only."""


def warn(message: str) -> None:
    """Print `message` as a GitHub Actions warning annotation (plain text elsewhere)."""
    print(f"::warning::{message}", flush=True)
