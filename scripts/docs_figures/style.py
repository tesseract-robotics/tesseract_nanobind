"""One restrained style for every docs figure, rendered once per docs theme.

Each figure is drawn twice, for the light (`default`) and dark (`slate`) palette of the docs
theme, into `<name>.svg` and `<name>-dark.svg`. A page embeds both with the theme's
`#only-light` / `#only-dark` URL fragments, so ink, grid and mesh shading always sit on the
background they were chosen for. Backgrounds are transparent.

Data colours are mid-tones that keep at least 3:1 contrast against both page backgrounds;
the categorical order is fixed so a planner or joint has the same colour on every page.
"""

from __future__ import annotations

from collections.abc import Callable, Generator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Final

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.typing import RcKeyType  # noqa: E402

REPO_ROOT: Final[Path] = Path(__file__).resolve().parents[2]
FIGURES_DIR: Final[Path] = REPO_ROOT / "docs" / "assets" / "figures"

# in: the docs content column is about 690 px wide at 96 px/in; figures render at that width
# so 1 pt of type is 1.33 px on the page, the same as the body text's scale.
COLUMN_WIDTH_IN: Final[float] = 7.2
# pt: body text in the docs is 16 px (12 pt); figure type sits one step below, like a caption.
BASE_FONT_PT: Final[float] = 9.5
# dpi of the raster previews and of rasterised mesh layers inside the SVGs: 2x a 96 dpi screen.
RASTER_DPI: Final[int] = 192


@dataclass(frozen=True)
class Theme:
    """The ink and surface colours of one docs palette."""

    name: str
    suffix: str
    ink: str  # titles, labels, primary strokes
    muted: str  # tick labels, secondary annotation
    grid: str  # grid lines and spines
    surface: str  # neutral fill (robot meshes, obstacle bodies)
    page: str  # the docs page background (previews only; the SVGs stay transparent)


LIGHT: Final[Theme] = Theme(
    name="light",
    suffix="",
    ink="#1f2328",
    muted="#59636e",
    grid="#d1d9e0",
    surface="#b8bfc8",
    page="#ffffff",
)
DARK: Final[Theme] = Theme(
    name="dark",
    suffix="-dark",
    ink="#e6edf3",
    muted="#9aa4af",
    grid="#3d444d",
    surface="#6e7781",
    page="#1e2129",
)
THEMES: Final[tuple[Theme, Theme]] = (LIGHT, DARK)

# Categorical data colours, fixed order. Mid-tones: each keeps >= 3:1 contrast against
# both #ffffff and #1e2129.
BLUE: Final[str] = "#4a7fd0"
ORANGE: Final[str] = "#cc7722"
GREEN: Final[str] = "#2f9e6e"
PURPLE: Final[str] = "#8b6fd6"
RED: Final[str] = "#d1495b"
TEAL: Final[str] = "#1f9eb5"
CATEGORICAL: Final[tuple[str, ...]] = (BLUE, ORANGE, GREEN, PURPLE, RED, TEAL)

# Frame axes: the robotics convention x red, y green, z blue, at mid-tone.
AXIS_X: Final[str] = "#d64545"
AXIS_Y: Final[str] = "#3a9d4a"
AXIS_Z: Final[str] = "#3f73d6"
AXIS_COLORS: Final[tuple[str, str, str]] = (AXIS_X, AXIS_Y, AXIS_Z)


def _rc(theme: Theme) -> dict[RcKeyType, Any]:
    return {
        "font.family": "DejaVu Sans",
        "font.size": BASE_FONT_PT,
        "axes.titlesize": BASE_FONT_PT + 0.5,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.labelsize": BASE_FONT_PT,
        "axes.edgecolor": theme.grid,
        "axes.labelcolor": theme.ink,
        "axes.titlecolor": theme.ink,
        "axes.facecolor": "none",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": theme.grid,
        "grid.linewidth": 0.6,
        "xtick.color": theme.grid,
        "ytick.color": theme.grid,
        "xtick.labelcolor": theme.muted,
        "ytick.labelcolor": theme.muted,
        "xtick.labelsize": BASE_FONT_PT - 1,
        "ytick.labelsize": BASE_FONT_PT - 1,
        "legend.frameon": False,
        "legend.fontsize": BASE_FONT_PT - 0.5,
        "legend.labelcolor": theme.ink,
        "text.color": theme.ink,
        "figure.facecolor": "none",
        "savefig.facecolor": "none",
        "savefig.transparent": True,
        "lines.linewidth": 1.6,
        "lines.solid_capstyle": "round",
        # Deterministic SVGs: fixed id salt, glyphs as paths (no dependence on page fonts).
        "svg.hashsalt": "tesseract-docs-figures",
        "svg.fonttype": "path",
    }


@contextmanager
def themed(theme: Theme) -> Generator[None]:
    """Matplotlib rc settings of `theme` for the duration of the block."""
    with plt.rc_context(_rc(theme)):
        yield


def render(
    name: str,
    draw: Callable[[Theme], Figure],
    *,
    out_dir: Path = FIGURES_DIR,
    preview_dir: Path | None = None,
) -> list[Path]:
    """Draw `name` once per theme and write the SVGs (and optional PNG previews).

    Args:
        name: File stem, e.g. `"collision_distance"`.
        draw: Builds the figure for a theme; called inside that theme's rc context.
        out_dir: Directory of the SVGs.
        preview_dir: When given, also writes `<name><suffix>.png` at `RASTER_DPI` on the
            page colour, for checking a figure as the reader sees it.

    Returns:
        The written SVG paths.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for theme in THEMES:
        with themed(theme):
            fig = draw(theme)
            svg = out_dir / f"{name}{theme.suffix}.svg"
            fig.savefig(svg, format="svg", metadata={"Date": None, "Creator": None})
            written.append(svg)
            if preview_dir is not None:
                preview_dir.mkdir(parents=True, exist_ok=True)
                fig.savefig(
                    preview_dir / f"{name}{theme.suffix}.png",
                    dpi=RASTER_DPI,
                    transparent=False,
                    facecolor=theme.page,
                )
        plt.close(fig)
    return written
