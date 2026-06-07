"""Generate SVG diagrams of task composer pipelines via TaskComposerNode.getDotgraph().

Renders the DOT graph of each pipeline documented in
docs/user-guide/task-composer.md to docs/assets/dotgraphs/<Pipeline>.svg, plus
one annotated example (FreespacePipeline_annotated.svg) from a real planning
run with a rigged contact-check failure, showing the green/red result overlay.

Node UUIDs and execution times differ per run, so regenerated SVGs never diff
clean — only regenerate when pipeline *structure* changes (tesseract_planning
version bump):

    pixi run docs-dotgraphs

Requires graphviz (`dot`) on PATH and the built tesseract_robotics package.
"""

import argparse
import re
import shutil
import subprocess
from os import environ
from pathlib import Path

import numpy as np
from loguru import logger

# Importing the package resolves TESSERACT_TASK_COMPOSER_CONFIG_FILE for both
# installed wheels (bundled data) and dev envs (conda share/).
from tesseract_robotics.planning import JointTarget, MotionProgram, Robot, TaskComposer
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles
from tesseract_robotics.tesseract_collision import (
    CollisionEvaluatorType,
    ContactManagerConfig,
)
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerPluginFactory,
)
from tesseract_robotics.tesseract_task_composer_planning import ContactCheckProfile

# Pipelines woven into docs/user-guide/task-composer.md — keep the two in sync.
DOCUMENTED_PIPELINES = (
    "FreespacePipeline",
    "CartesianPipeline",
    "TrajOptPipeline",
    "OMPLPipeline",
    "DescartesFPipeline",
    "RasterFtPipeline",
)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "docs" / "assets" / "dotgraphs"

# dump() emits debug-grade labels; these fields are noise in documentation
# graphs. The I/O key maps are near-identical on every node (profiles/
# environment identity mappings), so they go too. Kept: task name, and on
# annotated graphs Time + Status Msg.
DROPPED_LABEL_FIELDS = (
    "Type:",
    "UUID:",
    "Namespace:",
    "Abort Terminal:",
    "Conditional:",
    "Status Code:",
    "Inputs:",
    "Outputs:",
)

# DOT label value: quoted string with backslash escapes (handles \" inside).
# dump() emits node labels as `label="..."` but cluster labels as `label = "..."`.
_LABEL_RE = re.compile(r'label\s*=\s*"((?:[^"\\]|\\.)*)"')


def clean_dot_labels(dot_source: str) -> str:
    """Strip debug-only fields from `dump()` node labels for docs rendering.

    Graph structure (nodes, edges, shapes, colors) is untouched — only the
    label text shrinks. Segments are separated by DOT's ``\\n`` / ``\\l``
    escapes; a segment is dropped when it starts with one of
    `DROPPED_LABEL_FIELDS`.
    """

    def _clean(match: re.Match[str]) -> str:
        segments = re.split(r"\\[nl]", match.group(1))
        kept = [
            seg
            for seg in segments
            if seg.strip()
            and not seg.startswith("\t")  # I/O key lines are tab-indented
            and not seg.lstrip().startswith(DROPPED_LABEL_FIELDS)
        ]
        if not kept:
            return 'label=""'
        title, *rest = kept
        if not rest:
            return f'label="{title}"'
        body = "".join(f"{seg}\\l" for seg in rest)
        return f'label="{title}\\n{body}"'

    return _LABEL_RE.sub(_clean, dot_source)


def render_svg(dot_source: str, out_svg: Path) -> None:
    """Render DOT source to an SVG file via graphviz `dot`.

    Labels are passed through `clean_dot_labels` first — this script renders
    documentation graphs, not debug dumps.

    Args:
        dot_source: DOT graph text as returned by `TaskComposerNode.getDotgraph()`.
        out_svg: Destination SVG path; parent directories are created.

    Raises:
        subprocess.CalledProcessError: `dot` rejected the graph.
    """
    out_svg.parent.mkdir(parents=True, exist_ok=True)
    # bgcolor=white: mkdocs-material dark mode would otherwise render
    # graphviz's black-on-transparent text unreadably.
    subprocess.run(
        ["dot", "-Tsvg", "-Gbgcolor=white", "-o", str(out_svg)],
        input=clean_dot_labels(dot_source).encode(),
        check=True,
    )


def render_annotated_example(out_dir: Path) -> None:
    """Run FreespacePipeline with a rigged contact-check failure and render the
    result-annotated graph (successful nodes green + timing, failed nodes red).

    The 1.5 m contact margin makes every link pair count as colliding, so the
    planners succeed but DiscreteContactCheckTask aborts the run — one image
    shows both annotation colors. Mirrors the rigged fixture in
    tests/tesseract_task_composer/test_tesseract_task_composer.py.
    """
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    joint_names = robot.get_joint_names("manipulator")
    j_start = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    j_end = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(j_start, joint_names=joint_names)

    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(JointTarget(j_start))
        .move_to(JointTarget(j_end))
    )
    profiles = create_freespace_pipeline_profiles(planning_time=2.0)  # seconds (OMPL budget)

    forcing = ContactCheckProfile()
    forcing.contact_manager_config = ContactManagerConfig(1.5)
    forcing.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
    for profile_name in ("FREESPACE", "DEFAULT"):
        profiles.addProfile("DiscreteContactCheckTask", profile_name, forcing)

    composer = TaskComposer.from_config()
    composite = program.to_composite_instruction(joint_names, "tool0")
    assignCurrentStateAsSeed(composite, robot.env)

    task = composer.factory.createTaskComposerNode("FreespacePipeline")
    task_data = TaskComposerDataStorage()
    task_data.setData(
        task.getInputKeys().get("planning_input"),
        AnyPoly_wrap_CompositeInstruction(composite),
    )
    task_data.setData("environment", AnyPoly_wrap_EnvironmentConst(robot.env))
    task_data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    future = composer.executor.run(task, task_data)
    future.wait()
    if future.context.isSuccessful():
        raise RuntimeError(
            "rigged contact-check run unexpectedly succeeded — "
            "annotated example needs a failing node"
        )

    out_svg = out_dir / "FreespacePipeline_annotated.svg"
    render_svg(task.getDotgraph(future.context.task_infos), out_svg)
    logger.info("rendered annotated example -> {}", out_svg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help=f"SVG output directory (default: {DEFAULT_OUT_DIR})",
    )
    args = parser.parse_args()

    if shutil.which("dot") is None:
        raise FileNotFoundError(
            "graphviz `dot` not found on PATH — install it first "
            "(e.g. `brew install graphviz` / `apt install graphviz`)"
        )

    config_file = environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if not config_file:
        raise FileNotFoundError(
            "TESSERACT_TASK_COMPOSER_CONFIG_FILE not set — importing "
            "tesseract_robotics should have resolved it (broken install?)"
        )
    config = Path(config_file)
    if not config.is_file():
        raise FileNotFoundError(f"task composer config not found: {config}")

    factory = TaskComposerPluginFactory(FilesystemPath(str(config)), GeneralResourceLocator())

    for name in DOCUMENTED_PIPELINES:
        node = factory.createTaskComposerNode(name)
        if node is None:
            raise RuntimeError(f"pipeline {name!r} failed to load from {config}")
        out_svg = args.out_dir / f"{name}.svg"
        render_svg(node.getDotgraph(), out_svg)
        logger.info("rendered {} -> {}", name, out_svg)

    render_annotated_example(args.out_dir)


if __name__ == "__main__":
    main()
