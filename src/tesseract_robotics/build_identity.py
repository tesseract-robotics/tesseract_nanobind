"""The upstream sources this wheel's C++ stack was built from.

0.36.0.0rc1 builds against `nanobind-0.36` integration branches on the jf--- forks: upstream
`master` plus the open upstream pull requests listed here, so a release does not wait on upstream
review. `dependencies.rosinstall` pins the same SHAs; `tests/test_build_identity.py` checks that
the two agree.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class UpstreamSource:
    """One upstream repository as built: the integration branch head and what it carries."""

    repository: str
    fork: str
    sha: str
    base_master_sha: str  # upstream master the branch was cut from
    carried_pull_requests: tuple[int, ...]  # open upstream PRs merged into the branch


UPSTREAM_SOURCES: tuple[UpstreamSource, ...] = (
    UpstreamSource(
        repository="tesseract-robotics/tesseract",
        fork="jf---/tesseract",
        sha="4cf644d704bd8961c7cdaf193f72488fa501f5c0",
        base_master_sha="a9c22066",
        carried_pull_requests=(1396,),
    ),
    UpstreamSource(
        repository="tesseract-robotics/tesseract_planning",
        fork="jf---/tesseract_planning",
        sha="4e27d94b160f757e064dbad2a5277657cd33bbf8",
        base_master_sha="a6483823",
        carried_pull_requests=(789, 792, 794),
    ),
    UpstreamSource(
        repository="tesseract-robotics/trajopt",
        fork="jf---/trajopt",
        sha="98b85d559e9fc7cb60a4dbfeb6d3baa177c94df5",
        base_master_sha="963baf7d",
        carried_pull_requests=(628, 629),
    ),
)
