"""Profile helpers for the planning pipelines, split by planner.

- `trajopt`: TrajOpt (SCO) and TrajOptIfopt (SQP/OSQP) profiles
- `ompl`: OMPL move profiles and planner configurators
- `descartes`: Descartes move and solver profiles
- `time_param`: TOTG and ISP constructors
- `pipelines`: profiles for pipelines that run several planners, and per-pipeline defaults
- `names`: profile names and namespaces
"""

# The imports below are this package's public names: no `__all__`; each `X as X` is an explicit
# re-export (PEP 484), so type checkers and linters treat it as public.

from tesseract_robotics.planning.profiles.descartes import (
    create_descartes_default_profiles as create_descartes_default_profiles,
)
from tesseract_robotics.planning.profiles.names import (
    DESCARTES_DEFAULT_NAMESPACE as DESCARTES_DEFAULT_NAMESPACE,
)
from tesseract_robotics.planning.profiles.names import (
    DESCARTES_PROFILE_NAMES as DESCARTES_PROFILE_NAMES,
)
from tesseract_robotics.planning.profiles.names import (
    OMPL_DEFAULT_NAMESPACE as OMPL_DEFAULT_NAMESPACE,
)
from tesseract_robotics.planning.profiles.names import (
    OMPL_PROFILE_NAMES as OMPL_PROFILE_NAMES,
)
from tesseract_robotics.planning.profiles.names import (
    STANDARD_PROFILE_NAMES as STANDARD_PROFILE_NAMES,
)
from tesseract_robotics.planning.profiles.names import (
    TRAJOPT_DEFAULT_NAMESPACE as TRAJOPT_DEFAULT_NAMESPACE,
)
from tesseract_robotics.planning.profiles.names import (
    TRAJOPT_IFOPT_DEFAULT_NAMESPACE as TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
)
from tesseract_robotics.planning.profiles.names import (
    UPRIGHT_PROFILE_NAMES as UPRIGHT_PROFILE_NAMES,
)
from tesseract_robotics.planning.profiles.ompl import (
    OMPL_CONTACT_MARGIN_M as OMPL_CONTACT_MARGIN_M,
)
from tesseract_robotics.planning.profiles.ompl import (
    UnknownConfiguratorFieldError as UnknownConfiguratorFieldError,
)
from tesseract_robotics.planning.profiles.ompl import (
    UnknownPlannerTypeError as UnknownPlannerTypeError,
)
from tesseract_robotics.planning.profiles.ompl import (
    create_ompl_default_profiles as create_ompl_default_profiles,
)
from tesseract_robotics.planning.profiles.ompl import (
    create_ompl_planner_configurators as create_ompl_planner_configurators,
)
from tesseract_robotics.planning.profiles.pipelines import (
    create_cartesian_pipeline_profiles as create_cartesian_pipeline_profiles,
)
from tesseract_robotics.planning.profiles.pipelines import (
    create_freespace_ifopt_pipeline_profiles as create_freespace_ifopt_pipeline_profiles,
)
from tesseract_robotics.planning.profiles.pipelines import (
    create_freespace_pipeline_profiles as create_freespace_pipeline_profiles,
)
from tesseract_robotics.planning.profiles.pipelines import (
    create_raster_pipeline_profiles as create_raster_pipeline_profiles,
)
from tesseract_robotics.planning.profiles.pipelines import (
    default_profiles_for as default_profiles_for,
)
from tesseract_robotics.planning.profiles.time_param import (
    create_iterative_spline_parameterization as create_iterative_spline_parameterization,
)
from tesseract_robotics.planning.profiles.time_param import (
    create_time_optimal_parameterization as create_time_optimal_parameterization,
)
from tesseract_robotics.planning.profiles.trajopt import (
    create_trajopt_default_profiles as create_trajopt_default_profiles,
)
from tesseract_robotics.planning.profiles.trajopt import (
    create_trajopt_ifopt_default_profiles as create_trajopt_ifopt_default_profiles,
)
from tesseract_robotics.planning.profiles.trajopt import (
    create_trajopt_profile_pair as create_trajopt_profile_pair,
)
from tesseract_robotics.planning.profiles.trajopt import (
    create_trajopt_upright_profile_pair as create_trajopt_upright_profile_pair,
)
from tesseract_robotics.planning.profiles.trajopt import (
    create_trajopt_upright_profiles as create_trajopt_upright_profiles,
)
