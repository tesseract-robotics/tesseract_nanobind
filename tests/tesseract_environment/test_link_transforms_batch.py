"""Environment.link_transforms_batch: one call for many joint states, each row bitwise getState's."""

import numpy as np
import numpy.testing as nptest
import pytest

from .test_tesseract_environment import _IIWA_JOINTS, _fresh_env

_LINKS = ["tool0", "link_3", "base_link"]


@pytest.mark.parametrize("m", [0, 1, 257])
def test_each_row_is_get_state_bitwise(m):
    env = _fresh_env()
    values = np.random.default_rng(m).uniform(-2.0, 2.0, size=(m, len(_IIWA_JOINTS)))
    stack = env.link_transforms_batch(_IIWA_JOINTS, values, _LINKS)
    assert stack.shape == (m, len(_LINKS), 4, 4)
    assert stack.dtype == np.float64 and stack.flags.c_contiguous
    for row, q in enumerate(values):
        transforms = env.getState(_IIWA_JOINTS, q).link_transforms
        for column, link in enumerate(_LINKS):
            nptest.assert_array_equal(stack[row, column], transforms[link].matrix)


def test_leaves_the_environment_state_alone():
    env = _fresh_env()
    env.link_transforms_batch(_IIWA_JOINTS, np.full((3, len(_IIWA_JOINTS)), 0.4), _LINKS)
    nptest.assert_array_equal(env.getCurrentJointValues(), np.zeros(len(_IIWA_JOINTS)))


def test_reads_strided_input_by_value():
    env = _fresh_env()
    wide = np.random.default_rng(3).uniform(-1.0, 1.0, size=(5, 2 * len(_IIWA_JOINTS)))
    strided = wide[:, ::2]
    nptest.assert_array_equal(
        env.link_transforms_batch(_IIWA_JOINTS, strided, _LINKS),
        env.link_transforms_batch(_IIWA_JOINTS, np.array(strided), _LINKS),
    )


def test_refuses_unknown_joint_and_link_names_and_a_column_mismatch():
    env = _fresh_env()
    values = np.zeros((2, len(_IIWA_JOINTS)))
    with pytest.raises(ValueError, match="not_a_joint"):
        env.link_transforms_batch([*_IIWA_JOINTS[:-1], "not_a_joint"], values, _LINKS)
    with pytest.raises(ValueError, match="not_a_link"):
        env.link_transforms_batch(_IIWA_JOINTS, values, ["tool0", "not_a_link"])
    with pytest.raises(ValueError, match="joint_ids length"):
        env.link_transforms_batch(_IIWA_JOINTS, values[:, :-1], _LINKS)
