"""gh-178: margin arithmetic and constructors of CollisionMarginPairData / CollisionMarginData.

Expected values follow upstream collision_margin_data.cpp (tesseract 0.35.0).
"""

import pytest

from tesseract_robotics.tesseract_common import (
    CollisionMarginData,
    CollisionMarginPairData,
    CollisionMarginPairOverrideType,
)

# Margins in metres.
MARGIN_AB = 0.05
MARGIN_AC = 0.02
MARGIN_BD = 0.07
DEFAULT_MARGIN = 0.03
INCREMENT = 0.01
SCALE = 2.0
# m: a few float additions/multiplications of centimetre margins round well below this.
MARGIN_ATOL = 1e-12


def _approx(value):
    return pytest.approx(value, abs=MARGIN_ATOL)


def _pairs():
    return CollisionMarginPairData({("a", "b"): MARGIN_AB, ("a", "c"): MARGIN_AC})


def test_pair_data_from_dict():
    data = _pairs()
    assert data.getCollisionMargin("a", "b") == _approx(MARGIN_AB)
    assert data.getCollisionMargin("c", "a") == _approx(MARGIN_AC)


def test_pair_data_from_dict_orders_keys():
    """The constructor orders each key, as setCollisionMargin does."""
    data = CollisionMarginPairData({("b", "a"): MARGIN_AB})
    assert data.getCollisionMargin("a", "b") == _approx(MARGIN_AB)


def test_pair_data_increment_then_scale():
    data = _pairs()
    data.incrementMargins(INCREMENT)
    data.scaleMargins(SCALE)
    assert data.getCollisionMargin("a", "b") == _approx((MARGIN_AB + INCREMENT) * SCALE)
    assert data.getCollisionMargin("a", "c") == _approx((MARGIN_AC + INCREMENT) * SCALE)
    assert data.getMaxCollisionMargin() == _approx((MARGIN_AB + INCREMENT) * SCALE)


def test_pair_data_arithmetic_on_empty_is_noop():
    data = CollisionMarginPairData()
    data.incrementMargins(INCREMENT)
    data.scaleMargins(SCALE)
    assert data.empty()
    assert data.getMaxCollisionMargin() is None


def test_pair_data_max_margin():
    data = _pairs()
    assert data.getMaxCollisionMargin() == _approx(MARGIN_AB)
    assert data.getMaxCollisionMargin("a") == _approx(MARGIN_AB)
    assert data.getMaxCollisionMargin("c") == _approx(MARGIN_AC)


def test_pair_data_max_margin_unknown_object_is_none():
    """Upstream returns an empty optional: no pair margin involves the object."""
    assert _pairs().getMaxCollisionMargin("z") is None


def test_pair_data_apply_modify_keeps_untouched_pairs():
    data = _pairs()
    data.apply(
        CollisionMarginPairData({("b", "d"): MARGIN_BD}), CollisionMarginPairOverrideType.MODIFY
    )
    assert data.getCollisionMargin("a", "b") == _approx(MARGIN_AB)
    assert data.getCollisionMargin("b", "d") == _approx(MARGIN_BD)
    assert data.getMaxCollisionMargin() == _approx(MARGIN_BD)


def test_pair_data_apply_replace_drops_untouched_pairs():
    data = _pairs()
    data.apply(
        CollisionMarginPairData({("b", "d"): MARGIN_BD}), CollisionMarginPairOverrideType.REPLACE
    )
    assert data.getCollisionMargin("a", "b") is None
    assert data.getCollisionMargin("b", "d") == _approx(MARGIN_BD)


def test_pair_data_apply_none_changes_nothing():
    data = _pairs()
    data.apply(
        CollisionMarginPairData({("b", "d"): MARGIN_BD}), CollisionMarginPairOverrideType.NONE
    )
    assert data.getCollisionMargin("b", "d") is None
    assert data.getMaxCollisionMargin() == _approx(MARGIN_AB)


def test_margin_data_from_default_and_pairs():
    data = CollisionMarginData(DEFAULT_MARGIN, _pairs())
    assert data.getDefaultCollisionMargin() == _approx(DEFAULT_MARGIN)
    assert data.getCollisionMargin("a", "b") == _approx(MARGIN_AB)
    assert data.getCollisionMargin("x", "y") == _approx(DEFAULT_MARGIN)


def test_margin_data_from_pairs_only():
    data = CollisionMarginData(_pairs())
    assert data.getDefaultCollisionMargin() == 0.0
    assert data.getCollisionMargin("a", "c") == _approx(MARGIN_AC)


def test_margin_data_from_scalar_still_dispatches():
    assert CollisionMarginData(DEFAULT_MARGIN).getDefaultCollisionMargin() == _approx(
        DEFAULT_MARGIN
    )


def test_margin_data_object_max_margin():
    data = CollisionMarginData(DEFAULT_MARGIN, _pairs())
    # Pair margin above the default wins.
    assert data.getMaxCollisionMargin("a") == _approx(MARGIN_AB)
    # Pair margin below the default: the default wins.
    assert data.getMaxCollisionMargin("c") == _approx(DEFAULT_MARGIN)
    # No pair involves the object: the default applies.
    assert data.getMaxCollisionMargin("z") == _approx(DEFAULT_MARGIN)


def test_margin_data_increment_then_scale():
    data = CollisionMarginData(DEFAULT_MARGIN, _pairs())
    data.incrementMargins(INCREMENT)
    data.scaleMargins(SCALE)
    assert data.getDefaultCollisionMargin() == _approx((DEFAULT_MARGIN + INCREMENT) * SCALE)
    assert data.getCollisionMargin("a", "b") == _approx((MARGIN_AB + INCREMENT) * SCALE)


def test_margin_data_apply():
    data = CollisionMarginData(DEFAULT_MARGIN, _pairs())
    data.apply(
        CollisionMarginPairData({("b", "d"): MARGIN_BD}), CollisionMarginPairOverrideType.REPLACE
    )
    assert data.getCollisionMargin("a", "b") == _approx(DEFAULT_MARGIN)
    assert data.getCollisionMargin("b", "d") == _approx(MARGIN_BD)
