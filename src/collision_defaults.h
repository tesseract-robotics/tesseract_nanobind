#pragma once

// Upstream collision defaults the bindings restate as keyword defaults, read from upstream
// objects so they cannot drift from the headers.

#include <tesseract/collision/types.h>

// [m] CollisionCheckConfig's default longest_valid_segment_length (types.h), from a
// default-constructed config. Shared by collision.CollisionCheckConfig and
// trajopt_ifopt.TrajOptCollisionConfig.
inline const double LONGEST_VALID_SEGMENT_LENGTH_DEFAULT =
    tesseract::collision::CollisionCheckConfig().longest_valid_segment_length;
