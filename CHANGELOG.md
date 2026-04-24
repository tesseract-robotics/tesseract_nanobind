# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased] — targeting 0.35.0.0

### Added

- `CompositeInstruction.push_back` now accepts a `CompositeInstruction` in addition to a plain `Instruction`, enabling nested composites (e.g. raster programs) to be built from Python the same way they are in C++ ([#51], [6d17314]).
  - contributed by [@Joelkang]

[Unreleased]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.2...HEAD
[@Joelkang]: https://github.com/Joelkang
[#51]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/51
[6d17314]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/6d17314a0b5aa1db45ffb9d9b18a8b3e048d3d06
