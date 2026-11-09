# Audit contract fixture: a second module whose binding (the test's `bound` lookup) uses the
# Python name `RenamedSettings` for another C++ class, `other::Options`, so the name alone does
# not say which class a parameter names. Fixture class names stay unique, so it declares none.
def rival() -> None: ...
