# Self-test fixture: two failing blocks

```python
raise RuntimeError("first failure")
```

```python
raise RuntimeError("second failure")
```

<!-- snippet: skip (fixture: must never run) -->
```python
raise RuntimeError("after the failure")
```
