## Logging Best Practices in Embedded C
**Debug Logging in Algorithms:** When adding debug logs to complex algorithms like plateau detection, log function entry with key parameters, buffer state changes, calibration triggers, and all decision branches (including calculations like delta/epsilon and counting logic). Use appropriate format specifiers: %.3f for general floats, %.4f for very small values like noise_std.

**SEARCH/REPLACE Patterns:** For large code modifications, construct SEARCH blocks with exact whitespace and indentation match. Include sufficient surrounding context for unique matching. Break into smaller changes if whitespace alignment becomes challenging to prevent tool failures.

## Plateau Detection Debugging
**Algorithm State Visibility:** In flow rate plateau detection, debug logs should cover circular buffer operations, first-measurement handling, noise calibration completion, delta comparisons, and confirm count accumulation to trace when plateau is detected.
