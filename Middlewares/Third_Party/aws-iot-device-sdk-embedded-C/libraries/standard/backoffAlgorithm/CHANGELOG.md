# Changelog for backoffAlgorithm Library

## v1.0.1 (February 2020)

### Changes

- [#24](https://github.com/FreeRTOS/backoffAlgorithm/pull/24) Fix MISRA 10.4 and 10.7 rule violations, and add documentation of MISRA compliance.
- [#18](https://github.com/FreeRTOS/backoffAlgorithm/pull/18), [#19](https://github.com/FreeRTOS/backoffAlgorithm/pull/19), and [#20](https://github.com/FreeRTOS/backoffAlgorithm/pull/20) Documentation fixes.

## v1.0.0 (December 2020)

This is the first release of the backoffAlgorithm library in this repository.

The backoffAlgorithm library is a utility library to calculate backoff period using an exponential backoff with jitter algorithm for retrying network operations (like failed network connection with server).
This library uses the "Full Jitter" strategy for the exponential backoff with jitter algorithm.
More information about the algorithm can be seen in the [Exponential Backoff and Jitter](https://aws.amazon.com/blogs/architecture/exponential-backoff-and-jitter/) AWS blog.