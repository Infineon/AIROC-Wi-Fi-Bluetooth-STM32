# MISRA Compliance

The backoffAlgorithm library files conform to the [MISRA C:2012](https://www.misra.org.uk/MISRAHome/MISRAC2012/tabid/196/Default.aspx)
guidelines, with some noted exceptions. Compliance is checked with Coverity static analysis.
Deviations from the MISRA standard are listed below:

### Ignored by [Coverity Configuration](tools/coverity/misra.config)
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Directive 4.9 | Advisory | Allow inclusion of function like macros. |
| Rule 3.1 | Required | Allow nested comments. C++ style `//` comments are used in example code within Doxygen documentation blocks. |
| Rule 2.4 | Advisory | Allow unused tags. Some compilers warn if types are not tagged. |

### Flagged by Coverity
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Rule 8.7 | Advisory | API functions are not used by the library; however, they must be externally visible in order to be used by an application. |

