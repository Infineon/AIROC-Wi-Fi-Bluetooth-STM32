
# MISRA Compliance

The jobs library files conform to the [MISRA C:2012](https://www.misra.org.uk/MISRAHome/MISRAC2012/tabid/196/Default.aspx)
guidelines, with some noted exceptions. Compliance is checked with Coverity static analysis.
Deviations from the MISRA standard are listed below:

### Ignored by [Coverity Configuration](tools/coverity/misra.config)
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Directive 4.9 | Advisory | Allow inclusion of function like macros. |
| Rule 3.1 | Required | Allow nested comments. C++ style `//` comments are used in example code within Doxygen documentation blocks. |
| Rule 20.12 | Required | Allow use of `assert()`, which uses a parameter in both expanded and raw forms. |

### Flagged by Coverity
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Rule 2.5 | Advisory | A macro is not used by the library; however, it exists to be used by an application. |
| Rule 8.7 | Advisory | API functions are not used by the library; however, they must be externally visible in order to be used by an application. |

### Suppressed with Coverity Comments
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Rule 10.1 | Required | A variable of an enum type is used to iterate over contiguous values. |
