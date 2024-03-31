# MISRA Compliance

The AWS IoT Over-the-air Update library files conform to the [MISRA C:2012](https://www.misra.org.uk/MISRAHome/MISRAC2012/tabid/196/Default.aspx)
guidelines, with some noted exceptions. Compliance is checked with Coverity static analysis.
Deviations from the MISRA standard are listed below:

### Ignored by [Coverity Configuration](tools/coverity/misra.config)
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Directive 4.5 | Advisory | Allow names that MISRA considers ambiguous (such as LogInfo and LogError). |
| Directive 4.8 | Advisory | Allow inclusion of unused types. Header files for a specific port, which are needed by all files, may define types that are not used by a specific file. |
| Directive 4.9 | Advisory | Allow inclusion of function like macros. The `assert` macro is used throughout the library for parameter validation, and logging is done using function like macros. |
| Rule 2.4 | Advisory | Allow unused tags. Some compilers warn if types are not tagged. |
| Rule 2.5 | Advisory | Allow unused macros. Library headers may define macros intended for the application's use, but are not used by a specific file. |
| Rule 3.1 | Required | Allow nested comments. C++ style `//` comments are used in example code within Doxygen documentation blocks. |
| Rule 11.5 | Advisory | Allow casts from `void *`. Fields may be passed as `void *`, requiring a cast to the correct data type before use. |
| Rule 21.1 | Required | Allow use of all macro names. For compatibility, libraries may define macros introduced in C99 for use with C90 compilers. |
| Rule 21.2 | Required | Allow use of all macro and identifier names. For compatibility, libraries may define macros introduced in C99 for use with C90 compilers. |

### Flagged by Coverity
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Rule 4.12 | Required | This rule prohibits the use of malloc and free from stdlib.h because of undefined behavior. We define a malloc interface in OTA and on POSIX it is implemented with malloc/free. The design for our OTA library is to let user choose whether they want to pass buffers to us or not. Dynamic allocation is used only when they do not provide these buffers. Further, we have unit tests with memory, and address sanitizer enabled to ensure we're not leaking or free memory that's not dynamically allocated. |
| Rule 14.3 | Required | This is a warning on `otaconfigAllowDowngrade` being a constant and used in controlling expression. This macro is one of the OTA library configuration and it's set to 0 when running the static analysis. But users can change it when they build their application. So this is a false positive. |
| Rule 21.5 | Required | This rule prohibits the use of signal.h because of undefined behavior. However, the warning is in OS porting implementation on POSIX, which has well defined behavior. We're using the timer functionality from POSIX so we deviate from this rule. |
| Rule 8.6  | Required | `OTA_JsonFileSignatureKey` is an extern variable declared but not defined in OTA library. This variable shall be defined in OTA platform abstraction layer implementation, which is found in other repositories. |

### Suppressed with Coverity Comments
| Deviation | Category | Justification |
| :-: | :-: | :-: |
| Rule 21.3 | Required | This is explained in rule 4.12 from section above. We define a malloc and free interface so that our OTA library can be ported to any OS. |
| Rule 21.8 | Required | One of the OTA platform abstraction layer interfaces `abort` is flagged for this violation. This is implemented by a platform abstraction layer and always called through the OTA PAL interface. |
| Rule 10.1 | Required | Use of POSIX specific macro `O_CREAT` and `O_RDWR` is flagged for this violation. We use these 2 macros with one POSIX API `mq_open` in the POSIX OS implementation. |
| Rule 2.2 | Required | This rule prohibits dead code for the string arrays `pOtaAgentStateStrings` and `pOtaEventStrings`. These are used only for logging which is disabled during static analysis. |