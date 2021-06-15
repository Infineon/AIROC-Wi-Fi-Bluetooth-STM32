// Trace definitions for script application. 
//
#ifdef __cplusplus
extern "C" {
#endif
    extern void BTU_trace_debug(const char* p_fmt, ...);
    extern void BTU_trace_error(const char* p_fmt, ...);
#if defined(MESH_HOST_MODE) && !defined(WIN32)
extern void mesh_trace_debug(const char* p_fmt, ...);
#endif

#ifdef __cplusplus
}
#endif

/**
 *  @ingroup     gentypes
 *
 *  @{
 */
/** Debug trace macro */
#if defined(MESH_HOST_MODE) && !defined(WIN32)
#define WICED_BT_TRACE          mesh_trace_debug
#else
#define WICED_BT_TRACE          BTU_trace_debug
#endif

/** Debug trace array macro */
#define WICED_BT_TRACE_ARRAY(ptr,len,string)     WICED_BT_TRACE("%s %A",string,ptr,len);
/** Error trace array macro */
#define WICED_BT_TRACE_CRIT     BTU_trace_error

