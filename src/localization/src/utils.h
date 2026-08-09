
#define RCL_CHECK(fn)                                                                             \
    {                                                                                             \
        rcl_ret_t ret = fn;                                                                       \
        if ((ret != RCL_RET_OK))                                                                  \
        {                                                                                         \
            (void)                                                                                \
                fprintf(stderr, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)ret); \
            return 1;                                                                             \
        }                                                                                         \
    }

#define RCL_TRY(fn)                          \
    {                                        \
        rcl_ret_t ret = fn;                  \
        if ((ret != RCL_RET_OK)) return ret; \
    }

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : ((val) < (min)) ? (min) : (val))
