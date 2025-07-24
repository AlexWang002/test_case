#ifndef _PLANET_UTC_MGR_H
#define _PLANET_UTC_MGR_H

#ifdef __cplusplus
extern "C"{
#else
#include <stdbool.h>
#endif
#include <stdint.h>
    enum manage_sync_state {
        MANAGE_SYNC_UNLOCKED,
        MANAGE_SYNC_LOCKED
    };

    enum init_clock_source {
        ICS_NONE,
        ICS_GNSS,
        ICS_RTC,
        ICS_DILINK,
        ICS_SHUTDOWNTIME,
        ICS_MAC,
        ICS_SOC1,
        ICS_DEFAULT
    };

    /**
    *   @details 管理面时间库初始化接口
    *   @return  小于0 on error, 0 on Successful initialization.
    */
    int32_t tmsync_planet_time_mgr_init();
    int32_t planet_time_mgr_init();

    /**
    *   @details 管理面时间库反初始化接口，用于释放库资源
    */
    void tmsync_planet_time_mgr_deinit();
    void planet_time_mgr_deinit();

    /**
    *   @details 获取管理面时间（utc时间）
    *   @param[inout] utcTime(返回的utc时间), 单位为ns
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_utc_time(uint64_t *utctime);
    int32_t get_utc_time(uint64_t *utctime);

    /**
    *   @details 通过utc时间戳设置管理面时间
    *   @param[in] utcTimeStamp（utc时间戳, ns）
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_set_utc_time(uint64_t utcTimeStamp);
    int32_t set_utc_time(uint64_t utcTimeStamp);

    /**
    *   @details 获取时区时间（utc时间+时区）
    *   @param[inout] localTime(返回的本地时间), 单位为ns
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_local_time(uint64_t *localtime);
    int32_t get_local_time(uint64_t *localtime);

    /**
    *   @details 获取当前同步状态
    *   @param[inout] state(返回的当前同步状态)
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_sync_state(enum manage_sync_state *state);
    int32_t get_sync_state(enum manage_sync_state *state);

    /**
    *   @details 获取管理面时间当前初始化状态
    *   @param[inout] init_state(返回的管理面时钟初始化状态)
    *   @param[inout] clocksource(返回的管理面时钟初始化源)
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_utc_clocksource_init_status(bool *init_state, enum init_clock_source *clocksource);
    int32_t get_utc_clocksource_init_status(bool *init_state, enum init_clock_source *clocksource);

//#if defined(dilinkSwitch) || defined(n3Switch) 
    /**
    *   @details 获取系统时间(CLOCK_REALTIME)初始化状态
    *   @param[inout] init_state(返回的系统时钟初始化状态)
    *   @param[inout] clocksource(返回的系统时钟初始化源)
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_systime_clocksource_init_status(bool *init_state, enum init_clock_source *clocksource);
    int32_t get_systime_clocksource_init_status(bool *init_state, enum init_clock_source *clocksource);
//#endif

    /**
    *   @details 获取当前同步偏差,如果返回0指示当前没有gnss信号
    *   @return  当前的同步偏差,单位为ns  -2 on error
    */
    int64_t tmsync_get_utc_sync_offset(void);
    int64_t get_utc_sync_offset(void);

    /**
    *   @details 获取当前同步偏差,如果返回0指示当前没有gnss信号
    *   @return  当前的同步偏差,单位为ms  -2 on error
    */
    int64_t tmsync_get_utc_sync_offset_ms(void);
    int64_t get_utc_sync_offset_ms(void);

    enum TMSYNC_GET_UTC_AND_PTP_TIME{
        MANAGE_SUCCESS = 0,
        INTERNAL_ERROR_GET_UTC_TIME_FAILED = 1<<0,
        PTP_TIME_UNDEFINED=1<<1,
        PTPCLOCKID_INVALID=1<<2,
        INTERNAL_ERROR_PTPSTATE_UNLOCKED=1<<3,
    };

    /**
    *   @details 获取系统时间（不含时区）
    *   @param[inout] systime(返回的sys时间), 单位为ns
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_sys_time(uint64_t *systime);
    int32_t get_sys_time(uint64_t *systime);

    /**
    *   @details 获取单调时间
    *   @param[inout] monotime(返回的mono时间), 单位为ns
    *   @return  小于0 on error, 0 on Successful
    */
    int32_t tmsync_get_mono_time(uint64_t *monotime);
    int32_t get_mono_time(uint64_t *monotime);

#ifdef __cplusplus
}
#endif
#endif
