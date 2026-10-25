

#include "music_task.h"

#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "music.h"
#include "music_calibrate.h"
#include "music_canon.h"
#include "music_castle_in_the_sky.h"
#include "music_error.h"
#include "music_referee.h"
#include "music_start.h"
#include "music_unity.h"
#include "music_you.h"
#include "music_see_you_again.h"
#include "music_hao_yun_lai.h"
#include "music_meow.h"
#include "music_gong_xi_fa_cai.h"
#include "music_deja_vu.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t music_high_water;
#endif

#define STEP_INIT 1
#define STEP_NORMAL 2

#define is_play_cali()                                                                 \
    (is_play == CALI_BEGIN || is_play == CALI_MIDDLE_TIME || is_play == CALI_GIMBAL || \
     is_play == CALI_IMU || is_play == CALI_CHASSIS)

// Enum Declarations
typedef enum {
    PLAY_NONE = 0,
    POWER_UP,
    CALI_BEGIN,
    CALI_MIDDLE_TIME,
    CALI_GIMBAL,
    CALI_IMU,
    CALI_CHASSIS,
} Playing_e;

// Variable Declarations
static uint8_t music_step = STEP_INIT;

static Playing_e is_play = POWER_UP;

static const CaliBuzzerState_e * cali_buzzer_state;

static void MusicInit(void);
static void MusicPlay(void);

void music_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(MUSIC_TASK_INIT_TIME);
    // 初始化音乐
    MusicInit();

    while (1) {
        MusicPlay();
        // 系统延时
        vTaskDelay(MUSIC_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        music_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void MusicInit(void)
{
    cali_buzzer_state = Subscribe(CALI_BUZZER_STATE_NAME);

    music_step = STEP_INIT;

    // MusicStartInit();
    // MusicRefereeInit();
    // MusicErrorInit();

    // MusicYouInit();
    // MusicUnityInit();
    // MusicCanonInit();
    // MusicCastleInTheSkyInit();
    // MusicSeeYouAgainInit();
    // MusicHaoYunLaiInit();
    // MusicMeowInit();
    // MusicGongXiFaCaiInit();
    // MusicDejaVuInit();
}

static void MusicPlay(void)
{
    // MusicYouPlay();
    // MusicUnityPlay();
    // MusicCanonPlay();
    // MusicCastleInTheSkyPlay();
    if (music_step == STEP_INIT) {
        if (MusicStartPlay()) {
            music_step = STEP_NORMAL;
            is_play = PLAY_NONE;
        }
    } else {
        // MusicRefereeDisconnectPlay();
        // MusicErrorPlay();
        // MusicHaoYunLaiPlay();
        // MusicMeowPlay();
        // MusicGongXiFaCaiPlay();
        // MusicDejaVuPlay();

        switch (*cali_buzzer_state) {
            case CALI_BUZZER_OFF: {
                if (is_play_cali()) {
                    cali_buzzer_off();
                    is_play = PLAY_NONE;
                }
            } break;
            case CALI_BUZZER_BEGIN: {
                cali_buzzer_begin();
                is_play = CALI_BEGIN;
            } break;
            case CALI_BUZZER_MIDDLE_TIME: {
                cali_buzzer_middle();
                is_play = CALI_MIDDLE_TIME;
            } break;
            case CALI_BUZZER_GIMBAL: {
                cali_buzzer_gimbal();
                is_play = CALI_GIMBAL;
            } break;
            case CALI_BUZZER_IMU: {
                cali_buzzer_imu();
                is_play = CALI_IMU;
            } break;
            case CALI_BUZZER_CHASSIS: {
                cali_buzzer_chassis();
                is_play = CALI_CHASSIS;
            } break;
            default:
                break;
        }
    }
}
