#include "music_hao_yun_lai.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
#define D_do 294
#define D_re 330
#define D_mi 370
#define D_fa 392
#define D_so 440
#define D_la 494
#define D_si 554
// clang-format on

#define ONE_FOURTH 250
#define ONE_FOURTH_HALF 125
#define ONE_FOURTH_HALF_HALF 63

#define NOTE_NUM 50
static Note Notes[NOTE_NUM];  // Array of notes

static uint32_t last_note_id = 0;  // Index of the last note
static uint32_t write_id = 1;      // Index of the note to be written
static uint32_t play_id = 1;       // Index of the note to be played

static uint32_t start_time = 0;  // Start time of the music
static uint32_t now = 0;

/*-------------------- Private functions --------------------*/
static void WriteNote(int note, float Long)
{
    Notes[write_id].note = note;
    Notes[write_id].Long = Long;
    Notes[write_id].end = Notes[write_id - 1].end + Long;
    write_id++;
}

static void SleepNote(float Long) { WriteNote(0, Long); }

/*-------------------- User functions --------------------*/
void MusicHaoYunLaiPlay(void)
{
    now = HAL_GetTick();
    if (now - start_time >= Notes[play_id].end) {
        play_id++;
        if (play_id > last_note_id) {
            play_id = 1;
            start_time = now;
        }

        buzzer_note(Notes[play_id].note, 0.1);
    }
}

void MusicHaoYunLaiInit(void)
{
    SleepNote(500);

    // do re mi fa so la si
    // 1  2  3  4  5  6  7

    // 好运来祝你
    // 6 3`_ 2`_ 2` 1`_ 6_
    WriteNote(D_la, ONE_FOURTH);           // 6
    WriteNote(D_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WriteNote(D_re * 2, ONE_FOURTH);       // 2
    WriteNote(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WriteNote(D_la, ONE_FOURTH_HALF);      // 6_

    // 好运来，
    // 5 1`_ 2`_ 6 -
    WriteNote(D_so, ONE_FOURTH);           // 5
    WriteNote(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WriteNote(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WriteNote(D_la, ONE_FOURTH * 2);       // 6

    // 好运带来了
    // 6 2` 1` 6_ 5_
    WriteNote(D_la, ONE_FOURTH);       // 6
    WriteNote(D_re * 2, ONE_FOURTH);   // 2`
    WriteNote(D_do * 2, ONE_FOURTH);   // 1`
    WriteNote(D_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(D_so, ONE_FOURTH_HALF);  // 5_

    // 喜和爱,
    // 2 5_ 6_ 3 -
    WriteNote(D_re, ONE_FOURTH);       // 2
    WriteNote(D_so, ONE_FOURTH_HALF);  // 5_
    WriteNote(D_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(D_mi, ONE_FOURTH * 2);   // 3

    // 好运来我们
    // 3 6_ 5_ 6 6_ 5_
    WriteNote(D_mi, ONE_FOURTH);       // 3
    WriteNote(D_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(D_so, ONE_FOURTH_HALF);  // 5_
    WriteNote(D_la, ONE_FOURTH);       // 6
    WriteNote(D_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(D_so, ONE_FOURTH_HALF);  // 5_

    // 好运来，
    // 6 2`_ 1`_ 2` -
    WriteNote(D_la, ONE_FOURTH);           // 6
    WriteNote(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WriteNote(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WriteNote(D_re * 2, ONE_FOURTH * 2);   // 2`

    //迎着好运兴旺发达
    // 1`-_ 1`__ 1`_ 2`_ 3`_ 3`_ 2`_ 1`_
    WriteNote(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 1`-_
    WriteNote(D_do * 2, ONE_FOURTH_HALF_HALF);                    // 1`__
    WriteNote(D_do * 2, ONE_FOURTH_HALF);                         // 1`_
    WriteNote(D_re * 2, ONE_FOURTH_HALF);                         // 2`_
    WriteNote(D_mi * 2, ONE_FOURTH_HALF);                         // 3`_
    WriteNote(D_mi * 2, ONE_FOURTH_HALF);                         // 3`_
    WriteNote(D_re * 2, ONE_FOURTH_HALF);                         // 2`_
    WriteNote(D_do * 2, ONE_FOURTH_HALF);                         // 1`_

    // 通四海。
    // 5 1`_ 6_ 6 -
    WriteNote(D_so, ONE_FOURTH);           // 5
    WriteNote(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WriteNote(D_la, ONE_FOURTH_HALF);      // 6_
    WriteNote(D_la, ONE_FOURTH * 2);           // 6

    // 尾奏

    last_note_id = write_id - 1;
    write_id = 1;
}
/*------------------------------ End of File ------------------------------*/
