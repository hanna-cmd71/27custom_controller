#include "music_see_you_again.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
#define G_do 392
#define G_re 440
#define G_mi 494
#define G_fa 523
#define G_so 587
#define G_la 659
#define G_si 698
// clang-format on

#define ONE_FOURTH 250
#define ONE_FOURTH_HALF 125
#define ONE_FOURTH_HALF_HALF 63

#define NOTE_NUM 300
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
void MusicSeeYouAgainPlay(void)
{
    now = HAL_GetTick();
    if (now - start_time >= Notes[play_id].end) {
        play_id++;
        if (play_id > last_note_id) {
            play_id = 1;
            start_time = now;
        }

        buzzer_note(Notes[play_id].note, 0.04);
    }
}

void MusicSeeYouAgainInit(void)
{
    SleepNote(500);

    // do re mi fa so la si
    // 1  2  3  4  5  6  7

    // 前奏
    // 5 2 1 5 5 1 2 3 2 1 2
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);  // 3`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__

    // 5 2 1 5 5 1 2 3 2 1 2
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);  // 3`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__

    // 5 2 1 5 5 1 2 3 2 1 2
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_so, ONE_FOURTH_HALF);           // 5_
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);  // 3`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_re * 2, ONE_FOURTH_HALF_HALF);  // 2`__

    // It`s been a
    // 5 2 1 5 5 1 3 5
    WriteNote(G_so, ONE_FOURTH_HALF);      // 5_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);  // 2`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);  // 1`_
    WriteNote(G_so, ONE_FOURTH_HALF);      // 5_
    WriteNote(G_so, ONE_FOURTH_HALF);      // 5_
    WriteNote(G_do, ONE_FOURTH_HALF);      // 1_
    WriteNote(G_mi, ONE_FOURTH_HALF);      // 3_
    WriteNote(G_so, ONE_FOURTH_HALF);      // 5_

    // long day without
    // 6 5 5 0 1
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);         // 6-
    WriteNote(G_so, ONE_FOURTH_HALF);                      // 5_
    WriteNote(G_so, ONE_FOURTH);                           // 5
    WriteNote(0, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 0-_
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);                 // 1_

    // you my friend And I`ll
    // 2 2 1 2 3 0 3 5
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);       // 1_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_mi, ONE_FOURTH);            // 3
    WriteNote(0, ONE_FOURTH_HALF);          // 0_
    WriteNote(G_mi, ONE_FOURTH_HALF_HALF);  // 3__
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);  // 5__

    // tell you all about it when I
    // 6 7 6 5 3 2 2 1
    WriteNote(G_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(G_si, ONE_FOURTH_HALF);  // 7_
    WriteNote(G_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(G_so, ONE_FOURTH_HALF);  // 5_
    WriteNote(G_mi, ONE_FOURTH_HALF);  // 3_
    WriteNote(G_re, ONE_FOURTH_HALF);  // 2_

    // see you again We`ve come a
    // 2 2 2 1 0 0 1 3 5
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);       // 1_
    WriteNote(0, ONE_FOURTH);               // 0
    WriteNote(0, ONE_FOURTH_HALF_HALF);     // 0__
    WriteNote(G_do, ONE_FOURTH_HALF_HALF);  // 1__
    WriteNote(G_mi, ONE_FOURTH_HALF_HALF);  // 3__
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);  // 5__

    // long way from
    // 6- 5_ 5 0-_ 1__
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);         // 6-
    WriteNote(G_so, ONE_FOURTH_HALF);                      // 5_
    WriteNote(G_so, ONE_FOURTH);                           // 5
    WriteNote(0, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 0-_
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);                 // 1__

    // where we began Oh I`ll
    // 2_ 2_ 1_ 3_ 0 2__ 3__ 5_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);       // 1_
    WriteNote(G_mi, ONE_FOURTH_HALF);       // 3_
    WriteNote(0, ONE_FOURTH);               // 0
    WriteNote(G_re, ONE_FOURTH_HALF_HALF);  // 2__
    WriteNote(G_mi, ONE_FOURTH_HALF_HALF);  // 3__
    WriteNote(G_so, ONE_FOURTH_HALF);       // 5_

    // tell you all about it when I
    // 6_ 1`_ 2`_ 3`_ 2`_ 1`_ 5__ 6__ 1`_
    WriteNote(G_la, ONE_FOURTH_HALF);           // 6_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);      // 5__
    WriteNote(G_la, ONE_FOURTH_HALF_HALF);      // 6__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`_

    // see you again When I
    // 3`_ 3`_ 3`_ 2`_ 0 5__ 6__ 1`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(0, ONE_FOURTH);                   // 0
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);      // 5__
    WriteNote(G_la, ONE_FOURTH_HALF_HALF);      // 6__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`_

    // see you again
    // 3`_ 3`_ 3`_ 2`_ 0 0
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);  // 2`_
    WriteNote(0, ONE_FOURTH);              // 0
    WriteNote(0, ONE_FOURTH);              // 0

    // 说唱部分
    WriteNote(0, 200);  // 0

    // how could we not
    // 0 0 0 1`__ 1`__ 1`__ 3`__
    WriteNote(0, ONE_FOURTH);                   // 0
    WriteNote(0, ONE_FOURTH);                   // 0
    WriteNote(0, ONE_FOURTH);                   // 0
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`__
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);  // 3`__

    // talk about family when family`s all that we got
    // 3`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 6-_ 3`__
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);       // 3`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF * 11);  // 1`__
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);   // 6-_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);       // 3`__

    // everything I went through you were standing there by my side and
    // 1`_ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 6-_ 6__
    WriteNote(G_do * 2, ONE_FOURTH_HALF);            // 1`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF * 10);  // 1`__
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);   // 6-_
    WriteNote(G_la, ONE_FOURTH_HALF);                // 6__

    // now you gonna be with me for the last ride/it`s been a
    // 3`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`__ 1`_ 1_ 3_ 5_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF_HALF);      // 3`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF * 7);  // 1`__
    WriteNote(G_do * 2, ONE_FOURTH_HALF);           // 1`_
    WriteNote(G_do, ONE_FOURTH_HALF);               // 1_
    WriteNote(G_mi, ONE_FOURTH_HALF);               // 3_
    WriteNote(G_so, ONE_FOURTH_HALF);               // 5_

    // long day without
    // 6- 5_ 5 0-_ 1_
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);         // 6-
    WriteNote(G_so, ONE_FOURTH_HALF);                      // 5_
    WriteNote(G_so, ONE_FOURTH);                           // 5
    WriteNote(0, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 0-_
    WriteNote(G_do, ONE_FOURTH_HALF);                      // 1_

    // you my friend. And I`ll
    // 2_ 2_ 1_ 2_ 3 0 3_ 5_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);       // 1_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_mi, ONE_FOURTH);            // 3
    WriteNote(0, ONE_FOURTH_HALF);          // 0_
    WriteNote(G_mi, ONE_FOURTH_HALF_HALF);  // 3__
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);  // 5__

    // tell you all about it when I
    // 6_ 7_ 6_ 5_ 3_ 2_ 2_ 1_
    WriteNote(G_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(G_si, ONE_FOURTH_HALF);  // 7_
    WriteNote(G_la, ONE_FOURTH_HALF);  // 6_
    WriteNote(G_so, ONE_FOURTH_HALF);  // 5_
    WriteNote(G_mi, ONE_FOURTH_HALF);  // 3_
    WriteNote(G_re, ONE_FOURTH_HALF);  // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);  // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);  // 1_

    // see you again. We`ve come a
    // 2_ 2_ 2_ 1_ 0 0__ 1__ 3__ 5__
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);       // 1_
    WriteNote(0, ONE_FOURTH);               // 0
    WriteNote(0, ONE_FOURTH_HALF_HALF);     // 0__
    WriteNote(G_do, ONE_FOURTH_HALF_HALF);  // 1__
    WriteNote(G_mi, ONE_FOURTH_HALF_HALF);  // 3__
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);  // 5__

    // long way from
    // 6- 5_ 5 0-_ 1__
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);         // 6-
    WriteNote(G_so, ONE_FOURTH_HALF);                      // 5_
    WriteNote(G_so, ONE_FOURTH);                           // 5
    WriteNote(0, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 0-_
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);                 // 1__

    // where we began. Oh I`ll
    // 2_ 2_ 1_ 3_ 0 2__ 3__ 5_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_re, ONE_FOURTH_HALF);       // 2_
    WriteNote(G_do, ONE_FOURTH_HALF);       // 1_
    WriteNote(G_mi, ONE_FOURTH_HALF);       // 3_
    WriteNote(0, ONE_FOURTH);               // 0
    WriteNote(G_re, ONE_FOURTH_HALF_HALF);  // 2__
    WriteNote(G_mi, ONE_FOURTH_HALF_HALF);  // 3__
    WriteNote(G_so, ONE_FOURTH_HALF);       // 5_

    // tell you all about it when I
    // 6_ 1`_ 2`_ 3`_ 2`_ 1`_ 5__ 6__ 1`_
    WriteNote(G_la, ONE_FOURTH_HALF);           // 6_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(G_do * 2, ONE_FOURTH_HALF);       // 1`_
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);      // 5__
    WriteNote(G_la, ONE_FOURTH_HALF_HALF);      // 6__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`_

    // see you again. When I
    // 3`_ 3`_ 3`_ 2`_ 0 5__ 6__ 1`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);       // 3`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);       // 2`_
    WriteNote(0, ONE_FOURTH);                   // 0
    WriteNote(G_so, ONE_FOURTH_HALF_HALF);      // 5__
    WriteNote(G_la, ONE_FOURTH_HALF_HALF);      // 6__
    WriteNote(G_do * 2, ONE_FOURTH_HALF_HALF);  // 1`_

    // see you again ah ah
    // 3`_ 3`_ 3`_ 2`_ 0 1`_ 7_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(G_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WriteNote(G_re * 2, ONE_FOURTH_HALF);  // 2`_
    WriteNote(0, ONE_FOURTH);              // 0
    WriteNote(G_do * 2, ONE_FOURTH_HALF);  // 1`_
    WriteNote(G_si, ONE_FOURTH_HALF);      // 7_

    // oh oh ah ah
    // 6- 5- 1`_ 7_
    WriteNote(G_la, ONE_FOURTH + ONE_FOURTH_HALF);  // 6-
    WriteNote(G_so, ONE_FOURTH + ONE_FOURTH_HALF);  // 5-
    WriteNote(G_do * 2, ONE_FOURTH_HALF);           // 1`_
    WriteNote(G_si, ONE_FOURTH_HALF);               // 7_

    // oh ah oh ah oh
    // 6-_ 7__ 6_ 5_ 3 0
    WriteNote(G_la, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6-_
    WriteNote(G_si, ONE_FOURTH_HALF_HALF);                    // 7__
    WriteNote(G_la, ONE_FOURTH_HALF);                         // 6_
    WriteNote(G_so, ONE_FOURTH_HALF);                         // 5_
    WriteNote(G_mi, ONE_FOURTH);                              // 3
    WriteNote(0, ONE_FOURTH);                                 // 0

    // 尾奏

    last_note_id = write_id - 1;
    write_id = 1;
}
/*------------------------------ End of File ------------------------------*/
