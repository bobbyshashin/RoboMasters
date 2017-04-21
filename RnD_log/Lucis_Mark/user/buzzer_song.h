#ifndef	__BUZZER_SONG_H
#define	__BUZZER_SONG_H

#include "buzzer.h"

extern const MUSIC_NOTE START_UP[];
extern const MUSIC_NOTE SUCCESSFUL_SOUND[];
extern const MUSIC_NOTE FAIL_SOUND[];
extern const MUSIC_NOTE BIRTHDAY_SONG[];

extern const MUSIC_NOTE CLICK_LEFT[] ;
extern const MUSIC_NOTE CLICK_RIGHT[] ;
extern const MUSIC_NOTE CLICK_DOWN[] ;
extern const MUSIC_NOTE CLICK_UP[] ;
extern const MUSIC_NOTE CLICK_CENTER[] ;
extern const MUSIC_NOTE CLICK_SMALL_BUTTON[] ;
extern const MUSIC_NOTE CLICK_MUSIC[];



#define	SUCCESSFUL_MUSIC			buzzer_play_song(SUCCESSFUL_SOUND, 100, 0)
#define	FAIL_MUSIC 						buzzer_play_song(FAIL_SOUND, 120, 100)
#define CLICK_MUSIC_SOUND   	buzzer_play_song(CLICK_MUSIC, 120, 100)
#define voltage_music          buzzer_play_song(START_UP,100,100)





#endif /* __BUZZER_SONG_H */
