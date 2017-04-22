#include "buzzer_song.h"
//if wanna make more music, need to make extern const in .h file as well
const MUSIC_NOTE START_UP[] = {{NOTE_G,6},{NOTE_B,6},{NOTE_D,7},{NOTE_G,7},{NOTE_G,7},{NOTE_END}};
const MUSIC_NOTE SUCCESSFUL_SOUND[] = {{NOTE_C,7}, {NOTE_D,7}, {NOTE_E,7}, {NOTE_END}};
const MUSIC_NOTE FAIL_SOUND[] = {{NOTE_C, 4}, {NOTE_C, 4}, {NOTE_END}};
const MUSIC_NOTE CLICK_LEFT[] = {{NOTE_G, 7}, {NOTE_END}};
const MUSIC_NOTE CLICK_RIGHT[] = {{1, 7}, {NOTE_END}};
const MUSIC_NOTE CLICK_DOWN[] = {{2, 7}, {NOTE_END}};
const MUSIC_NOTE CLICK_UP[] = {{3, 7}, {NOTE_END}};
const MUSIC_NOTE CLICK_CENTER[] = {{4, 7}, {NOTE_END}};
const MUSIC_NOTE CLICK_SMALL_BUTTON[] = {{5, 7}, {NOTE_END}};
const MUSIC_NOTE CLICK_MUSIC[] = {{6,7} ,{NOTE_END}};








const MUSIC_NOTE BIRTHDAY_SONG[] = {
	{NOTE_D,6},{NOTE_REST},{NOTE_D,6},
	{NOTE_E,6},{NOTE_E,6},{NOTE_E,6},
	{NOTE_D,6},{NOTE_D,6},{NOTE_D,6},
	{NOTE_G,6},{NOTE_G,6},{NOTE_G,6},
	{NOTE_Fs,6},{NOTE_Fs,6},{NOTE_Fs,6},
	{NOTE_Fs,6},{NOTE_Fs,6},{NOTE_REST},
	
	{NOTE_D,6},{NOTE_REST},{NOTE_D,6},
	{NOTE_E,6},{NOTE_E,6},{NOTE_E,6},
	{NOTE_D,6},{NOTE_D,6},{NOTE_D,6},
	{NOTE_A,6},{NOTE_A,6},{NOTE_A,6},
	{NOTE_G,6},{NOTE_G,6},{NOTE_G,6},
	{NOTE_G,6},{NOTE_G,6},{NOTE_REST},
	
	{NOTE_D,6},{NOTE_REST},{NOTE_D,6},
	{NOTE_D,7},{NOTE_D,7},{NOTE_D,7},	
	{NOTE_B,6},{NOTE_B,6},{NOTE_B,6},	
	{NOTE_G,6},{NOTE_G,6},{NOTE_G,6},	
	{NOTE_Fs,6},{NOTE_Fs,6},{NOTE_Fs,6},
	{NOTE_E,6},{NOTE_E,6},{NOTE_E,6},
	
	{NOTE_C,7},{NOTE_REST},{NOTE_C,7},
	{NOTE_B,6},{NOTE_B,6},{NOTE_B,6},	
	{NOTE_G,6},{NOTE_G,6},{NOTE_G,6},	
	{NOTE_A,6},{NOTE_A,6},{NOTE_A,6},	
	{NOTE_G,6},{NOTE_G,6},{NOTE_G,6},	
	{NOTE_G,6},{NOTE_G,6},{NOTE_END}
};
