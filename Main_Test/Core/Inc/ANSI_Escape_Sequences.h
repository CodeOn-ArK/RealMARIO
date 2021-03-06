/*
 * ANSI_Escape_Sequences.h
 *
 *  Created on: 28-Nov-2021
 *      Author: ark & programish
 */

#include <stdint.h>

typedef struct pos{
	uint16_t line_no;
	uint16_t col_no;

}pos_line_col;

typedef struct object_t{
	//position
	pos_line_col obj_max_top_pos;
	pos_line_col obj_max_left_pos;
	//color
	char *color_str;	//stores the color macro
	//style
	char *obj_style;	//stores the style macro
	//data
	char draw_obj[10][20];


}object_t;
#ifndef INC_ANSI_ESCAPE_SEQUENCES_H_
#define INC_ANSI_ESCAPE_SEQUENCES_H_

#define ANSI_ESC									"\033["
#define DOC											"%d"
#define SOC											"%s"

#define ANSI_256_COLOR_FG(x) 			 			ANSI_ESC"38;5;"#x"m"
#define ANSI_256_COLOR_F	 						ANSI_ESC"38;5;"DOC"m"
#define ANSI_256_COLOR_BG(x) 			 			ANSI_ESC"48;5;"#x"m"
#define ANSI_TERMINATE 								ANSI_ESC"0m\r\n"
#define ANSI_COLOR_BLACK 							ANSI_ESC"0;31m"


/*
 * CURSOR CONTROLS
 */
#define ANSI_BRING_CURSOR_HOME						ANSI_ESC"H"
#define ANSI_MOVE_CURSOR_TO_POS(LINE,COL)			ANSI_ESC DOC";"DOC"f"
#define ANSI_MOVE_CURSOR_TO_POS_LINE_COL(LINE,COL)	ANSI_ESC#LINE";"#COL"f"
#define ANSI_MOVE_CURSOR_TO_POS_LINE(LINE)			ANSI_ESC#LINE";"DOC"f"
#define ANSI_MOVE_CURSOR_TO_POS_COL(COL)			ANSI_ESC DOC";"#COL"f"
#define ANSI_MOVE_CURSOR_UP(x) 						ANSI_ESC#x"A"
#define ANSI_MOVE_CURSOR_DOWN(x) 					ANSI_ESC#x"B"
#define ANSI_MOVE_CURSOR_RIGHT(x) 					ANSI_ESC#x"C"
#define ANSI_MOVE_CURSOR_LEFT(x) 					ANSI_ESC#x"D"
#define ANSI_MOVE_CURSOR_BEG_NXT(x) 				ANSI_ESC#x"E"
#define ANSI_MOVE_CURSOR_BEG_PREV(x) 				ANSI_ESC#x"F"
#define ANSI_MOVE_CURSOR_COL(x) 					ANSI_ESC#x"G"
#define ANSI_REQ_CURSOR_POS(x) 						ANSI_ESC"6n"
#define ANSI_MOVE_CURSOR_1_LINE_UP_OR_SCROLL 		"\033 M"
#define ANSI_SAVE_CURSOR_POS_DEC 					"\033 7"
#define ANSI_RESTORE_CURSOR_POS_DEC 				"\033 8"
#define ANSI_SAVE_CURSOR_POS_SOC(x) 				ANSI_ESC"s"
#define ANSI_RESTORE_CURSOR_POS_SOC(x) 				ANSI_ESC"u"


/*
 * ERASE FUNCTIONS
 */
#define ANSI_CLR_SCR								ANSI_ESC"2J"
#define ANSI_ERASE_IN_DISPLAY						ANSI_ESC"J"
#define ANSI_ERASE_CURSOR_TO_END					ANSI_ESC"0J"
#define ANSI_ERASE_CURSOR_TO_BEG					ANSI_ESC"1J"
#define ANSI_ERASE_SAVED_LINES						ANSI_ESC"3J"
#define ANSI_ERASE_IN_LINE							ANSI_ESC"K"
#define ANSI_ERASE_CURSOR_TO_END_OF_LINE			ANSI_ESC"0K"
#define ANSI_ERASE_START_OF_LINE_TO_CURSOR			ANSI_ESC"1K"
#define ANSI_ERASE_ENTIRE_LINE						ANSI_ESC"2K"

/*
 * GRAPHICS MODE
 */
#define ANSI_RESET_ALL_MODES						ANSI_ESC"0m"
#define ANSI_SET_BOLD_MODE							ANSI_ESC"1m"
#define ANSI_SET_DIM_FAINT_MODE						ANSI_ESC"2m"
#define ANSI_SET_ITALIC_MODE						ANSI_ESC"3m"
#define ANSI_SET_UNDERLINE_MODE						ANSI_ESC"4m"
#define ANSI_SET_BLINKING_MODE						ANSI_ESC"5m"
#define ANSI_SET_REVERSE_MODE						ANSI_ESC"7m"
#define ANSI_SET_HIDDEN_MODE						ANSI_ESC"8m"
#define ANSI_SET_STRIKETHROUGH_MODE					ANSI_ESC"9m"
#define ANSI_RESET_BOLD_MODE						ANSI_ESC"22m"
#define ANSI_RESET_DIM_FAINT_MODE					ANSI_ESC"22m"
#define ANSI_RESET_ITALIC_MODE						ANSI_ESC"23m"
#define ANSI_RESET_UNDERLINE_MODE					ANSI_ESC"24m"
#define ANSI_RESET_BLINKING_MODE					ANSI_ESC"25m"
#define ANSI_RESET_REVERSE_MODE						ANSI_ESC"27m"
#define ANSI_RESET_HIDDEN_MODE						ANSI_ESC"28m"
#define ANSI_RESET_STRIKETHROUGH_MODE				ANSI_ESC"29m"

/*
 * 8-16 bit COLOR CODES
 */
#define ANSI_SET_FG_RGB(r,g,b)						ANSI_ESC"38;2;"#r";"#g";"#b"m"
#define ANSI_SET_BG_RGB(r,g,b)						ANSI_ESC"48;2;"#r";"#g";"#b"m"

/*************************
 * SCREEN MODES
 *************************/
/*
 * SET MODE
 */
#define ANSI_40X25_MONOCHROME_TXT					ANSI_ESC"=0h"
#define ANSI_40X25_COLOR_TXT						ANSI_ESC"=1h"
#define ANSI_80X25_MONOCHROME_TXT					ANSI_ESC"=2h"
#define ANSI_80X25_COLOR_TXT						ANSI_ESC"=3h"
#define ANSI_320X200_4_COLOR_GRAPHICS				ANSI_ESC"=4h"
#define ANSI_320X200_MONOCHROME_GRAPHICS			ANSI_ESC"=5h"
#define ANSI_620X200_MONOCHROME_GRAPHICS			ANSI_ESC"=6h"
#define ANSI_ENABLE_LINE_WRAPPING					ANSI_ESC"=7h"
#define ANSI_320X200_COLOR_GRAPHICS					ANSI_ESC"=13h"
#define ANSI_640X200_COLOR_16_COLOR_GRAPHICS		ANSI_ESC"=14h"
#define ANSI_640X350_MONOCHROME_2_COLOR_GRAPHICS	ANSI_ESC"=15h"
#define ANSI_640X350_COLOR_16_COLOR_GRAPHICS		ANSI_ESC"=16h"
#define ANSI_640X480_MONOCHROME_2_COLOR_GRAPHICS	ANSI_ESC"=17h"
#define ANSI_640X480_COLOR_16_COLOR_GRAPHICS		ANSI_ESC"=18h"
#define ANSI_320X200_COLOR_256_COLOR_GRAPHICS		ANSI_ESC"=19h"

/*
 *COMMON PRIVATE MODE
 */
#define ANSI_CURSOR_INVISIBLE						ANSI_ESC"?251"
#define ANSI_MAKE_CURSOR_INVISIBLE					ANSI_ESC"?25l"
#define ANSI_MAKE_CURSOR_VISIBLE					ANSI_ESC"?25h"
#define ANSI_RESTORE_SCREEN							ANSI_ESC"?47l"
#define ANSI_SAVE_SCREEN							ANSI_ESC"?47h"
#define ANSI_ENABLE_ALTERNATIVE_BUFFER				ANSI_ESC"?1049h"
#define ANSI_DISABLE_ALTERNATIVE_BUFFER				ANSI_ESC"?1049l"

/*
 * 		ANSI_CLR_SCR
		 ANSI_ESC"1m"ANSI_ESC "97m    ?????????"ANSI_ESC"0m\r\n"
		 ANSI_ESC"11m"ANSI_ESC"97m ???????????????????????? "ANSI_ESC"0m\r\n"
		 ANSI_ESC"11m"ANSI_ESC"97m?????????"ANSI_ESC"46m????????????"ANSI_ESC"40m???"ANSI_ESC"46m???"ANSI_ESC"40m???"ANSI_ESC"46m???"ANSI_ESC"0m\r\n"
		 ANSI_ESC"11m"ANSI_ESC"97m?????????"ANSI_ESC"46m???   "ANSI_ESC"22m"ANSI_ESC"30m??? ???"ANSI_ESC"0m"ANSI_ESC"36m???"ANSI_ESC"0m\r\n"
		 ANSI_ESC"11m"ANSI_ESC"97m ???"ANSI_ESC"46m  "ANSI_ESC"0m"ANSI_ESC"1m"ANSI_ESC"97m?????????????????? "ANSI_ESC"22m"ANSI_ESC"31m??????"ANSI_ESC"0m\r\n"
		 ANSI_ESC"122m"ANSI_ESC"31m??????"ANSI_ESC"0m"ANSI_ESC"1m"ANSI_ESC"41m"ANSI_ESC"97m???"ANSI_ESC"46m???   "ANSI_ESC"41m?????????"ANSI_ESC"0m"ANSI_ESC"22m"ANSI_ESC"31m????????????"ANSI_ESC"0m\r\n"
		 ANSI_ESC"11m"ANSI_ESC"97m ????????????????????????  "ANSI_ESC"22m"ANSI_ESC"31m???"ANSI_ESC"0m\r\n"
		 ANSI_ESC"11m"ANSI_ESC"97m ???????????? ????????????"ANSI_ESC"0m\r\n"
 */
#endif /* INC_ANSI_ESCAPE_SEQUENCES_H_ */
