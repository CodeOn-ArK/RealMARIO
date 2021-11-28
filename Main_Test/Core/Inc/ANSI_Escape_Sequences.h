/*
 * ANSI_Escape_Sequences.h
 *
 *  Created on: 28-Nov-2021
 *      Author: ark
 */

#ifndef INC_ANSI_ESCAPE_SEQUENCES_H_
#define INC_ANSI_ESCAPE_SEQUENCES_H_

#define ANSI_ESC						"\033["

#define ANSI_256_COLOR_FG(x) 			 ANSI_ESC"38;5;"#x"m"
#define ANSI_256_COLOR_BG(x) 			 ANSI_ESC"48;5;"#x"m"
#define ANSI_TERMINATE 					 ANSI_ESC"0m\r\n"
#define ANSI_COLOR_BLACK 				 ANSI_ESC"0;31m"


/*
 * CURSOR CONTROLS
 */
#define ANSI_BRING_CURSOR_HOME				ANSI_ESC"H"
#define ANSI_MOVE_CURSOR_TO_POS(x,y)		ANSI_ESC#x";"#y"f"
/*
#define
#define
#define
#define
#define
#define
*/

/*
 * ERASE FUNCTIONS
 */
#define ANSI_CLR_SCR							ANSI_ESC"2J"
#define ANSI_ERASE_IN_DISPLAY					ANSI_ESC"J"
/*
#define
#define
#define
#define
#define
#define
#define
*/

/*
 * GRAPHICS MODE
 */

/*
 * 8-16 bit COLOR CODES
 */

/*************************
 * SCREEN MODES
 *************************/
/*
 * SET MODE
 */

/*
 *COMMON PRIVATE MODE
 */

#endif /* INC_ANSI_ESCAPE_SEQUENCES_H_ */
