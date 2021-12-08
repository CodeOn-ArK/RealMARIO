/*
 * ANSI_anim.c
 *
 *  Created on: 04-Dec-2021
 *      Author: ark
 */

#include "ANSI_Escape_Sequences.h"
#include "main.h"
object_t main_char_body;

void object_init(object_t *obj){
	if(obj == NULL){
		//Init the main character
	obj = (object_t *)pvPortMalloc(sizeof(object_t));

	obj->obj_max_top_pos.col_no 	= 20;
	obj->obj_max_top_pos.line_no 	= 0;

	obj->color_str 					= NULL;
	obj->obj_style 					= NULL;

	//obj->draw_obj[5][6] = {hello,hello};
	strcpy(obj->draw_obj[0], (char *) ANSI_MOVE_CURSOR_TO_POS_LINE(20) ANSI_ESC"1m"ANSI_ESC "97m    ▄▄▄"ANSI_ESC"0m");
	strcpy(obj->draw_obj[1], (char *) ANSI_MOVE_CURSOR_TO_POS_LINE(21) ANSI_ESC"11m"ANSI_ESC"97m ▄█████▄▄ "ANSI_ESC"0m");
	strcpy(obj->draw_obj[2], (char *) ANSI_MOVE_CURSOR_TO_POS_LINE(22) ANSI_ESC"11m"ANSI_ESC"97m███"ANSI_ESC"46m▀▀▀▀"ANSI_ESC"40m▀"ANSI_ESC"46m▀"ANSI_ESC"40m▀"ANSI_ESC"46m▀"ANSI_ESC"0m");
	strcpy(obj->draw_obj[3], (char *) ANSI_MOVE_CURSOR_TO_POS_LINE(23) ANSI_ESC"11m"ANSI_ESC"97m███"ANSI_ESC"46m▄   "ANSI_ESC"22m"ANSI_ESC"30m▀ ▀"ANSI_ESC"0m"ANSI_ESC"36m▀"ANSI_ESC"0m");
	strcpy(obj->draw_obj[4], (char *) ANSI_MOVE_CURSOR_TO_POS_LINE(24) ANSI_ESC"11m"ANSI_ESC"97m ▄"ANSI_ESC"46m  "ANSI_ESC"0m"ANSI_ESC"1m"ANSI_ESC"97m█████▄ "ANSI_ESC"22m"ANSI_ESC"31m█▄"ANSI_ESC"0m");
	strcpy(obj->draw_obj[5], (char *) ANSI_MOVE_CURSOR_TO_POS_LINE(25) ANSI_ESC"122m"ANSI_ESC"31m▀▀"ANSI_ESC"0m"ANSI_ESC"1m"ANSI_ESC"41m"ANSI_ESC"97m▄"ANSI_ESC"46m▄   "ANSI_ESC"41m▄▄▄"ANSI_ESC"0m"ANSI_ESC"22m"ANSI_ESC"31m▀██▀"ANSI_ESC"0m");
/*
	printmsg(obj->draw_obj);
	main_char_body = *obj;
	printmsg(main_char_body.draw_obj);
*/
	}
}
