#include <stdio.h>
#include <string.h>

/*
#include <io.h>
#include <ansi/colors.h>
#include <ansi/style.h>
*/
#include "../../ansi_anim/inc/io.h"
#include "../../ansi_anim/ansilib/inc/graphics/common.h"
#include "../../ansi_anim/ansilib/inc/ansi/common.h"
#include "../../ansi_anim/ansilib/inc/ansi/colors.h"
#include "../../ansi_anim/ansilib/inc/ansi/style.h"

static char *get_mode_str(mode_t m, char *mode_str)
{
    int  barf = 1;
    char temp[4];

    memset(mode_str, 0, (size_t) 32);

    while ( m ) {

        if ( m & 1 ) {
            sprintf(temp, "%i;", barf);
            strcat(mode_str, temp);
        }

        /* No six? Skip six! */
        barf = barf == 5 ? 7 : barf + 1;
        m >>= 1;
    }

    return mode_str;
}

void set_style(const style_t *s)
{
    char mode_str[32],
         buf[32],
         temp[32];

    sprintf(buf, CSI);

    get_mode_str(s->mode_bm, mode_str);
    strcat(buf, *mode_str ? mode_str : "0;");

    if ( s->fg ) {
        sprintf(temp, "%i;", s->fg);
        strcat(buf, temp);
    }

    if ( s->bg ) {
        sprintf(temp, "%i;", s->bg);
        strcat(buf, temp);
    }

    if ( buf[strlen(buf) - 1] == ';' )
        buf[strlen(buf) - 1] = 'm';
    else
        strcat(buf, "m");

    printws(NULL, buf);
}

void reset_all(void)
{
    printws(NULL, CSI"0m");
}
