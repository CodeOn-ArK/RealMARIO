//#include <ansi/cursor.h>
#include "../../ansi_anim/ansilib/inc/ansi/cursor.h"

point_t get_screen_size(void)
{
    point_t ret, orig_pos = get_cursor_pos();

    /* Quoting from
     * https://viewsourcecode.org/snaptoken/kilo/03.rawInputAndOutput.html#window-size-the-hard-way
     *
     * We use a very large value, 999, which should ensure that the
     * cursor reaches the right and bottom edges of the screen.
     *
     * The C and B commands are specifically documented to stop the
     * cursor from going past the edge of the screen. The reason we
     * don’t use the <esc>[999;999H command is that the documentation
     * doesn’t specify what happens when you try to move the cursor
     * off-screen.
     */
    nudge_cursor(RIGHT, 999);
    nudge_cursor(DOWN, 999);
    ret = get_cursor_pos();

    move_cursor(orig_pos);
    return ret;
}
