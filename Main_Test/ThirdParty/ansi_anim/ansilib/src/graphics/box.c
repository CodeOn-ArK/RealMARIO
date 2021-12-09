#include <stdint.h>

#include "../../ansi_anim/ansilib/inc/ansi/cursor.h"
#include "../../ansi_anim/ansilib/inc/graphics/box.h"
#include "../../ansi_anim/ansilib/inc/graphics/shapes.h"
/*
#include <ansi/cursor.h>
#include <graphics/box.h>
#include <graphics/shapes.h>
*/

void draw(const box_t *box)
{
    switch ( box->type ) {
    case RECTANGLE:
        move_cursor(box->origin);
        draw_rectangle(box->width, box->height, '#');
        break;
    }
}

void erase(const box_t *box)
{
    switch ( box->type ) {
    case RECTANGLE:
        move_cursor(box->origin);
        draw_rectangle(box->width, box->height, ' ');
        break;
    }
}

void nudge(box_t *box, dir_t dir, int32_t steps)
{
    switch ( dir ) {
    case UP:
        box->origin.y -= steps;
        break;

    case DOWN:
        box->origin.y += steps;
        break;

    case LEFT:
        box->origin.x -= steps;
        break;

    case RIGHT:
        box->origin.x += steps;
        break;
    }
}
