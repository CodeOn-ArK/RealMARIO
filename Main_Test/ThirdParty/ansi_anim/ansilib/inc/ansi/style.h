#ifndef __ANSI_STYLE_H_
# define __ANSI_STYLE_H_

/*# include <ansi/colors.h>*/
#include "../../ansi_anim/ansilib/inc/ansi/colors.h"

typedef struct {
    color_t fg, bg;
    font_mode_t mode_bm;
} style_t;

void set_style(const style_t *);
void reset_all(void);

#endif  /* __ANSI_STYLE_H_ */
