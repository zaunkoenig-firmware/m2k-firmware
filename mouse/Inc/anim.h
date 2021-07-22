#pragma once
#include "cmsis_compiler.h"

struct Xy {
	int16_t x, y;
};

// move in xy direction for len cycles
// remember that positive y is downwards
struct Anim {
	uint32_t len;
	struct Xy xy;
};

void anim_set_scale(int scale);

void anim_add(int reps, int len_seq, struct Anim seq[]);

struct Xy anim_read(void);

#define ANIM(l, _x, _y) ((struct Anim){.len = (l), .xy = (struct Xy){.x = (_x), .y = (_y)}})

#define LEN_LONG 300
#define LEN_MEDIUM 125
#define SPEED_FAST 4
#define SPEED_MEDIUM 2
#define SPEED_SLOW 1

#define PAUSE ANIM(300,  0,  0)

#define UP_L    ANIM(LEN_LONG / SPEED_FAST,  0, -SPEED_FAST)
#define DOWN_L  ANIM(LEN_LONG / SPEED_FAST,  0,  SPEED_FAST)
#define LEFT_L  ANIM(LEN_LONG / SPEED_FAST, -SPEED_FAST,  0)
#define RIGHT_L ANIM(LEN_LONG / SPEED_FAST,  SPEED_FAST,  0)

#define UP    ANIM(LEN_MEDIUM / SPEED_MEDIUM,  0, -SPEED_MEDIUM)
#define DOWN  ANIM(LEN_MEDIUM / SPEED_MEDIUM,  0,  SPEED_MEDIUM)
#define LEFT  ANIM(LEN_MEDIUM / SPEED_MEDIUM, -SPEED_MEDIUM,  0)
#define RIGHT ANIM(LEN_MEDIUM / SPEED_MEDIUM,  SPEED_MEDIUM,  0)

#define UP_SLOW    ANIM(LEN_MEDIUM / SPEED_SLOW,  0, -SPEED_SLOW)
#define DOWN_SLOW  ANIM(LEN_MEDIUM / SPEED_SLOW,  0,  SPEED_SLOW)
#define LEFT_SLOW  ANIM(LEN_MEDIUM / SPEED_SLOW, -SPEED_SLOW,  0)
#define RIGHT_SLOW ANIM(LEN_MEDIUM / SPEED_SLOW,  SPEED_SLOW,  0)

#define anim_cw(reps) anim_add((reps), 4, (struct Anim[]){UP_SLOW, RIGHT_SLOW, DOWN_SLOW, LEFT_SLOW})
#define anim_ccw(reps) anim_add((reps), 4, (struct Anim[]){RIGHT_SLOW, UP_SLOW, LEFT_SLOW, DOWN_SLOW})
#define anim_one(reps) anim_add((reps), 2, (struct Anim[]){RIGHT_SLOW, DOWN_SLOW})
#define anim_eight(reps) anim_add((reps), 8, (struct Anim[]){DOWN_SLOW, RIGHT_SLOW, DOWN_SLOW, LEFT_SLOW, UP_SLOW, RIGHT_SLOW, UP_SLOW, LEFT_SLOW})

#define anim_lg_updown(reps) anim_add((reps), 2, (struct Anim[]){UP_L, DOWN_L})
#define anim_lg_downup(reps) anim_add((reps), 2, (struct Anim[]){DOWN_L, UP_L})

#define anim_updown(reps) anim_add((reps), 2, (struct Anim[]){UP, DOWN})
#define anim_downup(reps) anim_add((reps), 2, (struct Anim[]){DOWN, UP})

#define anim_updown_pause(reps) anim_add((reps), 3, (struct Anim[]){UP, DOWN, PAUSE})
#define anim_downup_pause(reps) anim_add((reps), 3, (struct Anim[]){DOWN, UP, PAUSE})
#define anim_rightleft_pause(reps) anim_add((reps), 3, (struct Anim[]){RIGHT, LEFT, PAUSE})
