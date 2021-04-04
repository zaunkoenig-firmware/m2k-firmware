#include <stdint.h>
#include "anim.h"

#define BUF_SIZE 128
static struct Anim buf[BUF_SIZE] = {0}; // circular fifo buffer
int anim_buf_head = 0, anim_buf_tail = 0; // tail is one past end index

void anim_add(int reps, int len_seq, struct Anim seq[])
{
	for (int i = 0; i < reps; i++) {
		for (int j = 0; j < len_seq; j++) {
			buf[anim_buf_tail] = seq[j];
			anim_buf_tail = (anim_buf_tail + 1) % BUF_SIZE;
		}
	}
}

struct Xy anim_read(void)
{
	if (anim_buf_head == anim_buf_tail)
		return (struct Xy){0};
	struct Xy ret = buf[anim_buf_head].xy;
	buf[anim_buf_head].len--;
	if (buf[anim_buf_head].len == 0)
		anim_buf_head = (anim_buf_head + 1) % BUF_SIZE;
	return ret;
}
