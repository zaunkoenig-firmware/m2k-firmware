/* MIT License
 *
 * Copyright (c) 2022 Zaunkoenig GmbH
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdint.h>
#include "anim.h"

#define BUF_SIZE 128
static struct Anim buf[BUF_SIZE] = {0}; // circular fifo buffer
static int anim_buf_head = 0, anim_buf_tail = 0; // tail is one past end index
static int anim_time_scale = 1; // slow down animation by this factor

void anim_set_scale(int scale)
{
	anim_time_scale = scale;
}

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

	static int count = 0;
	count = (count + 1) % anim_time_scale;
	if (count > 0)
		return (struct Xy){0};

	// step through animation only after anim_time_scale calls to anim_read()
	struct Xy ret = buf[anim_buf_head].xy;
	buf[anim_buf_head].len--;
	if (buf[anim_buf_head].len == 0)
		anim_buf_head = (anim_buf_head + 1) % BUF_SIZE;
	return ret;
}
