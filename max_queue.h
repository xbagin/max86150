#ifndef MAX_QUEUE_H
#define MAX_QUEUE_H

#include <inttypes.h>

struct queue_t {
	int head;
	int tail;
	int element_size;
	uint8_t *buffer;
	int buffer_size;
};

int queue_init(struct queue_t *q, uint8_t *q_buf, int element_size, int q_buf_size);
int queue_reset(struct queue_t *q);
int enqueue(struct queue_t *q, uint8_t *element);
int dequeue(struct queue_t *q, uint8_t *element);
int is_queue_empty(struct queue_t *q);

#endif
