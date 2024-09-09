#include "max_queue.h"
#include <string.h>

int queue_init(struct queue_t *q, uint8_t *q_buf, int element_size, int q_buf_size) {
	if (q == NULL) {
		return -1;
	}
	q->buffer = q_buf;
	q->buffer_size = q_buf_size;
	q->element_size = element_size;
	return queue_reset(q);
}

int queue_reset(struct queue_t *q) {
	if (q == NULL) {
		return -1;
	}
	q->head = -1;
	q->tail = -1;
	if (q->buffer == NULL || q->buffer_size < 1 || q->buffer_size < q->element_size) {
		return -1;
	}
	memset(q->buffer, 0, q->buffer_size);
	return 0;
}

int enqueue(struct queue_t *q, uint8_t *element) {
	if (q == NULL || q->buffer == NULL || element == NULL) {
		return -1;
	}
	if (q->tail < 0) {
		q->head = 0;
		q->tail = 0;
	} else {
		// if sth in ... check H & T ovelap, ! ovf
		if (q->tail > q->head) {
			int new_tail = (q->tail + q->element_size) % q->buffer_size;
			if (new_tail < q->head + q->element_size && new_tail > q->head) {
				return -1;
			}
		} else if (q->tail < q->head) {
			if (q->tail + q->element_size > q->head) {
				return -1;
			}
		} else {  // q->tail == q->head
			return -2;
		}
	}
	memcpy(&q->buffer[q->tail], element, q->element_size);
	q->tail = (q->tail + q->element_size) % q->buffer_size;
	return 0;
}

int dequeue(struct queue_t *q, uint8_t *element) {
	if (q == NULL || q->buffer == NULL || q->head < 0) {
		return -1;
	}
	memcpy(element, &q->buffer[q->head], q->element_size);
	memset(&q->buffer[q->head], 0, q->element_size);
	q->head = (q->head + q->element_size) % q->buffer_size;
	if (q->head == q->tail) {
		q->head = -1;
		q->tail = -1;
	}
	return 0;
}

int is_queue_empty(struct queue_t *q) {
	return q->head == -1 && q->tail == -1;
}
