#include <stdint.h>

struct ecb_t {
	uint8_t key[16];
	uint8_t in[16];
	uint8_t out[16];
};

void SystemInit() {
}

#define ECB_BASE 0x4000E000
#define ECB_TASKS_START 0x000
#define ECB_TASKS_STOP 0x004
#define ECB_EVENTS_END 0x100
#define ECB_EVENTS_ERROR 0x104
#define ECB_DATAPTR 0x504

#define MMIO(base, offset) (*((volatile uint32_t*)(base + offset)))

void _start() {
	struct ecb_t ecb;
	int i=0;
	ecb.in[i++] = 0x00;
	ecb.in[i++] = 0x00;
	ecb.in[i++] = 0x01;
	ecb.in[i++] = 0x01;

	ecb.in[i++] = 0x03;
	ecb.in[i++] = 0x03;
	ecb.in[i++] = 0x07;
	ecb.in[i++] = 0x07;

	ecb.in[i++] = 0x0f;
	ecb.in[i++] = 0x0f;
	ecb.in[i++] = 0x1f;
	ecb.in[i++] = 0x1f;

	ecb.in[i++] = 0x3f;
	ecb.in[i++] = 0x3f;
	ecb.in[i++] = 0x7f;
	ecb.in[i++] = 0x7f;

	for (i=0;i<16;i++) {
		ecb.key[i] = 0;
		ecb.out[i] = 0;
	}

	MMIO(ECB_BASE, ECB_DATAPTR) = &ecb;
	MMIO(ECB_BASE, ECB_TASKS_START) = 1;

	while(1) {
	}
}
