/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_console.h>

/*
 * Include these files as needed.
 * See config.mk PLATFORM_xxx configuration parameters.
 */
#include <sbi_utils/ipi/aclint_mswi.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/serial/uart8250.h>
#include <sbi_utils/timer/aclint_mtimer.h>

#define PLATFORM_PLIC_ADDR		0xc000000
#define PLATFORM_PLIC_NUM_SOURCES	128
#define PLATFORM_HART_COUNT BLACKPARROT_HART_COUNT
#define PLATFORM_CLINT_ADDR     0x300000
#define PLATFORM_ACLINT_MTIMER_FREQ	10000000
#define PLATFORM_ACLINT_MSWI_ADDR   (PLATFORM_CLINT_ADDR + \
					CLINT_MSWI_OFFSET)
#define PLATFORM_ACLINT_MTIMER_ADDR (PLATFORM_CLINT_ADDR + \
					CLINT_MTIMER_OFFSET)
#define PLATFORM_UART_ADDR		0x09000000
#define PLATFORM_UART_INPUT_FREQ	10000000
#define PLATFORM_UART_BAUDRATE		115200

#if 0
static struct plic_data plic = {
	.addr = PLATFORM_PLIC_ADDR,
	.num_src = PLATFORM_PLIC_NUM_SOURCES,
};
#endif

uint64_t* getchar_ptr  = (uint64_t*)(0x00100000);
uint64_t* putchar_ptr  = (uint64_t*)(0x00101000);
uint64_t poweroff_base = (uint64_t)(0x00102000);

static struct aclint_mswi_data mswi = {
	.addr = PLATFORM_ACLINT_MSWI_ADDR,
	.size = ACLINT_MSWI_SIZE,
	.first_hartid = 0,
	.hart_count = PLATFORM_HART_COUNT,
};

static struct aclint_mtimer_data mtimer = {
	.mtime_freq = PLATFORM_ACLINT_MTIMER_FREQ,
	.mtime_addr = PLATFORM_ACLINT_MTIMER_ADDR +
			ACLINT_DEFAULT_MTIME_OFFSET,
	.mtime_size = ACLINT_DEFAULT_MTIME_SIZE,
	.mtimecmp_addr = PLATFORM_ACLINT_MTIMER_ADDR +
			ACLINT_DEFAULT_MTIMECMP_OFFSET,
	.mtimecmp_size = ACLINT_DEFAULT_MTIMECMP_SIZE,
	.first_hartid = 0,
	.hart_count = PLATFORM_HART_COUNT,
	.has_64bit_mmio = TRUE,
};

static int bp_system_reset_check(u32 type, u32 reason)
{
	switch(type) {
		case SBI_SRST_RESET_TYPE_SHUTDOWN:
			return 1;
	}
	return 0;
}
static void bp_system_reset(u32 type, u32 reason)
{
	for (int i = 0; i < BLACKPARROT_HART_COUNT; i++) {
		*(uint64_t*)(poweroff_base + (i << 3)) = type;
	}
}

static struct sbi_system_reset_device bp_reset = {
	.name = "bp-reset",
	.system_reset_check = bp_system_reset_check,
	.system_reset = bp_system_reset
};

/*
 * Platform early initialization.
 */
static int platform_early_init(bool cold_boot)
{
	if(cold_boot)
		sbi_system_reset_add_device(&bp_reset);
	return 0;
}

/*
 * Platform final initialization.
 */
static int platform_final_init(bool cold_boot)
{
	return 0;
}


void bp_console_putc(char ch)
{
	*putchar_ptr = ch;
}

int bp_console_getc(void)
{
	int ch = *getchar_ptr;
	return ch;
}

static struct sbi_console_device bp_console = {
	.name = "bp-console",
	.console_putc = bp_console_putc,
	.console_getc = bp_console_getc
};

/*
 * Initialize the platform console.
 */
static int platform_console_init(void)
{
#if 0
	/* Example if the generic UART8250 driver is used */
	return uart8250_init(PLATFORM_UART_ADDR, PLATFORM_UART_INPUT_FREQ,
			     PLATFORM_UART_BAUDRATE, 0, 1);
#endif
	sbi_console_set_device(&bp_console);
	return 0;
}

/*
 * Initialize the platform interrupt controller for current HART.
 */
static int platform_irqchip_init(bool cold_boot)
{
#if 0
	u32 hartid = current_hartid();
	int ret;

	/* Example if the generic PLIC driver is used */
	if (cold_boot) {
		ret = plic_cold_irqchip_init(&plic);
		if (ret)
			return ret;
	}

	return plic_warm_irqchip_init(&plic, 2 * hartid, 2 * hartid + 1);
#endif
	return 0;
}

/*
 * Initialize IPI for current HART.
 */
static int platform_ipi_init(bool cold_boot)
{
	int ret;

	/* Example if the generic ACLINT driver is used */
	if (cold_boot) {
		ret = aclint_mswi_cold_init(&mswi);
		if (ret)
			return ret;
	}

	return aclint_mswi_warm_init();
}

/*
 * Initialize platform timer for current HART.
 */
static int platform_timer_init(bool cold_boot)
{
	int ret;

	/* Example if the generic ACLINT driver is used */
	if (cold_boot) {
		ret = aclint_mtimer_cold_init(&mtimer, NULL);
		if (ret)
			return ret;
	}

	return aclint_mtimer_warm_init();
}

/*
 * Platform descriptor.
 */
const struct sbi_platform_operations platform_ops = {
	.early_init		= platform_early_init,
	.final_init		= platform_final_init,
	.console_init		= platform_console_init,
	.irqchip_init		= platform_irqchip_init,
	.ipi_init		= platform_ipi_init,
	.timer_init		= platform_timer_init
};
const struct sbi_platform platform = {
	.opensbi_version	= OPENSBI_VERSION,
	.platform_version	= SBI_PLATFORM_VERSION(0x0, 0x00),
	.name			= "BlackParrot",
	.features		= SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count		= PLATFORM_HART_COUNT,
	.hart_stack_size	= SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr	= (unsigned long)&platform_ops
};
