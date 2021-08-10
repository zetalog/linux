/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smart-core.cn>
 */

#define pr_fmt(fmt) "rvtimer: " fmt
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/tick.h>
#include <linux/sched_clock.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#ifndef CONFIG_RISCV_M_MODE
#include <asm/clint.h>
#endif

/* Default register offsets */
#define RV_TIMER_CMP_OFF	0x208
#define RV_TIMER_VAL_OFF	0x40

static u64 __iomem *rvtimer_timer_cmp;
static u64 __iomem *rvtimer_timer_val;
static unsigned long rvtimer_timer_freq;
static unsigned int rvtimer_timer_irq;

#ifdef CONFIG_RISCV_M_MODE
u64 __iomem *clint_time_val;
EXPORT_SYMBOL(clint_time_val);
#endif

#ifdef CONFIG_64BIT
#define rvtimer_get_cycles()	readq_relaxed(rvtimer_timer_val)
#else
#define rvtimer_get_cycles()	readl_relaxed(rvtimer_timer_val)
#define rvtimer_get_cycles_hi()	readl_relaxed(((u32 *)rvtimer_timer_val) + 1)
#endif

#ifdef CONFIG_64BIT
static u64 notrace rvtimer_get_cycles64(void)
{
	return rvtimer_get_cycles();
}
#else /* CONFIG_64BIT */
static u64 notrace rvtimer_get_cycles64(void)
{
	u32 hi, lo;

	do {
		hi = rvtimer_get_cycles_hi();
		lo = rvtimer_get_cycles();
	} while (hi != rvtimer_get_cycles_hi());

	return ((u64)hi << 32) | lo;
}
#endif /* CONFIG_64BIT */

static u64 rvtimer_rdtime(struct clocksource *cs)
{
	return rvtimer_get_cycles64();
}

static struct clocksource rvtimer_clocksource = {
	.name		= "rvtimer_clocksource",
	.rating		= 300,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= rvtimer_rdtime,
};

static int rvtimer_clock_next_event(unsigned long delta,
				    struct clock_event_device *ce)
{
	void __iomem *r = rvtimer_timer_cmp +
			  cpuid_to_hartid_map(smp_processor_id());

	csr_set(CSR_IE, IE_TIE);
	writeq_relaxed(rvtimer_get_cycles64() + delta, r);
	return 0;
}

static DEFINE_PER_CPU(struct clock_event_device, rvtimer_clock_event) = {
	.name		= "rvtimer_clockevent",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 100,
	.set_next_event	= rvtimer_clock_next_event,
};

static int rvtimer_timer_starting_cpu(unsigned int cpu)
{
	struct clock_event_device *ce = per_cpu_ptr(&rvtimer_clock_event, cpu);

	ce->cpumask = cpumask_of(cpu);
	clockevents_config_and_register(ce, rvtimer_timer_freq,
					100, 0x7fffffff);

	enable_percpu_irq(rvtimer_timer_irq,
			  irq_get_trigger_type(rvtimer_timer_irq));
	return 0;
}

static int rvtimer_timer_dying_cpu(unsigned int cpu)
{
	disable_percpu_irq(rvtimer_timer_irq);
	return 0;
}

static irqreturn_t rvtimer_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evdev = this_cpu_ptr(&rvtimer_clock_event);

	csr_clear(CSR_IE, IE_TIE);
	evdev->event_handler(evdev);

	return IRQ_HANDLED;
}

static int __init rvtimer_timer_init_dt(struct device_node *np)
{
	int rc;
	u32 i, nr_irqs;
	void __iomem *base;
	struct of_phandle_args oirq;
	u32 cmp_off, val_off;

	/*
	 * Ensure that timer device interrupt is RV_IRQ_TIMER. If it's
	 * anything else then we ignore the device.
	 */
	nr_irqs = of_irq_count(np);
	for (i = 0; i < nr_irqs; i++) {
		if (of_irq_parse_one(np, i, &oirq)) {
			pr_err("%pOFP: failed to parse irq %d.\n", np, i);
			continue;
		}

		if ((oirq.args_count != 1) ||
		    (oirq.args[0] != RV_IRQ_TIMER)) {
			pr_err("%pOFP: invalid irq %d (hwirq %d)\n",
			       np, i, oirq.args[0]);
			return -ENODEV;
		}

		/* Find parent irq domain and map timer irq */
		if (!rvtimer_timer_irq &&
		    oirq.args[0] == RV_IRQ_TIMER &&
		    irq_find_host(oirq.np))
			rvtimer_timer_irq = irq_of_parse_and_map(np, i);
	}

	/* If timer irq not found then fail */
	if (!rvtimer_timer_irq) {
		pr_err("%pOFP: timer irq not found\n", np);
		return -ENODEV;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%pOFP: could not map registers\n", np);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "smarco,rvtimer-cmp-off",
				 &cmp_off))
		cmp_off = RV_TIMER_CMP_OFF;
	if (of_property_read_u32(np, "smarco,rvtimer-val-off",
				 &val_off))
		cmp_off = RV_TIMER_VAL_OFF;

	rvtimer_timer_cmp = base + cmp_off;
	rvtimer_timer_val = base + val_off;
	rvtimer_timer_freq = riscv_timebase;

#ifdef CONFIG_RISCV_M_MODE
	/* The odd naming scheme that time_val is public */
	clint_time_val = rvtimer_timer_val;
#endif

	pr_info("%pOFP: timer running at %ld Hz\n", np, rvtimer_timer_freq);

	rc = clocksource_register_hz(&rvtimer_clocksource, rvtimer_timer_freq);
	if (rc) {
		pr_err("%pOFP: clocksource register failed [%d]\n", np, rc);
		goto fail_iounmap;
	}

	sched_clock_register(rvtimer_get_cycles64, 64, rvtimer_timer_freq);

	rc = request_percpu_irq(rvtimer_timer_irq, rvtimer_timer_interrupt,
				 "rvtimer-timer", &rvtimer_clock_event);
	if (rc) {
		pr_err("registering percpu irq failed [%d]\n", rc);
		goto fail_iounmap;
	}

	rc = cpuhp_setup_state(CPUHP_AP_CLINT_TIMER_STARTING,
			       "clockevents/rvtimer/timer:starting",
			       rvtimer_timer_starting_cpu,
			       rvtimer_timer_dying_cpu);
	if (rc) {
		pr_err("%pOFP: cpuhp setup state failed [%d]\n", np, rc);
		goto fail_free_irq;
	}

	return 0;

fail_free_irq:
	free_irq(rvtimer_timer_irq, &rvtimer_clock_event);
fail_iounmap:
	iounmap(base);
	return rc;
}

TIMER_OF_DECLARE(rvtimer_timer, "smarco,rvtimer0", rvtimer_timer_init_dt);
