// Shared ADC and digital endstop input
//
// Copyright (C) 2026  Klipper developers
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_in_read
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "trsync.h" // trsync_do_trigger

struct shared_adc_endstop {
    struct timer timer;
    struct gpio_adc adc;
    struct gpio_in digital;
    uint32_t adc_rest_time, adc_sample_time, adc_next;
    uint32_t endstop_rest_time, endstop_sample_time, endstop_next;
    uint32_t adc_sum;
    struct trsync *ts;
    uint16_t adc_value;
    uint8_t adc_sample_count, adc_count, endstop_sample_count;
    uint8_t flags, trigger_count, trigger_reason;
};

enum {
    SAE_PIN_HIGH = 1 << 0,
    SAE_DIGITAL = 1 << 1,
    SAE_ADC_READY = 1 << 2,
};

static struct task_wake shared_adc_wake;

static uint_fast8_t shared_endstop_oversample_event(struct timer *timer);

// PIO_PDSR is synchronized to the peripheral clock.  Allow that synchronizer
// and the configured pull resistor to settle after changing an analog pin back
// to PIO control before servicing an immediate state query.
static void
shared_gpio_settle(void)
{
    uint32_t end = timer_read_time() + timer_from_us(10);
    while (timer_is_before(timer_read_time(), end))
        ;
}

static uint_fast8_t
shared_adc_event(struct timer *timer)
{
    struct shared_adc_endstop *s =
        container_of(timer, struct shared_adc_endstop, timer);
    uint32_t delay = gpio_adc_sample(s->adc);
    if (delay) {
        s->timer.waketime += delay;
        return SF_RESCHEDULE;
    }
    s->adc_sum += gpio_adc_read(s->adc);
    if (++s->adc_count < s->adc_sample_count) {
        s->timer.waketime += s->adc_sample_time;
        return SF_RESCHEDULE;
    }
    s->adc_value = s->adc_sum / s->adc_sample_count;
    s->adc_sum = 0;
    s->adc_count = 0;
    s->flags |= SAE_ADC_READY;
    sched_wake_task(&shared_adc_wake);
    s->adc_next += s->adc_rest_time;
    s->timer.waketime = s->adc_next;
    return SF_RESCHEDULE;
}

static uint_fast8_t
shared_endstop_event(struct timer *timer)
{
    struct shared_adc_endstop *s =
        container_of(timer, struct shared_adc_endstop, timer);
    uint8_t val = gpio_in_read(s->digital);
    uint32_t next = s->timer.waketime + s->endstop_rest_time;
    if ((val ? ~s->flags : s->flags) & SAE_PIN_HIGH) {
        s->timer.waketime = next;
        return SF_RESCHEDULE;
    }
    s->endstop_next = next;
    s->timer.func = shared_endstop_oversample_event;
    return shared_endstop_oversample_event(timer);
}

static uint_fast8_t
shared_endstop_oversample_event(struct timer *timer)
{
    struct shared_adc_endstop *s =
        container_of(timer, struct shared_adc_endstop, timer);
    uint8_t val = gpio_in_read(s->digital);
    if ((val ? ~s->flags : s->flags) & SAE_PIN_HIGH) {
        s->timer.func = shared_endstop_event;
        s->timer.waketime = s->endstop_next;
        s->trigger_count = s->endstop_sample_count;
        return SF_RESCHEDULE;
    }
    if (!--s->trigger_count) {
        trsync_do_trigger(s->ts, s->trigger_reason);
        return SF_DONE;
    }
    s->timer.waketime += s->endstop_sample_time;
    return SF_RESCHEDULE;
}

void
command_config_shared_adc_endstop(uint32_t *args)
{
    struct shared_adc_endstop *s = oid_alloc(
        args[0], command_config_shared_adc_endstop, sizeof(*s));
    s->adc = gpio_adc_setup(args[1]);
    s->digital = gpio_in_setup(args[1], args[2]);
    gpio_adc_restore(s->adc);
}
DECL_COMMAND(command_config_shared_adc_endstop,
             "config_shared_adc_endstop oid=%c pin=%u pull_up=%c");

void
command_query_shared_adc(uint32_t *args)
{
    struct shared_adc_endstop *s = oid_lookup(
        args[0], command_config_shared_adc_endstop);
    sched_del_timer(&s->timer);
    gpio_adc_cancel_sample(s->adc);
    s->adc_next = args[1];
    s->adc_sample_time = args[2];
    s->adc_sample_count = args[3];
    s->adc_rest_time = args[4];
    s->adc_sum = 0;
    s->adc_count = 0;
    s->flags &= ~SAE_ADC_READY;
    if (!s->adc_sample_count || (s->flags & SAE_DIGITAL))
        return;
    s->timer.func = shared_adc_event;
    s->timer.waketime = s->adc_next;
    sched_add_timer(&s->timer);
}
DECL_COMMAND(command_query_shared_adc,
             "query_shared_adc oid=%c clock=%u sample_ticks=%u"
             " sample_count=%c rest_ticks=%u");

void
command_shared_endstop_home(uint32_t *args)
{
    struct shared_adc_endstop *s = oid_lookup(
        args[0], command_config_shared_adc_endstop);
    sched_del_timer(&s->timer);
    gpio_adc_cancel_sample(s->adc);
    s->flags &= ~SAE_ADC_READY;
    if (!args[3]) {
        s->ts = NULL;
        s->flags = 0;
        gpio_adc_restore(s->adc);
        if (s->adc_sample_count) {
            s->adc_next = timer_read_time() + s->adc_rest_time;
            s->timer.func = shared_adc_event;
            s->timer.waketime = s->adc_next;
            sched_add_timer(&s->timer);
        }
        return;
    }
    gpio_in_reset(s->digital, args[8]);
    s->timer.waketime = args[1];
    s->endstop_sample_time = args[2];
    s->endstop_sample_count = args[3];
    s->endstop_rest_time = args[4];
    s->timer.func = shared_endstop_event;
    s->trigger_count = args[3];
    s->flags = SAE_DIGITAL | (args[5] ? SAE_PIN_HIGH : 0);
    s->ts = trsync_oid_lookup(args[6]);
    s->trigger_reason = args[7];
    sched_add_timer(&s->timer);
}
DECL_COMMAND(command_shared_endstop_home,
             "shared_endstop_home oid=%c clock=%u sample_ticks=%u"
             " sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c"
             " trigger_reason=%c pull_up=%c");

void
command_shared_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct shared_adc_endstop *s = oid_lookup(
        oid, command_config_shared_adc_endstop);
    uint8_t flags = s->flags;
    uint8_t temporary = !(flags & SAE_DIGITAL);
    if (temporary) {
        sched_del_timer(&s->timer);
        gpio_adc_cancel_sample(s->adc);
        s->flags &= ~SAE_ADC_READY;
        gpio_in_reset(s->digital, args[1]);
        shared_gpio_settle();
    }
    uint8_t value = gpio_in_read(s->digital);
    if (temporary) {
        gpio_adc_restore(s->adc);
        if (s->adc_sample_count) {
            s->adc_next = timer_read_time() + s->adc_rest_time;
            s->timer.func = shared_adc_event;
            s->timer.waketime = s->adc_next;
            sched_add_timer(&s->timer);
        }
    }
    sendf("shared_endstop_state oid=%c homing=%c next_clock=%u"
          " pin_value=%c", oid, !!(flags & SAE_DIGITAL),
          s->endstop_next, value);
}
DECL_COMMAND(command_shared_endstop_query_state,
             "shared_endstop_query_state oid=%c pull_up=%c");

void
shared_adc_task(void)
{
    if (!sched_check_wake(&shared_adc_wake))
        return;
    uint8_t oid;
    struct shared_adc_endstop *s;
    foreach_oid(oid, s, command_config_shared_adc_endstop) {
        irq_disable();
        if (!(s->flags & SAE_ADC_READY)) {
            irq_enable();
            continue;
        }
        uint16_t value = s->adc_value;
        uint32_t next = s->adc_next;
        s->flags &= ~SAE_ADC_READY;
        irq_enable();
        sendf("shared_adc_state oid=%c next_clock=%u value=%hu",
              oid, next, value);
    }
}
DECL_TASK(shared_adc_task);
