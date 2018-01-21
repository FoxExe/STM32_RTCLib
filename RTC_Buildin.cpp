#include "RTC_Buildin.h"

#include <libmaple/libmaple.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
#include <libmaple/bitband.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/exti.h>

#define EXTI_RTC_ALARM_BIT	17	// Bug. Must be in exti.h

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#elif defined(ARDUINO_ARCH_SAMD)
// nothing special needed
#elif defined(ARDUINO_SAM_DUE)
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

/* Control register high (CRH) */
#define RTC_CRH_OWIE_BIT	2
#define RTC_CRH_ALRIE_BIT	1
#define RTC_CRH_SECIE_BIT	0
#define RTC_CRH_OWIE		BIT(RTC_CRH_OWIE_BIT)
#define RTC_CRH_ALRIE		BIT(RTC_CRH_ALRIE_BIT)
#define RTC_CRH_SECIE		BIT(RTC_CRH_SECIE_BIT)

/* Control register low (CRL) */
#define RTC_CRL_RTOFF_BIT	5
#define RTC_CRL_CNF_BIT		4
#define RTC_CRL_RSF_BIT		3
#define RTC_CRL_OWF_BIT		2
#define RTC_CRL_ALRF_BIT	1
#define RTC_CRL_SECF_BIT	0
#define RTC_CRL_RTOFF	BIT(RTC_CRL_RTOFF_BIT)
#define RTC_CRL_CNF		BIT(RTC_CRL_CNF_BIT)
#define RTC_CRL_RSF		BIT(RTC_CRL_RSF_BIT)
#define RTC_CRL_OWF		BIT(RTC_CRL_OWF_BIT)
#define RTC_CRL_ALRF	BIT(RTC_CRL_ALRF_BIT)
#define RTC_CRL_SECF	BIT(RTC_CRL_SECF_BIT)

#define handle_irq(dier_sr, irq_mask, handlers, iid, handled_irq) do {  \
	if ((dier_sr) & (irq_mask)) {                                   \
		void (*__handler)(void) = (handlers)[iid];                  \
		if (__handler) {                                            \
			__handler();                                            \
			handled_irq |= (irq_mask);                              \
		}                                                           \
	}                                                               \
} while (0)

void RTC_Buildin::begin(ClockSources src)
{
	bkp_init();
	bkp_enable_writes();
	RCC_BASE->BDCR &= ~RCC_BDCR_RTCSEL;

	uint32 prescaler = 0;

	switch (src)
	{
	case RTC_Buildin::RTCSEL_NONE:
		RCC_BASE->BDCR = RCC_BDCR_RTCSEL_NONE;
		break;
	default:	// LSE by default
	case RTC_Buildin::RTCSEL_DEFAULT:
	case RTC_Buildin::RTCSEL_LSE:
		rcc_start_lse();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSE;
		prescaler = 0x7FFF;
		break;
	case RTC_Buildin::RTCSEL_LSI:
		rcc_start_lsi();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSI;
		prescaler = 0x9C3F;
		break;
	case RTC_Buildin::RTCSEL_HSE:
		rcc_start_hse();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_HSE;
		prescaler = 0xF423;
		break;
	}

	bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_RTCEN_BIT, 1);	// Enable RTC

	_clear_sync();
	_wait_sync();
	_wait_finished();
	
	if (prescaler != 0)
		_set_prescaler_load(prescaler);
}

void RTC_Buildin::adjust(const DateTime & dt)
{
	_set_clock(dt.unixtime());
}

DateTime RTC_Buildin::now()
{
	return (uint32_t)_get_clock();
}

void RTC_Buildin::CreateAlarm(voidFuncPtr function, DateTime dt)
{
	//_set_alarm();
}

inline void RTC_Buildin::_wait_finished()
{
	while (*bb_perip(&regs->CRL, RTC_CRL_RTOFF_BIT) == 0);
}

inline void RTC_Buildin::_clear_sync()
{
	_wait_finished();
	*bb_perip(&regs->CRL, RTC_CRL_RSF_BIT) = 0;

}

inline void RTC_Buildin::_wait_sync()
{
	while (*bb_perip(&regs->CRL, RTC_CRL_RSF_BIT) == 0);
}

inline void RTC_Buildin::_enter_config_mode()
{
	_wait_finished();
	*bb_perip(&regs->CRL, RTC_CRL_CNF_BIT) = 1;
}

inline void RTC_Buildin::_exit_config_mode()
{
	_wait_finished();
	*bb_perip(&regs->CRL, RTC_CRL_CNF_BIT) = 0;
}

inline void RTC_Buildin::_enable_irq(_interrupts interrupt)
{
	_wait_finished();
	if (interrupt == RTC_INT_ALARM_OTHER) { // Enabling this interrupt allows waking up from deep sleep via WFI.
		*bb_perip(&EXTI_BASE->IMR, EXTI_RTC_ALARM_BIT) = 1;
		*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 1;
	}
	else *bb_perip(&regs->CRH, interrupt) = 1;
}

inline void RTC_Buildin::_disable_irq(_interrupts interrupt)
{
	_wait_finished();
	if (interrupt == RTC_INT_ALARM_OTHER) {
		*bb_perip(&EXTI_BASE->IMR, EXTI_RTC_ALARM_BIT) = 0;
		*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 0;
	}
	else *bb_perip(&regs->CRH, interrupt) = 0;

}

void RTC_Buildin::_attach_interrupt(_interrupts interrupt, voidFuncPtr handler)
{
	handlers[interrupt] = handler;
	_enable_irq(interrupt);
	switch (interrupt) {
		case RTC_INT_EVERYSECOND: nvic_irq_enable(NVIC_RTC); break;
		case RTC_INT_OVERFLOW: nvic_irq_enable(NVIC_RTC); break;
		case RTC_INT_ALARM_GLOBAL: nvic_irq_enable(NVIC_RTC); break;
		case RTC_INT_ALARM_OTHER: nvic_irq_enable(NVIC_RTCALARM); break;
	}
}

void RTC_Buildin::_detach_interrupt(_interrupts interrupt)
{
	_disable_irq(interrupt);
	handlers[interrupt] = NULL;
}

void RTC_Buildin::__irq_rtc(void)
{
	uint32 dsr = regs->CRH & regs->CRL;
	void(**hs)(void) = handlers;
	uint32 handled = 0;

	handle_irq(dsr, RTC_CRL_SECF, hs, RTC_INT_EVERYSECOND, handled);
	handle_irq(dsr, RTC_CRL_ALRF, hs, RTC_INT_ALARM_GLOBAL, handled);
	handle_irq(dsr, RTC_CRL_OWF, hs, RTC_INT_OVERFLOW, handled);

	regs->CRL &= ~handled;
}

void RTC_Buildin::__irq_rtcalarm(void)
{
	void(*handler)(void) = handlers[RTC_INT_ALARM_OTHER];
	if (handler) {
		handler();
		*bb_perip(&EXTI_BASE->PR, EXTI_RTC_ALARM_BIT) = 1;
		//asm volatile("nop");		// See comment in exti.c. Doesn't seem to be required.
		//asm volatile("nop");
	}
}

void RTC_Buildin::_set_prescaler_load(uint32 value)
{
	_clear_sync();
	_wait_sync();
	_wait_finished();
	_enter_config_mode();
	regs->PRLH = (value >> 16) & 0xffff;
	regs->PRLL = value & 0xffff;
	_exit_config_mode();
	_wait_finished();
}

uint32 RTC_Buildin::_get_divider()
{
	uint32 h, l;
	_clear_sync();
	_wait_sync();
	_wait_finished();
	h = regs->DIVH & 0x000f;
	l = regs->DIVL & 0xffff;
	return (h << 16) | l;
}

void RTC_Buildin::_set_alarm(uint32 value) {
	_clear_sync();
	_wait_sync();
	_wait_finished();
	_enter_config_mode();
	regs->ALRH = (value >> 16) & 0xffff;
	regs->ALRL = value & 0xffff;
	_exit_config_mode();
	_wait_finished();
}

inline void RTC_Buildin::_enable_alarm_event()
{
	*bb_perip(&EXTI_BASE->EMR, EXTI_RTC_ALARM_BIT) = 1;
	*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 1;
}

inline void RTC_Buildin::_disable_alarm_event()
{
	*bb_perip(&EXTI_BASE->EMR, EXTI_RTC_ALARM_BIT) = 0;
	*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 0;
}

inline int RTC_Buildin::_is_second()
{
	return *bb_perip(&regs->CRL, RTC_CRL_SECF_BIT);
}

inline int RTC_Buildin::_is_alarm()
{
	return *bb_perip(&regs->CRL, RTC_CRL_ALRF_BIT);
}

inline int RTC_Buildin::_is_overflow()
{
	return *bb_perip(&regs->CRL, RTC_CRL_OWF_BIT);
}

uint32 RTC_Buildin::_get_clock()
{
	_clear_sync();
	_wait_sync();
	_wait_finished();
	return ((regs->CNTH & 0xffff) << 16) | (regs->CNTL & 0xffff);
}

void RTC_Buildin::_set_clock(uint32 value)
{
	_clear_sync();
	_wait_sync();
	_wait_finished();
	_enter_config_mode();
	regs->CNTH = (value >> 16) & 0xffff;
	regs->CNTL = value & 0xffff;
	_exit_config_mode();
	_wait_finished();
}
