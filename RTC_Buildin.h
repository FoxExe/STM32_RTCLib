#pragma once

#include <Arduino.h>
#include <RTClib.h>

class RTC_Buildin
{
public:
	enum ClockSources {
		RTCSEL_DEFAULT,
		RTCSEL_NONE,
		RTCSEL_LSE,
		RTCSEL_LSI,
		RTCSEL_HSE,
	};

	void begin(const DateTime& dt, ClockSources src = RTCSEL_DEFAULT) { begin(RTCSEL_DEFAULT); adjust(dt); }
	void adjust(const DateTime& dt);
	DateTime now();

	void begin(ClockSources src = RTCSEL_DEFAULT);	// Real init
	void CreateAlarm(voidFuncPtr function, DateTime dt);

private:
	uint32 _get_clock();
	void _set_clock(uint32 value);

protected:
	struct _regs {
		__io uint32 CRH;		// Control register high
		__io uint32 CRL;		// Control register high
		__io uint32 PRLH;		// Prescaler load register high
		__io uint32 PRLL;		// Prescaler load register low
		__io uint32 DIVH;		// Prescaler divider register high
		__io uint32 DIVL;		// Prescaler divider register low
		__io uint32 CNTH;		// Counter register high
		__io uint32 CNTL;		// Counter register low
		__io uint32 ALRH;		// Alarm register high
		__io uint32 ALRL;		// Alarm register low
	};

	enum _interrupts {
		RTC_INT_EVERYSECOND = 0,	// Counter seconds interrupt
		RTC_INT_ALARM_GLOBAL = 1,	// RTC alarm global interrupt (i.e. __irq_rtc())
		RTC_INT_OVERFLOW = 2,		// Counter overflow interrupt
		RTC_INT_ALARM_OTHER = 3,	// RTC alarm specific interrupt (i.e. __irq_rtcalarm(), wake up from halt/sleep)
		RTC_INT_TOTAL = 4,
	};

	_regs *regs = (struct _regs*)0x40002800;			// Direct access to RTC registers
	voidFuncPtr handlers[RTC_INT_TOTAL] = { NULL, };	// Interrupt handlers

	inline void _wait_finished();
	inline void _clear_sync();
	inline void _wait_sync();
	inline void _enter_config_mode();
	inline void _exit_config_mode();
	inline void _enable_irq(_interrupts interrupt);
	inline void _disable_irq(_interrupts interrupt);

	void _attach_interrupt(_interrupts interrupt, voidFuncPtr handler);
	void _detach_interrupt(_interrupts interrupt);
	void __irq_rtc(void);
	void __irq_rtcalarm(void);

	void _set_prescaler_load(uint32 value);
	uint32 _get_divider();

	void _set_alarm(uint32 value);
	inline void _enable_alarm_event();
	inline void _disable_alarm_event();
	inline int _is_second();
	inline int _is_alarm();
	inline int _is_overflow();
};

