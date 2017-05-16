#pragma once

#include <libmaple/libmaple.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
#include <libmaple/bitband.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/exti.h>

class RTC
{
private:
	typedef struct rtc_reg_map {
		__io uint32_t CRH;		/**< Control register high */
		__io uint32_t CRL;		/**< Control register high */
		__io uint32_t PRLH;		/**< Prescaler load register high */
		__io uint32_t PRLL;		/**< Prescaler load register low */
		__io uint32_t DIVH;		/**< Prescaler divider register high */
		__io uint32_t DIVL;		/**< Prescaler divider register low */
		__io uint32_t CNTH;		/**< Counter register high */
		__io uint32_t CNTL;		/**< Counter register low */
		__io uint32_t ALRH;		/**< Alarm register high */
		__io uint32_t ALRL;		/**< Alarm register low */
	} rtc_reg_map;

	typedef struct rtc_dev {
		rtc_reg_map *regs;			// Register map
		voidFuncPtr handlers[4];     // Up to 4 user IRQ handlers
	} rtc_dev;

	static rtc_dev RTCDEV;

	typedef enum rtc_interrupt_id {
		RTC_SECONDS_INTERRUPT = 0,	/**< Counter seconds interrupt */
		RTC_ALARM_GLOBAL_INTERRUPT = 1,	/**< RTC alarm global interrupt (i.e. __irq_rtc()) */
		RTC_OVERFLOW_INTERRUPT = 2,	/**< Counter overflow interrupt */
		RTC_ALARM_SPECIFIC_INTERRUPT = 3		/**< RTC alarm specific interrupt (i.e. __irq_rtcalarm(), wake up from halt/sleep) */
	} rtc_interrupt_id;

	typedef enum rtc_clk_src {
		RTCSEL_DEFAULT = 0,
		RTCSEL_NONE = 0x10,
		RTCSEL_LSE = 0x11,
		RTCSEL_LSI = 0x12,
		RTCSEL_HSE = 0x13,
	} rtc_clk_src;

	static inline void rtc_wait_finished();
	static inline void rtc_clear_sync();
	static inline void rtc_wait_sync();
	static inline void rtc_enter_config_mode();
	static inline void rtc_exit_config_mode();
	static inline void rtc_enable_irq(uint8 interrupt);
	static inline void rtc_disable_irq(uint8 interrupt);
	static inline void rtc_enable_alarm_event();
	static inline void rtc_disable_alarm_event();
	static inline void dispatch_multiple_rtc_irq();
	static inline int rtc_is_second();
	static inline int rtc_is_alarm();
	static inline int rtc_is_overflow();
	void rtc_set_prescaler_load(uint32_t value);
	uint32_t rtc_get_divider();
	void __irq_rtc(void);
	void __irq_rtcalarm(void);
	void rtc_set_alarm(uint32_t value);

	int8 timezone = 0;
public:
	typedef struct dt
	{
		int8_t timezone;	// Possible -12...0...+12 (Hours)
		uint32_t timestamp;
		uint16_t year;
		uint8_t month;
		uint8_t day;
		uint8_t hour;
		uint8_t minute;
		uint8_t second;
	} dt;

	dt DateTime;

	RTC();
	RTC(rtc_clk_src src);

	~RTC();

	void SetTS(uint32_t timeStamp);
	uint32_t GetTS();

	void SetInterrupt(voidFuncPtr functionName, rtc_interrupt_id intterruptID);
	void UnsetInterrupt(rtc_interrupt_id intterruptID);

	void SetTZ(int8 tzoffset);
	void UpdateDT();
	void SetTS(uint32_t timeStamp);

	uint32_t NtpToUtc(uint32_t timeStamp);
	uint32_t UtcToNtp(uint32_t timeStamp);

	String PrintDate();
	String PrintTime();
};

