#include "RTC.h"

#define EXTI_RTC_ALARM_BIT	17		// the extra exti interrupts (16,17,18,19) should be defined in exti.h (BUG)
#define RTC_CRH_OWIE_BIT	2
#define RTC_CRH_ALRIE_BIT	1
#define RTC_CRH_SECIE_BIT	0
#define RTC_BASE			((struct rtc_reg_map*)0x40002800)

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

// Added by Fox
#define LEAPYEAR(year)		(!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)		(LEAPYEAR(year) ? 366 : 365)
#define UNIX_START_YEAR		1970
#define SECS_M	60
#define SECS_H	3600
#define SECS_D	86400
#define SECS_W	604800
const uint8_t mlen[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
// ===========================

RTC::RTC() {
	RTC(RTCSEL_LSE);	 // Best for STM32F103Cxx board.
}

RTC::RTC(rtc_clk_src src) {
	// Initialize vars
	RTCDEV = { RTC_BASE, { NULL, NULL, NULL, NULL } };
	DateTime = { 0,0,0,0,0,0,0,0 };

	// Enable backup registers for save RTC data
	bkp_init();
	bkp_enable_writes();

	RCC_BASE->BDCR &= ~RCC_BDCR_RTCSEL;

	switch (src) {
	case RTCSEL_DEFAULT:
	case RTCSEL_LSI:
		rcc_start_lsi();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSI;
		rtc_set_prescaler_load(0x9C3F);		// ~39999
		break;
	case RTCSEL_LSE:
		rcc_start_lse();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSE;
		rtc_set_prescaler_load(0x7fff);		// =32767
		break;
	case RTCSEL_HSE:	// This selection uses HSE/128 as the RTC source (i.e. 64 kHz with an 8 mHz xtal)
		rcc_start_hse();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_HSE;
		rtc_set_prescaler_load(0xF423);		// ~62499
		break;
	case RTCSEL_NONE:
		RCC_BASE->BDCR = RCC_BDCR_RTCSEL_NONE;
		break;
	}

	// Enable the RTC
	bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_RTCEN_BIT, 1);

	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
}

RTC::~RTC() {

}

uint32_t RTC::GetTS() {
	uint32_t h, l;
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	h = RTCDEV.regs->CNTH & 0xffff;
	l = RTCDEV.regs->CNTL & 0xffff;
	return (h << 16) | l;
}

// timeStamp = seconds since 1970 (UTC)
void RTC::SetTS(uint32_t timeStamp) {
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	rtc_enter_config_mode();
	RTCDEV.regs->CNTH = (timeStamp >> 16) & 0xffff;
	RTCDEV.regs->CNTL = timeStamp & 0xffff;
	rtc_exit_config_mode();
	rtc_wait_finished();

	DateTime.timestamp = timeStamp + DateTime.timezone * SECS_H;
	UpdateDT();
}

void RTC::SetInterrupt(voidFuncPtr functionName, rtc_interrupt_id intterruptID) {
	RTCDEV.handlers[intterruptID] = functionName;
	rtc_enable_irq(intterruptID);
	switch (intterruptID) {
		case RTC_SECONDS_INTERRUPT: nvic_irq_enable(NVIC_RTC); break;
		case RTC_OVERFLOW_INTERRUPT: nvic_irq_enable(NVIC_RTC); break;
		case RTC_ALARM_GLOBAL_INTERRUPT: nvic_irq_enable(NVIC_RTC); break;
		case RTC_ALARM_SPECIFIC_INTERRUPT: nvic_irq_enable(NVIC_RTCALARM); break; // The alarm specific interrupt can wake us from deep sleep.
	}
}

void RTC::UnsetInterrupt(rtc_interrupt_id intterruptID) {
	rtc_disable_irq(intterruptID);
	RTCDEV.handlers[intterruptID] = NULL;
}

void RTC::rtc_set_prescaler_load(uint32_t value) {
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	rtc_enter_config_mode();
	RTCDEV.regs->PRLH = (value >> 16) & 0xffff;
	RTCDEV.regs->PRLL = value & 0xffff;
	rtc_exit_config_mode();
	rtc_wait_finished();
}

uint32_t RTC::rtc_get_divider() {
	uint32_t h, l;
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	h = RTCDEV.regs->DIVH & 0x000f;
	l = RTCDEV.regs->DIVL & 0xffff;
	return (h << 16) | l;
}

void RTC::rtc_set_alarm(uint32_t value) {
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	rtc_enter_config_mode();
	RTCDEV.regs->ALRH = (value >> 16) & 0xffff;
	RTCDEV.regs->ALRL = value & 0xffff;
	rtc_exit_config_mode();
	rtc_wait_finished();
}

void RTC::rtc_wait_finished() {
	while (*bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_RTOFF_BIT) == 0);
}

void RTC::rtc_clear_sync() {
	rtc_wait_finished();
	*bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_RSF_BIT) = 0;
}

void RTC::rtc_wait_sync() {
	while (*bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_RSF_BIT) == 0);
}

void RTC::rtc_enter_config_mode() {
	rtc_wait_finished();
	*bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_CNF_BIT) = 1;
}

void RTC::rtc_exit_config_mode() {
	rtc_wait_finished();
	*bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_CNF_BIT) = 0;
}

void RTC::rtc_enable_irq(uint8 interrupt) {
	rtc_wait_finished();

	// Enabling this interrupt allows waking up from deep sleep via WFI.
	if (interrupt == RTC_ALARM_SPECIFIC_INTERRUPT) {
		*bb_perip(&EXTI_BASE->IMR, EXTI_RTC_ALARM_BIT) = 1;
		*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 1;
	}
	else
		*bb_perip(&(RTCDEV.regs)->CRH, interrupt) = 1;
}

void RTC::rtc_disable_irq(uint8 interrupt) {
	rtc_wait_finished();
	if (interrupt == RTC_ALARM_SPECIFIC_INTERRUPT) {
		*bb_perip(&EXTI_BASE->IMR, EXTI_RTC_ALARM_BIT) = 0;
		*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 0;
	}
	else
		*bb_perip(&(RTCDEV.regs)->CRH, interrupt) = 0;
}

void RTC::rtc_enable_alarm_event() {
	*bb_perip(&EXTI_BASE->EMR, EXTI_RTC_ALARM_BIT) = 1;
	*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 1;
}

void RTC::rtc_disable_alarm_event() {
	*bb_perip(&EXTI_BASE->EMR, EXTI_RTC_ALARM_BIT) = 0;
	*bb_perip(&EXTI_BASE->RTSR, EXTI_RTC_ALARM_BIT) = 0;
}

int RTC::rtc_is_second() {
	return *bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_SECF_BIT);
}

int RTC::rtc_is_alarm() {
	return *bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_ALRF_BIT);
}

int RTC::rtc_is_overflow() {
	return *bb_perip(&(RTCDEV.regs)->CRL, RTC_CRL_OWF_BIT);
}

void RTC::dispatch_multiple_rtc_irq() {
	uint32_t dsr = RTCDEV.regs->CRH & RTCDEV.regs->CRL;
	void(**hs)(void) = RTCDEV.handlers;
	uint32_t handled = 0;
	handle_irq(dsr, RTC_CRL_SECF, hs, RTC_SECONDS_INTERRUPT, handled);
	handle_irq(dsr, RTC_CRL_ALRF, hs, RTC_ALARM_GLOBAL_INTERRUPT, handled);
	handle_irq(dsr, RTC_CRL_OWF, hs, RTC_OVERFLOW_INTERRUPT, handled);
	RTCDEV.regs->CRL &= ~handled;
}

void RTC::__irq_rtc(void) {
	dispatch_multiple_rtc_irq();
}

void RTC::__irq_rtcalarm(void) {
	void(*handler)(void) = RTCDEV.handlers[RTC_ALARM_SPECIFIC_INTERRUPT];
	if (handler) {
		handler();
		*bb_perip(&EXTI_BASE->PR, EXTI_RTC_ALARM_BIT) = 1;
		//asm volatile("nop");		// See comment in exti.c. Doesn't seem to be required.
		//asm volatile("nop");
	}
}


// Added by Fox
void RTC::SetTZ(int8 tzoffset)
{
	DateTime.timezone = tzoffset * SECS_H;
	UpdateDT();
}

// TODO: Is it correct?
void RTC::UpdateDT() {
	lastTS = DateTime.timestamp;
	DateTime.timestamp = GetTS() + DateTime.timezone * SECS_D;	// RTC in UTC format

	if (DateTime.timestamp != 0 && DateTime.timestamp == lastTS)
		return;		// No need to update DT, its already correct
	
	// Found somewhere in google...
	uint32_t seconds = DateTime.timestamp % SECS_D;	// Maybe use "lastTS". Its save 4 byte in ram...
	uint16_t days = 0;
	DateTime.year = UNIX_START_YEAR;	// Temp
	DateTime.month = 0;

	DateTime.second = seconds % 60;
	seconds /= 60;
	DateTime.minute = seconds % 60;
	seconds /= 60;
	DateTime.hour = seconds % 24;
	days = seconds / 24;
	//DateTime.wday = (days + FIRST_DAY_OF_WEEK) % 7;	// Day of week

	while (days >= YEARSIZE(DateTime.year))
	{
		days -= YEARSIZE(DateTime.year);
		DateTime.year++;
	}
	// now days = days in current year
	while (true)
	{
		uint8_t daysm = 0;
		if (DateTime.month == 2 && LEAPYEAR(DateTime.year))
			daysm = mlen[DateTime.month - 1] + 1;	// 29 days in February if leap year
		else
			daysm = mlen[DateTime.month - 1];
		if (daysm > days)
			break;
		else
			DateTime.month++;	// Month start from 1
	}
	DateTime.day = days + 1;
}

// TODO: Is it correct?
void RTC::SetTSFromDT(dt dateTime)
{
	dateTime.timestamp  = dateTime.second;
	dateTime.timestamp += dateTime.minute * SECS_M;
	dateTime.timestamp += dateTime.hour * SECS_H;
	dateTime.timestamp += dateTime.day * SECS_D;
	
	while (dateTime.month > 1)
	{
		if (dateTime.month == 2 && LEAPYEAR(dateTime.year))
			dateTime.timestamp += (mlen[dateTime.month - 1] + 1) * SECS_D;
		else
			dateTime.timestamp += (mlen[dateTime.month - 1]) * SECS_D;
		dateTime.month--;
	}

	while (dateTime.year > 1970)
	{
		dateTime.year--;
		dateTime.timestamp += YEARSIZE(dateTime.year) * SECS_D;
	}

	SetTS(dateTime.timestamp - DateTime.timezone * SECS_D);
}

String RTC::PrintDate() {
	char buff[11];
	sprintf(buff, PSTR("%02d/%02d/%04d"), DateTime.day, DateTime.month, DateTime.year);
	return String(buff);
}

String RTC::PrintTime() {
	char buff[9];
	sprintf(buff, PSTR("%02d:%02d:%02d"), DateTime.hour, DateTime.minute, DateTime.second);
	return String(buff);
}
