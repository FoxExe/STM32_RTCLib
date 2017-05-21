/******************************************************************************
* The MIT License
*
* Copyright (c) 2010 LeafLabs LLC.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*****************************************************************************/

#include "RTClock.h"


RTC::RTC() {
	RTC(RTCSEL_LSE);	// Best for STM32F1xx
}

RTC::RTC(rtc_clk_src src) {
	bkp_init();
	bkp_enable_writes();
	RCC_BASE->BDCR &= ~RCC_BDCR_RTCSEL;

	switch (src) {
	case RTCSEL_DEFAULT:
	case RTCSEL_LSI:
		rcc_start_lsi();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSI;
		break;
	case RTCSEL_LSE:
		rcc_start_lse();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSE;
		break;
	case RTCSEL_HSE:
		rcc_start_hse();
		RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_HSE;
		break;
	case RTCSEL_NONE:
		RCC_BASE->BDCR = RCC_BDCR_RTCSEL_NONE;
		break;
	}
	bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_RTCEN_BIT, 1); // Enable the RTC
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();

	switch (src) {
	case RTCSEL_DEFAULT:
	case RTCSEL_LSI:
		rtc_set_prescaler_load(39999);
		break;
	case RTCSEL_LSE:
		rtc_set_prescaler_load(32767);	// External clock 32.676 Hz
		break;
	case RTCSEL_HSE:
		rtc_set_prescaler_load(62499);
		break;
	case RTCSEL_NONE:
		break;
	}

}

/*
RTC::~RTC() {
//to implement
}
*/

void RTC::setTime(uint32_t time_stamp) {
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	rtc_enter_config_mode();
	RTCDEV->regs->CNTH = (time_stamp >> 16) & 0xffff;
	RTCDEV->regs->CNTL = time_stamp & 0xffff;
	rtc_exit_config_mode();
	rtc_wait_finished();

	DateTime.timestamp = time_stamp + DateTime.timezone * SECS_H;
	UpdateDT();
}

/**
* @brief Returns the rtc's counter (i.e. time/date) value.
*
* This value is likely to be inaccurate if the counter is running
* with a low prescaler.
*/
uint32_t RTC::getTime() {
	uint32_t h, l;
	rtc_clear_sync();
	rtc_wait_sync();
	rtc_wait_finished();
	h = RTCDEV->regs->CNTH & 0xffff;
	l = RTCDEV->regs->CNTL & 0xffff;
	return (h << 16) | l;
}

void RTC::createAlarm(voidFuncPtr function, uint32_t alarm_time) {
	rtc_set_alarm(alarm_time);
	rtc_attach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT, function);
}

void RTC::setAlarmTime(uint32_t alarm_time) {
	rtc_set_alarm(alarm_time);
}

void RTC::attachSecondsInterrupt(voidFuncPtr function) {
	rtc_attach_interrupt(RTC_SECONDS_INTERRUPT, function);
}

void RTC::detachSecondsInterrupt() {
	rtc_detach_interrupt(RTC_SECONDS_INTERRUPT);
}

void RTC::SetTZ(int8 tzoffset)
{
	DateTime.timezone = tzoffset * SECS_H;
	UpdateDT();
}

// TODO: Is it correct?
void RTC::UpdateDT() {
	lastTS = DateTime.timestamp;
	DateTime.timestamp = getTime() + DateTime.timezone * SECS_D;	// RTC in UTC format

	if (DateTime.timestamp != 0 && DateTime.timestamp == lastTS)
		return;		// No need to update DT, its already correct

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
	dateTime.timestamp = dateTime.second;
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

	setTime(dateTime.timestamp - DateTime.timezone * SECS_D);
}