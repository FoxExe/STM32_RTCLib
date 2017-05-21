/**
 * @file   RTClock.h
 * @author Rod Gilchrist <rod@visibleassets.com>, Allester Fox <fox.axon@yandex.ru>
 * @brief  Real Time Clock Class implementation.
 *
 */

#include <utility/rtc_util.h>

#ifndef _RTCLOCK_H_
#define _RTCLOCK_H_

// Added by Fox
#define LEAPYEAR(year)		(!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)		(LEAPYEAR(year) ? 366 : 365)
#define UNIX_START_YEAR		1970
#define SECS_M	60
#define SECS_H	3600
#define SECS_D	86400
#define SECS_W	604800


class RTC {
private:
	int8 timezone = 0;
	uint32_t lastTS = 0;
	const uint8_t mlen[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

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
	//~RTClock(); //to implement

	void setTime(uint32_t time_stamp);
	uint32_t getTime();

	void createAlarm(voidFuncPtr function, uint32_t alarm_time_t);
	void setAlarmTime(uint32_t alarm_time);

	void attachSecondsInterrupt(voidFuncPtr function);
	void detachSecondsInterrupt();

	void SetTS(uint32_t timeStamp);
	uint32_t GetTS();
	void SetTZ(int8 tzoffset);
	void UpdateDT();
	void SetTSFromDT(dt dateTime);

};

#endif // _RTCLOCK_H_
                                                                                        