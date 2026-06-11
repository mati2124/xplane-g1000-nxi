#include "avionics/NavDatabase.h"

#include <cstdio>
#include <ctime>

namespace avionics {
namespace {

// AIRAC cycles run on a continuous 28-day cadence; cycle numbers reset to 01
// at the first effective date of each calendar year. Cycle 1501 was effective
// 08-JAN-2015, which anchors the whole calendar.
constexpr int kAiracCycleDays = 28;
constexpr int kEpochYear = 2015;
constexpr int kEpochMonth = 1;
constexpr int kEpochDay = 8;

constexpr const char* kMonthNames[] = {"JAN", "FEB", "MAR", "APR",
                                       "MAY", "JUN", "JUL", "AUG",
                                       "SEP", "OCT", "NOV", "DEC"};

// Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's
// civil-days algorithm), so cycle arithmetic is plain integer math.
long daysFromCivil(int y, int m, int d) {
  y -= m <= 2;
  const long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<long>(doe) - 719468;
}

void civilFromDays(long z, int& y, int& m, int& d) {
  z += 719468;
  const long era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const long yr = static_cast<long>(yoe) + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  d = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
  m = static_cast<int>(mp + (mp < 10 ? 3 : -9));
  y = static_cast<int>(yr + (m <= 2));
}

long airacEpochDays() {
  return daysFromCivil(kEpochYear, kEpochMonth, kEpochDay);
}

long floorDiv(long a, long b) { return (a >= 0 ? a : a - b + 1) / b; }

// Step index (28-day steps from the epoch) of the first AIRAC effective date
// in `year`, i.e. that year's cycle 01.
long firstStepOfYear(int year) {
  const long diff = daysFromCivil(year, 1, 1) - airacEpochDays();
  // Smallest n with epoch + 28n >= Jan 1 of `year` (ceiling division).
  return floorDiv(diff + kAiracCycleDays - 1, kAiracCycleDays);
}

long todayDaysUtc() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &now);
#else
  gmtime_r(&now, &utc);
#endif
  return daysFromCivil(utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday);
}

std::string formatDate(long days) {
  int y = 0;
  int m = 0;
  int d = 0;
  civilFromDays(days, y, m, d);
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%02d-%s-%02d", d, kMonthNames[m - 1],
                y % 100);
  return buf;
}

}  // namespace

NavDatabaseInfo navDatabaseInfoForCycle(int cycleYYCC) {
  NavDatabaseInfo info;
  const int yy = cycleYYCC / 100;
  const int cc = cycleYYCC % 100;
  // A year holds at most 14 28-day cycles; reject anything outside that.
  if (cycleYYCC < 100 || cc < 1 || cc > 14) return info;

  const int year = 2000 + yy;
  const long effectiveDays =
      airacEpochDays() + (firstStepOfYear(year) + cc - 1) * kAiracCycleDays;
  const long expiresDays = effectiveDays + kAiracCycleDays;

  info.available = true;
  info.expired = todayDaysUtc() >= expiresDays;
  char cycleBuf[8];
  std::snprintf(cycleBuf, sizeof(cycleBuf), "%02d%02d", yy, cc);
  info.cycle = cycleBuf;
  info.effective = formatDate(effectiveDays);
  info.expires = formatDate(expiresDays);
  return info;
}

int currentAiracCycle() {
  const long step =
      floorDiv(todayDaysUtc() - airacEpochDays(), kAiracCycleDays);
  const long stepDays = airacEpochDays() + step * kAiracCycleDays;
  int y = 0;
  int m = 0;
  int d = 0;
  civilFromDays(stepDays, y, m, d);
  const int cc = static_cast<int>(step - firstStepOfYear(y)) + 1;
  return (y - 2000) * 100 + cc;
}

}  // namespace avionics
