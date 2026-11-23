// Rate gate on receive time: keep a message only when at least 1/rate has passed since the last
// kept one. The next deadline advances by the period (not from "now") so the kept rate stays at
// the configured value instead of drifting lower with jitter; a long gap resets it.
// A message up to 20% of the period early is accepted: with input rate == output rate (e.g. a
// 2 Hz costmap gated at 2 Hz) receive jitter would otherwise drop every few messages. Because
// the deadline still advances by whole periods, the average kept rate does not exceed the rate.
#ifndef NAV_DATA_RECORDER__THROTTLE_HPP_
#define NAV_DATA_RECORDER__THROTTLE_HPP_

#include <cstdint>

namespace nav_data_recorder
{

class Throttle
{
public:
  explicit Throttle(double rate_hz = 0.0)
  : period_ns_(rate_hz > 0.0 ? static_cast<int64_t>(1e9 / rate_hz) : 0),
    early_ns_(period_ns_ / 5) {}

  bool enabled() const {return period_ns_ > 0;}

  bool accept(int64_t now_ns)
  {
    if (period_ns_ <= 0) {
      return true;
    }
    // first message, or the clock went back (by more than reordering/early acceptance explain)
    if (!started_ || now_ns < next_ns_ - 2 * period_ns_) {
      started_ = true;
      next_ns_ = now_ns + period_ns_;
      return true;
    }
    if (now_ns < next_ns_ - early_ns_) {
      return false;
    }
    next_ns_ += period_ns_;
    if (next_ns_ <= now_ns) {  // long gap: restart from now
      next_ns_ = now_ns + period_ns_;
    }
    return true;
  }

private:
  int64_t period_ns_;
  int64_t early_ns_;
  int64_t next_ns_{0};
  bool started_{false};
};

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__THROTTLE_HPP_
