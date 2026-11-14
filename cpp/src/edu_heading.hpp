// Logical heading of the educational turtle, independent of translational drift.
#ifndef HANPPIE_EDU_HEADING_HPP
#define HANPPIE_EDU_HEADING_HPP

#include "hanppie/ctrl.hpp"

namespace hanppie {
namespace edu {
namespace detail {

class HeadingTarget {
public:
    explicit HeadingTarget(double reference = 0) : reference_(reference), offset_(0) {}
    double next(double clockwiseDegrees) const {
        return ctrl::angleDifference(offset_ + clockwiseDegrees, 0);
    }
    double correction(double nextOffset, double nativeYaw, int polarity) const {
        return ctrl::angleDifference(reference_ + polarity * nextOffset, nativeYaw) * polarity;
    }
    void commit(double nextOffset) { offset_ = nextOffset; }

private:
    double reference_;
    double offset_;
};

}  // namespace detail
}  // namespace edu
}  // namespace hanppie
#endif
