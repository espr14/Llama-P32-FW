#include "PrusaGcodeSuite.hpp"
#include "M45.hpp"

void PrusaGcodeSuite::M45() {
    /// home XY if needed
    /// home Z at safe position (get reference height)
    /// cycle over 9 points
    /// go to point
    /// scan 32x32 array
    /// calculate center
    /// calculate XY skews
    /// pick median
    /// set XY skew
}
