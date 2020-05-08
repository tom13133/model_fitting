#include <types.hpp>

namespace model_fitting  {

double Distance(const Point& p1, const Point& p2)
{
  const double dx = p1.x() - p2.x();
  const double dy = p1.y() - p2.y();
  const double dz = p1.z() - p2.z();
  return sqrt(dx * dx + dy * dy + dz * dz);
}


}//namespace model_fitting
