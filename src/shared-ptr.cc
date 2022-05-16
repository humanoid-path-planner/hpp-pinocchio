// convert CollisionGeometryPtr back and forth between pinocchio and hpp
// ref. https://stackoverflow.com/a/71575543

#include <hpp/pinocchio/shared-ptr.hh>

namespace hpp {
namespace pinocchio {

using std_ptr = std::shared_ptr<hpp::fcl::CollisionGeometry>;
using boost_ptr = boost::shared_ptr<hpp::fcl::CollisionGeometry>;

std_ptr as_std_shared_ptr(boost_ptr ptr) {
  if (!ptr) return nullptr;
  auto pq = std::make_shared<boost_ptr>(std::move(ptr));
  return std_ptr(pq, pq.get()->get());
}

boost_ptr as_boost_shared_ptr(std_ptr ptr) {
  if (!ptr) return nullptr;
  auto pq = boost::make_shared<std_ptr>(std::move(ptr));
  return boost_ptr(pq, pq.get()->get());
}

}  // namespace pinocchio
}  // namespace hpp
