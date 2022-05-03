// convert CollisionGeometryPtr back and forth between pinocchio and hpp
// ref. https://stackoverflow.com/a/71575543

#include <hpp/pinocchio/shared-ptr.hh>

namespace hpp {
namespace pinocchio {

std::shared_ptr<hpp::fcl::CollisionGeometry> as_std_shared_ptr(
    boost::shared_ptr<hpp::fcl::CollisionGeometry> bp) {
  if (!bp) return nullptr;
  // a std shared pointer to boost shared ptr.  Yes.
  auto pq = std::make_shared<boost::shared_ptr<hpp::fcl::CollisionGeometry>>(
      std::move(bp));
  // aliasing ctor.  Hide the double shared ptr.  Sneaky.
  return std::shared_ptr<hpp::fcl::CollisionGeometry>(pq, pq.get()->get());
}

boost::shared_ptr<hpp::fcl::CollisionGeometry> as_boost_shared_ptr(
    std::shared_ptr<hpp::fcl::CollisionGeometry> bp) {
  if (!bp) return nullptr;
  // a std shared pointer to boost shared ptr.  Yes.
  auto pq = boost::make_shared<std::shared_ptr<hpp::fcl::CollisionGeometry>>(
      std::move(bp));
  // aliasing ctor.  Hide the double shared ptr.  Sneaky.
  return boost::shared_ptr<hpp::fcl::CollisionGeometry>(pq, pq.get()->get());
}

}  // namespace pinocchio
}  // namespace hpp
