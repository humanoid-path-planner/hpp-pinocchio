#ifndef HPP_PINOCCHIO_SHARED_PTR_HH
#define HPP_PINOCCHIO_SHARED_PTR_HH

#include <hpp/fcl/fwd.hh>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>

// include boost after pinocchio
#include <boost/make_shared.hpp>

namespace hpp {
namespace pinocchio {

std::shared_ptr<hpp::fcl::CollisionGeometry> as_std_shared_ptr(
    boost::shared_ptr<hpp::fcl::CollisionGeometry> bp);

boost::shared_ptr<hpp::fcl::CollisionGeometry> as_boost_shared_ptr(
    std::shared_ptr<hpp::fcl::CollisionGeometry> bp);

}  // namespace pinocchio
}  // namespace hpp
#endif  // HPP_PINOCCHIO_UTIL_HH
