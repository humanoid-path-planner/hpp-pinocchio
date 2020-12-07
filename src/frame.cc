// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
// hpp-pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

# include <hpp/pinocchio/frame.hh>

# include <pinocchio/multibody/geometry.hpp>
# include <pinocchio/multibody/joint/fwd.hpp>
# include <pinocchio/algorithm/jacobian.hpp>
# include <pinocchio/algorithm/frames.hpp>

# include <hpp/pinocchio/device.hh>
# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/joint.hh>
# include <hpp/pinocchio/joint-collection.hh>

namespace hpp {
  namespace pinocchio {
    namespace {
      void moveFrame (Model& model, GeomModel& geomModel, const FrameIndex& pF, const Transform3f& new_jMf)
      {
        ::pinocchio::Frame& f = model.frames[pF];
        const Transform3f old_fMj = f.placement.inverse();
        for (GeomIndex i = 0; i < geomModel.geometryObjects.size(); ++i) {
          ::pinocchio::GeometryObject& go = geomModel.geometryObjects[i];
          if (go.parentFrame == pF)
            go.placement = new_jMf * old_fMj * go.placement;
        }
        f.placement = new_jMf;
      }
    }

    Frame::Frame (DeviceWkPtr_t device, FrameIndex indexInFrameList ) 
      :devicePtr_(device)
      ,frameIndex_(indexInFrameList)
    {
      assert (devicePtr_.lock());
      assert (devicePtr_.lock()->modelPtr());
      assert (std::size_t(frameIndex_)<model().frames.size());
    }

    void Frame::selfAssert() const
    {
      assert(devicePtr_.lock());
      assert(devicePtr_.lock()->modelPtr()); assert(devicePtr_.lock()->dataPtr());
      assert(devicePtr_.lock()->model().frames.size()>std::size_t(frameIndex_));
    }

    inline Model&        Frame::model()       { selfAssert(); return devicePtr_.lock()->model(); }
    inline const Model&  Frame::model() const { selfAssert(); return devicePtr_.lock()->model(); }
    const ::pinocchio::Frame&  Frame::pinocchio() const { return model().frames[index()]; }
    inline ::pinocchio::Frame& Frame::pinocchio()       { return model().frames[index()]; }

    DeviceData& Frame::data() const
    {
      return devicePtr_.lock()->d();
    }

    Frame Frame::parentFrame () const
    {
      FrameIndex idParent = model().frames[frameIndex_].previousFrame;
      return Frame(devicePtr_,idParent);
    }

    bool Frame::isFixed () const
    {
      return pinocchio().type != ::pinocchio::JOINT;
    }

    JointPtr_t Frame::joint () const
    {
      return Joint::create(devicePtr_, pinocchio().parent);
    }

    bool Frame::isRootFrame () const
    {
      return index() == 0;
    }

    const std::string&  Frame::name() const 
    {
      selfAssert();
      return pinocchio().name;
    }

    Transform3f Frame::currentTransformation () const 
    {
      return currentTransformation (data());
    }

    Transform3f Frame::currentTransformation (const DeviceData& d) const 
    {
      selfAssert();
      const ::pinocchio::Frame f = model().frames[frameIndex_];
      if (f.type == ::pinocchio::JOINT)
        return d.data_->oMi[f.parent];
      else
        return d.data_->oMi[f.parent] * f.placement;
    }

    JointJacobian_t Frame::jacobian (const DeviceData& d) const 
    {
      selfAssert();
      assert(robot()->computationFlag() & JACOBIAN);
      JointJacobian_t jacobian = JointJacobian_t::Zero(6,model().nv);
      ::pinocchio::getFrameJacobian(model(),*d.data_,frameIndex_,::pinocchio::LOCAL,jacobian);
      return jacobian;
    }

    void Frame::setChildList()
    {
      selfAssert();
      if (!children_.empty()) return;
      if (!isFixed()) return;
      const Model& m = model();

      std::vector<bool> visited (m.frames.size(), false);
      std::vector<bool> isChild (m.frames.size(), false);

      FrameIndex k = frameIndex_;
      while (k > 0) {
        visited[k] = true;
        k = m.frames[k].previousFrame;
      }
      visited[0] = true;

      for (FrameIndex i = m.frames.size() - 1; i > 0; --i) {
        if (visited[i]) continue;
        visited[i] = true;
        k = m.frames[i].previousFrame;
        while (m.frames[k].type != ::pinocchio::JOINT) {
          if (k == frameIndex_ || k == 0) break;
          // if (visited[k]) {
            // std::vector<FrameIndex>::iterator _k = 
              // std::find(children_.begin(), children_.end(), k);
            // if (_k != children_.end())
              // children_.erase(_k);
            // k = frameIndex_;
            // break;
          // }
          visited[k] = true;
          k = m.frames[k].previousFrame;
        }
        if (k == frameIndex_)
          children_.push_back(i);
      }
    }

    const Transform3f& Frame::positionInParentJoint () const
    {
      return pinocchio().placement;
    }

    Transform3f Frame::positionInParentFrame () const
    {
      selfAssert();
      const Model& m = model();
      const ::pinocchio::Frame f = m.frames[index()];
      return m.frames[f.previousFrame].placement.inverse() 
        * ((f.type == ::pinocchio::FIXED_JOINT) ? f.placement : m.jointPlacements[f.parent]);
    }

    void Frame::positionInParentFrame (const Transform3f& p)
    {
      selfAssert();
      setChildList();

      devicePtr_.lock()->invalidate();
      Model& m = model();
      GeomModel& geomModel = devicePtr_.lock()->geomModel();
      ::pinocchio::Frame& me = pinocchio();
      bool isJoint = (me.type == ::pinocchio::JOINT);
      Transform3f fMj = (isJoint ? m.jointPlacements[me.parent].inverse() : me.placement.inverse());
      if (isJoint)
        m.jointPlacements[me.parent] = m.frames[me.previousFrame].placement * p;
      else
        me.placement = m.frames[me.previousFrame].placement * p;

      std::vector<bool> visited (m.frames.size(), false);
      for (std::size_t i = 0; i < children_.size(); ++i) {
        FrameIndex k = children_[i];
        if (m.frames[k].type == ::pinocchio::JOINT)
          k = m.frames[k].previousFrame;
        while (k != frameIndex_) {
          if (visited[k]) break;
          visited[k] = true;
          moveFrame (m, geomModel, k, me.placement * fMj * m.frames[k].placement);
          k = m.frames[k].previousFrame;
        }
      }

      // Update joint placements
      for (std::size_t i = 0; i < children_.size(); ++i) {
        FrameIndex k = children_[i];
        const ::pinocchio::Frame f = m.frames[k];
        if (f.type == ::pinocchio::JOINT) {
          m.jointPlacements[f.parent]
            = me.placement * fMj * m.jointPlacements[f.parent];
        }
      }
    }

    std::ostream& Frame::display (std::ostream& os) const 
    {
      os
        << "Frame " << frameIndex_
        << (isFixed() ? " (Fixed)" : "")
        << " : " << name() << '\n';
      return os << std::endl;
    }

  } // namespace pinocchio
} // namespace hpp
