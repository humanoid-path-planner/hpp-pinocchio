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
# include <pinocchio/multibody/joint/joint.hpp>
# include <pinocchio/algorithm/jacobian.hpp>
# include <pinocchio/algorithm/frames.hpp>

# include <hpp/pinocchio/device.hh>
# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace pinocchio {
    namespace {
      void moveFrame (Model& model, GeomModel& geomModel, const FrameIndex& pF, const Transform3f& new_jMf)
      {
        se3::Frame& f = model.frames[pF];
        const Transform3f old_fMj = f.placement.inverse();
        for (GeomIndex i = 0; i < geomModel.geometryObjects.size(); ++i) {
          se3::GeometryObject& go = geomModel.geometryObjects[i];
          if (go.parentFrame == pF)
            go.placement = new_jMf * old_fMj * go.placement;
        }
        f.placement = new_jMf;
      }
    }

    Frame::Frame (DevicePtr_t device, FrameIndex indexInFrameList ) 
      :devicePtr_(device)
      ,frameIndex_(indexInFrameList)
    {
      assert (devicePtr_);
      assert (devicePtr_->modelPtr());
      assert (std::size_t(frameIndex_)<model().frames.size());
      setChildList();
    }

    inline void Frame::selfAssert() const 
    {
      assert(devicePtr_);
      assert(devicePtr_->modelPtr()); assert(devicePtr_->dataPtr());
      assert(devicePtr_->model().frames.size()>std::size_t(frameIndex_));
    }

    inline Model&        Frame::model()       { selfAssert(); return devicePtr_->model(); }
    inline const Model&  Frame::model() const { selfAssert(); return devicePtr_->model(); }
    inline Data &        Frame::data()        { selfAssert(); return devicePtr_->data (); }
    inline const Data &  Frame::data()  const { selfAssert(); return devicePtr_->data (); }
    const se3::Frame&  Frame::pinocchio() const { return model().frames[index()]; }
    inline se3::Frame& Frame::pinocchio()       { return model().frames[index()]; }

    Frame Frame::parentFrame () const
    {
      FrameIndex idParent = model().frames[frameIndex_].previousFrame;
      return Frame(devicePtr_,idParent);
    }

    bool Frame::isFixed () const
    {
      return pinocchio().type == se3::FIXED_JOINT;
    }

    Joint Frame::joint () const
    {
      // if (pinocchio().type == se3::FIXED_JOINT) {
        // HPP_THROW(std::logic_error, "Frame " << name() << " is fixed");
      // }
      return Joint(devicePtr_, pinocchio().parent);
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
      selfAssert();
      const Data & d = data();
      const se3::Frame f = model().frames[frameIndex_];
      if (f.type == se3::JOINT)
        return d.oMi[f.parent];
      else
        return d.oMi[f.parent] * f.placement;
    }

    JointJacobian_t Frame::jacobian (const bool local) const 
    {
      selfAssert();
      assert(robot()->computationFlag() & Device::JACOBIAN);
      JointJacobian_t jacobian = JointJacobian_t::Zero(6,model().nv);
      if(local) se3::getFrameJacobian<true> (model(),data(),frameIndex_,jacobian);
      else      se3::getFrameJacobian<false>(model(),data(),frameIndex_,jacobian);
      return jacobian;
    }

    void Frame::setChildList()
    {
      assert(devicePtr_->modelPtr()); assert(devicePtr_->dataPtr());
      children_.clear();
      if (!isFixed()) return;
      const Model& model = devicePtr_->model();

      std::vector<bool> visited (model.frames.size(), false);
      std::vector<bool> isChild (model.frames.size(), false);

      FrameIndex k = frameIndex_;
      while (k > 0) {
        visited[k] = true;
        k = model.frames[k].previousFrame;
      }
      visited[0] = true;

      for (FrameIndex i = model.frames.size() - 1; i > 0; --i) {
        if (visited[i]) continue;
        visited[i] = true;
        k = model.frames[i].previousFrame;
        while (model.frames[k].type != se3::JOINT) {
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
          k = model.frames[k].previousFrame;
        }
        if (k == frameIndex_)
          children_.push_back(i);
      }
    }

    /*
    std::size_t  Frame::numberChildFrames () const
    {
      return children_.size();
    }

    FramePtr_t  Frame::childFrame (std::size_t rank) const
    {
      selfAssert();
      assert(rank<children_.size());
      return Frame (devicePtr_, children_[rank]);
    }
    */

    Transform3f Frame::positionInParentFrame () const
    {
      selfAssert();
      const Model& m = model();
      const se3::Frame f = m.frames[index()];
      return m.frames[f.previousFrame].placement.inverse() 
        * ((f.type == se3::FIXED_JOINT) ? f.placement : m.jointPlacements[f.parent]);
    }

    void Frame::positionInParentFrame (const Transform3f& p)
    {
      selfAssert();
      devicePtr_->invalidate();
      Model& model = devicePtr_->model();
      GeomModel& geomModel = devicePtr_->geomModel();
      se3::Frame& me = pinocchio();
      bool isJoint = (me.type == se3::JOINT);
      Transform3f fMj = (isJoint ? model.jointPlacements[me.parent].inverse() : me.placement.inverse());
      if (isJoint)
        model.jointPlacements[me.parent] = model.frames[me.previousFrame].placement * p;
      else
        me.placement = model.frames[me.previousFrame].placement * p;

      std::vector<bool> visited (model.frames.size(), false);
      for (std::size_t i = 0; i < children_.size(); ++i) {
        FrameIndex k = children_[i];
        if (model.frames[k].type == se3::JOINT)
          k = model.frames[k].previousFrame;
        while (k != frameIndex_) {
          if (visited[k]) break;
          visited[k] = true;
          moveFrame (model, geomModel, k, me.placement * fMj * model.frames[k].placement);
          k = model.frames[k].previousFrame;
        }
      }

      // Update joint placements
      for (std::size_t i = 0; i < children_.size(); ++i) {
        FrameIndex k = children_[i];
        const se3::Frame f = model.frames[k];
        if (f.type == se3::JOINT) {
          model.jointPlacements[f.parent]
            = me.placement * fMj * model.jointPlacements[f.parent];
        }
      }
    }

    /*
    BodyPtr_t  Frame::linkedBody () const 
    {
      return BodyPtr_t( new Body(devicePtr_,frameIndex_) );
    }
    */

    std::ostream& Frame::display (std::ostream& os) const 
    {
      os
        << "Frame " << frameIndex_
        << (isFixed() ? " (Fixed)" : "")
        << " : " << name() << '\n';
      // for (unsigned int iChild=0; iChild < numberChildFrames (); iChild++)
      // {
      // os << "\"" << name () << "\"->\"" << childFrame(iChild)->name () << "\""
      // << std::endl;
      // }
      return os << std::endl;
    }

  } // namespace pinocchio
} // namespace hpp
