/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Zachary Kingston */

#include "ompl/base/ConstrainedStateSpace.h"

#include "ompl/base/PlannerDataGraph.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

#include <boost/graph/iteration_macros.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/// ConstrainedMotionValidator

/// Public

ompl::base::ConstrainedMotionValidator::ConstrainedMotionValidator(SpaceInformation *si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ConstrainedStateSpace>())
{
    ConstrainedStateSpace::checkSpace(si);
}

ompl::base::ConstrainedMotionValidator::ConstrainedMotionValidator(const SpaceInformationPtr &si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ConstrainedStateSpace>())
{
    ConstrainedStateSpace::checkSpace(si.get());
}

bool ompl::base::ConstrainedMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    return ss_.traverseManifold(s1, s2);
}

bool ompl::base::ConstrainedMotionValidator::checkMotion(const State *s1, const State *s2,
                                                       std::pair<State *, double> &lastValid) const
{
    // Invoke the manifold-traversing algorithm to save intermediate states
    std::vector<ompl::base::State *> stateList;
    const ConstrainedStateSpace::StateType *const as1 = s1->as<ConstrainedStateSpace::StateType>();
    const ConstrainedStateSpace::StateType *const as2 = s2->as<ConstrainedStateSpace::StateType>();
    bool reached = ss_.traverseManifold(s1, s2, false, &stateList);

    // We are supposed to be able to assume that s1 is valid. However, it's not
    // on rare occasions, and I don't know why. This makes stateList empty.
    if (stateList.empty())
    {
        if (lastValid.first)
            ss_.copyState(lastValid.first, as1);
        lastValid.second = 0;
        return false;
    }

    double distanceTraveled = 0;
    for (std::size_t i = 0; i < stateList.size() - 1; i++)
    {
        if (!reached)
            distanceTraveled += ss_.distance(stateList[i], stateList[i + 1]);
        ss_.freeState(stateList[i]);
    }

    if (!reached && lastValid.first)
    {
        // Check if manifold traversal stopped early and set its final state as
        // lastValid.
        ss_.copyState(lastValid.first, stateList.back());
        // Compute the interpolation parameter of the last valid
        // state. (Although if you then interpolate, you probably won't get this
        // exact state back.)
        double approxDistanceRemaining = ss_.distance(lastValid.first, as2);
        lastValid.second = distanceTraveled / (distanceTraveled + approxDistanceRemaining);
    }

    ss_.freeState(stateList.back());
    return reached;
}

void ompl::base::ConstrainedStateSpace::setSpaceInformation(const SpaceInformationPtr &si)
{
    // Check that the object is valid
    if (!si.get())
        throw ompl::Exception("ompl::base::ProjectedStateSpace::setSpaceInformation(): "
                             "si is nullptr.");
    if (si->getStateSpace().get() != this)
        throw ompl::Exception("ompl::base::ProjectedStateSpace::setSpaceInformation(): "
                             "si for ProjectedStateSpace must be constructed from the same state space object.");

    // Save only a raw pointer to prevent a cycle
    si_ = si.get();
    si_->setStateValidityCheckingResolution(delta_);
}

void ompl::base::ConstrainedStateSpace::checkSpace(const SpaceInformation *si)
{
    if (!dynamic_cast<ConstrainedStateSpace *>(si->getStateSpace().get()))
        throw ompl::Exception("ompl::base::ConstrainedStateSpace(): "
                             "si needs to use an ProjectedStateSpace!");
}

void ompl::base::ConstrainedStateSpace::interpolate(const State *from, const State *to, const double t,
                                                  State *state) const
{
    // Get the list of intermediate states along the manifold.
    std::vector<State *> stateList;

    if (!traverseManifold(from, to, true, &stateList))
        stateList.push_back(si_->cloneState(to)->as<State>());

    piecewiseInterpolate(stateList, t, state);

    for (State *state : stateList)
        freeState(state);
}

unsigned int ompl::base::ConstrainedStateSpace::piecewiseInterpolate(const std::vector<State *> &stateList, const double t,
                                                           State *state) const
{
    std::size_t n = stateList.size();
    auto d = new double[n];

    // Compute partial sums of distances between intermediate states.
    d[0] = 0;
    for (std::size_t i = 1; i < n; i++)
        d[i] = d[i - 1] + distance(stateList[i - 1], stateList[i]);

    // Find the two adjacent states that t lies between.
    unsigned int i = 0;
    double tt;
    if (d[n - 1] == 0)
    {
        // Corner case where total distance is 0.
        i = n - 1;
        tt = t;
    }
    else
    {
        while (i < n - 1 && d[i] / d[n - 1] <= t)
            i++;
        tt = t - d[i - 1] / d[n - 1];
    }

    // Linearly interpolate between these two states.
    RealVectorStateSpace::interpolate(stateList[i > 0 ? i - 1 : 0], stateList[i], tt, state);
    delete[] d;

    return i;
}

void ompl::base::ConstrainedStateSpace::dumpGraph(const PlannerData::Graph &graph, std::ostream &out, const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;

    BGL_FORALL_EDGES(edge, graph, PlannerData::Graph)
    {
        std::vector<State *> stateList;
        const State *const source = boost::get(vertex_type, graph, boost::source(edge, graph))->getState();
        const State *const target = boost::get(vertex_type, graph, boost::target(edge, graph))->getState();

        if (!asIs)
            traverseManifold(source, target, true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            v << target->as<StateType>()->constVectorView().transpose() << "\n";
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount - 3 << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            for (State *state : stateList)
                freeState(state);
            continue;
        }
        StateType *to, *from = stateList[0]->as<StateType>();
        v << from->constVectorView().transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i]->as<StateType>();
            from = stateList[i - 1]->as<StateType>();
            v << to->constVectorView().transpose() << "\n";
            v << from->constVectorView().transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount - 3 : vcount - 4) << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            freeState(stateList[i - 1]);
            reset = false;
        }
        freeState(stateList.back());
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}

void ompl::base::ConstrainedStateSpace::dumpPath(ompl::geometric::PathGeometric &path, std::ostream &out,
                                           const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;

    const std::vector<State *> &waypoints = path.getStates();
    for (std::size_t i = 0; i < waypoints.size() - 1; i++)
    {
        std::vector<State *> stateList;
        const State *const source = waypoints[i];
        const State *const target = waypoints[i + 1];

        if (!asIs)
            traverseManifold(source, target, true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            v << target->as<StateType>()->constVectorView().transpose() << "\n";
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount - 3 << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            for (State *state : stateList)
                freeState(state);
            continue;
        }
        StateType *to, *from = stateList[0]->as<StateType>();
        v << from->constVectorView().transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i]->as<StateType>();
            from = stateList[i - 1]->as<StateType>();
            v << to->constVectorView().transpose() << "\n";
            v << from->constVectorView().transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount - 3 : vcount - 4) << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            freeState(stateList[i - 1]);
            reset = false;
        }
        freeState(stateList.back());
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}
