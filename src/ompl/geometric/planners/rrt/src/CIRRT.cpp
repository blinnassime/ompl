/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/rrt/CIRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <ompl/base/SpaceInformation.h>
#include "ompl/tools/config/SelfConfig.h"
#include <limits>


#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <typeinfo>

#include <zmq.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#include <Eigen/Core>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
//
// PI
#include <boost/math/constants/constants.hpp>
//
#include <thread>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

using namespace std;

//robot_model::RobotModelPtr kinematic_model;
//robot_state::RobotStatePtr kinematic_state;
//const robot_model::JointModelGroup *joint_model_group;
//ros::NodeHandle nh; 
//ros::Publisher robot_state_publisher;
 

ompl::geometric::CIRRT::CIRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "CIRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CIRRT::setRange, &CIRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &CIRRT::setGoalBias, &CIRRT::getGoalBias, "0.:.05:1.");

    dimension_ = 10;

    //for (int i=0; i<dimension_; i++) q_user_.push_back(0);
    

    running_ = true;
    threads_.push_back(std::thread(&ompl::geometric::CIRRT::InteractiveThread, this, "thread argument"));

    sleep(30);
    cout << "END CONSTRUCTOR\n";

}


void ompl::geometric::CIRRT::InteractiveThread(std::string msg){
    cout << "starting thread\n";
    void *context = zmq_ctx_new ();
    void *responder = zmq_socket (context, ZMQ_REP);
    int rc = zmq_bind (responder, "tcp://*:5555");
    assert (rc == 0);
    rc += 0;

    ros::NodeHandle nh; 

    ////////////////////////INTERACTIVE SHOW

    /* Load the robot model */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    /* Get a shared pointer to the model */
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    /* Get the configuration for the joints in the right arm of the PR2*/
    const robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("right_arm");

    // Advertise the state publisher
    ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1); 

    std::vector<double> positions(46);
    positions.clear(); 
    //for (int i=0; i<46; i++) positions[i]=0;

    // version zmq
    if(1)
    while(running_)
    {


        //* receive configuration using zmq server
        double config[14];
        for (int j=0; j<15; j++)
        {
            cout << j<<" ";
            char buffer [7];
            zmq_recv (responder, buffer, sizeof(char)*7, 0); 
            //printf("\nrec:");
            string value_s="";
            string start_s="start  ";
            for(int i=0; i<7; i++){
                //printf("%x ", buffer[i]);
                value_s.push_back(buffer[i]);
            }   
            //printf("%s \n", buffer);        
            //value_s.push_back('\0');

            //cout << "\ncomparaison "; 
            //for (int i=0; i<8; i++)
            //{ 
                //printf("%X %X  ", start_s[i], value_s[i]);
            //}cout <<endl; 
            double value = 0;  
            if(value_s==start_s)
            {   
                //cout << "\nstart value\n";  
            }   
            else {
                //cout << "not start value\n";  
                value = std::stof(value_s);
                config[j-1] = value;
                //cout << "\t" << value; 
            }   
            //sleep (1);          //  Do some 'work'
            zmq_send (responder, buffer, 0, 0); 
        }
        printf("\nrec:");
        for (int j=0; j<14; j++)
            cout << config[j] << "\t";
        cout << endl; 

        
       



        //*
        // set configuration vector to incoming configuration
        //std::vector<double> positions(46);
        // for pr2 left arm
        //for (int i=33; i<40; i++){
            //double val = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-33];
            //cout << " rand " << val;
            //positions[i] = buffer[i-33];
        //}

        // set configuration vector to incoming configuration
        // for baxter
        for (int i=17; i<24; i++){
            //double val = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17];
            //cout << " rand " << val;
            positions[i] = config[i-17];
        }
        
        q_u_mut_.lock();
        q_user_ = positions; 
        q_u_mut_.unlock();


        // set robot to configuration vector
        kinematic_state->setVariablePositions(positions);     
        
        // send the message to the RobotState display 
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
        robot_state_publisher.publish(msg);
        ros::spinOnce();
        usleep(10000);

        





        ////////////////////////INTERACTIVE SHOW

    }
    cout << "ending interactive thread\n";
}


ompl::geometric::CIRRT::~CIRRT()
{
    cout << "destructor\n";
    running_ = false;
    //threads_[0].join();
    freeMemory();
}

void ompl::geometric::CIRRT::clear()
{
    cout << "CIRRT::clear()\n";
    //running_ = false;
    //threads_[0].join();
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::CIRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

void ompl::geometric::CIRRT::freeMemory()
{

    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::CIRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    rstate_ = rmotion->state;
    base::State *xstate = si_->allocState();

    static unsigned int iter = 0;
    while (!ptc)
    {
        //double alpha = 1;
        double alpha = 0.5;

        double r = rand();
        r = r / RAND_MAX;

        std::cout << "CI-RRT iteration " <<++iter<< " r=" << r;
        if (r<alpha){
            cout << " random\n";
            /* sample random state (with goal biasing) */
            if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            {
                goal_s->sampleGoal(rstate_);
            }
            else
            {
                sampler_->sampleUniform(rstate_);
            }

        }
        else
        {
            // take human input
            cout << " human\n";
            // set configuration vector to incoming configuration for baxter right arm
            q_u_mut_.lock(); 
            for (int i=17; i<24; i++){
                rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17] = q_user_[i];
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17] = 0;
            }
            q_u_mut_.unlock();
        }

        cout << "q_user_=";
        for (int i=0; i<q_user_.size(); i++) cout << q_user_[i] << " "; cout << endl;
        cout << "rstate_=";
        for (int i=0; i<46; i++){
            double round = rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
            round = round < 0.000001 ? 0 : round;
            cout<< round << " ";
        }
        cout <<endl;
        cout <<endl;



       /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate_;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate_);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate_, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
        running_ = false;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::CIRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
