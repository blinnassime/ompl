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

#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"



#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <typeinfo>
#include <ompl/base/samplers/UniformValidStateSampler.h>
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
#include <boost/thread/mutex.hpp>
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
// static mutex
boost::mutex ompl::geometric::RRTConnect::mut_;
// global variables to move
ros::NodeHandle nh; 
robot_state::RobotState* rsptr;
ros::Publisher robot_state_publisher;
double a=1.1;
double q_user__[14];
double b=2.2;

ompl::geometric::RRTConnect::RRTConnect(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "RRTConnectIntermediate" : "RRTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTConnect::setRange, &RRTConnect::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;

    // initialize state display
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    rsptr = (new robot_state::RobotState(kinematic_model));
    robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1); 

    // not used right now
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_frame","/moveit_visual_markers"));

    // cirrt parameters
    delay_= 500000;
    running_ = true;
    dimension_ = 24;
    //for (int i=0; i<14; i++) q_user_[i]=0;
    threads_.push_back(std::thread(&ompl::geometric::RRTConnect::InteractiveThread, this, "thread argument"));
    sleep(0);
}


void ompl::geometric::RRTConnect::InteractiveThread(std::string msg){
    cout << "starting thread\n";
    void *context = zmq_ctx_new ();
    void *responder = zmq_socket (context, ZMQ_REP);
    int rc = zmq_bind (responder, "tcp://*:5555");
    assert (rc == 0);
    rc += 0;


    static const std::string PLANNING_GROUP = "both_arms";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    //Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = -0.4;
    box_pose.position.z = 1.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    // Show text in Rviz of status
    //visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    //visual_tools.trigger();
    // Sleep to allow MoveGroup to recieve and process the collision object message
    ros::Duration(1.0).sleep();

    const rviz_visual_tools::colors& color;
    const Eigen::Affine3d& tip_pose(1,1,1) ;


    if(1)
    while(running_)
    {
        cout << "thread running" <<endl;

        //* receive configuration using zmq server
        double config[46];
        memset (config, 0, 46*sizeof(double));
   
        if(1) 
        for (int j=0; j<15; j++)
        {
            //cout <<">"<< j<<"<"<<endl;
            char buffer [7];
            zmq_recv (responder, buffer, sizeof(char)*7, 0); 
            //printf("\nrec:");
            string value_s;//="";
            string start_s="start  ";
            for(int i=0; i<7; i++){
                //printf("%x ", buffer[i]);
                value_s.push_back(buffer[i]);
            }   
            //printf("%s \n", buffer);        

            /* //print comparison
            cout << "\ncomparison "; 
            for (int i=0; i<7; i++)
            { 
                printf("%X %X  ", start_s[i], value_s[i]);
            }cout <<endl;
            for (int i=0; i<value_s.length(); i++) printf("%X.",value_s[i]);
            cout << endl;
            for (int i=0; i<start_s.length(); i++) printf("%X.",start_s[i]);
            cout << endl;
            //*/ 
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
            zmq_send (responder, buffer, 0, 0); 
        }
        //cout << endl;
        //cout << "config:";
        //for (int i=0; i<14; i++) printf("%lf\t",config[i]);
        //cout << endl;

        //std::vector<double> config_v(dimension_);
        //for (int i=0; i<14; i++){
            //config_v[i] = config[i];
            //cout << "config_v["<<i<<"]="<<config_v[i]<<" ";
        //}
        //cout<<endl;
        //displayState(config_v); 

        //mut_.lock();
        //for (int i=8; i<15; i++){
            //q_user_[i] = config[i-8];
        //}
        //for (int i=17; i<24; i++){
            //q_user_[i] = config[i-10];
        //}
        //mut_.unlock();
        mut_.lock();
        //cout << "q_user_ before copy" << endl;
        //for (int i=0; i<14; i++) cout << i << ":" << q_user__[i] << " ";
        //cout << endl;
        for (int i=0; i<14; i++){
            //if(config[i]!=0){
                //cout << "copying q_user__["<<i<<"] <- config["<<i<<"]i="<<config[i];
                q_user__[i] = config[i];
            //}
            //else {
                //q_user_[i] = place_middle[i];
                //cout << "copying q_user_["<<i<<"] <- place_middle["<<i<<"]i="<<place_middle[i];
            //}
            //cout << "   q_user_["<<i<<"]="<<q_user_[i]<<endl;
        }
        //cout << endl;
        //cout << "q_user__ after copy" << endl;
        //for (int i=0; i<14; i++) cout << i << ":" << q_user__[i] << " ";
        //cout << endl;
        mut_.unlock();

        
        usleep(delay_);
    }
    cout << "ending interactive thread\n";
}



ompl::geometric::RRTConnect::~RRTConnect()
{
    freeMemory();
}

void ompl::geometric::RRTConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRTConnect::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RRTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::RRTConnect::GrowState ompl::geometric::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* check if we have moved at all */
        if (si_->distance(nmotion->state, tgi.xstate) < std::numeric_limits<double>::epsilon())
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false;
    }
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it
    // receives as argument is valid,
    // so we check that one first
    if (addIntermediateStates_)
    {
        std::vector<base::State *> states;
        const unsigned int count =
            1 + si_->distance(nmotion->state, dstate) / si_->getStateValidityCheckingResolution();
        ompl::base::State *nstate = nmotion->state;
        if (tgi.start)
            si_->getMotionStates(nstate, dstate, states, count, true, true);
        else
            si_->getStateValidityChecker()->isValid(dstate) &&
                si_->getMotionStates(dstate, nstate, states, count, true, true);
        if (states.empty())
            return TRAPPED;
        bool adv = si_->distance(states.back(), tgi.start ? dstate : nstate) <= 0.01;
        reach = reach && adv;
        si_->freeState(states[0]);
        Motion *motion;
        for (std::size_t i = 1; i < states.size(); i++)
        {
            if (adv)
            {
                /* create a motion */
                motion = new Motion;
                motion->state = states[i];
                motion->parent = nmotion;
                motion->root = nmotion->root;
                tgi.xmotion = motion;
                nmotion = motion;
                tree->add(motion);
            }
            else
                si_->freeState(states[i]);
        }
        if (reach)
            return REACHED;
        else if (adv)
            return ADVANCED;
        else
            return TRAPPED;
    }
    else
    {
        bool validMotion =
            tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                        si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

        if (validMotion)
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->root = nmotion->root;
            tgi.xmotion = motion;

            tree->add(motion);
            if (reach)
                return REACHED;
            else
                return ADVANCED;
        }
        else
            return TRAPPED;
    }
}

ompl::base::PlannerStatus ompl::geometric::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    auto *rmotion = new Motion(si_);
    rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    static unsigned int iter = 0;
    while (!ptc)
    {
        //mut_.lock(); 
        //cout << "first q_user_ in planner" << endl;
        //for (int i=0; i<14; i++){
            //cout << " qu "<<i<<"="<< q_user__[i] ;
        //}   
        //cout << endl;
        //mut_.unlock();

        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }


        ///////////////////////////////////////////////// I RRT //////////////////////
        usleep(delay_);
        /*mut_.lock();
        char c;
        cout << "input key"<<endl;
        cin >> c;
        mut_.unlock();//*/
        double alpha = 1.0;
        double r = rand();
        r = r / RAND_MAX;
        std::cout << "CI-RRT-Connect iteration " <<++iter<< " r=" << r;
        if (r<alpha){
            cout << "\trandom\n";

            /* sample random state */
            sampler_->sampleUniform(rstate);
            //for (int i=1; i<14; i++)
                //rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0;
        }
        if(0)if (r>=alpha)
        {
            cout << "\thuman\n";
            // take human input
            // set configuration vector to incoming configuration
            mut_.lock(); 
            //cout << "second q_user_ in planner" << endl;
            for (int i=0; i<14; i++){
                rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = q_user__[i];
                //cout << " qu "<<i<<"="<< q_user__[i] ;
            }   
            //cout << endl;
            mut_.unlock();
        
            
            //ompl::base::UniformValidStateSampler validss (si_.get());
            //ompl::base::State* valid_state = si_->allocState();
            //bool success = false;
            //success = validss.sampleNear(valid_state, rstate, 0.01 );
            //cout << "sample valid state near rstate="<< success<<endl;
            //rstate = valid_state;
        }
        /////////////////////////////////////////////////////////////////////////////////


        for (int i=0; i<7; i++)
            rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0;
        for (int i=8; i<14; i++)
            rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0;
        std::vector<double> config_v(dimension_);
        for (int i=0; i<14; i++){
            config_v[i] = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
            //cout << "config_v["<<i<<"]="<<config_v[i]<<" ";
        }
        cout<<endl;
        displayState(config_v); 

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED){
                cout << "REACHED :)" << endl;
                si_->copyState(rstate, tgi.xstate);
            }
            else cout << "NOT REACHED" << endl;
            
            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
        }
        else cout << "TRAPPED" << endl;
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = boost::lexical_cast<std::string>(distanceBetweenTrees_);
}



void ompl::geometric::RRTConnect::displayState(std::vector<double>& config){
    short int dimension = 14; 
    //cout << "dans fonction ";
    //for (int i=0; i<dimension; i++)
        //cout << "config["<<i<<"]="<<config[i]<<" ";
    //cout <<endl;
    std::vector<double> positions(40);
    for (int i=0; i<dimension; i++) positions[i]=0;
    //* set positions vector to incoming configuration, positions used for display
    // for baxter
    for (int i=0; i<8; i++) positions[i]=0;
    for (int i=8; i<15; i++){
        positions[i] = config[i-8];
    }
    positions[15] = positions[16] = 0;
    for (int i=17; i<24; i++){
        positions[i] = config[i-10];
    }
    //*/

    //cout << "positions after copy" << endl;
    //for (int i=0; i<24; i++) cout << i << ":" << positions[i] << " ";
    //cout << endl; 

    /* make position equal r_state_ = the configuration sent to the planner
    // means display the planner generated configuration
    mut_.lock();
    double val = 0;
    cout << "position from planner ";
    for (int i=8; i<15; i++){
    val = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-8]; 
    positions[i] = val;
    cout << " "<<i<<"="<<val;
    }
    positions[15] = positions[16] = 0;
    for (int i=17; i<24; i++){
    val = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-10]; 
    positions[i] = val;
    cout << " "<<i<<"="<<val;
    }
    mut_.unlock();

    //*
    cout << "positions after copy 2" << endl;
    for (int i=0; i<24; i++) cout << i << ":" << positions[i] << " ";
    cout << endl; 
    //*/

    // set robot to configuration vector
    //kinematic_state->setVariablePositions(positions);     
    rsptr->setVariablePositions(positions);     
    //cout << 1 << endl; 
    // send the message to the RobotState display 
    moveit_msgs::DisplayRobotState msg;
    //cout << 2 << endl;
    robot_state::robotStateToRobotStateMsg(*rsptr, msg.state);
    //cout << 3 << endl;
    robot_state_publisher.publish(msg);
    //cout << 4 << endl;
    ros::spinOnce();
    //usleep(10000);
}
