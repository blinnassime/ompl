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

#include <ompl/base/samplers/UniformValidStateSampler.h>
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

    // dimension of baxter robot
    dimension_ = 24;
    for (int i=0; i<dimension_; i++) q_user_.push_back(0);
    
    delay_ = 0;
    running_ = true;
    threads_.push_back(std::thread(&ompl::geometric::CIRRT::InteractiveThread, this, "thread argument"));

    //sleep(100);
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

    /* PUBLISH RANDOM ARM POSITIONS */
    ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1); 

    std::vector<double> positions(dimension_);
    //positions.clear(); 
    for (int i=0; i<dimension_; i++) positions[i]=0;

    double place_middle[14] = {-0.3776, 1.0466, -1.72227, 1.881, 0.4172, 1.4776, -0.1108,0.9139, 1.0262, 1.4381, 2.1246, -0.5097,1.2282, -0.0564};

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

        //cout << endl;

        /* receive configuration using zmq server previous version before yichen
           double buffer [10];
        //cout <<"waiting for receive" << endl;
        zmq_recv (responder, buffer, sizeof(double)*10, 0);
        //printf("\nrec:");
        //for(int i=0; i<10; i++) printf("%lf ", buffer[i]);
        //cout <<"waiting for send\n";
        zmq_send (responder, buffer, 0, 0);
        //cout <<"sent\n\n";


        //*/

        // current variable values 
        const double * var_values = kinematic_state->getVariablePositions();
        // dimension of the robot 
        std::size_t dimension = kinematic_model->getVariableCount();
        // names of the planning group 
        const std::vector< std::string>& group_name_vector_ref = joint_model_group->getJointModelNames();


        const std::vector< std::string> & name_vector_ref = kinematic_state->getVariableNames();
        var_values = kinematic_state->getVariablePositions();

        std::vector< std::string> nc_group_name_vector(group_name_vector_ref);
        unsigned long int length_name_vector_ref = group_name_vector_ref.size();

        nc_group_name_vector.erase(nc_group_name_vector.cbegin()+3);
        nc_group_name_vector.erase(nc_group_name_vector.cbegin()+5);




        // set configuration vector to incoming configuration
        // for pr2 left arm
        //for (int i=33; i<40; i++){
            //double val = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-33];
            //cout << " rand " << val;
            //positions[i] = buffer[i-33];
        //}

        //cout << "config:";
        //for (int i=0; i<14; i++) printf("%lf\t",config[i]);
        //cout << endl;
        //* set positions vector to incoming configuration, positions used for display
        // for baxter
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
        
        //* make position equal r_state_ = the configuration sent to the planner
        // means display the planner generated configuration
        q_u_mut_.lock();
        double val = 0;
        for (int i=8; i<15; i++){
            val = rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-8]; 
            positions[i] = val;
        }
        positions[15] = positions[16] = 0;
        for (int i=17; i<24; i++){
            val = rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-10]; 
            positions[i] = val;
        }
        q_u_mut_.unlock();

        //*
        //cout << "positions after copy 2" << endl;
        //for (int i=0; i<24; i++) cout << i << ":" << positions[i] << " ";
        //cout << endl; 
        usleep(delay_);
        //*/


        //q_u_mut_.lock();
        //for (int i=8; i<15; i++){
            //q_user_[i] = config[i-8];
        //}
        //for (int i=17; i<24; i++){
            //q_user_[i] = config[i-10];
        //}
        //q_u_mut_.unlock();
        //cout << "q_user_ before copy" << endl;
        //for (int i=0; i<24; i++) cout << i << ":" << q_user_[i] << " ";
        //cout << endl;
        q_u_mut_.lock();
        for (int i=0; i<14; i++){
            //if(config[i]!=0){
                //cout << "copying q_user_["<<i<<"] <- config["<<i<<"]i="<<config[i];
                q_user_[i] = config[i];
            //}
            //else {
                //q_user_[i] = place_middle[i];
                //cout << "copying q_user_["<<i<<"] <- place_middle["<<i<<"]i="<<place_middle[i];
            //}
            //cout << "   q_user_["<<i<<"]="<<q_user_[i]<<endl;
        }
        //q_user_ = positions; 
        q_u_mut_.unlock();
        //cout << "q_user_ after copy" << endl;
        //for (int i=0; i<24; i++) cout << i << ":" << q_user_[i] << " ";
        //cout << endl;
        //cout << endl;
        
        

        /*
        q_u_mut_.lock();
        cout << "q_user_ after copy" << endl;
        for (int i=0; i<24; i++) cout << i << ":" << q_user_[i] << " ";
        cout << endl; 
        q_u_mut_.unlock();
        //*/

        /*
        cout << "after recieve\n";
        cout << "incoming config:"<<endl;
        for (int k=0; k<46; k++)
            cout << config[k] << "\tcfg " << k <<":";
        cout << endl;
        cout << "buffer positions:"<<endl;
        for (int i=0; i<46; i++)
            cout << positions[i] << "\tpos " << i <<":";
        cout << endl;
        cout << "q_user_ vairable:"<<endl;
        for (int i=0; i<46; i++)
            cout << q_user_[i] << "$";
        cout << endl;
        //*/ 

        // set robot to configuration vector
        kinematic_state->setVariablePositions(positions);     
        //cout << 1 << endl; 
        // send the message to the RobotState display 
        moveit_msgs::DisplayRobotState msg;
        //cout << 2 << endl;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
        //cout << 3 << endl;
        robot_state_publisher.publish(msg);
        //cout << 4 << endl;
        ros::spinOnce();
        //usleep(10000);

        // print values
        //std::size_t dimension = kinematic_model->getVariableCount();
        //const double* var_values = kinematic_state->getVariablePositions();
            //for (unsigned int i = 0; i<dimension; i++) cout <<" pdim " << i << "=" << var_values[i];
            //cout << endl;
            //cout << endl;
        



        //set robot to random position
        if(0)
        for (int cnt = 0; cnt < 1 && ros::ok(); cnt++)
        {

            if (!running_) break;
            kinematic_state->setToRandomPositions(joint_model_group);


            /* get a robot state message describing the pose in kinematic_state */
            moveit_msgs::DisplayRobotState msg;
            robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

            /* send the message to the RobotState display */
            robot_state_publisher.publish(msg);

            /* let ROS send the message, then wait a while */
            ros::spinOnce();
            //loop_rate.sleep();

            //sleep(1);
            //usleep(100000);
        }

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

    const ompl::base::StateSpacePtr& ssptr = si_->getStateSpace(); 
    cout << "dim=" << ssptr->getDimension();


    //const ompl::base::ParamSet& param = ssptr->params();
    //cout << "param size="<<param.size()<<endl;
    //std::vector<std::string> names;
    //param.getParamNames(names);
    //for (int i=0; i<names.size(); i++) cout << names[i];
    //cout <<endl;

    static unsigned int iter = 0;
    while (!ptc)
    {
        //cout << "state dimension=" << rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->getDimension()<< endl;
        //ompl::base::RealVectorStateSpace::StateType* st_ptr = rstate_->as<ompl::base::RealVectorStateSpace::StateType>();
        //cout << "dimension=" << rstate_->getDimension() << endl;
         

        usleep(delay_);
        //double alpha = 1;
        //double alpha = 0.9;
        //double alpha = 0.5;
        //double alpha = 0.1;
        double alpha = 0.5;

        double r = rand();
        r = r / RAND_MAX;

        if (r<alpha){
        std::cout << "CI-RRT iteration " <<++iter<< " r=" << r<< "\trandom\n";
        }
        // always sample random to populate every joint with random values
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate_);
        }
        else
        {
            sampler_->sampleUniform(rstate_);
        }

        /* print configuration
        cout << "rstate_     =";
        for (int i=0; i<15; i++){
            double round = rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]; 
            cout << i<<"="<<round << "*";} cout <<endl;
        cout <<endl;
        q_u_mut_.lock(); 
        cout << "q_user_=";
        for (int i=0; i<dimension_; i++) cout << i<<"="<<q_user_[i]<<"*"; cout << endl;
        q_u_mut_.unlock();
        //*/
        
        bool valid = false;
        if(1)if (r>=alpha)
        {
            std::cout << "CI-RRT iteration " <<++iter<< " r=" << r<< "\thuman\n";
            // take human input
            // set configuration vector to incoming configuration for baxter right arm
            q_u_mut_.lock(); 
            //for (int i=8; i<15; i++){
                //if (q_user_[i]!=0)
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-8] = q_user_[i];
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17] = 0;
            //}
            //for (int i=17; i<24; i++){
                //if (q_user_[i]!=0)
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-9] = q_user_[i];
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17] = 0;
            //}
            for (int i=0; i<14; i++){
                //if (q_user_[i]!=0)
                rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = q_user_[i];
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0.001*iter*i;
            }
            //cout << "rstate_after=";
            //for (int i=0; i<15; i++){
                //double round = rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]; 
                //cout << i<<"="<<round << "*";} cout <<endl;
            //cout <<endl;
            q_u_mut_.unlock();
            ompl::base::UniformValidStateSampler validss (si_.get());
            ompl::base::State* valid_state = si_->allocState();
            bool success = false;
            //success = validss.sample(valid_state, rstate_, (const double) 0.05 );
            success = validss.sampleNear(valid_state, rstate_, 0.01 );
            cout << "sample valid state near rstate="<< success<<endl;
            rstate_ = valid_state;
        }
        
         



        valid = si_->isValid(rstate_);
        cout << "isValid="<<valid<<endl;



       /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate_;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate_);
        if (d > maxDistance_)
        {
            cout << "distance big ";
            si_->getStateSpace()->interpolate(nmotion->state, rstate_, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
        cout << "\t\t\t\tVALID " << endl;
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
            {
                cout << "4a " << endl;
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                cout << "4b " << endl;
                approxdif = dist;
                approxsol = motion;
            }
        }else cout << "not valid" << endl;
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        cout << "5a " << endl;
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        cout << "5b " << endl;
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

    cout << "freeing space..." << endl;

    si_->freeState(xstate);
        cout << "6 " << endl;
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

        cout << "7 " << endl;
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
