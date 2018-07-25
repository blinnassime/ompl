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

    dimension_ = 14;
    for (int i=0; i<dimension_; i++) q_user_.push_back(0);
    

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

    std::vector<double> positions(46);
    //positions.clear(); 
    for (int i=0; i<46; i++) positions[i]=0;

    // version zmq
    if(1)
    while(running_)
    {
        cout << "thread running" <<endl;


        //* receive configuration using zmq server
        double config[46];
        memset (config, 0, 46*sizeof(double));
       
    
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
        cout << endl;


        //for (int i=0; i<14; i++) printf("%lf\t",config[i]);
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


        //* read joint angles in function of names and such
        const std::vector< std::string> & name_vector_ref = kinematic_state->getVariableNames();
        var_values = kinematic_state->getVariablePositions();

        //const std::vector< std::string> & group_name_vector_ref = joint_model_group->getJointModelNames();
        std::vector< std::string> nc_group_name_vector(group_name_vector_ref);
        unsigned long int length_name_vector_ref = group_name_vector_ref.size();

        //cout << "dimension "<<dimension<< " length vector " << length_name_vector_ref << endl;
        //for (int i = 0; i<dimension; i++) cout <<" dim " << i << "=" << var_values[i];
        //cout << endl;
        //cout <<"variables names ";
        //for (int i = 0; i<dimension; i++) cout << i << " " << name_vector_ref[i] << " ";
        //cout << endl;
        //cout <<"group variables names ";
        //for (int i = 0; i<length_name_vector_ref; i++) cout << i << " " << group_name_vector_ref[i] << " ";
        //cout << endl;
        //cout <<"nc group variables names avant" << endl;
        //for (int i = 0; i<nc_group_name_vector.size(); i++)
            //cout << nc_group_name_vector[i] << " ";
        //cout << endl;
        nc_group_name_vector.erase(nc_group_name_vector.cbegin()+3);
        nc_group_name_vector.erase(nc_group_name_vector.cbegin()+5);
        //cout <<"nc group variables names après " << endl;
        //for (int i = 0; i<nc_group_name_vector.size(); i++)
            //cout << nc_group_name_vector[i] << " ";
        //cout << endl;
        /*
        cout <<"group variables positions ";
        for (int i = 0; i<nc_group_name_vector.size(); i++) 
            cout << kinematic_state->getVariablePosition(nc_group_name_vector[i])<< " ";
        cout << endl;
        cout <<"variables positions ";
        for (int i = 0; i<length_name_vector_ref; i++) 
            cout << kinematic_state->getVariablePosition(name_vector_ref[i])<< " ";
        cout << endl;
        //*/


        //set robot to random position
        //kinematic_state->setToRandomPositions(joint_model_group);
        //cout << "random positions\n";
        //for (unsigned int i = 0; i<dimension; i++) cout <<" rdim " << i << "=" << var_values[i];
            //cout << endl;
            //cout << endl;


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
        for (int i=8; i<15; i++){
            positions[i] = config[i-8];
        }
        positions[15] = positions[16] = 0;
        for (int i=17; i<24; i++){
            positions[i] = config[i-10];
        }
        //cout << "après copie" << endl;
        //for (int i=0; i<24; i++) cout << i << " " << positions[i] << " ";
        //cout << endl; 
        
        q_u_mut_.lock();
        q_user_ = positions; 
        q_u_mut_.unlock();
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
        /*
        q_u_mut_.lock();
        for (int i=0; i<dimension_; i++)
            q_user_[i] = 0;
        q_u_mut_.unlock();
        //*/

        // set robot to configuration vector
        kinematic_state->setVariablePositions(positions);     
        
        // send the message to the RobotState display 
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
        robot_state_publisher.publish(msg);
        ros::spinOnce();
        //usleep(10000);

        // print values
        //std::size_t dimension = kinematic_model->getVariableCount();
        //const double* var_values = kinematic_state->getVariablePositions();
            //for (unsigned int i = 0; i<dimension; i++) cout <<" pdim " << i << "=" << var_values[i];
            //cout << endl;
            //cout << endl;
        




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
            usleep(100000);
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

    static unsigned int iter = 0;
    while (!ptc)
    {
        double alpha = 1;
        //double alpha = 0.5;
        //double alpha = 0.9;

        double r = rand();
        r = r / RAND_MAX;

        std::cout << "CI-RRT iteration " <<++iter<< " r=" << r;
        if (r<alpha){
            cout << "\trandom\n";
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
            cout << "\thuman\n";
            // set configuration vector to incoming configuration for baxter right arm
            q_u_mut_.lock(); 
            for (int i=17; i<24; i++){
                
                rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17] = q_user_[i];
                //rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-17] = 0;
            }
            q_u_mut_.unlock();
        }

        /* print configuration
        cout << "q_user_=";
        for (int i=0; i<q_user_.size(); i++) printf("%lf\t", q_user_[i]); cout << endl;
        cout << "rstate_=";
        for (int i=0; i<46; i++){
            double round = rstate_->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
            round = round < 0.000001 ? 0 : round;
            cout<< round << " ";
        }
        cout <<endl;
        cout <<endl;
        //*/



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



// this was in the main thread after random sampling
        //std::vector<double> positions(46);
        //for (int i=33; i<40; i++){
            //double val = rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i-33];
            //cout << " rand " << val;
            //positions[i] = val;
        //}
        //cout << endl;
        //cout << endl;
    
        //sleep(5);

        //for (int i=0; i<10; i++)
            //rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0;
        //for (int i=0; i<10; i++) cout << " " <<
        //rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
        //cout << endl;
       


        /*
           if (r<alpha){
        //q_u_mut_.lock();
        for (int i=0; i<dimension_; i++)
        rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]
        //= q_user_[i];
        = 0;
        //q_u_mut_.unlock();
        }
        //*/
        /*
           std::cout << "valeurs\n";
           for (int i=0; i<10; i++){
           std::cout <<
           rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]
           << " ";
           }
           std::cout << "\n";
        //*/
        /*
           if (r<alpha){
           rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0;
           }
           std::cout << "valeurs 2\n";
           for (int i=0; i<10; i++){
           std::cout <<
           rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]
           << " ";
           }
           std::cout << "\n";
        //*/
        //usleep(100000);
        //
        //
        //
        //
 

// ancien constructeur
/*
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

    sleep(100);
    cout << "FIN CONSTRUCTEUR\n";

}
*
*
*/


// tentatives d'affichage obstacles

    /* mesh visu ok
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.mesh_resource = "file:///home/nblin/ws_moveit/hole_small.stl";
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0.32;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    sleep(1);
    vis_pub.publish( marker );
    //*/
   




    /* collision mesh affiche trop tard
    // The :move_group_interface:`MoveGroup` class can be easily 
    //   // setup using just the name
    //   // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroup group("left_arm");
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // (Optional) Create a publisher for visualizing plans in Rviz.
    //ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory; 
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    moveit_msgs::CollisionObject toilet;
    Eigen::Vector3d scale(0.5,0.5,0.5);
    shapes::Mesh* m = shapes::createMeshFromResource("file:///home/nblin/ws_moveit/hole_small.dae", scale);
    shape_msgs::Mesh toilet_mesh;
    shapes::ShapeMsg toilet_mesh_msg;
    shapes::constructMsgFromShape(m,toilet_mesh_msg);
    toilet_mesh = boost::get<shape_msgs::Mesh>(toilet_mesh_msg);
    toilet.meshes.resize(1);
    toilet.meshes[0] = toilet_mesh;
    toilet.mesh_poses.resize(1);
    toilet.mesh_poses[0].position.x = 1.7;
    toilet.mesh_poses[0].position.y = -1;
    toilet.mesh_poses[0].position.z = 0.3+0.05;
    toilet.mesh_poses[0].orientation.w= 0.921;
    toilet.mesh_poses[0].orientation.x= 0.0;
    toilet.mesh_poses[0].orientation.y= 0.0;
    toilet.mesh_poses[0].orientation.z= -0.389;
    //pub_co.publish(toilet);

    toilet.meshes.push_back(toilet_mesh);
    toilet.mesh_poses.push_back(toilet.mesh_poses[0]);
    toilet.operation = toilet.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(toilet);  
    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");  
    planning_scene_interface.addCollisionObjects(collision_objects);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(toilet);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    sleep(2.0);

    ros::ServiceClient planning_scene_diff_client =
        nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);




    ros::spinOnce();
    sleep(2.0);
    //group.setStartState(*group.getCurrentState());
    //group.setPoseTarget(target_pose1);
    //*/



    /* collision box affiche trop tard
    //ros::Publisher coll_pub = nh.advertise<moveit_msgs::CollisionObject>( "visualization_marker", 0 );
    //ros::Publisher coll_pub = nh.advertise<moveit_msgs::CollisionObject>( "/move_group/monitored_planning_scene", 0 );
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
    moveit::planning_interface::MoveGroup group("right_arm");
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.8;

    // A pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.6;
    box_pose.position.y = 0.4;
    box_pose.position.z =  1.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(collision_object); 
    ROS_INFO("Add an object into the world");  
    planning_scene_interface.addCollisionObjects(collision_objects);
    //coll_pub.publish(collision_object);
    
    //arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req;
    //arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res;
    //set_planning_scene_diff.call(set_planning_scene_diff_req, set_planning_scene_diff_res);
    
    // Sleep so we have time to see the object in RViz
    sleep(2.0);
    //*/


    /*
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "myns";
    //marker.id = k+current_cone_pos.left.x.size()+current_cone_pos.right.x.size()+1;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = "file://models/v1-01/model.dae";
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.b = 0.0f;
    marker.color.g = 0.0f;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    visualizer_.publish(marker);  
    */  
    /*
    ros::Publisher add_collision_object_pub;
    add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
    sleep(1.0); // To make sure the node can publish  
    moveit_msgs::CollisionObject co;    
    shapes::Mesh* m = shapes::createMeshFromResource("file:///home/nblin/ws_moveit/hole.dae"); 
    ROS_INFO("mesh loaded");
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;  
    shapes::constructMsgFromShape(m, co_mesh_msg);    
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);  
    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = co_mesh;
    co.header.frame_id = "mainulator";
    co.id = "ring";     

    geometry_msgs::Pose pose;
    co.mesh_poses[0].position.x = 0.75;
    co.mesh_poses[0].position.y = 0.0;
    co.mesh_poses[0].position.z = 0.525;
    co.mesh_poses[0].orientation.w= 1.0;
    co.mesh_poses[0].orientation.x= 0.0;
    co.mesh_poses[0].orientation.y= 0.0;
    co.mesh_poses[0].orientation.z= 0.0;   

    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;

    add_collision_object_pub.publish(co);
    ROS_INFO("Collision object published");
    sleep(2);
    //*/

    
    /* librairie manquante
    ros::Publisher object_in_map_pub_;
    object_in_map_pub_  = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 10);

    ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server

    //add the cylinder into the collision space
    mapping_msgs::CollisionObject cylinder_object;
    cylinder_object.id = "pole";
    cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    cylinder_object.header.frame_id = "base_link";
    cylinder_object.header.stamp = ros::Time::now();
    geometric_shapes_msgs::Shape object;
    object.type = geometric_shapes_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = .1;
    object.dimensions[1] = .75;
    geometry_msgs::Pose pose;
    pose.position.x = .6;
    pose.position.y = -.6;
    pose.position.z = .375;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    cylinder_object.shapes.push_back(object);
    cylinder_object.poses.push_back(pose);

    cylinder_object.id = "pole";
    object_in_map_pub_.publish(cylinder_object);

    ROS_INFO("Should have published");

    ros::Duration(2.0).sleep();
    // moveit_msgs::PlanningScene
    //*/



        /* read joint angles in function of names and such
        const std::vector< std::string> & name_vector_ref = kinematic_state->getVariableNames();
        var_values = kinematic_state->getVariablePositions();

        const std::vector< std::string> & group_name_vector_ref = joint_model_group->getJointModelNames();
        std::vector< std::string> nc_group_name_vector(group_name_vector_ref);
        unsigned long int length_name_vector_ref = group_name_vector_ref.size();

        cout << "dimension "<<dimension<< " length vector " << length_name_vector_ref << endl;
        for (int i = 0; i<dimension; i++) cout <<" dim " << i << "=" << var_values[i];
        cout << endl;
        cout <<"variables names ";
        for (int i = 0; i<dimension; i++) cout << name_vector_ref[i] << " ";
        cout << endl;
        cout <<"group variables names ";
        for (int i = 0; i<length_name_vector_ref; i++) cout << group_name_vector_ref[i] << " ";
        cout << endl;
        cout <<"nc group variables names avant" << endl;
        for (int i = 0; i<nc_group_name_vector.size(); i++)
            cout << nc_group_name_vector[i] << " ";
        cout << endl;
        nc_group_name_vector.erase(nc_group_name_vector.cbegin()+3);
        nc_group_name_vector.erase(nc_group_name_vector.cbegin()+5);
        cout <<"nc group variables names après " << endl;
        for (int i = 0; i<nc_group_name_vector.size(); i++)
            cout << nc_group_name_vector[i] << " ";
        cout << endl;
        cout <<"group variables positions ";
        for (int i = 0; i<nc_group_name_vector.size(); i++) 
            cout << kinematic_state->getVariablePosition(nc_group_name_vector[i])<< " ";
        cout << endl;
        cout <<"variables positions ";
        for (int i = 0; i<length_name_vector_ref; i++) 
            cout << kinematic_state->getVariablePosition(name_vector_ref[i])<< " ";
        cout << endl;
        //*/
