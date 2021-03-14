#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <unistd.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string>


#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

// #include "counterAttack.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

namespace gazebo{
	

	class counterAttack : public WorldPlugin{

	public:

		// counterAttack : WorldPlugin(){}

		void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/){
			// Make sure the ROS node for Gazebo has already been initialized                                                                                    
		    if (!ros::isInitialized())
		    {
		      // ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
		      //   << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		      // return;

		    	int argc = 0;
				char **argv = NULL;
	  			ros::init(argc, argv, "counter_attack",ros::init_options::NoSigintHandler);
		    }

		    this->world = _world;
	    	// Create a new transport node
    		transport::NodePtr node(new transport::Node());
    		// Initialize the node with the world name
    		node->Init(_world->Name());
    		this->factoryPub = node->Advertise<msgs::Factory>("~/factory");
		    // this->phy = this->world->GetPhysicsEngine();
		    // Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("counter_attack"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<geometry_msgs::Twist>(
			      "/counterAttack",
			      1,
			      boost::bind(&counterAttack::OnRosMsg, this, _1),
			      ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&counterAttack::QueueThread, this));

			  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	          std::bind(&counterAttack::OnUpdate, this));
		}

		public: void OnUpdate(){
			if(this->x>0){
				for (auto model : this->world->Models())
		        {
		            std::string model_name = model->GetName();
		            if(this->model_id==-1){
		            	if (model_name.find("counter_sphere") != std::string::npos)
			            {
			                // std::cout<<"om";
			                ROS_WARN("FORCE APPLIED[X,Y,Z]=[%f,%f,%f]", this->x_axis_force, this->y_axis_force, this->z_axis_force);
		        			model->GetLink("link")->SetForce(ignition::math::Vector3d(this->x_axis_force, this->y_axis_force, this->z_axis_force));
		    				this->x = 0;
			            }
		            }
		            else{
		            	sprintf(this->str, "%d", this->model_id);
		            	char str1[100] = "counter_sphere_";
		            	std::string s;
		            	s = strcat(str1,str);
		            	// std::cout<<s;
		            	if (model_name.find(s) != std::string::npos)
			            {
			                // std::cout<<"om";
			                ROS_WARN("FORCE APPLIED[X,Y,Z]=[%f,%f,%f]", this->x_axis_force, this->y_axis_force, this->z_axis_force);
		        			model->GetLink("link")->SetForce(ignition::math::Vector3d(this->x_axis_force, this->y_axis_force, this->z_axis_force));
		    				this->x = 0;
			            }
		            }
		            
		          
		        }
			}
			
		}

		/// \brief Handle an incoming message from ROS
		public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr& _msg)
		{
			      // Create the message
			      msgs::Factory msg;

			      // Model file to load
			      msg.set_sdf_filename("model://counter_sphere");

			      // Pose to initialize the model to
			      msgs::Set(msg.mutable_pose(),
			          ignition::math::Pose3d(
			            ignition::math::Vector3d(_msg->linear.x,_msg->linear.y, _msg->linear.z),
			            ignition::math::Quaterniond(0, 0, 0)));
			      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

			      // Send the message
					this->factoryPub->Publish(msg);
					this->x = 1;
					this->model_id++;
					this->x_axis_force = _msg->angular.x;
					this->y_axis_force = _msg->angular.y;
					this->z_axis_force = _msg->angular.z;
		 //    }
			// printf("received message %f, model loaded\n",_msg->data);
		    
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
		  static const double timeout = 0.01;
		  while (this->rosNode->ok())
		  {
		    this->rosQueue.callAvailable(ros::WallDuration(timeout));
		  }
		}

	private:
		// Pointer to the model
    	// boost::shared_ptr<gazebo::physics::Model> model;
		/// \brief A node use for ROS transport
		std::unique_ptr<ros::NodeHandle> rosNode;
		/// \brief A ROS subscriber
		ros::Subscriber rosSub;
		/// \brief A ROS callbackqueue that helps process messages
		ros::CallbackQueue rosQueue;
		/// \brief A thread the keeps running the rosQueue
		std::thread rosQueueThread;
		// physics::PhysicsEnginePtr phy;
		physics::WorldPtr world;
		// Pointer to the update event connection
    	event::ConnectionPtr updateConnection;
    	// Create a new transport node
	    transport::PublisherPtr factoryPub;

	    int model_id = -2;

    	float x_axis_force = 1000.0;
    	float y_axis_force = 0.0;
    	float z_axis_force = 0.0;
    	char str[100];
		int x = 0;
	};


	
	GZ_REGISTER_WORLD_PLUGIN(counterAttack)

}