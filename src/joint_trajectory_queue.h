
#include <ros/ros.h>

#include <boost/string.h>
#include <std_msgs/UInt16.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

class JointTrajectoryQueue
{
    public:
        JointTrajectoryQueue(ros::NodeHandle &n):
            _nh(n)
        {
            _pub_controller_command = _nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);
            _sub_queue_request = _nh.subscribe("queue/request", 1, &JointTrajectoryQueue::queueRequestCb, this);
        }
        ~JointTrajectoryQueue()
        {
            shutdown();
        }

        void shutdown()
        {
            _pub_controller_command.shutdown();
            _sub_queue_request.shutdown();
        }

        void stop()
        {
            trajectory_msgs::JointTrajectory empty;
            //empty.joint_names = joint_names_;
            _pub_controller_command.publish(empty);
        }

        void set(const trajectory_msgs::JointTrajectory& traj)
        {
            //reset point queue
            _queue.clear();
            //reset joint names
            _joint_names.clear();
            for(int i=0; i<traj.joint_names.size(); i++)
            {
                _joint_names.push_back(traj.joint_names[i]);
            }

            add(traj);

            doSend(_tune_request_size);
        }
        
        void add(const trajectory_msgs::JointTrajectory& traj)
        {
            for(int i=0; i<traj.points.size(); i++)
                _queue.insert(_queue.begin(), traj.points[i]);
        }

        void queueRequestCb(const std_msgs::UInt16ConstPtr &msg)
        {
            //auto-adjust queue to max size client supports
            if (msg->data > _tune_request_size) _tune_request_size = msg->data;

            doSend(msg->data);
        }

        void doSend(uint16_t amount)
        {
            trajectory_msgs::JointTrajectory traj_msg;

            for(int i=0; i<amount && !_queue.empty(); i++)
            {
                trajectory_msgs::JointTrajectoryPoint point = _queue.back();
                _queue.pop_back();

                traj_msg.points.push_back(point);
            }

            _pub_controller_command.publish(traj_msg);
        }
        
    protected:
        uint16_t _tune_request_size=1;
        ros::NodeHandle _nh;
        ros::Publisher _pub_controller_command;
        ros::Subscriber _sub_queue_request;
        std::vector<string> _joint_names;
        std::vector<trajectory_msgs::JointTrajectoryPoint> _queue;
};
