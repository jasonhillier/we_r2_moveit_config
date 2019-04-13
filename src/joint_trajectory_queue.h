
#include <ros/ros.h>

#include <std_msgs/UInt16.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#define DEFAULT_RESPONSE_TX_SIZE    5

class JointTrajectoryQueue
{
    public:
        JointTrajectoryQueue(ros::NodeHandle &n):
            _nh(n)
        {
            _pub_controller_command = _nh.advertise<trajectory_msgs::JointTrajectoryPoint>("command/point", false);
            _sub_queue_request = _nh.subscribe("command/request", 2, &JointTrajectoryQueue::queueRequestCb, this);
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
            trajectory_msgs::JointTrajectoryPoint empty;
            empty.time_from_start = ros::Duration(-1);
            _pub_controller_command.publish(empty);
        }

        void set(const trajectory_msgs::JointTrajectory& traj)
        {
            //reset point queue
            _queue.clear();

            add(traj);

            doSend(_tune_response_size);
        }
        
        void add(const trajectory_msgs::JointTrajectory& traj)
        {
            for(int i=0; i<traj.points.size(); i++)
                _queue.insert(_queue.begin(), traj.points[i]);

            ROS_INFO("[joint_trajectory_queue] added %i points to queue", (int)traj.points.size());
        }

        void queueRequestCb(const std_msgs::UInt16ConstPtr &msg)
        {
            //auto-adjust queue to max size client supports
            if (msg->data > _tune_response_size) _tune_response_size = msg->data;

            doSend(msg->data);
        }

        void doSend(uint16_t amount)
        {
            std::vector<trajectory_msgs::JointTrajectoryPoint> txQueue;

            for(int i=0; i<amount && !_queue.empty(); i++)
            {
                _pub_controller_command.publish( _queue.back() );
                _queue.pop_back();
            }

            ROS_INFO("[joint_trajectory_queue] buffer queue has %i points remaning", (int)_queue.size());
        }
        
    protected:
        uint16_t _tune_response_size=DEFAULT_RESPONSE_TX_SIZE;
        ros::NodeHandle _nh;
        ros::Publisher _pub_controller_command;
        ros::Subscriber _sub_queue_request;
        std::vector<trajectory_msgs::JointTrajectoryPoint> _queue;
};
