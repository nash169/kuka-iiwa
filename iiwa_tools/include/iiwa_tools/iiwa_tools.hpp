#include <ros/ros.h>
#include <iiwa_tools/jacobian.h>

namespace iiwa_tools
{
    class IiwaTools
    {
    public:
        IiwaTools();

        ~IiwaTools();

        bool GetJacobian(iiwa_tools::jacobian::Request& req, iiwa_tools::jacobian::Response& res);
    protected:
        void LoadParams();
        
        void InitRbdyn();

        ros::NodeHandle nh_;

        std::string robot_description_, 
                    jacobian_service_name;

        ros::ServiceServer jacobian_server_;

    };

} // namespace iiwa_tools