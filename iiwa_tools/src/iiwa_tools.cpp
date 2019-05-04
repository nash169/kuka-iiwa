#include <iiwa_tools/iiwa_tools.hpp>

namespace iiwa_tools
{
    IiwaTools::IiwaTools()
    {
        ROS_INFO_STREAM("Starting Iiwa IK server..");
        LoadParams();
        InitRbdyn();

        jacobian_server_ = nh_.advertiseService(_jacobian_service_name, &IiwaTools::get_jacobian, this);
        ROS_INFO_STREAM("Started Iiwa Jacobian server..");


    }
    
    IiwaTools::~IiwaTools()
    {
    }

    bool IiwaTools::GetJacobian(iiwa_tools::jacobian::Request& req, iiwa_tools::jacobian::Response& res)
    {
        if (req.joint_angles.size() != _rbd_indices.size() || req.joint_angles.size() != req.joint_velocities.size()) {
            ROS_ERROR_STREAM("The requested joint size is not the same as the robot's size or some field is missing!");
            return false;
        }

        // copy RBDyn for thread-safety
        // TO-DO: Check if it takes time
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        // Get joint positions and velocities
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i];
            double pos = req.joint_angles[i];
            // wrap in [-pi,pi]
            pos = wrap_angle(pos);

            // The velocity should not be necessary for the computation of the Jacobian
            double vel = req.joint_velocities[i];

            rbdyn_urdf.mbc.q[rbd_index][0] = pos;
            rbdyn_urdf.mbc.alpha[rbd_index][0] = vel;
        }

        // Compute jacobian
        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

        // TO-DO: Check if we need this
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        Eigen::MatrixXd jac_mat = jac.jacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        // Fill response
        res.jacobian.layout.dim.resize(2);
        res.jacobian.layout.data_offset = 0;
        res.jacobian.layout.dim[0].size = jac_mat.rows();
        res.jacobian.layout.dim[0].stride = jac_mat.cols();
        res.jacobian.layout.dim[1].size = jac_mat.cols();
        res.jacobian.layout.dim[1].stride = 0;
        res.jacobian.data.resize(jac_mat.rows() * jac_mat.cols());

        for (size_t i = 0; i < jac_mat.rows(); i++) {
            for (size_t j = 0; j < jac_mat.cols(); j++) {
                set_multi_array(res.jacobian, i, j, jac_mat(i, j));
            }
        }

        return true;
    }

    void IiwaTools::LoadParams()
    {
        ros::NodeHandle n_p("~");

        n_p.param<std::string>("tools/robot_description", robot_description_, "/robot_description");
        n_p.param<std::string>("tools/jacobian_service_name", jacobian_service_name_, "iiwa_jacobian_server");
    }

    void IiwaTools::InitRbdyn()
    {

    }

    double get_multi_array(const std_msgs::Float64MultiArray& array, size_t i, size_t j)
    {
        assert(array.layout.dim.size() == 2);
        size_t offset = array.layout.data_offset;

        return array.data[offset + i * array.layout.dim[0].stride + j];
    }

    void set_multi_array(std_msgs::Float64MultiArray& array, size_t i, size_t j, double val)
    {
        assert(array.layout.dim.size() == 2);
        size_t offset = array.layout.data_offset;

        array.data[offset + i * array.layout.dim[0].stride + j] = val;
    }
}