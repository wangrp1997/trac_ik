 /* trac_ik_wrap.i */
 %module trac_ik_wrap

// Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>

 %{
 /* Includes the header in the wrapper code */
 #include "../../trac_ik_lib/include/trac_ik/trac_ik.hpp"
 #include <urdf/model.h>
 #include <kdl_parser/kdl_parser.hpp>
 #include <limits>
 #include <kdl/frames.hpp>
 %}

 // We need this or we will get on runtime
 // NotImplementedError: Wrong number or type of arguments for overloaded function
 %include <std_string.i>
 %include <std_vector.i>

// From http://stackoverflow.com/a/8752983
// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}

// USEFUL DOCS: http://www.swig.org/Doc1.3/SWIG.html


// Ignore original constructors as they are not useful in Python
// Note the full namespacing
%ignore TRAC_IK::TRAC_IK::TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime=0.005, double _eps=1e-5, SolveType _type=Speed);
%ignore TRAC_IK::TRAC_IK::TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param="/robot_description", double _maxtime=0.005, double _eps=1e-5, SolveType _type=Speed);

// Ignore the runKDL and runNLOPT methods as they fail to be wrapped (and are private anyways)
%ignore TRAC_IK::runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in);
%ignore TRAC_IK::runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in);

// Ignore other methods that we will wrap in a more usable way
%ignore TRAC_IK::getKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_);
%ignore TRAC_IK::setKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_);

// All variables will use const reference typemaps
// This eases dealing with std::vectors
%naturalvar;

// Include visibility_control.hpp first to ensure TRAC_IK_PUBLIC is defined
%include "../../trac_ik_lib/include/trac_ik/visibility_control.hpp"

// Parse the original header file to generate wrappers
%include "../../trac_ik_lib/include/trac_ik/trac_ik.hpp"

// Create a more Python friendly constructor
// Note: Exception handling is done inside %extend block to ensure all exceptions are caught
%extend TRAC_IK::TRAC_IK {
    // Based on trac_ik_kinematics_plugin.cpp implementation
    // As we can't access the private variables of the TRAC_IK class from this extension
    // this is the only way I found to make another constructor
    // thanks to: http://stackoverflow.com/questions/33564645/how-to-add-an-alternative-constructor-to-the-target-language-specifically-pytho
    TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& urdf_string,
      double timeout, double epsilon, const std::string& solve_type="Speed"){
      
      try {
        urdf::Model robot_model;

        robot_model.initString(urdf_string);

        // ROS2: Using RCLCPP_DEBUG instead of ROS_DEBUG_STREAM_NAMED
        // RCLCPP_DEBUG(rclcpp::get_logger("trac_ik"), "Reading joints and links from URDF");

        KDL::Tree tree;

        if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
          // ROS2: Using exception instead of ROS_FATAL
          throw std::runtime_error("Failed to extract kdl tree from xml robot description");
        }

        KDL::Chain chain;

        if(!tree.getChain(base_link, tip_link, chain)) {
          // ROS2: Using exception instead of ROS_FATAL
          throw std::runtime_error("Couldn't find chain " + base_link + " to " + tip_link);
        }

      uint num_joints_;
      num_joints_ = chain.getNrOfJoints();
      
      std::vector<KDL::Segment> chain_segs = chain.segments;

      urdf::JointConstSharedPtr joint;

      std::vector<double> l_bounds, u_bounds;

      KDL::JntArray joint_min, joint_max;

      joint_min.resize(num_joints_);
      joint_max.resize(num_joints_);

      std::vector<std::string> link_names_;
      std::vector<std::string> joint_names_;

      uint joint_num=0;
      for(unsigned int i = 0; i < chain_segs.size(); ++i) {

        link_names_.push_back(chain_segs[i].getName());
        joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
        // Defensive: getJoint() can return null; dereferencing would segfault.
        // If this KDL segment has no joint (fixed), it's safe to skip.
        // Otherwise, this is a URDF inconsistency -> throw a readable error.
        if (!joint) {
          if (chain_segs[i].getJoint().getType() == KDL::Joint::None) {
            continue;
          }
          throw std::runtime_error(std::string("URDF joint not found: ") + chain_segs[i].getJoint().getName());
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
          joint_num++;
          assert(joint_num<=num_joints_);
          float lower, upper;
          int hasLimits;
          joint_names_.push_back(joint->name);
          if ( joint->type != urdf::Joint::CONTINUOUS ) {
            // Some URDFs may omit limits; treat as error to avoid undefined behavior downstream.
            if (!joint->limits) {
              throw std::runtime_error(std::string("URDF joint has no limits: ") + joint->name);
            } else if(joint->safety) {
              lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
              upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
            } else {
              lower = joint->limits->lower;
              upper = joint->limits->upper;
            }
            hasLimits = 1;
          }
          else {
            hasLimits = 0;
          }
          if(hasLimits) {
            joint_min(joint_num-1)=lower;
            joint_max(joint_num-1)=upper;
          }
          else {
            joint_min(joint_num-1)=std::numeric_limits<float>::lowest();
            joint_max(joint_num-1)=std::numeric_limits<float>::max();
          }
          // ROS2: Commented out ROS_DEBUG_STREAM
          // RCLCPP_DEBUG(rclcpp::get_logger("trac_ik"), "IK Using joint " << chain_segs[i].getName() << " " << joint_min(joint_num-1) << " " << joint_max(joint_num-1));
        }
      }

      if (joint_num != num_joints_) {
        throw std::runtime_error(
          std::string("Joint limit extraction mismatch: expected ") +
          std::to_string(num_joints_) + " joints, got " + std::to_string(joint_num)
        );
      }


      TRAC_IK::SolveType solvetype;

      if (solve_type == "Manipulation1")
        solvetype = TRAC_IK::Manip1;
      else if (solve_type == "Manipulation2")
        solvetype = TRAC_IK::Manip2;
      else if (solve_type == "Distance")
        solvetype = TRAC_IK::Distance;
      else {
          if (solve_type != "Speed") {
              // ROS2: Commented out ROS_WARN_STREAM_NAMED
              // RCLCPP_WARN(rclcpp::get_logger("trac_ik"), solve_type << " is not a valid solve_type; setting to default: Speed");
          }
          solvetype = TRAC_IK::Speed;
      }
        // This constructor call may also throw exceptions, so it's inside the try block
        TRAC_IK::TRAC_IK* newX = new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, timeout, epsilon, solvetype);
        return newX;
      } catch (const std::runtime_error& e) {
        // Re-throw runtime_error - SWIG will convert it to Python RuntimeError
        throw;
      } catch (const std::exception& e) {
        // Convert other std::exception to runtime_error so SWIG can handle it
        throw std::runtime_error(std::string("TRAC_IK constructor failed: ") + e.what());
      } catch (...) {
        // Catch any other exception (including non-standard exceptions)
        throw std::runtime_error("TRAC_IK constructor failed with unknown exception");
      }
    }

    // original call:
    // int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());
        
    // note that as a comment here https://bitbucket.org/traclabs/trac_ik/issues/18/possible-bug-with-quaternions-in-carttojnt
    // explains, the pose is in reference to the base
    // of the chain... 
    std::vector<double> CartToJnt(const std::vector<double> q_init,
     const double x, const double y, const double z, 
     const double rx, const double ry, const double rz, const double rw, 
     // bounds x y z
     const double boundx=0.0, const double boundy=0.0, const double boundz=0.0,
     // bounds on rotation x y z
     const double boundrx=0.0, const double boundry=0.0, const double boundrz=0.0)
    {

      KDL::Frame frame;
      // ROS2: Directly construct KDL::Frame from position and quaternion
      KDL::Vector pos(x, y, z);
      KDL::Rotation rot = KDL::Rotation::Quaternion(rx, ry, rz, rw);
      frame = KDL::Frame(rot, pos);

      KDL::JntArray in(q_init.size()), out(q_init.size());

      for (uint z=0; z < q_init.size(); z++)
          in(z) = q_init[z];

      KDL::Twist bounds = KDL::Twist::Zero();
      bounds.vel.x(boundx);
      bounds.vel.y(boundy);
      bounds.vel.z(boundz);
      bounds.rot.x(boundrx);
      bounds.rot.y(boundry);
      bounds.rot.z(boundrz);

      int rc = $self->CartToJnt(in, frame, out, bounds);
      std::vector<double> vout;
      // If no solution, return empty vector which acts as None
      if (rc == -3)
          return vout;

      for (uint z=0; z < q_init.size(); z++)
          vout.push_back(out(z));

      return vout;
    }

    // Convenience method to check that calls to IK have the correct
    // number of qinit elements
    int getNrOfJointsInChain(){
      KDL::Chain chain;
      $self->getKDLChain(chain);
      return (int) chain.getNrOfJoints();
    }

    // Convenience method to get the list of joint names in the KDL chain (safe; no URDF parsing)
    // Note: urdf_string is ignored, kept only for API compatibility with existing Python wrapper.
    std::vector<std::string> getJointNamesInChain(const std::string& urdf_string){
      (void)urdf_string;
      KDL::Chain chain;
      $self->getKDLChain(chain);
      std::vector<KDL::Segment> chain_segs = chain.segments;
      std::vector<std::string> joint_names_;
      for(unsigned int i = 0; i < chain_segs.size(); ++i) {
        const KDL::Joint& j = chain_segs[i].getJoint();
        if (j.getType() != KDL::Joint::None) {
          joint_names_.push_back(j.getName());
        }
      }
      return joint_names_;
    }


    // Convenience method to get the list of link names as used internally
    std::vector<std::string> getLinkNamesInChain(){
      KDL::Chain chain;
      $self->getKDLChain(chain);
      std::vector<KDL::Segment> chain_segs = chain.segments;
      std::vector<std::string> link_names_;
      for(unsigned int i = 0; i < chain_segs.size(); ++i) {
        link_names_.push_back(chain_segs[i].getName());
      }
      return link_names_;
    }


    // Get KDL limits
    std::vector<double> getLowerBoundLimits(){
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      std::vector<double> lb;
      $self->getKDLLimits(lb_, ub_);
      for(unsigned int i=0; i < lb_.rows(); i++){
        lb.push_back(lb_(i));
      }
      return lb;
    }

    std::vector<double> getUpperBoundLimits(){
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      std::vector<double> ub;
      $self->getKDLLimits(lb_, ub_);
      for(unsigned int i=0; i < ub_.rows(); i++){
        ub.push_back(ub_(i));
      }
      return ub;
    }


    // Set KDL limits, Python takes care of checking number of limits
    void setKDLLimits(const std::vector<double> lb, const std::vector<double> ub) {
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      lb_.resize(lb.size());
      for(unsigned int i=0; i < lb.size(); i++){
        lb_(i) = lb[i];
      }
      ub_.resize(ub.size());
      for(unsigned int i=0; i < ub.size(); i++){
        ub_(i) = ub[i];
      }
      $self->setKDLLimits(lb_, ub_);
    }

};

