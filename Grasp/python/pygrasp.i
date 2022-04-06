%module pygrasp

%{
    #include <Grasp/GraspVars.hpp>
    #include <Grasp/GraspPlannerParams.hpp>
    #include <Grasp/GraspResult.hpp>

    #include <Grasp/GraspExecutor.hpp>
    #include <Grasp/TestGramacyExecutor.hpp>
    #include <Grasp/GraspPlanner.hpp>
%}

%include "std_string.i"
%include "std_vector.i"
namespace std {
    %template(vectord) vector<double>;
}

%include <eigen.i>

%eigen_typemaps(Eigen::Vector3f)
%eigen_typemaps(Eigen::Matrix3f)
%eigen_typemaps(Eigen::Matrix4f)
%eigen_typemaps(Eigen::MatrixXf)

%include "Grasp/GraspVars.hpp"
%include "Grasp/GraspResult.hpp"

namespace Grasp {
    class GraspExecutor {
    public:
        virtual GraspResult executeQueryGrasp(const std::vector<double>& query) = 0;
    };

    class TestGramacyExecutor : public GraspExecutor {
    public:
        
        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };

    struct GraspPlannerParams {
        std::string robot_file;
        std::string eef_name;
        std::string preshape;
        std::string object_file;

        bool eef_pose = false;
        Eigen::Vector3f eef_position, eef_orientation;

        bool obj_pose = false;
        Eigen::Vector3f obj_position, obj_orientation;

        float timeout;
        float min_quality;
        bool force_closure;

        GraspPlannerParams();

        GraspPlannerParams(
            const std::string& _robot_file,
            const std::string& _eef_name,
            const std::string& _preshape,
            const std::string& _object_file,
            const float _timeout, const float _min_quality, const bool _force_closure
        );
    };

    class GraspPlanner : public GraspExecutor {
    public:
        GraspPlanner(const GraspPlannerParams& params);

        GraspResult executeQueryGrasp(const std::vector<double>& query);
    };
}

%inline %{
    Eigen::Vector3f test_eigen_numpy_type() {
        return Eigen::Vector3f(1.1, 2.2, 3.3);
    }
%}
