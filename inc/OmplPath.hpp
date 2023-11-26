#include <Eigen/Dense>

#include <ompl/base/Path.h>
#include <ompl/base/State.h>
#include <ompl/util/ClassForward.h>

OMPL_CLASS_FORWARD(OmplPath);

class OmplPath {
  public:
    explicit OmplPath(const ompl::base::PathPtr& path);
    ~OmplPath();

    /*\brief Get configuration along path at position s in [0,1] */
    Eigen::VectorXd GetConfigAt(float s);

    float GetLength() const;

  private:
    ompl::base::PathPtr path_;
    ompl::base::State* tmpState_;
    std::vector<float> lengths_;
    float total_length_;
};
