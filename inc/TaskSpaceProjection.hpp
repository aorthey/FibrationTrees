#include <dart/dart.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>

class ProjectionJointSpaceToR3 : public ompl::multilevel::Projection
{
public:
    ProjectionJointSpaceToR3(const ompl::base::StateSpacePtr& bundle, const ompl::base::StateSpacePtr& base, const dart::dynamics::SkeletonPtr& skeleton)
      : ompl::multilevel::Projection(bundle, base), skeleton_(skeleton)
    {
        type_ = ompl::multilevel::PROJECTION_TASK_SPACE;
    }

    void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
    {
        double *xBundleValues = xBundle->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        Eigen::VectorXd config = Eigen::VectorXd::Zero(getBundle()->getDimension());
        OMPL_INFORM("Bundle: ", getBundle()->getDimension());
        for(size_t dim = 0; dim < getBundle()->getDimension(); dim++)
        {
          config[dim] = xBundleValues[dim];
        }
        skeleton_->setConfiguration(config);

        auto endeffector = skeleton_->getBodyNode(skeleton_->getNumBodyNodes() - 1)->getName();
        const auto& frame = skeleton_->getBodyNode(endeffector)->getTransform().translation();

        double *angles = xBase->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        angles[0] = frame[0];
        angles[1] = frame[1];
        angles[2] = frame[2];

        OMPL_INFORM("Project state");
        getBundle()->printState(xBundle);
        getBase()->printState(xBase);
    }

    void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
    {
        double *xBaseValues = xBase->as<ompl::base::RealVectorStateSpace::StateType>()->values;

        Eigen::Vector3d frame;
        frame[0] = xBaseValues[0];
        frame[1] = xBaseValues[1];
        frame[2] = xBaseValues[2];

        auto endeffector = skeleton_->getBodyNode(skeleton_->getNumBodyNodes() - 1)->getName();

        const auto& old_translation = skeleton_->getBodyNode(endeffector)->getTransform().translation();

        if(skeleton_->getBodyNode(endeffector)==nullptr) 
        {
          std::cout << "[ERROR] Could not get body " << endeffector << " in " << skeleton_->getName() << std::endl;
          return;
        }

        auto config = GetRandomPosition(skeleton_);
        skeleton_->setConfiguration(config);

        auto ik = skeleton_->getBodyNode(endeffector)->getOrCreateIK();
        ik->getTarget()->setTranslation(frame);

        if(ik->solveAndApply(config, true)) 
        {
          std::cout << "Found ik solution" << std::endl;
        }

        // const auto& current_translation = skeleton_->getBodyNode(endeffector)->getTransform().translation();
        // std::cout << "Old translation     : " << old_translation << std::endl;
        // std::cout << "Desired Translation : " << translation << std::endl;
        // std::cout << "New Translation     : " << current_translation << std::endl;
        // std::cout << "IK Error            : " << (translation - current_translation).norm() << std::endl;

        auto result = ik->getPositions();
        double *angles = xBundle->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (uint k = 0; k < getBundle()->getDimension(); k++)
        {
            angles[k] = result[k];
        }
        OMPL_INFORM("Lift state");
        getBase()->printState(xBase);
        getBundle()->printState(xBundle);
    }

private:
  dart::dynamics::SkeletonPtr skeleton_;
};
