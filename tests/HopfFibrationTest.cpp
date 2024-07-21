#include <gtest/gtest.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/base/spaces/special/SphereStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include "projections/HopfFibration.hpp"
#include "validators/HopfFibrationMotionValidator.hpp"
#include "State.hpp"
#include "Common.hpp"

ompl::base::State* MakeQuaternion(const ompl::base::StateSpacePtr& space, double x, double y, double z, double w) {
  auto goal_state = space->allocState();
  auto goal = goal_state->as<ompl::base::SO3StateSpace::StateType>();
  auto norm = std::sqrt(x * x + y * y + z * z + w * w);
  goal->x = x/norm;
  goal->y = y/norm;
  goal->z = z/norm;
  goal->w = w/norm;
  return goal;
}

class HopfFibrationTest : public testing::Test {
 protected:
  HopfFibrationTest()
  {
    total_space = std::make_shared<ompl::base::SO3StateSpace>();
    base_space = std::make_shared<ompl::base::SphereStateSpace>();

    factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(total_space);
    child = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(base_space);

    hopf_fibration = std::make_shared<HopfFibration>(factor, child);
    factor->addChild(child, hopf_fibration);
    hopf_motion_validator = std::make_shared<HopfFibrationMotionValidator>(factor, hopf_fibration);
    factor->setMotionValidator(hopf_motion_validator);

    factor->setup();

  }
  std::shared_ptr<ompl::base::SO3StateSpace> total_space;
  std::shared_ptr<ompl::base::SphereStateSpace> base_space;

  std::shared_ptr<ompl::multilevel::FactoredSpaceInformation> factor;
  std::shared_ptr<ompl::multilevel::FactoredSpaceInformation> child;
  std::shared_ptr<HopfFibration> hopf_fibration;
  std::shared_ptr<HopfFibrationMotionValidator> hopf_motion_validator;

};

TEST_F(HopfFibrationTest, SO3Interpolation) {
  auto start = MakeQuaternion(total_space, 0.47, 0.81, -0.33, 0.06);
  auto goal = MakeQuaternion(total_space, -0.39, -0.80, 0.02, 0.45);

  std::cout << "Desired Goal :" << hopf_fibration->toVector(goal) << std::endl;
  total_space->printState(goal, std::cout);
  int nd = total_space->validSegmentCount(start, goal);
  EXPECT_GT(nd, 1);

  auto result = total_space->allocState();

  auto q1 = Eigen::Quaternion(0.06, 0.47, 0.81, -0.33);
  auto q2 = Eigen::Quaternion(0.45, -0.39, -0.80, 0.02);

  for (int i = 1; i <= nd; i++)
  {
    auto s = (double)i / (double)nd;
    total_space->interpolate(start, goal, s, result);
    //Eigen::Quaternionf 
    auto q3 = q1.slerp(s, q2);

    auto so3_result = result->as<ompl::base::SO3StateSpace::StateType>();
    EXPECT_NEAR(so3_result->w, q3.w(), 1e-2);
    EXPECT_NEAR(so3_result->x, q3.x(), 1e-2);
    EXPECT_NEAR(so3_result->y, q3.y(), 1e-2);
    EXPECT_NEAR(so3_result->z, q3.z(), 1e-2);
  }
  total_space->interpolate(start, goal, 1.0, result);

  std::cout << "Interpolated Goal: " << hopf_fibration->toVector(result) << std::endl;
  total_space->printState(result, std::cout);

  EXPECT_NEAR(total_space->distance(goal, result), 0.0, Epsilon);
  EXPECT_TRUE(total_space->equalStates(goal, result));

  total_space->freeState(result);
  total_space->freeState(start);
  total_space->freeState(goal);
}

TEST_F(HopfFibrationTest, FiberProjectionTest) {
  auto new_state = total_space->allocState();
  auto base_state = base_space->allocState();
  auto fiber_space = hopf_fibration->getFiberSpace();
  auto fiber_state = fiber_space->allocState();

  auto state = MakeQuaternion(total_space, 0.337715, -0.52596, 0.656841, -0.421753);
  hopf_fibration->projectFiber(state, fiber_state);
  EXPECT_NEAR(fiber_state->as<ompl::base::SO2StateSpace::StateType>()->value, 0.895566, 1e-6);
  hopf_fibration->project(state, base_state);
  hopf_fibration->lift(base_state, fiber_state, new_state);
  EXPECT_NEAR(total_space->distance(state, new_state), 0.0, 1e-6);

  total_space->freeState(state);
  total_space->freeState(new_state);
  base_space->freeState(base_state);
  fiber_space->freeState(fiber_state);
}

TEST_F(HopfFibrationTest, HopfPropagationStaysInsideEdgeRestriction) {
  auto start_s3 = MakeQuaternion(total_space, 0.337715, -0.52596, 0.656841, -0.421753);
  auto goal_s3 = MakeQuaternion(total_space, 0.07731, 0.120403, -0.832812, -0.534743);

  auto start_s2 = base_space->allocState();
  auto goal_s2 = base_space->allocState();
  auto tmp_base_state = base_space->allocState();

  hopf_fibration->project(start_s3, start_s2);
  hopf_fibration->project(goal_s3, goal_s2);

  child->printState(start_s2);
  EXPECT_NEAR(start_s2->as<ompl::base::SphereStateSpace::StateType>()->getPhi(), 1.57, 1e-2);
  EXPECT_NEAR(goal_s2->as<ompl::base::SphereStateSpace::StateType>()->getPhi(), 1.57, 1e-2);
  child->printState(goal_s2);

  auto interpolated_states = hopf_motion_validator->propagateMotion(start_s3, goal_s3);
  EXPECT_GT(interpolated_states.size(), 2u);

  for(const auto& state : interpolated_states) {
    hopf_fibration->project(state, tmp_base_state);
    EXPECT_NEAR(tmp_base_state->as<ompl::base::SphereStateSpace::StateType>()->getPhi(), 1.57, 1e-2);
  }

  base_space->freeState(tmp_base_state);
  base_space->freeState(start_s2);
  base_space->freeState(goal_s2);
  total_space->freeState(start_s3);
  total_space->freeState(goal_s3);
}

TEST_F(HopfFibrationTest, ContinuousStereographicProjection) {
  auto start_s3 = MakeQuaternion(total_space, 0.337715, -0.52596, 0.656841, -0.421753);
  auto goal_s3 = MakeQuaternion(total_space, 0.07731, 0.120403, -0.832812, -0.534743);

  auto interpolated_states = hopf_motion_validator->propagateMotion(start_s3, goal_s3);
  EXPECT_GT(interpolated_states.size(), 2u);

  for(size_t k = 1; k < interpolated_states.size(); k++) {
    auto s1 = interpolated_states.at(k-1);
    auto s2 = interpolated_states.at(k);

    auto v1 = hopf_fibration->toVector(s1);
    auto v2 = hopf_fibration->toVector(s2);

    auto d = (v2 - v1).norm();
    EXPECT_LT(d, kStereographicDistanceThreshold);
  }
  total_space->freeState(start_s3);
  total_space->freeState(goal_s3);
}

TEST_F(HopfFibrationTest, RepairedStereographicProjection) {
  auto start_s3 = MakeQuaternion(total_space, 0.308412 , -0.480323,  -0.690915,  0.443631);
  auto goal_s3 = MakeQuaternion(total_space, -0.0264851,  0.12706 , -0.886483 , -0.444187);

  auto interpolated_states = hopf_motion_validator->propagateMotion(start_s3, goal_s3);
  EXPECT_GT(interpolated_states.size(), 2u);

  for(size_t k = 1; k < interpolated_states.size(); k++) {
    auto s1 = interpolated_states.at(k-1);
    auto s2 = interpolated_states.at(k);

    auto v1 = hopf_fibration->toVector(s1);
    auto v2 = hopf_fibration->toVector(s2);

    auto d = (v2 - v1).norm();
    EXPECT_LT(d, kStereographicDistanceThreshold);
  }
  total_space->freeState(start_s3);
  total_space->freeState(goal_s3);
}

TEST_F(HopfFibrationTest, RepairedStereographicProjection2) {
  auto start_s3 = MakeQuaternion(total_space, 0.539193, -0.839744, 0.0538855, -0.0345995);

  auto goal_s3 = MakeQuaternion(total_space,  0.544485 , 0.740865, 0.386016, 0.0751439);

  auto interpolated_states = hopf_motion_validator->propagateMotion(start_s3, goal_s3);
  EXPECT_GT(interpolated_states.size(), 2u);

  for(size_t k = 1; k < interpolated_states.size(); k++) {
    auto s1 = interpolated_states.at(k-1);
    auto s2 = interpolated_states.at(k);

    auto v1 = hopf_fibration->toVector(s1);
    auto v2 = hopf_fibration->toVector(s2);

    auto d = (v2 - v1).norm();
    EXPECT_LT(d, kStereographicDistanceThreshold);
  }
  total_space->freeState(start_s3);
  total_space->freeState(goal_s3);
}


TEST_F(HopfFibrationTest, PathStaysOnPathRestriction) {

  std::vector<ompl::base::State*> states;
  states.push_back(MakeQuaternion(total_space, 0.0148385,-0.0231096,0.841154,-0.540099));
  states.push_back(MakeQuaternion(total_space, 0.0171937,-0.00452338,0.853199,-0.521282));
  states.push_back(MakeQuaternion(total_space, 0.0193171,0.0157657,0.864295,-0.502366));
  states.push_back(MakeQuaternion(total_space, 0.0212334,0.0379395,0.87431,-0.483417));
  states.push_back(MakeQuaternion(total_space, 0.0229454,0.0622366,0.883081,-0.464507));
  states.push_back(MakeQuaternion(total_space, 0.0244584,0.0889151,0.89041,-0.445719));
  states.push_back(MakeQuaternion(total_space, 0.0257798,0.118249,0.896049,-0.427141));
  states.push_back(MakeQuaternion(total_space, 0.0269195,0.150523,0.899688,-0.408876));
  states.push_back(MakeQuaternion(total_space, 0.0278909,0.186016,0.90095,-0.391037));
  states.push_back(MakeQuaternion(total_space, 0.0287107,0.224988,0.899369,-0.373752));
  states.push_back(MakeQuaternion(total_space, 0.0293995,0.267648,0.894391,-0.357163));
  states.push_back(MakeQuaternion(total_space, 0.0299827,0.314111,0.88536,-0.341428));
  states.push_back(MakeQuaternion(total_space, 0.0304905,0.364346,0.871535,-0.326723));
  states.push_back(MakeQuaternion(total_space, 0.0309586,0.41811,0.85212,-0.313236));
  states.push_back(MakeQuaternion(total_space, 0.0312448,0.451582,0.837583,-0.305863));
  states.push_back(MakeQuaternion(total_space, 0.0449733,0.444763,0.848611,-0.282884));
  states.push_back(MakeQuaternion(total_space, 0.0563763,0.444041,0.85561,-0.259962));
  states.push_back(MakeQuaternion(total_space, 0.065585,0.45096,0.857867,-0.237483));
  states.push_back(MakeQuaternion(total_space, 0.072817,0.467139,0.854332,-0.215862));
  states.push_back(MakeQuaternion(total_space, 0.0783835,0.494114,0.843487,-0.195543));
  states.push_back(MakeQuaternion(total_space, 0.0827086,0.532953,0.823281,-0.176999));
  states.push_back(MakeQuaternion(total_space, 0.0863446,0.58362,0.791268,-0.160706));
  states.push_back(MakeQuaternion(total_space, 0.08997,0.644151,0.745209,-0.1471));
  states.push_back(MakeQuaternion(total_space, 0.0943521,0.710094,0.684275,-0.136499));
  states.push_back(MakeQuaternion(total_space, 0.100261,0.774972,0.610513,-0.128995));
  states.push_back(MakeQuaternion(total_space, 0.108345,0.832163,0.529429,-0.12438));
  states.push_back(MakeQuaternion(total_space, 0.119027,0.877263,0.448692,-0.122138));
  states.push_back(MakeQuaternion(total_space, 0.132459,0.9092,0.375552,-0.121532));
  states.push_back(MakeQuaternion(total_space, 0.148559,0.929514,0.314821,-0.121742));
  states.push_back(MakeQuaternion(total_space, 0.167089,0.940821,0.268434,-0.121984));
  states.push_back(MakeQuaternion(total_space, 0.187727,0.945621,0.236173,-0.121581));
  states.push_back(MakeQuaternion(total_space, 0.210118,0.945793,0.216636,-0.119978));
  states.push_back(MakeQuaternion(total_space, 0.233899,0.942553,0.207984,-0.11674));
  states.push_back(MakeQuaternion(total_space, 0.258717,0.936597,0.208352,-0.111536));
  states.push_back(MakeQuaternion(total_space, 0.284227,0.928278,0.216041,-0.104119));
  states.push_back(MakeQuaternion(total_space, 0.310099,0.917737,0.229571,-0.0943127));
  states.push_back(MakeQuaternion(total_space, 0.336011,0.905001,0.247681,-0.0819975));
  states.push_back(MakeQuaternion(total_space, 0.361654,0.890046,0.269299,-0.0671037));
  states.push_back(MakeQuaternion(total_space, 0.386731,0.872828,0.293514,-0.049603));
  states.push_back(MakeQuaternion(total_space, 0.410954,0.853311,0.319542,-0.0295042));
  states.push_back(MakeQuaternion(total_space, 0.434045,0.831476,0.346705,-0.0068486));
  states.push_back(MakeQuaternion(total_space, 0.455741,0.807329,0.374414,0.0182928));
  states.push_back(MakeQuaternion(total_space, 0.475788,0.7809,0.402147,0.0458219));
  states.push_back(MakeQuaternion(total_space, 0.493948,0.752247,0.429445,0.0756159));
  states.push_back(MakeQuaternion(total_space, 0.509996,0.721456,0.455897,0.107529));
  states.push_back(MakeQuaternion(total_space, 0.523725,0.688638,0.481141,0.141396));
  states.push_back(MakeQuaternion(total_space, 0.52783,0.677047,0.489384,0.153317));

  auto start_state = base_space->allocState();
  hopf_fibration->project(states.front(), start_state);

  auto goal_state = base_space->allocState();
  hopf_fibration->project(states.back(), goal_state);

  auto tmp_base_state = base_space->allocState();

  for(size_t k = 1; k < states.size(); k++) {
    auto s1 = states.at(k-1);
    auto s2 = states.at(k);

    //Check that states are close-by stereographically
    auto v1 = hopf_fibration->toVector(s1);
    auto v2 = hopf_fibration->toVector(s2);
    auto d = (v2 - v1).norm();
    EXPECT_LT(d, kStereographicDistanceThreshold);
  }

  for(const auto& state : states) {
    total_space->freeState(state);
  }

  base_space->freeState(tmp_base_state);
  base_space->freeState(goal_state);
  base_space->freeState(start_state);
}

