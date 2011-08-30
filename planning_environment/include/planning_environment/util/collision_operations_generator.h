/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author E. Gil Jones
 *********************************************************************/

#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>
#include <fstream>

#include <planning_environment/models/collision_models.h>
#include <yaml-cpp/yaml.h>

namespace planning_environment 
{

class CollisionOperationsGenerator {

public:

  enum DisableType {
    ADJACENT,
    ALWAYS,
    DEFAULT,
    OFTEN,
    OCCASIONALLY,
    NEVER
  };

  enum SamplingSafety
  {
    VerySafe,
    Safe,
    Normal,
    Fast,
    VeryFast
  };

  CollisionOperationsGenerator(CollisionModels* cm);
  
  typedef std::pair<std::string, std::string> StringPair;
  typedef std::map<std::string, double> CollidingJointValues;

  void generateAdjacentInCollisionPairs(std::vector<StringPair>& adjacent_in_collision_pairs);
  
  void generateAlwaysInCollisionPairs(std::vector<StringPair>& always_in_collision_pairs,
                                      std::vector<CollidingJointValues>& in_collision_joint_values);

  void generateDefaultInCollisionPairs(std::vector<StringPair>& default_in_collision_pairs,
                                       std::vector<CollidingJointValues>& in_collision_joint_values);

  void generateOftenInCollisionPairs(std::vector<StringPair>& often_in_collision_pairs,
                                     std::vector<double>& collision_percentage,
                                     std::vector<CollidingJointValues>& in_collision_joint_values);
  
  void generateOccasionallyAndNeverInCollisionPairs(std::vector<StringPair>& occasionally_in_collision_pairs,
                                                    std::vector<StringPair>& never_in_collision_pairs,
                                                    std::vector<double>& collision_percentage,
                                                    std::vector<CollidingJointValues>& in_collision_joint_values);
  
  void enablePairCollisionChecking(const StringPair& pair);  
  void disablePairCollisionChecking(const StringPair& pair);
  void disablePairCollisionChecking(const std::vector<StringPair>& pair_vec);

  void enableAllCollisions();
  void outputYamlStringOfSavedResults(YAML::Emitter& outy, const std::map<DisableType, std::vector<StringPair> >& disable_types);
  //void outputFileOfSavedResults();

  void performanceTestSavedResults(std::map<CollisionOperationsGenerator::DisableType, std::vector<CollisionOperationsGenerator::StringPair> >& disable_types);

  void generateSamplingStructures(const std::map<std::string, bool>& add_map);

  inline void setSafety(SamplingSafety safety)
  {
    switch(safety)
    {
      case VerySafe:
        establish_always_num_ = 5000;
        establish_often_num_ = 15000;
        establish_often_percentage_ = 0.5;
        establish_occasional_num_ = 1000000;
        performance_testing_num_ = 5000;
        break;

      case Safe:
        establish_always_num_ = 10000;
        establish_often_num_ = 5000;
        establish_often_percentage_ = 0.5;
        establish_occasional_num_ = 100000;
        performance_testing_num_ = 1000;
        break;

      case Normal:
        establish_always_num_ = 1000;
        establish_often_num_ = 1000;
        establish_often_percentage_ = 0.5;
        establish_occasional_num_ = 20000;
        performance_testing_num_ = 1000;
        break;

      case Fast:
        establish_always_num_ = 100;
        establish_often_num_ = 500;
        establish_often_percentage_ = 0.5;
        establish_occasional_num_ = 1000;
        performance_testing_num_ = 100;
        break;

      case VeryFast:
        establish_always_num_ = 100;
        establish_often_num_ = 100;
        establish_often_percentage_ = 0.5;
        establish_occasional_num_ = 500;
        performance_testing_num_ = 10;
        break;
    }
  }

  unsigned int establish_always_num_;
  unsigned int establish_often_num_;
  double establish_often_percentage_;
  unsigned int establish_occasional_num_;
  unsigned int performance_testing_num_;

protected:

  void accumulateAdjacentLinksRecursive(const planning_models::KinematicModel::LinkModel* parent,
                                        std::vector<StringPair>& adjacencies);

  void sampleAndCountCollisions(unsigned int num);

  void buildOutputStructures(unsigned int num, double low_value, double high_value, 
                             std::vector<StringPair>& meets_threshold_collision,
                             std::vector<double>& collision_percentages, 
                             std::vector<CollidingJointValues>& joint_values,
                             std::map<std::string, std::map<std::string, double> >& percentage_num);
  
  void resetCountingMap();

  void generateRandomState(planning_models::KinematicState& state);

  std::map<std::string, std::pair<double, double> > joint_bounds_map_;
  std::map<std::string, std::map<std::string, unsigned int> > collision_count_map_;
  std::map<std::string, std::map<std::string, CollidingJointValues> > collision_joint_values_;

  planning_environment::CollisionModels* cm_;

};
}
