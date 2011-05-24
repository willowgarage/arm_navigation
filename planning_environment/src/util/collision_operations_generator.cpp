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

#include <planning_environment/util/collision_operations_generator.h>
#include <yaml-cpp/yaml.h>

static const unsigned int ESTABLISH_ALWAYS_NUM = 100;
static const unsigned int ESTABLISH_OFTEN_NUM = 500;
static const double ESTABLISH_OFTEN_PERCENTAGE = .5;
static const unsigned int ESTABLISH_OCCASIONAL_NUM = 1000;
static const unsigned int PERFORMANCE_TESTING_NUM = 2000;

using namespace planning_environment;

inline double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

CollisionOperationsGenerator::CollisionOperationsGenerator(planning_environment::CollisionModels* cm) 
{
  cm_ = cm;

  enableAllCollisions();

}

void CollisionOperationsGenerator::generateAlwaysInCollisionPairs(std::vector<CollisionOperationsGenerator::StringPair>& always_in_collision,
                                                                  std::vector<CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values)
{
  sampleAndCountCollisions(ESTABLISH_ALWAYS_NUM);
  std::vector<double> percentages;
  std::map<std::string, std::map<std::string, double> > percentage_num;
  buildOutputStructures(ESTABLISH_ALWAYS_NUM, 1.0, 1.0,
                        always_in_collision, percentages, in_collision_joint_values, percentage_num);
  
}

//assumes that always in collision pairs have been generated
void CollisionOperationsGenerator::generateDefaultInCollisionPairs(std::vector<CollisionOperationsGenerator::StringPair>& default_in_collision,
                                                                  std::vector<CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values)
{
  default_in_collision.clear();
  in_collision_joint_values.clear();

  planning_models::KinematicState state(cm_->getKinematicModel());
  
  state.setKinematicStateToDefault();    
  CollidingJointValues cjv;
  state.getKinematicStateValues(cjv);
  std::vector<planning_environment_msgs::ContactInformation> contacts;
  cm_->getAllCollisionsForState(state,
                                contacts);
  
  for(unsigned int i = 0; i < contacts.size(); i++) {
    planning_environment_msgs::ContactInformation& contact = contacts[i];
    default_in_collision.push_back(StringPair(contact.contact_body_1, contact.contact_body_2));
    in_collision_joint_values.push_back(cjv);
  }
}

void CollisionOperationsGenerator::generateOftenInCollisionPairs(std::vector<CollisionOperationsGenerator::StringPair>& often_in_collision,
                                                                 std::vector<double>& percentages, 
                                                                 std::vector<CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values)
{
  sampleAndCountCollisions(ESTABLISH_OFTEN_NUM);
  std::map<std::string, std::map<std::string, double> > percentage_num;
  buildOutputStructures(ESTABLISH_OFTEN_NUM, ESTABLISH_OFTEN_PERCENTAGE, 1.0,
                        often_in_collision, percentages, in_collision_joint_values, percentage_num);
  
}

void CollisionOperationsGenerator::generateOccasionallyAndNeverInCollisionPairs(std::vector<CollisionOperationsGenerator::StringPair>& occasionally_in_collision_pairs,
                                                                                std::vector<CollisionOperationsGenerator::StringPair>& never_in_collision_pairs,
                                                                                std::vector<double>& collision_percentages,
                                                                                std::vector<CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values)
{
  occasionally_in_collision_pairs.clear(); 
  collision_percentages.clear();
  never_in_collision_pairs.clear();
  in_collision_joint_values.clear();

  std::vector<CollisionOperationsGenerator::StringPair> first_in_collision_pairs;
  std::vector<CollisionOperationsGenerator::StringPair> second_in_collision_pairs;

  std::vector<CollisionOperationsGenerator::CollidingJointValues> first_in_collision_joint_values;
  std::vector<CollisionOperationsGenerator::CollidingJointValues> second_in_collision_joint_values;

  std::map<std::string, std::map<std::string, double> > first_percentage_num;
  std::map<std::string, std::map<std::string, double> > second_percentage_num;

  sampleAndCountCollisions(ESTABLISH_OCCASIONAL_NUM);
  buildOutputStructures(ESTABLISH_OCCASIONAL_NUM, 1.0/(ESTABLISH_OCCASIONAL_NUM*1.0), 1.0,
                        first_in_collision_pairs, collision_percentages, first_in_collision_joint_values, first_percentage_num);

  ROS_INFO_STREAM("First in collision size " << first_in_collision_pairs.size());

  sampleAndCountCollisions(ESTABLISH_OCCASIONAL_NUM);
  buildOutputStructures(ESTABLISH_OCCASIONAL_NUM, 1.0/(ESTABLISH_OCCASIONAL_NUM*1.0), 1.0,
                        second_in_collision_pairs, collision_percentages, second_in_collision_joint_values, second_percentage_num);

  ROS_INFO_STREAM("Second in collision size " << second_in_collision_pairs.size());

  collision_percentages.clear();
  for(std::map<std::string, std::map<std::string, double> >::iterator it = first_percentage_num.begin();
      it != first_percentage_num.end();
      it++) {
    for(std::map<std::string, double>::iterator it2 = it->second.begin();
        it2 != it->second.end();
        it2++) {
      bool first_never = (it2->second == 0);
      bool second_never = (second_percentage_num[it->first][it2->first] == 0);
      
      if(first_never && second_never) {
        never_in_collision_pairs.push_back(StringPair(it->first, it2->first));
      } else if(second_never) {
        occasionally_in_collision_pairs.push_back(StringPair(it->first, it2->first));
        collision_percentages.push_back(first_percentage_num[it->first][it2->first]);
        for(unsigned int i = 0; i < first_in_collision_pairs.size(); i++) {
          if((first_in_collision_pairs[i].first == it->first && first_in_collision_pairs[i].second == it2->first) ||
             (first_in_collision_pairs[i].first == it2->first && first_in_collision_pairs[i].second == it->first)) {
            in_collision_joint_values.push_back(first_in_collision_joint_values[i]);
          }
        }
      } else if(first_never){
        occasionally_in_collision_pairs.push_back(StringPair(it->first, it2->first));
        collision_percentages.push_back(second_percentage_num[it->first][it2->first]);
        for(unsigned int i = 0; i < second_in_collision_pairs.size(); i++) {
          if((second_in_collision_pairs[i].first == it->first && second_in_collision_pairs[i].second == it2->first) ||
             (second_in_collision_pairs[i].first == it2->first && second_in_collision_pairs[i].second == it->first)) {
            in_collision_joint_values.push_back(second_in_collision_joint_values[i]);
          }
        }
      }
    }
  }
  ROS_INFO_STREAM("Diff pairs num " << occasionally_in_collision_pairs.size());
}
  
void CollisionOperationsGenerator::enableAllCollisions() {
  collision_space::EnvironmentModel::AllowedCollisionMatrix altered_acm = cm_->getCurrentAllowedCollisionMatrix();
  altered_acm.changeEntry(false);
  cm_->setAlteredAllowedCollisionMatrix(altered_acm);
}

void CollisionOperationsGenerator::disablePairCollisionChecking(const CollisionOperationsGenerator::StringPair& pair) {
  collision_space::EnvironmentModel::AllowedCollisionMatrix altered_acm = cm_->getCurrentAllowedCollisionMatrix();
  altered_acm.changeEntry(pair.first, pair.second, true);
  cm_->setAlteredAllowedCollisionMatrix(altered_acm);
}

void CollisionOperationsGenerator::disablePairCollisionChecking(const std::vector<CollisionOperationsGenerator::StringPair>& pair_vec) {
  collision_space::EnvironmentModel::AllowedCollisionMatrix altered_acm = cm_->getCurrentAllowedCollisionMatrix();
  for(unsigned int i = 0; i < pair_vec.size(); i++) {
    altered_acm.changeEntry(pair_vec[i].first, pair_vec[i].second, true);
  }
  cm_->setAlteredAllowedCollisionMatrix(altered_acm);
}

void CollisionOperationsGenerator::generateSamplingStructures(const std::map<std::string, bool>& add_map) {
  joint_bounds_map_.clear();
  
  const std::vector<planning_models::KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
  //assuming that 0th is world joint, which we don't want to include
  for(unsigned int i = 1; i < jmv.size(); i++) {
    const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
    for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin();
        it != joint_bounds.end();
        it++) {
      if(joint_bounds_map_.find(it->first) != joint_bounds_map_.end()) {
        ROS_WARN_STREAM("Have repeat DOF names for " << it->first);
        continue;
      }
      if(add_map.find(it->first) != add_map.end()) {
        if(!add_map.find(it->first)->second) {
          continue;
        }
      }
      if(it->second.first > it->second.second) {
        ROS_WARN_STREAM("Lower bound for DOF " << it->first << " is greater than upper bound");
      } else if(it->second.first == -DBL_MAX) {
        ROS_WARN_STREAM("Some non-root DOF " << it->first << " has negative inf lower bound");
      } 
      if(it->second.second == DBL_MAX) {
        ROS_WARN_STREAM("Some non-root DOF " << it->first << " has inf upper bound");
      } 
      joint_bounds_map_[it->first] = it->second;        
    }
  }
}

void CollisionOperationsGenerator::sampleAndCountCollisions(unsigned int num) {
  resetCountingMap();
  
  planning_models::KinematicState state(cm_->getKinematicModel());
  
  for(unsigned int i = 0; i < num; i++) {
    generateRandomState(state);
    
    std::vector<planning_environment_msgs::ContactInformation> contacts;
    cm_->getAllCollisionsForState(state,
                                  contacts);

    if(i != 0 && i % 10000 == 0) {
      ROS_INFO_STREAM("On iteration " << i);
    }
    
    for(unsigned int i = 0; i < contacts.size(); i++) {
      planning_environment_msgs::ContactInformation& contact = contacts[i];
      if(collision_count_map_.find(contact.contact_body_1) == collision_count_map_.end()) {
        ROS_WARN_STREAM("Problem - have no count for collision body " << contact.contact_body_1);
      }
      if(collision_count_map_.find(contact.contact_body_2) == collision_count_map_.end()) {
        ROS_WARN_STREAM("Problem - have no count for collision body " << contact.contact_body_2);
      }
      collision_count_map_[contact.contact_body_1][contact.contact_body_2]++;
      collision_count_map_[contact.contact_body_2][contact.contact_body_1]++;
      CollidingJointValues cjv;
      state.getKinematicStateValues(cjv);
      collision_joint_values_[contact.contact_body_1][contact.contact_body_2] = cjv;
      collision_joint_values_[contact.contact_body_2][contact.contact_body_1] = cjv;
    }
  }
}

void CollisionOperationsGenerator::buildOutputStructures(unsigned int num, double low_threshold, double high_threshold, 
                                                         std::vector<CollisionOperationsGenerator::StringPair>& meets_threshold_collision,
                                                         std::vector<double>& percentages,
                                                         std::vector<CollisionOperationsGenerator::CollidingJointValues>& joint_values,
                                                         std::map<std::string, std::map<std::string, double> >& percentage_num){
  meets_threshold_collision.clear();
  percentages.clear();
  joint_values.clear();
  percentage_num.clear();
  
  bool do_output = false;
  for(std::map<std::string, std::map<std::string, unsigned int> >::iterator it = collision_count_map_.begin();
      it != collision_count_map_.end();
      it++) {
    for(std::map<std::string, unsigned int>::iterator it2 = it->second.begin();
        it2 != it->second.end();
        it2++) {
      if(it->first == it2->first) {
        continue;
      }  
      //if we've already registered this pair continue
      if(percentage_num.find(it->first) != percentage_num.end()) {
        if(percentage_num.find(it->first)->second.find(it2->first) != 
           percentage_num.find(it->first)->second.end()) {
          continue;
        }
      }
      if(percentage_num.find(it2->first) != percentage_num.end()) {
        if(percentage_num.find(it2->first)->second.find(it->first) != 
           percentage_num.find(it2->first)->second.end()) {
          continue;
        }
      }
      double per = (it2->second*1.0)/(num*1.0);
      percentage_num[it->first][it2->first] = per;
      percentage_num[it2->first][it->first] = per;
      if(per >= low_threshold && per <= high_threshold) {
        meets_threshold_collision.push_back(StringPair(it->first, it2->first));
        percentages.push_back(per);
        joint_values.push_back(collision_joint_values_[it->first][it2->first]);
      }
      if(do_output) {
        ROS_INFO_STREAM("Per between " << it->first << " and " << it2->first << " is " << per  << " low " << low_threshold << " high " << high_threshold);
      }
    }
    do_output = false;
  }
}

void CollisionOperationsGenerator::performanceTestSavedResults(std::map<CollisionOperationsGenerator::DisableType, std::vector<CollisionOperationsGenerator::StringPair> >& disable_types) {
  enableAllCollisions();
  ros::WallTime n1 = ros::WallTime::now();
  sampleAndCountCollisions(PERFORMANCE_TESTING_NUM);
  ROS_INFO_STREAM("With no collisions disabled full collision check take an average of " 
                  << (ros::WallTime::now()-n1).toSec()/(PERFORMANCE_TESTING_NUM/1.0) << " seconds.");
  for(std::map<DisableType, std::vector<StringPair> >::iterator it = disable_types.begin(); 
      it != disable_types.end(); it++) {
    disablePairCollisionChecking(it->second);
    std::string com = "Disabling ";
    if(it->first == ALWAYS) {
      com = "Always";
    } else if(it->first == DEFAULT) {
      com = "Default";
    } else if(it->first == OFTEN) {
      com = "Often";
    } else if(it->first == OCCASIONALLY) {
      com = "Occasionally";
    } else {
      com = "Never";
    }
    com += " in collision pairs average full check time is ";
    n1 = ros::WallTime::now();
    sampleAndCountCollisions(PERFORMANCE_TESTING_NUM);
    ROS_INFO_STREAM(com << (ros::WallTime::now()-n1).toSec()/(PERFORMANCE_TESTING_NUM/1.0) << " seconds.");
  }
}
 

void CollisionOperationsGenerator::outputYamlStringOfSavedResults(YAML::Emitter& outy,  const std::map<CollisionOperationsGenerator::DisableType, std::vector<CollisionOperationsGenerator::StringPair> >& disable_types) {
  outy << YAML::Key << "default_collision_operations";
  outy << YAML::Value << YAML::BeginSeq; 
  for(std::map<DisableType, std::vector<StringPair> >::const_iterator it = disable_types.begin(); 
      it != disable_types.end(); it++) {
    std::string com;
    if(it->first == ALWAYS) {
      com = "Always";
    } else if(it->first == DEFAULT) {
      com = "Default";
    } else if(it->first == OFTEN) {
      com = "Often";
    } else if(it->first == OCCASIONALLY) {
      com = "Occasionally";
    } else {
      com = "Never";
    }
    com += " in collision";
    for(unsigned int i = 0; i < it->second.size(); i++) {
     outy << YAML::BeginMap; 
     outy << YAML::Key << "object1" << YAML::Value << it->second[i].first;
     outy << YAML::Key << "object2" << YAML::Value << it->second[i].second;
     outy << YAML::Key << "operation" << YAML::Value << "disable";
     outy << YAML::Comment(com);
     outy << YAML::EndMap;
    }
  }
}

void CollisionOperationsGenerator::resetCountingMap() {
  const std::vector<planning_models::KinematicModel::LinkModel*>& lmv = cm_->getKinematicModel()->getLinkModelsWithCollisionGeometry();
  
  collision_count_map_.clear();
  
  std::map<std::string, unsigned int> all_link_zero;
  for(unsigned int i = 0; i < lmv.size(); i++) {
    all_link_zero[lmv[i]->getName()] = 0;
  }
  for(unsigned int i = 0; i < lmv.size(); i++) {
    collision_count_map_[lmv[i]->getName()] = all_link_zero;
  }
  collision_joint_values_.clear();
}

void CollisionOperationsGenerator::generateRandomState(planning_models::KinematicState& state) {
  std::map<std::string, double> values;
  for(std::map<std::string, std::pair<double, double> >::iterator it = joint_bounds_map_.begin();
      it != joint_bounds_map_.end();
      it++) {
    values[it->first] = gen_rand(it->second.first, it->second.second);
    //ROS_INFO_STREAM("Value for " << it->first << " is " << values[it->first] << " bounds " << 
    //                it->second.first << " " << it->second.second);
  }
  state.setKinematicState(values);
}
