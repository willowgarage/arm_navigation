/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <collision_space/environmentODE.h>
#include <yaml-cpp/yaml.h>

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

static const unsigned int ESTABLISH_ALWAYS_NUM = 100;
static const unsigned int ESTABLISH_OFTEN_NUM = 500;
static const double ESTABLISH_OFTEN_PERCENTAGE = .5;
static const unsigned int PERFORMANCE_TESTING_NUM = 500;

class CollisionOperationsGenerator {

public:

  CollisionOperationsGenerator(const std::string& full_path_name) {
    
    ok_ = false;

    bool urdf_ok = urdf_model_.initFile(full_path_name);

    if(!urdf_ok) {
      ROS_WARN_STREAM("Urdf file " << full_path_name << " not ok");
      return;
    }
    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    const urdf::Link *root = urdf_model_.getRoot().get();

    //now this should work with an non-identity transform
    planning_models::KinematicModel::MultiDofConfig config("world_joint");
    config.type = "Floating";
    config.parent_frame_id = root->name;
    config.child_frame_id = root->name;
    multi_dof_configs.push_back(config);
    
    kmodel_ = new planning_models::KinematicModel(urdf_model_,gcs, multi_dof_configs);    
    if(kmodel_->getRoot() == NULL) {
      ROS_INFO_STREAM("Kinematic root is NULL");
      return;
    }

    ode_collision_model_ = new collision_space::EnvironmentModelODE();

    const std::vector<planning_models::KinematicModel::LinkModel*>& coll_links = kmodel_->getLinkModelsWithCollisionGeometry();
  
    std::vector<std::string> coll_names;
    for(unsigned int i = 0; i < coll_links.size(); i++) {
      coll_names.push_back(coll_links[i]->getName());
    }
    collision_space::EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names,false);
    std::map<std::string, double> default_link_padding_map;
    ode_collision_model_->setRobotModel(kmodel_, default_collision_matrix, 
                                        default_link_padding_map, 0.0, 1.0);
    generateSamplingStructures();
    ok_ = true;
  }

  ~CollisionOperationsGenerator() {
    delete ode_collision_model_;
    delete kmodel_;
  }

  void generateSamplingStructures() {
    const std::vector<planning_models::KinematicModel::JointModel*>& jmv = kmodel_->getJointModels();
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
        if(it->second.first > it->second.second) {
          ROS_WARN_STREAM("Lower bound for DOF " << it->first << " is greater than upper bound");
        } else if(it->second.first == -DBL_MAX) {
          ROS_WARN_STREAM("Some non-root DOF " << it->first << " has negative inf lower bound");
        } 
        if(it->second.second == DBL_MAX) {
          ROS_WARN_STREAM("Some non-root DOF " << it->first << " has inf upper[ bound");
        } 
        joint_bounds_map_[it->first] = it->second;        
      }
    }
  }

  void generateOutputCollisionOperations(unsigned int num, const std::string& output_file) {
    sampleAndCountCollisions(ESTABLISH_ALWAYS_NUM);
    
    ROS_INFO_STREAM("Established always in collision pairs");

    planning_models::KinematicState state(kmodel_);
    collision_space::EnvironmentModel::AllowedCollisionMatrix altered_acm = ode_collision_model_->getCurrentAllowedCollisionMatrix();

    altered_acm.changeEntry(false);
    for(unsigned int i = 0; i < always_in_collision_.size(); i++) {
      altered_acm.changeEntry(always_in_collision_[i].first,
                               always_in_collision_[i].second,
                               true);
    }    

    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);

    saved_always_in_collision_ = always_in_collision_;

    sampleAndCountCollisions(ESTABLISH_OFTEN_NUM);
    for(unsigned int i = 0; i < often_in_collision_.size(); i++) {
      altered_acm.changeEntry(often_in_collision_[i].first,
                              often_in_collision_[i].second,
                              true);
    }    
    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    saved_often_in_collision_ = often_in_collision_;
    often_percentage_map_ = percentage_map_;
    ROS_INFO_STREAM("Established often in collision pairs");
    
    sampleAndCountCollisions(num);
    emitYamlOutputStructures(output_file);
    printOutputStructures(output_file);
  }

  void sampleAndCountCollisions(unsigned int num) {
    resetCountingMap();

    planning_models::KinematicState state(kmodel_);

    ros::WallTime n1 = ros::WallTime::now();
    for(unsigned int i = 0; i < num; i++) {
      generateRandomState(state);
      ode_collision_model_->updateRobotModel(&state);
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
      ode_collision_model_->getAllCollisionContacts(allowed_contacts,
                                                    coll_space_contacts,
                                                    1);
      if(i != 0 && i % 1000 == 0) {
        ROS_INFO_STREAM("Num " << i << " getting all contacts takes " << (ros::WallTime::now()-n1).toSec());
        n1 = ros::WallTime::now();
      }
      for(unsigned int i = 0; i < coll_space_contacts.size(); i++) {
        collision_space::EnvironmentModel::Contact& contact = coll_space_contacts[i];
        if(collision_count_map_.find(contact.body_name_1) == collision_count_map_.end()) {
          ROS_WARN_STREAM("Problem - have no count for collision body " << contact.body_name_1);
        }
        if(collision_count_map_.find(contact.body_name_2) == collision_count_map_.end()) {
          ROS_WARN_STREAM("Problem - have no count for collision body " << contact.body_name_2);
        }
        collision_count_map_[contact.body_name_1][contact.body_name_2]++;
        collision_count_map_[contact.body_name_2][contact.body_name_1]++;
      }
    }
    buildOutputStructures(num);
  }

  void generateAndCompareOutputStructures(unsigned int num) {
    sampleAndCountCollisions(ESTABLISH_ALWAYS_NUM);
    
    ROS_INFO_STREAM("Established always in collision pairs");

    planning_models::KinematicState state(kmodel_);
    collision_space::EnvironmentModel::AllowedCollisionMatrix altered_acm = ode_collision_model_->getCurrentAllowedCollisionMatrix();

    altered_acm.changeEntry(false);
    for(unsigned int i = 0; i < always_in_collision_.size(); i++) {
      altered_acm.changeEntry(always_in_collision_[i].first,
                              always_in_collision_[i].second,
                              true);
    }    
    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    saved_always_in_collision_ = always_in_collision_;

    sampleAndCountCollisions(ESTABLISH_OFTEN_NUM);
    for(unsigned int i = 0; i < often_in_collision_.size(); i++) {
      altered_acm.changeEntry(often_in_collision_[i].first,
                              often_in_collision_[i].second,
                              true);
    }    
    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    saved_often_in_collision_ = often_in_collision_;
    ROS_INFO_STREAM("Established often in collision pairs");

    std::map<std::string, std::map<std::string, double> > first_percentage_map;
    sampleAndCountCollisions(num);
    first_percentage_map = percentage_map_;
    
    ROS_INFO_STREAM("Done with first");

    sampleAndCountCollisions(num);

    ROS_INFO_STREAM("Done with second");

    for(std::map<std::string, std::map<std::string, double> >::iterator it = first_percentage_map.begin();
        it != first_percentage_map.end();
        it++) {
      for(std::map<std::string, double>::iterator it2 = it->second.begin();
          it2 != it->second.end();
          it2++) {
        bool first_all = (it2->second == num);
        bool second_all = (percentage_map_[it->first][it2->first] == num);
        
        if(first_all != second_all) {
          ROS_INFO_STREAM("Links " << it->first << " and " << it2->first << " different all");
        }
        
        bool first_never = (it2->second == 0);
        bool second_never = (percentage_map_[it->first][it2->first] == 0);
        
        if(first_never != second_never) {
          ROS_INFO_STREAM("Links " << it->first << " and " << it2->first << " different never " << it2->second << " " << percentage_map_[it->first][it2->first]);
        }
      }
    }
  }

  void buildOutputStructures(unsigned int num) {
    always_in_collision_.clear();
    never_in_collision_.clear();    
    often_in_collision_.clear();
    occasionally_in_collision_.clear();
    percentage_map_.clear();
    for(std::map<std::string, std::map<std::string, unsigned int> >::iterator it = collision_count_map_.begin();
        it != collision_count_map_.end();
        it++) {
      for(std::map<std::string, unsigned int>::iterator it2 = it->second.begin();
          it2 != it->second.end();
          it2++) {
        if(it->first == it2->first) {
          continue;
        }
        bool already_in_lists = false;
        //if we've already registered this pair continue
        if(percentage_map_.find(it->first) != percentage_map_.end()) {
          if(percentage_map_.find(it->first)->second.find(it2->first) != 
             percentage_map_.find(it->first)->second.end()) {
            already_in_lists = true;
          }
        }
        if(percentage_map_.find(it2->first) != percentage_map_.end()) {
          if(percentage_map_.find(it2->first)->second.find(it->first) != 
             percentage_map_.find(it2->first)->second.end()) {
            already_in_lists = true;
          }
        }
        if(it2->second == num) {
          if(!already_in_lists) {
            always_in_collision_.push_back(std::pair<std::string, std::string>(it->first, it2->first));
          }
          percentage_map_[it->first][it2->first] = 1.0;
          percentage_map_[it2->first][it->first] = 1.0;
        } else if(it2->second == 0) {
          if(!already_in_lists) {
            never_in_collision_.push_back(std::pair<std::string, std::string>(it->first, it2->first));
          }
          percentage_map_[it->first][it2->first] = 0.0;
          percentage_map_[it2->first][it->first] = 0.0;          
        } else {
          double per = (it2->second*1.0)/(num*1.0);
          if(!already_in_lists) {
            if(per > ESTABLISH_OFTEN_PERCENTAGE) {
              often_in_collision_.push_back(std::pair<std::string, std::string>(it->first, it2->first));
            } else {
              occasionally_in_collision_.push_back(std::pair<std::string, std::string>(it->first, it2->first));
            }
          }
          percentage_map_[it->first][it2->first] = per;
          percentage_map_[it2->first][it->first] = per;                    
        }
      }
    }
  }

  void printOutputStructures(const std::string& filename) {
    std::ofstream outfile((filename+".txt").c_str());

    outfile << "Always in collision pairs: " << std::endl;
    for(unsigned int i = 0; i < saved_always_in_collision_.size(); i++) {
      outfile << saved_always_in_collision_[i].first << " " << saved_always_in_collision_[i].second << std::endl;
    }
    outfile << std::endl;

    outfile << "Often in collision pairs: " << std::endl;
    for(unsigned int i = 0; i < saved_often_in_collision_.size(); i++) {
      outfile << saved_often_in_collision_[i].first << " " << saved_often_in_collision_[i].second;
      outfile << " " << often_percentage_map_[saved_often_in_collision_[i].first][saved_often_in_collision_[i].second] << std::endl;
    }
    outfile << std::endl;

    outfile << "Never in collision pairs: " << std::endl;
    for(unsigned int i = 0; i < never_in_collision_.size(); i++) {
      outfile << never_in_collision_[i].first << " " << never_in_collision_[i].second << std::endl;
    }
    outfile << std::endl;

    outfile << "Occasionally in collision pairs: " << std::endl;
    for(unsigned int i = 0; i < occasionally_in_collision_.size(); i++) {
      outfile << occasionally_in_collision_[i].first << " " << occasionally_in_collision_[i].second;
      outfile << " " << percentage_map_[occasionally_in_collision_[i].first][occasionally_in_collision_[i].second] << std::endl;
    }
  }

  void emitYamlOutputStructures(const std::string& filename) {
    std::ofstream outfile((filename+".yaml").c_str());

    YAML::Emitter outy;
    outy << YAML::BeginMap;
    outy << YAML::Key << "default_collision_operations";
    outy << YAML::Value << YAML::BeginSeq; 
    for(unsigned int i = 0; i < saved_often_in_collision_.size(); i++) {
      outy << YAML::BeginMap; 
      outy << YAML::Key << "object1" << YAML::Value << saved_often_in_collision_[i].first;
      outy << YAML::Key << "object2" << YAML::Value << saved_often_in_collision_[i].second;
      outy << YAML::Key << "operation" << YAML::Value << "disable";
      outy << YAML::Comment("Often in collision");
      outy << YAML::EndMap;
    }
    for(unsigned int i = 0; i < saved_always_in_collision_.size(); i++) {
      outy << YAML::BeginMap; 
      outy << YAML::Key << "object1" << YAML::Value << saved_always_in_collision_[i].first;
      outy << YAML::Key << "object2" << YAML::Value << saved_always_in_collision_[i].second;
      outy << YAML::Key << "operation" << YAML::Value << "disable";
      outy << YAML::Comment("Always in collision");
      outy << YAML::EndMap;
    }
    for(unsigned int i = 0; i < never_in_collision_.size(); i++) {
      outy << YAML::BeginMap; 
      outy << YAML::Key << "object1" << YAML::Value << never_in_collision_[i].first;
      outy << YAML::Key << "object2" << YAML::Value << never_in_collision_[i].second;
      outy << YAML::Key << "operation" << YAML::Value << "disable";
      outy << YAML::Comment("Never in collision");
      outy << YAML::EndMap;
    }
    outy << YAML::EndMap;
    outfile << outy.c_str();
  }

  void performanceTestFromOutputStructures() {
    unsigned int num = PERFORMANCE_TESTING_NUM;
    planning_models::KinematicState state(kmodel_);
    collision_space::EnvironmentModel::AllowedCollisionMatrix altered_acm = ode_collision_model_->getCurrentAllowedCollisionMatrix();

    altered_acm.changeEntry(false);
    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    ros::WallTime n1 = ros::WallTime::now();
    for(unsigned int i = 0; i < num; i++) {
      generateRandomState(state);
      ode_collision_model_->updateRobotModel(&state);
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
      ode_collision_model_->getAllCollisionContacts(allowed_contacts,
                                                    coll_space_contacts,
                                                    1);
    }
    ROS_INFO_STREAM("All enabled collision check average " <<
                    (ros::WallTime::now()-n1).toSec()/(num*1.0));

    for(unsigned int i = 0; i < saved_always_in_collision_.size(); i++) {
      altered_acm.changeEntry(saved_always_in_collision_[i].first,
                               saved_always_in_collision_[i].second,
                               true);
    }    

    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    n1 = ros::WallTime::now();
    for(unsigned int i = 0; i < num; i++) {
      generateRandomState(state);
      ode_collision_model_->updateRobotModel(&state);
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
      ode_collision_model_->getAllCollisionContacts(allowed_contacts,
                                                    coll_space_contacts,
                                                    1);
    }
    ROS_INFO_STREAM("Always disabled collision check average " <<
                    (ros::WallTime::now()-n1).toSec()/(num*1.0));

    for(unsigned int i = 0; i < saved_often_in_collision_.size(); i++) {
      altered_acm.changeEntry(saved_often_in_collision_[i].first,
                               saved_often_in_collision_[i].second,
                               true);
    }    

    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    n1 = ros::WallTime::now();
    for(unsigned int i = 0; i < num; i++) {
      generateRandomState(state);
      ode_collision_model_->updateRobotModel(&state);
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
      ode_collision_model_->getAllCollisionContacts(allowed_contacts,
                                                    coll_space_contacts,
                                                    1);
    }
    ROS_INFO_STREAM("Often disabled collision check average " <<
                    (ros::WallTime::now()-n1).toSec()/(num*1.0));

    for(unsigned int i = 0; i < never_in_collision_.size(); i++) {
      altered_acm.changeEntry(never_in_collision_[i].first,
                               never_in_collision_[i].second,
                               true);
    }    
    ode_collision_model_->setAlteredCollisionMatrix(altered_acm);
    n1 = ros::WallTime::now();
    for(unsigned int i = 0; i < num; i++) {
      generateRandomState(state);
      ode_collision_model_->updateRobotModel(&state);
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
      ode_collision_model_->getAllCollisionContacts(allowed_contacts,
                                                    coll_space_contacts,
                                                    1);
    }
    ROS_INFO_STREAM("Never disabled collision check average " <<
                    (ros::WallTime::now()-n1).toSec()/(num*1.0));
    
  }


  bool isOk() const {
    return ok_;
  }

protected:

  void resetCountingMap() {
    const std::vector<planning_models::KinematicModel::LinkModel*>& lmv = kmodel_->getLinkModelsWithCollisionGeometry();

    collision_count_map_.clear();

    //assuming that 0th is world joint, which we don't want to include
    std::map<std::string, unsigned int> all_link_zero;
    for(unsigned int i = 0; i < lmv.size(); i++) {
      all_link_zero[lmv[i]->getName()] = 0;
    }
    for(unsigned int i = 0; i < lmv.size(); i++) {
      collision_count_map_[lmv[i]->getName()] = all_link_zero;
    }
  }

  void generateRandomState(planning_models::KinematicState& state) {
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

  bool ok_;

  collision_space::EnvironmentModel* ode_collision_model_;
  planning_models::KinematicModel* kmodel_;
  urdf::Model urdf_model_;

  std::map<std::string, std::pair<double, double> > joint_bounds_map_;
  std::map<std::string, std::map<std::string, unsigned int> > collision_count_map_;

  std::vector<std::pair<std::string, std::string> > always_in_collision_;
  std::vector<std::pair<std::string, std::string> > saved_always_in_collision_;
  std::vector<std::pair<std::string, std::string> > never_in_collision_;
  std::vector<std::pair<std::string, std::string> > often_in_collision_;
  std::vector<std::pair<std::string, std::string> > saved_often_in_collision_;
  std::vector<std::pair<std::string, std::string> > occasionally_in_collision_;
  std::map<std::string, std::map<std::string, double> > percentage_map_;
  std::map<std::string, std::map<std::string, double> > often_percentage_map_;

};

static const unsigned int TIMES = 50000;

int main(int argc, char** argv) {

  ros::init(argc, argv, "collision_operations_generator");

  srand(time(NULL));
  
  if(argc < 2) {
    ROS_INFO_STREAM("Must specify a urdf file");
    exit(0);
  }

  std::string urdf_file = argv[1];
  std::string output_file;
  if(argc == 3) {
    output_file = argv[2];
  }

  CollisionOperationsGenerator cog(urdf_file);

  if(!cog.isOk()) {
    ROS_INFO_STREAM("Something wrong with urdf");
    exit(0);
  }

  if(argc == 3) {
    cog.generateOutputCollisionOperations(TIMES, output_file);
    cog.performanceTestFromOutputStructures();
  } else {
    cog.generateAndCompareOutputStructures(TIMES);
  }

  ros::shutdown();
  exit(0);
}
  
