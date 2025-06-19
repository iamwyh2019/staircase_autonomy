#include <staircase_perception/core/multi_stair_manager.hpp>

void MultiRobotStairManager::computeStaircaseInfo(const stair_utility::StaircaseEstimate& new_estimate, stair_utility::StaircaseInfo& stair_info){
    // Compute the polygon of the staircase, and ensure are in order. Clear any left over data. 
    stair_info.stair_polygon.clear();

    Eigen::Vector3d begin_start, begin_end, final_start, final_end;

    begin_start =  new_estimate.steps[0].start_p;
    begin_end =  new_estimate.steps[0].end_p;
    final_start =  new_estimate.steps[new_estimate.stair_count - 1].start_p;
    final_end =  new_estimate.steps[new_estimate.stair_count - 1].end_p;

    getStairInfoFromCorners(begin_start, begin_end, final_start, final_end, stair_info);
}

MultiRobotStairManager::MultiRobotStairManager(const stair_utility::StairManagerParams& params){

    stair_manager_params_.yaw_threshold = params.yaw_threshold;
    stair_manager_params_.filter_type = params.filter_type;
    filter_type_ = params.filter_type;
    number_of_stairs_ = 0;

    std::cout << "\033[1;34m[Stair Manager] Initialized Multi-Robot Staircase Manager\033[0m" << std::endl;
}

int MultiRobotStairManager::addNewDetectedStaircase(const stair_utility::StaircaseEstimate& new_staircase, std::string robot_name, stair_utility::StaircaseEstimate& outgoing_staircase, stair_utility::SingleStaircaseSummary& updated_summary){
    int id;
    bool found_new = true;

    if (number_of_stairs_ == 0)
    {
        // First incoming stair. Add to list directly.
        id = getNewId();
        
        staircase_database_.emplace(id, std::make_shared<StaircaseModel>(id, new_staircase, filter_type_));
        staircase_estimate_database_.insert({id, std::unordered_map<std::string, stair_utility::StaircaseEstimate>({{robot_name, new_staircase}})});

        ids_.insert(id);
        number_of_stairs_++;
    }
    else
    {
        // STEP1 : Convert Measurement to Stair Spatial Parameters for correct checking.
        stair_utility::StaircaseInfo stair_info;
        computeStaircaseInfo(new_staircase, stair_info);

        for (auto it = staircase_database_.begin(); it != staircase_database_.end(); it++){
            int score = checkStaircaseSimilarity(it->second->staircase_info_, stair_info); 
            
            if(score >= 1){
                found_new = false;
                id = it->first;
                
                if(staircase_estimate_database_[id].find(robot_name) == staircase_estimate_database_[id].end()){
                    staircase_estimate_database_[id].insert({robot_name, new_staircase});
                }
                else{
                    staircase_estimate_database_[id][robot_name] = new_staircase;
                }
                break;
            }
        }
        if (found_new)
        {
            id = getNewId();
            staircase_database_.emplace(id, std::make_shared<StaircaseModel>(id, new_staircase, filter_type_));
            staircase_estimate_database_.insert({id, std::unordered_map<std::string, stair_utility::StaircaseEstimate>({{robot_name, new_staircase}})});

            ids_.insert(id);
            number_of_stairs_++;
        }
    }

    if(!found_new){
        // Merge or Update staircase here
        int count = 0;
        for(auto it = staircase_estimate_database_[id].begin();  it != staircase_estimate_database_[id].end(); it++){
            if(count == 0){
                staircase_database_[id]->reInitializeStaircase(it->second);
            }
            else{
                staircase_database_[id]->updateStaircase(it->second);
            }
            ++count;
        }
              
    }
    
    outgoing_staircase.stair_count = staircase_database_[id]->stair_count_;
    outgoing_staircase.stair_id =  id;
    outgoing_staircase.steps.clear();
    for(int k = 0; k < outgoing_staircase.stair_count; ++k){
        
        stair_utility::StairStep step; // Step width not used for estimates
        step.start_p = staircase_database_[id]->stair_aux_state_.segment((6 * k), 3);
        step.end_p = staircase_database_[id]->stair_aux_state_.segment((6 * k) + 3, 3);
        
        outgoing_staircase.steps.push_back(step);
    }
   
    updated_summary.id = id;    
    updated_summary.stair_depth = staircase_database_[id]->getStairDepth();
    updated_summary.stair_height = staircase_database_[id]->getStairHeight();
    updated_summary.stair_width = staircase_database_[id]->getStairWidth();
    updated_summary.stair_start_direction = staircase_database_[id]->getStairStartAngle();
    updated_summary.stair_end_direction = staircase_database_[id]->getStairEndAngle();
    std::cout << "\033[1;34m[Stair Manager] Processed Incoming data. Total stairs in database = " << number_of_stairs_ << " \033[0m" << std::endl;

    return id;
}

void MultiRobotStairManager::getStaircaseSummaries(std::vector<stair_utility::SingleStaircaseSummary> &stair_summaries){
    stair_summaries.clear();
    
    for(auto const& [stair_id, model_ptr] : staircase_database_){
        
        stair_utility::SingleStaircaseSummary summary;
                
        summary.id = stair_id;    
        summary.stair_depth = model_ptr->getStairDepth();
        summary.stair_height = model_ptr->getStairHeight();
        summary.stair_width = model_ptr->getStairWidth();
        summary.stair_start_direction = model_ptr->getStairStartAngle();
        summary.stair_end_direction = model_ptr->getStairEndAngle();

        summary.robot_list.clear();
        if (staircase_estimate_database_.count(stair_id)) {
            for (auto const& [robot_name, estimate] : staircase_estimate_database_.at(stair_id)) {
                summary.robot_list.push_back(robot_name);
            }
        }
        stair_summaries.push_back(summary);
    }
}