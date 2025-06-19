#ifndef _MULTI_STAIR_MANAGER_H_
#define _MULTI_STAIR_MANAGER_H_

#include "staircase_perception/utils/stair_utilities.hpp"
#include "staircase_perception/core/staircase_model.hpp"
#include <unordered_map>
#include <unordered_set>


class BaseStaircaseManager{
    public:
        BaseStaircaseManager(){};
        ~BaseStaircaseManager(){};

        stair_utility::StairManagerParams stair_manager_params_;
        int number_of_stairs_;

        bool areLinesIntersecting(const Eigen::Vector3d &pointA, const Eigen::Vector3d &pointB, const Eigen::Vector3d &pointC, const Eigen::Vector3d &pointD);
        int checkStaircaseSimilarity(const stair_utility::StaircaseInfo& curr_stairase_info, const stair_utility::StaircaseInfo& new_staircase_info); 
        
    protected:
        int getNewId();

        void getStairInfoFromCorners(const Eigen::Vector3d &begin_start, const Eigen::Vector3d &begin_end, const Eigen::Vector3d &final_start, const Eigen::Vector3d &final_end, stair_utility::StaircaseInfo& stair_info);

        std::unordered_set<int> ids_;
};


class SingleRobotStairManager: public BaseStaircaseManager{
    public:
        SingleRobotStairManager(){};
        ~SingleRobotStairManager(){};
        SingleRobotStairManager(const stair_utility::StairManagerParams& params);

        int addNewDetectedStaircase(const stair_utility::StaircaseMeasurement& new_staircase, stair_utility::StaircaseEstimate &estimate);
        
        stair_utility::StaircaseProcessingResult time_results_;
    private:

        void computeStaircaseInfo(const stair_utility::StaircaseMeasurement& new_staircase, stair_utility::StaircaseInfo& stair_info);

        stair_utility::StaircaseFilterType filter_type_;
        std::unordered_map<int, std::shared_ptr<StaircaseModel>> staircase_database_;

};


class MultiRobotStairManager: public BaseStaircaseManager{
    public:
        MultiRobotStairManager(){};
        ~MultiRobotStairManager(){};
        MultiRobotStairManager(const stair_utility::StairManagerParams& params);

        int addNewDetectedStaircase(const stair_utility::StaircaseEstimate& incoming_staircase, std::string robot_name, stair_utility::StaircaseEstimate& outgoing_staircase, stair_utility::SingleStaircaseSummary& updated_summary);
        void getStaircaseSummaries(std::vector<stair_utility::SingleStaircaseSummary> &stair_summaries);
    
    private:
        std::set<std::string> robot_list_;
        stair_utility::StaircaseFilterType filter_type_;

        void computeStaircaseInfo(const stair_utility::StaircaseEstimate& new_estimate, stair_utility::StaircaseInfo& stair_info);

        std::unordered_map<int, std::shared_ptr<StaircaseModel>> staircase_database_;
        std::unordered_map<int, std::unordered_map<std::string, stair_utility::StaircaseEstimate>> staircase_estimate_database_;
};

#endif