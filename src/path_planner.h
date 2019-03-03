# include <math.h>
# include <iostream>
# include <vector>
# include <cmath>

using namespace std;

int Path_planning(int lane, vector<double>left_lane_s, vector<double>left_lane_speed, vector<double>center_lane_s, vector<double>center_lane_speed, vector<double>right_lane_s, vector<double>right_lane_speed){

    // a vector to receive the costs
    vector<double>costs;
    int best_lane;

    // check the size of each incoming lane
    int left_lane_len = left_lane_s.size();
    int center_lane_len = center_lane_s.size();
    int right_lane_len = right_lane_s.size();

    // check the left lane
    if (left_lane_len > 0) {

        // get the distance to the closest car
        double left_closest = *min_element(left_lane_s.begin(), left_lane_s.end());
        // push the closest to the cost vector
        costs.push_back(left_closest);
    }

    // else push a cost of 5000 to the vector
    else {
        costs.push_back(5000);
    }

    // check center lane
    if (center_lane_len > 0) {
        double center_closest = *min_element(center_lane_s.begin(), center_lane_s.end());
        costs.push_back(center_closest);
    }
    else {
        costs.push_back(5000);
    }

    // check the right lane
    if (right_lane_len > 0) {
        double right_closest = *min_element(right_lane_s.begin(), right_lane_s.end());
        costs.push_back(right_closest);
    }
    else {
        costs.push_back(5000);
    }

    // if all lanes are wide open just keep lane
    if (costs[0] == 0 && costs[1] ==0 && costs[2] == 0) {

        // return lane
        best_lane = lane;
    }

    else {
        double best_cost = *max_element(costs.begin(), costs.end());
        if (costs[0] == best_cost) {
            best_lane = 0;
        }
        else if (costs[1] == best_cost) {
            best_lane = 1;
        }
        else {
            best_lane = 2;
        }
    }

    return best_lane;

}
                   