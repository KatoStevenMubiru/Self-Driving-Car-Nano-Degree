/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Kato Steven Mubiru
 **********************************************/

/**
 * @file cost_functions.cpp
 **/

#include "cost_functions.h"

using namespace std;

namespace cost_functions {
// COST FUNCTIONS

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight) {
  /*
  Penalizes trajectories whose coordinate(and derivatives)
  differ from the goal.
  */
  double cost = 0.0;
  vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);
  //////////////cout << "26 - Evaluating f and N derivatives Done. Size:" <<
  /// evals.size() << endl;

  for (size_t i = 0; i < evals.size(); i++) {
    double diff = fabs(evals[i] - goals[i]);
    cost += logistic(diff / sigma[i]);
  }
  ////////////cout << "diff_coeff Cost Calculated " << endl;
  return cost_weight * cost;
}

double collision_circles_cost_spiral(const std::vector<PathPoint>& spiral,
21                                     const std::vector<State>& obstacles) {
22  bool collision{false};
23  auto n_circles = CIRCLE_OFFSETS.size();
24
25  for (auto wp : spiral) {
26    if (collision) {
27      break;
28    }
29    double cur_x = wp.x;
30    double cur_y = wp.y;
31    double cur_yaw = wp.theta;  // This is already in rad.
32
33    for (size_t c = 0; c < n_circles && !collision; ++c) {
34      // TODO-Circle placement: Where should the circles be at? The code below
35      // is NOT complete. HINT: use CIRCLE_OFFSETS[c], sine and cosine to
36      // calculate x and y: cur_y + CIRCLE_OFFSETS[c] * std::sin/cos(cur_yaw)
37      auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
38      auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);
39
40      for (auto obst : obstacles) {
41        if (collision) {
42          break;
43        }
44        auto actor_yaw = obst.rotation.yaw;
45        for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
46          auto actor_center_x =
47              obst.location.x + CIRCLE_OFFSETS[c2] * std::cos(actor_yaw);
48          auto actor_center_y =
49              obst.location.y + CIRCLE_OFFSETS[c2] * std::sin(actor_yaw);
50
51          // TODO-Distance from circles to obstacles/actor: How do you calculate
52          // the distance between the center of each circle and the
53          // obstacle/actor
54          double dist = std::sqrt(std::pow(circle_center_x - actor_center_x, 2) + std::pow(circle_center_y - actor_center_y, 2));
55
56          collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
57        }
58      }
59    }
60  }
61  return (collision) ? COLLISION : 0.0;
62}
63
64double close_to_main_goal_cost_spiral(const std::vector<PathPoint>& spiral,
65                                      State main_goal) {
66  // The last point on the spiral should be used to check how close we are to
67  // the Main (center) goal. That way, spirals that end closer to the lane
68  // center-line, and that are collision free, will be prefered.
69  auto n = spiral.size();
70
71  // TODO-distance between last point on spiral and main goal: How do we
72  // calculate the distance between the last point on the spiral (spiral[n-1])
73  // and the main goal (main_goal.location). Use spiral[n - 1].x, spiral[n -
74  // 1].y and spiral[n - 1].z.
75  // Use main_goal.location.x, main_goal.location.y and main_goal.location.z
76  auto delta_x = main_goal.location.x - spiral[n - 1].x;
77  auto delta_y = main_goal.location.y - spiral[n - 1].y;
78  auto delta_z = main_goal.location.z - spiral[n - 1].z;  // If z is not considered, you can omit this line.
79
80  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z));
81
82  auto cost = logistic(dist);
83  return cost;
84}
85}  // namespace cost_functions