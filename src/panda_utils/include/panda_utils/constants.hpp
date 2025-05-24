#pragma once

#include <string>

namespace panda_interface_names {
const std::string forward_kine_service_name{"forward_kine"};
const std::string jacob_calc_service_name{"calculate_jacobian"};
const std::string joints_cmd_pos_service_name{"send_joints_pos_cmd"};
const std::string clik_service_name{"clik"};
} // namespace panda_interface_names
