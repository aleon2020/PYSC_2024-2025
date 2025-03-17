#include <memory>

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class FlyAction : public plansys2::ActionExecutorClient
{
public:
  FlyAction()
  : plansys2::ActionExecutorClient("fly", 200ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    progress_ += 0.05;

    std::string arg_drone = get_arguments()[0];
    std::string arg_from = get_arguments()[1];
    std::string arg_to = get_arguments()[2];
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving " << arg_drone
              << " from " << arg_from << " to " << arg_to
              << " ... ["
              << std::min(100.0, progress_ * 100.0)
              << "%]  " << std::flush;

    if (progress_ < 1.0) {
      send_feedback(progress_, "Fly running");
    } else {
      finish(true, 1.0, "Fly completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlyAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "fly"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
