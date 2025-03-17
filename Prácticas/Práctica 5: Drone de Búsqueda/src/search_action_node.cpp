#include <memory>

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class SearchAction : public plansys2::ActionExecutorClient
{
public:
  SearchAction()
  : plansys2::ActionExecutorClient("search", 1s)
  {
    init_ = true;
    num_photos_ = 0;
  }

// EJERCICIO 1: ACCIONES
private:
  void do_work()
  {
    if (init_) {
      init_ = false;
      std::cout << "\r\e[K" << std::flush;
      std::cout << "Taking photo ..." << std::endl;

    } else {
      num_photos_ += 1;
      int random_number_ = rand() % 100;
      send_feedback(num_photos_, "Taking photo");
      std::cout << "\r\e[K" << std::flush;
      std::cout << "Taking " << num_photos_ << " photo/s ..." << std::endl;

      // MODIFICACIÓN EJERCICIO 2: CONTROLADOR (PROBABILIDAD 10%)
      if (random_number_ < 10) {

      // if (random_number_ < 40) {
        std::cout << "\r\e[K" << std::flush;
        std::cout << "Search finished" << std::flush;
        finish(true, 1.0, "Search finished");
        num_photos_ = 0;
        init_ = true;
        std::cout << std::endl;
      }
    }
  }
  bool init_;
  int num_photos_;
};

// EJERCICIO 3: GESTIÓN DE FALLOS
// private:
//   void do_work()
//   {
//     if (init_) {
//       init_ = false;
//       std::cout << "\r\e[K" << std::flush;
//       std::cout << "Taking photo ..." << std::endl;
//       send_feedback(num_photos_, "Searching person");
//       return;
//     }

//     int random_number_ = rand() % 100;
//     num_photos_ ++;

//     if (num_photos_ > 10) {
//       num_photos_ = 0;
//       init_ = true;
//       finish(false, num_photos_, "Search finished");
//       std::cout << "\r\e[K" << std::flush;
//       std::cout << "Failed to find person" << std::flush;
//       std::cout << std::endl;
//       return;
//     }

//     std::cout << "\r\e[K" << std::flush;
//     std::cout << "Taking " << num_photos_ << " photo/s ..." << std::endl;

//     if (random_number_ < 10) {
//       std::cout << "\r\e[K" << std::flush;
//       std::cout << "Search finished" << std::flush;
//       finish(true, num_photos_, "Search finished");
//       num_photos_ = 0;
//       init_ = true;
//       std::cout << std::endl;
//     }
//   }
//   bool init_;
//   int num_photos_;
// };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "search"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}