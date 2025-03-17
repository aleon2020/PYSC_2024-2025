#include <memory>
#include <random> 

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DroneSearchController : public rclcpp::Node
{
public:
  bool finished;

  DroneSearchController()
  : rclcpp::Node("drone_search_controller"), state_(SEARCH_PERSON)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"drone_1", "drone"});
    problem_expert_->addInstance(plansys2::Instance{"base", "location"});
    problem_expert_->addInstance(plansys2::Instance{"z1", "location"});
    problem_expert_->addInstance(plansys2::Instance{"z2", "location"});
    problem_expert_->addInstance(plansys2::Instance{"z3", "location"});
    problem_expert_->addInstance(plansys2::Instance{"z4", "location"});
    problem_expert_->addInstance(plansys2::Instance{"pers_1", "person"});
    problem_expert_->addInstance(plansys2::Instance{"pers_2", "person"});
    problem_expert_->addInstance(plansys2::Instance{"pers_3", "person"});
    problem_expert_->addInstance(plansys2::Instance{"pers_4", "person"});

    problem_expert_->addPredicate(plansys2::Predicate("(drone_at drone_1 base)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected base z1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z1 base)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected base z4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z4 base)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z1 z2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z2 z1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z1 z3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z3 z1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at pers_1 z1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at pers_2 z2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at pers_3 z3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at pers_4 z4)"));
  }

  void step()
  {
    switch (state_) {
      case SEARCH_PERSON:
        {
          if (persons_.size() == 0) {
            finished = true;
            break;
          }

          get_next_person();
          problem_expert_->setGoal(plansys2::Goal("(and (person_found " + person_ + "))"));

          std::cout << "Searching " << person_ << std::endl;

          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = PERSON_FOUND;
          }
        }
        break;
      case PERSON_FOUND:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            if (action_feedback.action == "search") {
              std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion << "]";
            } else if (action_feedback.action != "INIT") {
              std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion * 100.0 << "%]";
            }
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << person_ << " find " << std::endl; 

              problem_expert_->removePredicate(plansys2::Predicate("(person_found " + person_ + ")"));
 
              state_ = SEARCH_PERSON;
            } else {

              // MODIFICACIÓN EJERCICIO 3: GESTIÓN DE FALLOS
              // for (const auto & action_feedback : feedback.action_execution_status) {
              //   if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
              //     std::cout << "[" << action_feedback.action << "] finished with error: " <<
              //       action_feedback.message_status << std::endl;
              //   }
              // }

              std::cout << "Failed to find " << person_ << std::endl;

              // MODIFICACIÓN EJERCICIO 3: GESTIÓN DE FALLOS
              // persons_.push_back(person_);

              state_ = SEARCH_PERSON;
            }
          }
        }
        break;
      default:
        break;
    }
  }

private:
  typedef enum {SEARCH_PERSON, PERSON_FOUND} StateType;
  StateType state_;

  std::vector<std::string> persons_ = std::vector<std::string> {"pers_1", "pers_2", "pers_3", "pers_4"};
  std::string person_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  void get_next_person() {
    std::shuffle(persons_.begin(), persons_.end(), std::mt19937{std::random_device{}()});
    person_ = persons_.back();
    persons_.pop_back();
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneSearchController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok() && !node->finished) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}