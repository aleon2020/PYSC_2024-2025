#include <memory>

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TrabajoPlanSys2Envidio33 : public rclcpp::Node
{
public:
  TrabajoPlanSys2Envidio33()
  : rclcpp::Node("trabajo_plansys2_envidio33_controller")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"peepo", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"entrance", "hall"});
    problem_expert_->addInstance(plansys2::Instance{"origin", "hall"});
    problem_expert_->addInstance(plansys2::Instance{"large_table1", "table"});
    problem_expert_->addInstance(plansys2::Instance{"large_table2", "table"});
    problem_expert_->addInstance(plansys2::Instance{"big_table1", "table"});
    problem_expert_->addInstance(plansys2::Instance{"big_table2", "table"});
    problem_expert_->addInstance(plansys2::Instance{"rubik_table", "table"});
    problem_expert_->addInstance(plansys2::Instance{"bookshelf1", "bookshelf"});
    problem_expert_->addInstance(plansys2::Instance{"bookshelf5", "bookshelf"});
    problem_expert_->addInstance(plansys2::Instance{"bookshelf6", "bookshelf"});
    problem_expert_->addInstance(plansys2::Instance{"bookshelf7", "bookshelf"});
    problem_expert_->addInstance(plansys2::Instance{"book1", "book"});
    problem_expert_->addInstance(plansys2::Instance{"book2", "book"});
    problem_expert_->addInstance(plansys2::Instance{"book3", "book"});
    problem_expert_->addInstance(plansys2::Instance{"book4", "book"});
    problem_expert_->addInstance(plansys2::Instance{"rubik_cube", "miscellaneus"});
    problem_expert_->addInstance(plansys2::Instance{"trash", "miscellaneus"});
    problem_expert_->addInstance(plansys2::Instance{"visitor1", "visitor"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at peepo origin)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_not_busy peepo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(gripper_free peepo)"));
    
    problem_expert_->addPredicate(plansys2::Predicate("(object_at book1 bookshelf1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at book2 bookshelf5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at book3 large_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at book4 large_table2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at rubik_cube rubik_table)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at trash big_table1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(visitor_at visitor1 entrance)"));

    problem_expert_->addPredicate(plansys2::Predicate("(noise_at large_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(noise_at large_table2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected origin entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance large_table2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected large_table2 entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance rubik_table)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected rubik_table entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance bookshelf1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bookshelf1 entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance big_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected big_table1 entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected large_table1 big_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected big_table1 large_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected large_table2 rubik_table)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected rubik_table large_table2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected large_table2 big_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected big_table1 large_table2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bookshelf5 bookshelf6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bookshelf6 bookshelf5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bookshelf6 bookshelf7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bookshelf7 bookshelf6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bookshelf7 big_table1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected big_table1 bookshelf7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected big_table1 big_table2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected big_table2 big_table1)"));

    problem_expert_->setGoal(
      plansys2::Goal(
        "(and (object_at book3 bookshelf1) (object_at book4 bookshelf6) (visitor_at visitor1 large_table1) (solved rubik_cube) (object_at trash entrance) (quiet large_table2) (quiet large_table1))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrabajoPlanSys2Envidio33>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
