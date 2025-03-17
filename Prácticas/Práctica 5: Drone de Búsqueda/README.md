[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/P7f30dhx)

# P4 - Drone de búsqueda

Disponemos de un sistema con un drone para aplicaciones de búsqueda y rescate.

El área de operación está compuesta por una base y por varias zonas en las que podría haber personas atrapadas (tal y como se muestra en la imagen). Inicialmente se sabe en qué zona están las personas atrapadas, pero no su ubicación exacta.

<p align="center">
  <img src="https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/media/search_scenario.png?raw=true">
</p>

El drone puede realizar las siguientes acciones:

* Volar entre dos zonas que estén conectadas.
* Detectar la posición exacta de una persona atrapada en una zona.

Para resolver el problema, el drone debe ir a cada una de las zonas en las que puede haber una persona y detectar la posición exacta de la persona para que pueda ser rescatada.

### Objetivo principal

El objetivo principal es encontrar a todas las personas que hay atrapadas en el escenario. No queremos que el planificador genere un plan "óptimo" que busque a todas las personas, porque unas pueden tener más prioridad que otras. Por este motivo, se deberá crear un controlador que utilice PlanSys2 para generar y controlar un plan independiente para la búsqueda de cada persona (de una en una).

## Antes de empezar

Este repositorio incluye un paquete de ROS 2 con la estructura necesaria para el sistema de control de la aplicación de búsqueda con el drone. Debes clonarlo dentro de tu workspace y compilarlo. Analiza bien los componentes incluidos en el paquete y estudia cómo están implementados.

En primer lugar, se ha creado un workspace y se ha clonado este repositorio en la carpeta `src/` del mismo y se le ha cambiado el nombre, ya que al clonarse, lo hará con el nombre `p4-drone-search-aleon2020`, por lo que lo se ha renombrado como `drone_search`.

```sh
mkdir -p plansys2_ws/src
```

```sh
cd src/
```

```sh
git clone https://<token>@github.com/Docencia-fmrico/p4-drone-search-aleon2020.git
```

```sh
mv p4-drone-search-aleon2020 drone_search
```

```sh
cd ..
```

```sh
colcon build --packages-select drone_search
```

Y por último, se ha añadido esta línea en el fichero .bashrc para que el workspace esté siempre activo.

```sh
source ~/plansys2_ws/install/setup.bash
```

### Dominio PDDL

Se proporciona una implementación del dominio en PDDL para el problema propuesto en [pddl/domain.pddl](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/pddl/domain.pddl). También se proporciona un problema de ejemplo en [pddl/problem.pddl](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/pddl/problem.pddl) y un archivo con comandos en [launch/commands.txt](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/launch/commands.txt) para la terminal de PlanSys2, pero éstos son únicamente para realizar pruebas sobre el PDDL.

Este es el plan generado al ejecutar el planificador POPF:

```sh
ros2 run popf popf domain.pddl problem.pddl 
```

```sh
0.000: (fly drone_1 base z1)  [4.000]
4.001: (search drone_1 pers_1 z1)  [10.000]
14.001: (fly drone_1 z1 z2)  [4.000]
18.001: (search drone_1 pers_2 z2)  [10.000]
28.001: (fly drone_1 z2 z1)  [4.000]
32.002: (fly drone_1 z1 z3)  [4.000]
36.002: (search drone_1 pers_3 z3)  [10.000]
46.002: (fly drone_1 z3 z1)  [4.000]
50.003: (fly drone_1 z1 base)  [4.000]
54.004: (fly drone_1 base z4)  [4.000]
58.004: (search drone_1 pers_4 z4)  [10.000]
```

## Ejercicio 1: Acciones

Implementa un nodo para ejecutar la acción `search` en un fichero llamado [search_action_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/search_action_node.cpp). Este nodo debe simular la acción de buscar una persona en una zona, y tendrá las siguientes características:

* Frecuencia de actualización: 1Hz. Se realizarán llamadas al método `do_work` cada segundo.
* En cada tick se tomará una imagen de la cámara y se lanzará el algoritmo de detección. El nodo llevará una cuenta del número de imágenes tomadas para enviar el feedback.
* En cada imagen existe una probabilidad del 40% de que la persona sea detectada.
* La acción finalizará con éxito cuando detecte a la persona.

Indica cómo has implementado el nodo para esta acción y qué más archivos has tenido que modificar para instalarlo y ejecutarlo:

**FICHERO [search_action_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/search_action_node.cpp)**

```c++
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
      if (random_number_ < 40) {
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
```

Para tener en cuenta esta acción a la hora de ejecutar el paquete, deben añadirse las siguientes líneas al fichero [CMakeLists.txt](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/CMakeLists.txt):

```sh
add_executable(search_action_node src/search_action_node.cpp)
ament_target_dependencies(search_action_node ${dependencies})

install(TARGETS
  search_action_node
  ...
)
```

Y por último, también deben añadirse las siguientes líneas al fichero [drone_search_launch.py](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/launch/drone_search_launch.py) para que se ejecute dicha acción al lanzar el paquete:

```sh
search_cmd = Node(
    package='drone_search',
    executable='search_action_node',
    name='search_action_node',
    namespace=namespace,
    output='screen',
    parameters=[])

ld.add_action(search_cmd)
```

**Nota**: Una vez implementada esta acción se puede probar el funcionamiento del sistema utilizando la terminal de PlanSys2 con los comandos disponibles en [pddl/commands.txt](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/launch/commands.txt).

Si se quiere lanzar el paquete utilizando la terminal de PlanSys2, se deben ejecutar las siguientes instrucciones, cada una de ellas en una terminal diferente:

```sh
ros2 launch drone_search drone_search_launch.py
```
```sh
ros2 run plansys2_terminal plansys2_terminal
```

Los comandos introducidos en la terminal de PlanSys2 para ejecutar este dominio se encuentran en el fichero [commands.txt](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/launch/commands.txt) de la carpeta [launch/](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/tree/main/launch), y son los siguientes:

```txt
set instance drone_1 drone
set instance base location
set instance z1 location
set instance z2 location
set instance z3 location
set instance z4 location
set instance pers_1 person
set instance pers_2 person
set instance pers_3 person
set instance pers_4 person

set predicate (drone_at drone_1 base)
set predicate (connected base z1)
set predicate (connected z1 base)
set predicate (connected base z4)
set predicate (connected z4 base)
set predicate (connected z1 z2)
set predicate (connected z2 z1)
set predicate (connected z1 z3)
set predicate (connected z3 z1)

set predicate (person_at pers_1 z1)
set predicate (person_at pers_2 z2)
set predicate (person_at pers_3 z3)
set predicate (person_at pers_4 z4)

set goal (and (person_found pers_1) (person_found pers_2) (person_found pers_3) (person_found pers_4))
```

Y por último, se adjunta un vídeo en el que se muestra cómo ejecutar el paquete utilizando el dominio proporcionado en la terminal de PlanSys2:

https://github.com/user-attachments/assets/26003029-3ad1-4565-a2d9-dba44f4c70bd

## Ejercicio 2: Controlador

Implementa un nodo para controlar la aplicación de búsqueda en un fichero llamado [drone_search_controller_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/drone_search_controller_node.cpp).

La lógica del controlador se guiará por los siguientes puntos:

1. El controlador deberá almacenar una lista de las personas que ya han sido encontradas.
2. Al inicio, se eligirá una persona aleatoria de la lista (que no haya sido encontrada aún), se actualizará el `goal` del problema, y se generará un nuevo plan.
4. Después, se ejecutará el nuevo plan y se comprobará tanto el feedback recibido como el resultado del plan si éste termina.
5. La ejecución del plan se detendrá cuando se cumpla el objetivo o si el plan falla.
6. Tras finalizar la ejecución del plan para buscar a la persona seleccionada, se volverá al punto 2 para elegir un nuevo objetivo y generar un plan para buscar a la nueva persona elegida.
7. El programa terminará una vez encontradas todas las personas del escenario.

Recomendaciones que no es necesario seguir:

* La lista de personas se puede almacenar dentro de la clase del controlador como un vector de strings. Conforme el drone va encontrando a las personas, estas pueden ser "marcadas" en otra estructura de datos (como `std::map`) o eliminadas del vector.
* Una forma sencilla (aunque algo bruta) de seleccionar un elemento aleatorio de un vector es utilizando la función [std::shuffle](https://en.cppreference.com/w/cpp/algorithm/random_shuffle) para reordenar el vector aleatoriamente. Esto se puede aplicar a la lista de personas para elegir qué persona será la siguiente. Ejemplo:

```c++
// Randomize the list of persons
std::shuffle(persons_.begin(), persons_.end(), std::mt19937{std::random_device{}()});
```

* Se recomienda implementar una máquina de estados similar a la vista en clase del ejemplo [plansys2_patrol_navigation_example](https://github.com/PlanSys2/ros2_planning_system_examples/blob/rolling/plansys2_patrol_navigation_example/src/patrolling_controller_node.cpp). Recuerda que cuando no queden más personas que buscar, el controlador debe dejar de ejecutar acciones.

Una vez implementado el controlador, ejecuta el launch del sistema en una terminal y el controlador en otra y muestra los resultados:

**FICHERO [drone_search_controller_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/drone_search_controller_node.cpp)**

```cpp
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
              std::cout << "Failed to find " << person_ << std::endl;
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
```

Para tener en cuenta esta acción a la hora de ejecutar el paquete, deben añadirse las siguientes líneas al fichero [CMakeLists.txt](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/CMakeLists.txt):

```sh
add_executable(drone_search_controller_node src/drone_search_controller_node.cpp)
ament_target_dependencies(drone_search_controller_node ${dependencies})

install(TARGETS
  drone_search_controller_node
  ...
)
```

Si se quiere lanzar el paquete utilizando el controlador, se deben ejecutar las siguientes instrucciones, cada una de ellas en una terminal diferente:

```sh
ros2 launch drone_search drone_search_launch.py
```
```sh
ros2 run drone_search drone_search_controller_node
```

Y por último, se adjunta un vídeo en el que se muestra cómo ejecutar el paquete utilizando el dominio proporcionado a través del controlador implementado:

https://github.com/user-attachments/assets/9ea096c5-439b-459c-93f3-e05af6f47d74

Ahora, modifica el *action performer* de la acción `search` para que la probabilidad de detectar a una persona en una imagen sea sólo del 10%. Vuelve a ejecutar el sistema de planificación con el controlador implementado y muestra los resultados:

Para tener en cuenta este cambio, se ha modificado únicamente la siguiente línea en el fichero [search_action_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/search_action_node.cpp):

```cpp
// ANTES
if (random_number_ < 40) {

// DESPUÉS
if (random_number_ < 10) {
```

Y por último, se adjunta un vídeo en el que se muestra cómo ejecutar el paquete utilizando el dominio proporcionado a través del controlador implementado teniendo en cuenta esta modificación:

https://github.com/user-attachments/assets/87310fcc-640e-4ba4-87a0-801aeab8b220

## Ejercicio 3: Gestión de fallos

Cuando la probabilidad de detección es baja, es posible que el drone esté demasiado tiempo buscando en una zona ralentizando el resto de la operación. Por este motivo, se pide modificar el nodo de la acción `search` para que sólo procese hasta un número máximo de imágenes. Si el drone no ha conseguido encontrar a la persona en menos de 10 imágenes, la acción deberá finalizar, con un resultado fallido. Revisa la API de la clase [ActionExecutorClient](https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/ActionExecutorClient.cpp) para ver cómo finalizar la acción de esta forma. Puedes fijar el número máximo de imágenes a 10 y modificar la probabilidad de detección a los valores que consideres convenientes para probar tu implementación.

Nuevo funcionamiento de la acción `search`:

* En cada imagen existe una probabilidad del 10% de que la persona sea detectada. Cuando esto ocurre, la acción debe finalizar con resultado exitoso.
* Si no se ha encontrado a la persona después de 10 imágenes, la acción deberá finalizar con resultado fallido.

Indica qué cambios has tenido que realizar:

Para tener en cuenta estos cambios, se ha modificado la parte "private" de la clase SearchAction en el fichero [search_action_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/search_action_node.cpp):

```cpp
// ANTES
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
      if (random_number_ < 10) {
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

// DESPUÉS
private:
  void do_work()
  {
    if (init_) {
      init_ = false;
      std::cout << "\r\e[K" << std::flush;
      std::cout << "Taking photo ..." << std::endl;
      send_feedback(num_photos_, "Searching person");
      return;
    }
    int random_number_ = rand() % 100;
    num_photos_ ++;
    if (num_photos_ > 10) {
      num_photos_ = 0;
      init_ = true;
      finish(false, num_photos_, "Search finished");
      std::cout << "\r\e[K" << std::flush;
      std::cout << "Failed to find person" << std::flush;
      std::cout << std::endl;
      return;
    }
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Taking " << num_photos_ << " photo/s ..." << std::endl;
    if (random_number_ < 10) {
      std::cout << "\r\e[K" << std::flush;
      std::cout << "Search finished" << std::flush;
      finish(true, num_photos_, "Search finished");
      num_photos_ = 0;
      init_ = true;
      std::cout << std::endl;
    }
  }
  bool init_;
  int num_photos_;
};
```

Y por último, se adjunta un vídeo en el que se muestra cómo ejecutar el paquete utilizando el dominio proporcionado a través del controlador implementado teniendo en cuenta estas modificaciones:

https://github.com/user-attachments/assets/a9357746-71f0-4288-ad1a-70b01f8987a7

Después de modificar la acción de búsqueda para que pueda fallar, ajusta el controlador para que tenga en cuenta estos fallos. Si uno de los planes falla, la persona que estaba siendo buscada seguirá en la lista de personas a buscar, pero el controlador deberá elegir de nuevo a una persona aleatoria y generar un nuevo plan.

Indica qué cambios has tenido que realizar y muestra el funcionamiento final:

Para tener en cuenta este cambio, se han añadido las siguientes líneas de código dentro del estado "PERSON_FOUND" de la máquina de estados implementada en el fichero [drone_search_controller_node.cpp](https://github.com/Docencia-fmrico/p4-drone-search-aleon2020/blob/main/src/drone_search_controller_node.cpp):

```cpp
// ANTES
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
        std::cout << "Failed to find " << person_ << std::endl;
        state_ = SEARCH_PERSON;
      }
    }
  }

// DESPUÉS
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
        for (const auto & action_feedback : feedback.action_execution_status) {
          if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
            std::cout << "[" << action_feedback.action << "] finished with error: " <<
              action_feedback.message_status << std::endl;
          }
        }
        std::cout << "Failed to find " << person_ << std::endl;
        persons_.push_back(person_);
        state_ = SEARCH_PERSON;
      }
    }
  }
```

Y por último, se adjunta un vídeo en el que se muestra cómo ejecutar el paquete utilizando el dominio proporcionado a través del controlador implementado teniendo en cuenta esta modificación:

https://github.com/user-attachments/assets/a4962d07-3c60-4152-a627-793df2f7afbd
