[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/rmR76L2s)

# P3 - Introducción a PlanSys2

El propósito de esta práctica es familiarizarse con el uso de PlanSys2, usando la terminal para definir el problema e implementando acciones sencillas en ROS 2.

Para ello, en este repositorio deberás crear un paquete de ROS 2 para implementar las acciones necesarias y para poder lanzar el sistema mediante `ros2 launch`. Es recomendable basarse en el ejemplo sencillo que se encuentra en [plansys2_simple_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example).

En primer lugar, he creado un workspace y he clonado este repositorio en la carpeta `src/` del mismo y le he cambiado el nombre, ya que al clonarse, lo hará con el nombre `p3-plansys2-intro-aleon2020`, por lo que lo he renombrado como `exploration_example`. 

```sh
mkdir -p plansys2_ws/src
```

```sh
cd src/
```

```sh
git clone https://<token>@github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020.git
```

```sh
mv p3-plansys2-intro-aleon2020 exploration_example
```

```sh
cd ..
```

```sh
colcon build --packages-select exploration_example
```

Y por último, he añadido esta línea en el fichero .bashrc para que el workspace esté siempre activo.

```sh
source ~/plansys2_ws/install/setup.bash
```

## Ejercicio 1

Crea un paquete de ROS 2 basado en CMake con nombre `exploration_example` que incluya al menos las siguientes dependencias:

* rclcpp
* plansys2_msgs
* plansys2_executor
* plansys2_bringup (sólo ejecución)
* plansys2_terminal (sólo ejecución)

Indica qué comandos has utilizado para crear el paquete, y si has copiado el paquete de ejemplo de [plansys2_simple_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example), indica qué cosas has cambiado en el paquete para adaptarlo.

Para crear el paquete, me he copiado en la carpeta `exploration_example/` todo el contenido del paquete [plansys2_simple_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example).

Una vez hecho esto, he modificado las líneas de cada uno de los ficheros correspondientes.

**FICHERO [plansys2_simple_example_launch.py](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/launch/exploration_example_launch.py)**

```py
example_dir = get_package_share_directory('exploration_example')
```

**FICHERO [CMakeLists.txt](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/CMakeLists.txt)**

```txt
project(exploration_example)
```

**FICHERO [package.xml](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/package.xml)**

```xml
<name>exploration_example</name>
```

## Ejercicio 2

Añade el código PDDL del dominio de la Práctica 1 (Space Exploration) a la carpeta [`pddl/`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/tree/main/pddl). Puedes utilizar el dominio de ejemplo que venía con la plantilla o cualquiera de tus modificaciones. Recuerda que este dominio debe definir al menos las siguientes acciones:

* **move**: El robot se mueve de un punto a otro.
* **collect**: El robot recoge una muestra con la pinza. La muestra pasa a estar siendo agarrada por la pinza.
* **drop**: El robot suelta una muestra en una ubicación.
* **analyse_soil**: El robot analiza el suelo en una ubicación.

**IMPORTANTE:** Es necesario modificar este dominio para que las acciones sean durativas. Puedes fijar la duración de las acciones a cualquier valor que consideres razonable.

**FICHERO [exploration_domain_example.pddl](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/pddl/exploration_domain_example.pddl)**

```pddl
(define (domain exploration_example_domain)
(:requirements :strips :typing :durative-actions)


; Types definition
(:types
  location
  robot
  sample
  soil
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (soil_at ?s - soil ?l - location)
  (soil_analysed ?sl - soil)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?s - sample)
  (occupied_robot ?r - robot)
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 10)
  :condition
    (and 
      (at start (robot_at ?r ?from))
      (at start (occupied_robot ?r))
    )
  :effect
    (and
      (at start (not (occupied_robot ?r)))
      (at end (robot_at ?r ?to))
      (at start (not (robot_at ?r ?from)))
      (at end (occupied_robot ?r))
    )
)

; Collect action. The robot collects a sample at a location.
; Both the robot and the sample must be in that location.
; The robot's gripper must be free (the robot can only hold 1 sample).
; Consequences:
;   - The sample is no longer at the given location.
;   - The robot is now carrying the sampple and its gripper is not free.
(:durative-action collect
  :parameters (?s - sample ?l - location ?r - robot)
  :duration (= ?duration 4)
  :condition 
    (and
      (at start (sample_at ?s ?l))
      (at start (robot_at ?r ?l))
      (at start (gripper_free ?r))
      (at start (occupied_robot ?r))
    )
  :effect
    (and
      (at start (not (occupied_robot ?r)))
      (at end (robot_carry ?r ?s))
      (at start (not (sample_at ?s ?l)))
      (at start (not (gripper_free ?r)))
      (at end (occupied_robot ?r))
    )
)

; Drop-off action. The robot drops a sample at a location.
; The robot must be in that location and must be carrying that sample.
; Consequences:
;   - The sample is now at the given location.
;   - The robot is no longer carrying the sample and its gripper is free.
(:durative-action drop
  :parameters (?s - sample ?l - location ?r - robot)
  :duration (= ?duration 1)
  :condition
    (and 
      (at start (robot_at ?r ?l))
      (at start (robot_carry ?r ?s))
      (at start (occupied_robot ?r))
    )
  :effect 
    (and 
      (at start (not (occupied_robot ?r)))
      (at end (sample_at ?s ?l))
      (at end (gripper_free ?r))
      (at start (not (robot_carry ?r ?s)))
      (at end (occupied_robot ?r))
    )
)

; Analyse Soil action. The robot samples and analyses the soil at a location.
; The robot must be in that location.
; Consequences:
;   - The soil is now analysed.
(:durative-action analyse_soil
  :parameters (?s - soil ?l - location ?r - robot)
  :duration (= ?duration 15)
  :condition
    (and 
      (at start (robot_at ?r ?l))
      (at start (soil_at ?s ?l))
      (at start (occupied_robot ?r))
    )
  :effect 
    (and
      (at start (not (occupied_robot ?r)))
      (at end (soil_analysed ?s))
      (at end (occupied_robot ?r))
    )
)

)
```

**FICHERO [exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/pddl/exploration_problem_example.pddl)**

```pddl
(define (problem exploration_example_problem)
(:domain exploration_example_domain)

; We define 3 different samples, 2 types of trash bins, one table and a robot
(:objects
  curiosity - robot
  base valley crater - location
  rock mineral - sample
  valley_soil crater_soil - soil
)

; Initially the robot is at the base
(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (sample_at rock crater)
  (sample_at mineral valley)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
  (occupied_robot curiosity)
)

; The goal is to make science!
(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
  )
)

)
```

Después de añadir y adaptar el dominio PDDL, añade un fichero [exploration_example_launch.py](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/launch/exploration_example_launch.py) a la carpeta [`launch/`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/tree/main/launch), o modifica el fichero del ejemplo, para que utilice el archivo PDDL con el dominio que acabas de incluir en el paquete.

Para ello, he renombrado el fichero `plansys2_simple_example_launch` a [`exploration_example_launch.py`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/launch/exploration_example_launch.py), y he modificado la siguiente línea para poder incluir el nuevo dominio en el paquete.

```sh
'model_file': example_dir + '/pddl/exploration_domain_example.pddl'
```

**Nota:** La carpeta [`pddl/`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/tree/main/pddl) debe ser marcada en el [`CMakeLists.txt`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/CMakeLists.txt) para ser instalada.

## Ejercicio 3

Para cada una de las acciones definidas en el dominio, es necesario implementar un nodo de ROS 2 (un *action performer*) en un archivo `.cpp` dentro de la carpeta [`src/`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/tree/main/src) del paquete. De momento, la implementación de las acciones no debe hacer nada complejo. Es suficiente con hacer un contador que haga terminar la acción en x segundos, siguiendo la implementación que puedes encontrar en las acciones del ejemplo propuesto.

Para cada acción es necesario realizar lo siguiente:

1. Implementar el nodo de ROS 2 en una clase que herede de `plansys2::ActionExecutorClient`.

A continuación se muestra un 'fichero esqueleto' correspondiente a cada uno de los ficheros CPP (ficheros [`move_action_node.cpp`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/src/move_action_node.cpp), [`collect_action_node.cpp`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/src/collect_action_node.cpp), [`drop_action_node.cpp`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/src/drop_action_node.cpp) y [`analyse_soil_action_node.cpp`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/src/analyse_soil_action_node.cpp)).

**FICHERO X_action_node.cpp**

```cpp
#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class XAction : public plansys2::ActionExecutorClient
{
public:
  XAction()
  : plansys2::ActionExecutorClient("X", t)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    progress_ += percentage;
    if (progress_ < 1.0) {
      send_feedback(progress_, "X running");
    } else {
      finish(true, 1.0, "X completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "X ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "X"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
```

**NOTA**: Para ajustar correctamente la duración de la acción en PlanSys2 acorde a la duración establecida en el dominio PDDL, se sigue la siguiente ecuación.

<p align="center">
duración acción PDDL (segundos) = t (segundos) / percentage (%)
</p>

2. Añadir las instrucciones necesarias al [`CMakeLists.txt`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/CMakeLists.txt) para que se genere el ejecutable de la acción y se instale.

```txt
add_executable(X_action_node src/X_action_node.cpp)
ament_target_dependencies(X_action_node ${dependencies})
```

3. Añadir la acción al fichero [exploration_example_launch.py](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/launch/exploration_example_launch.py) para que se ejecute al lanzarlo todo junto.

```py
    X_cmd = Node(
        package='exploration_example',
        executable='X_action_node',
        name='X_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    ld.add_action(X_cmd)
```

## Ejercicio 4

Una vez esté todo implementado, configurado y compilado / instalado, es posible lanzar el sistema con la siguiente instrucción:

```bash
ros2 launch exploration_example exploration_example_launch.py
```

En otra terminal, abre la terminal de Plansys2 con el siguiente comando:

```bash
ros2 run plansys2_terminal plansys2_terminal
```

En la terminal de Plansys2 debes ejecutar los comandos necesarios para definir el problema. Puedes basarte en el problema de la Práctica 1.

Indica qué comandos has introducido en la terminal de Plansys2 para definir el problema (objetos, predicados, etc.) y para definir el objetivo (goal) del problema:

Los comandos introducidos en la terminal de Plansys2 para definir todos los elementos del problema (objetos, predicados, estado inicial y estado objetivo) se encuentran en el fichero [`commands`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/blob/main/launch/commands) en la carpeta [`launch/`](https://github.com/Docencia-fmrico/p3-plansys2-intro-aleon2020/tree/main/launch).

```sh
set instance curiosity robot
set instance base location
set instance valley location
set instance crater location
set instance rock sample
set instance mineral sample
set instance valley_soil soil
set instance crater_soil soil

set predicate (robot_at curiosity base)

set predicate (gripper_free curiosity)

set predicate (sample_at rock crater)
set predicate (sample_at mineral valley)

set predicate (soil_at valley_soil valley)
set predicate (soil_at crater_soil crater)

set predicate (occupied_robot curiosity)

set goal (and (sample_at rock base) (sample_at mineral base) (soil_analysed valley_soil) (soil_analysed crater_soil))
```

```sh
get plan
```

```sh
0:	(move curiosity base crater)	[10]
10.001:	(analyse_soil crater_soil crater curiosity)	[15]
25.002:	(collect rock crater curiosity)	[4]
29.003:	(move curiosity crater base)	[10]
39.004:	(drop rock base curiosity)	[1]
40.005:	(move curiosity base valley)	[10]
50.006:	(collect mineral valley curiosity)	[4]
54.007:	(analyse_soil valley_soil valley curiosity)	[15]
69.008:	(move curiosity valley base)	[10]
79.009:	(drop mineral base curiosity)	[1]
```

```sh
run
```

```sh
Moving ... [100%]  
Analysing soil ... [100%]  
Collecting sample ... [100%]  
Moving ... [100%]  
Dropping sample ... [100%]  
Moving ... [100%]  
Collecting sample ... [100%]  
Analysing soil ... [100%]  
Moving ... [100%]  
Dropping sample ... [100%]
```

## Ejercicio 5

Vuelve a ejecutar el sistema de planificación y añade el problema PDDL a través de la terminal de PlanSys2, pero sin ejecutar aún el plan.

A continuación, deberás observar los topics `/actions_hub` y `/action_execution_info` y ejecutar la primera acción del plan desde la terminal.

Indica qué comandos has utilizado en la terminal de Plansys2 para obtener el plan y ejecutar la primera acción:

```sh
get plan
```

```sh
0:	(move curiosity base crater)	[10]
10.001:	(analyse_soil crater_soil crater curiosity)	[15]
25.002:	(collect rock crater curiosity)	[4]
29.003:	(move curiosity crater base)	[10]
39.004:	(drop rock base curiosity)	[1]
40.005:	(move curiosity base valley)	[10]
50.006:	(collect mineral valley curiosity)	[4]
54.007:	(analyse_soil valley_soil valley curiosity)	[15]
69.008:	(move curiosity valley base)	[10]
79.009:	(drop mineral base curiosity)	[1]
```

```sh
run num_actions 1
```

```sh
Moving ... [100%]
```

Finalmente, analiza los mensajes que se han publicado en los topics `/actions_hub` y `/action_execution_info`.
¿Qué información ha sido publicada en cada topic? ¿Qué nodos han publicado los mensajes?

**TOPIC /actions_hub**

```sh
ros2 topic echo /actions_hub
```

```sh
type: 1
node_id: executor
action: X
arguments:
- arg1
- ...
- argN
success: false
completion: 0.0
status: ''
---
type: 2
node_id: X_action_node
action: X
arguments:
- arg1
- ...
- argN
success: false
completion: 0.0
status: ''
---
type: 3
node_id: X_action_node
action: X
arguments:
- arg1
- ...
- argN
success: false
completion: 0.0
status: ''
---
type: 5
node_id: X_action_node
action: X
arguments:
- arg1
- ...
- argN
success: false
completion: percentage
status: X running
---
type: 6
node_id: X_action_node
action: X
arguments:
- arg1
- ...
- argN
success: true
completion: 1.0
status: X completed
---
```

La información mostrada por el topic `/actions_hub` se compone de cuatro tipos de bloques:

**TIPO 1: ANUNCIO DE ACCIÓN DISPONIBLE**

Se publica un mensaje de tipo 1 en el topic `/actions_hub`, lo que indica que una acción puede ser ejecutada. Este tipo de mensaje especifica el nombre de la acción y sus argumentos. Inicialmente, el estado de success está a false, completion es 0.0 y status está vacío.

**TIPO 2 Y 3: SOLICITUD DE EJECUCIÓN A LOS NODOS DISPONIBLES**

Los nodos candidatos para ejecutar la acción responden con un mensaje de tipo 2 y 3, indicando que están interesados en la acción. Estos nodos, identificados por node_id, informan que están dispuestos a ejecutar la acción con los argumentos proporcionados.

**TIPO 5: EJECUCIÓN DE LA ACCIÓN**

Una vez seleccionado un nodo para ejecutar la acción, se envía un mensaje de tipo 5, donde se actualiza el estado de ejecución. El campo completion cambia progresivamente desde 0.0 hasta un porcentaje intermedio mientras la acción está en ejecución, y el campo status se actualiza a X running, indicando que la acción se encuentra en ejecución.

**TIPO 6: FINALIZACIÓN DE LA ACCIÓN**

Al completar la acción, se publica un mensaje de tipo 6, donde completion alcanza 1.0 y status se actualiza a X completed.
Además, success se establece en true, indicando que la ejecución de la acción ha sido exitosa.

**TOPIC /action_execution_info**

```sh
ros2 topic echo /action_execution_info
```

```sh
status: 1
start_stamp:
  sec: t1
  nanosec: t2
status_stamp:
  sec: t1
  nanosec: t2
action_full_name: (action arg1 arg2 argN):ID
action: X
arguments:
- arg1
- arg2
- argN
duration:
  sec: duration
  nanosec: 0
completion: 0.0
message_status: ''
---
status: 2
start_stamp:
  sec: t1
  nanosec: t2
status_stamp:
  sec: t1
  nanosec: t2
action_full_name: (action arg1 arg2 argN):ID
action: X
arguments:
- arg1
- arg2
- argN
duration:
  sec: duration
  nanosec: 0
completion: t
message_status: X running
---
status: 4
start_stamp:
  sec: t1
  nanosec: t2
status_stamp:
  sec: t1
  nanosec: t2
action_full_name: (action arg1 arg2 argN):ID
action: X
arguments:
- arg1
- arg2
- argN
duration:
  sec: duration
  nanosec: 0
completion: 1.0
message_status: X completed
---
```

El topic `/action_execution_info` complementa la información de `/actions_hub`, proporcionando detalles sobre el estado de ejecución de cada acción. Como se puede observar, se tienen tres estados diferentes al mostrar el contenido del topic `/action_execution_info`.

* **status: 1**: Indica el inicio de la acción.

* **status: 2**: Indica que la acción se está ejecutando.

* **status: 4**: Indica que la acción ha finalizado su ejecución.

Basándote en la información publicada en el topic `/actions_hub`, explica cómo funciona el sistema de subasta de acciones de PlanSys2.

El sistema de subasta de acciones en PlanSys2 funciona mediante la publicación de mensajes en el topic `/actions_hub`, en el que diferentes nodos pueden participar en el proceso de asignación / ejecución de acciones. Además, permite que múltiples nodos compitan entre ellos para ejecutar una acción, donde se elige al mejor candidato disponible mientras se actualiza su estado en el topic.
