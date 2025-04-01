[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/dMkD-Mt7)

# Trabajo Final Planificación - La Biblioteca

El trabajo final consiste en implementar en equipo un sistema de planificación para un robot que trabaje en una biblioteca. El robot será capaz de realizar una serie de tareas (misiones) que pueden estar compuestas por una o más acciones. Como mínimo el robot debe poder ejecutar 3 tareas distintas. A continuación se muestra un ejemplo de posibles tareas a ejecutar:

* Atender visitas
* Recoger los libros de las mesas
* Resolver el cubo de Rubik
* Buscar un libro en las estanterías
* Recoger la basura
* Mandar callar a las personas ruidosas

Se dispone de libertad para elegir las tareas y para definir el comportamiento del robot, con las siguientes condiciones:

* Se deben definir varios "waypoints" en la biblioteca (entrada, estanterías, mesas, etc) para las distintas tareas.
* Al menos una de las tareas deberá inlcuir una componente aleatoria. Por ejemplo:
    * Que a la hora de buscar un libro, el robot vaya recorriendo estanterías hasta que lo encuentre, y que se pueda dar el caso en el que no exista el libro.
    * Que la acción de atender las visitas implique acercarse a la puerta a ver si hay alguien. Si hay una persona, se acompaña a esa persona a un punto de la biblioteca. Si no hay ninguna persona, la misión termina. Que haya una persona o no se puede decidir de forma aleatoria en el instante de "detectar" a la persona.

## Ejercicio 1 - Mundo simulado

### Mundo

Se utilizará un mundo simulado en gazebo de una biblioteca. Podéis encontrar un mundo funcional en [este repositorio](https://github.com/Juancams/aws-robomaker-bookstore-world/tree/ros2):

<p align="center">
  <img src="https://github.com/Docencia-fmrico/trabajo-plansys2-envidio33/blob/main/media/sim_world.png?raw=true">
</p>

### Robot

Se puede utilizar el modelo de cualquier robot, aunque se recomiendan los simuladores del [Kobuki](https://github.com/IntelligentRoboticsLabs/kobuki) o del [TIAGO](https://github.com/Tiago-Harmonic/tiago_harmonic), que ya se han utilizado en otras asignaturas.

Al utilizar un robot de verdad, las tareas de navegación deberán utilizar nav2 para que el robot se mueva de verdad. El resto de las acciones pueden seguir siendo sintéticas. Es decir, no es necesario que el robot interactúe con los objetos de la biblioteca.

Indicar qué simulador se ha utilizado, cómo ejecutarlo y cómo lanzar el sistema de planificación para que actúe sobre el robot:

A continuación, se detallan los pasos a seguir para instalar, configurar y compilar el paquete correspondiente al robot Kobuki, cuya simulación se llevará a cabo en Gazebo.

**PASO 1**: En primer lugar, se crea un nuevo workspace o se utiliza uno en el que ya esté instalado todo lo relativo a PlanSys2.

```sh
mkdir -p plansys2_ws/src
```

```sh
cd plansys2_ws/src/
```

**PASO 2**: Después, dentro de la carpeta `src/` del workspace, se debe clonar tanto el paquete correspondiente al sistema de planificación como el del Kobuki, del cual se deben instalar los ThirdParties.

**NOTA**: Se debe cambiar el nombre del paquete correspondiente al sistema de planificación debido a temas de legibilidad, ya que al clonarse, lo hará con el nombre `trabajo-plansys2-envidio33`, por lo que debe renombrarse como `trabajo_plansys2_envidio33`.

```sh
git clone https://<token>@github.com/Docencia-fmrico/trabajo-plansys2-forocoches.git
```

```sh
mv trabajo-plansys2-envidio33 trabajo_plansys2_envidio33
```

```sh
git clone https://<token>@github.com/IntelligentRoboticsLabs/kobuki.git
```

```sh
vcs import < kobuki/thirdparty.repos
```

**PASO 3**: Una vez hecho esto, se realizan las siguientes modificaciones en los ficheros indicados.

* Añade la siguiente línea en el fichero `ThirdParty/openni2_camera/openni2_camera/CMakeLists.txt`.

```txt
find_package(Boost 1.45.0 COMPONENTS date_time)
```

* Configura correctamente el mundo que se utilizará para la simulación en el fichero `kobuki/launch/simulation.launch.py`.

```py
    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('aws_robomaker_bookstore_world'),
            'worlds',
            'bookstore.world'))
```

**PASO 4**: Y por último, ya en el raíz del workspace, se ejecutan los siguientes comandos con el fin de compilar todos los paquetes añadidos al workspace.

```sh
cd ..
```

```sh
sudo rosdep init
```

```sh
rosdep update
```

```sh
rosdep install --from-paths src --ignore-src -r -y
```

```sh
colcon build --symlink-install --parallel-workers 1
```

Para verificar que todo se ha instalado, configurado y compilado correctamente, se ejecuta el modelo utilizando Gazebo para la simulación y RViz2 para la navegación, los cuales se ejecutan de la siguiente forma, cada uno de ellos en una terminal diferente.

```sh
ros2 launch kobuki simulation.launch.py
```

```sh
ros2 launch kobuki navigation_sim.launch.py
```

Y por último, para lanzar el paquete, se ejecutan los siguientes comandos, cada uno de ellos en una terminal diferente.

```sh
ros2 launch trabajo_plansys2_envidio33 trabajo_plansys2_envidio33.launch.py
```

```sh
ros2 run trabajo_plansys2_envidio33 trabajo_plansys2_envidio33_controller_node
```

## Ejercicio 2 - Dominio PDDL
### Domain.pddl
**Tipos**
```sh
(:types 
  hall table bookshelf - location
  book miscellaneus - prop
  visitor
  robot
)
```
Primero de todo definimos los distintos puntos de interés bajo el mismo tag _location_. De igual manera definimos el cubo y la basura mixta bajo un tag _prop_. Terminamos definiendo dos tipos distintos, uno para el robot/robots que operen en la librería, y otro para todos los visitantes.


**Predicados**
```sh
(:predicates
  (robot_at ?r - robot ?l - location)
  (object_at ?o - prop ?l - location)
  (visitor_at ?v - visitor ?l - location)
  (noise_at ?l - location)
  (connected ?l1 ?l2 - location)

  (gripper_free ?r - robot)
  (holding ?r - robot ?o - prop)
  (robot_not_busy ?r - robot)

  (solved ?m - miscellaneus)
  (book_found ?b - book ?l - bookshelf)
  (quiet ?l - location)
)
```
Creamos primero los predicados relacionados con la localización y navegación, ya que estos son los más básicos para el funcionamiento del robot. Luego añadimos unos predicados para comunicar información sobre el estado del robot, y concluimos con un grupo de predicados para representar información del entorno.


**Acciones**\
Queremos que nuestro robot pueda cumplir varios propósitos: Buscar libros, resolver cubos de rubik, mandar callar en las zonas más ruidosas, y guiar a los clientes por la tienda.

Una de las primeras acciones que tenemos que aplicar es la de moverse, ya que es necesaria para cumplir el resto de propósitos.
```sh
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 2)
  :condition
    (and 
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and
      (at start (not (robot_not_busy ?r)))
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (robot_not_busy ?r))
    )
)
```
Basandonos en esta accion creamos la accion para guiar a los clientes, ya que son muy similares en funcionamiento.
```sh
(:durative-action guide_visitor
  :parameters (?r - robot ?v - visitor ?from ?to - location)
  :duration (= ?duration 2)
  :condition 
    (and 
      (at start (robot_at ?r ?from)) 
      (at start (visitor_at ?v ?from))
      (at start (robot_not_busy ?r))
      (over all (connected ?from ?to))
    )
  :effect 
    (and 
      (at start (not (robot_not_busy ?r)))
      (at start (not (visitor_at ?v ?from))) 
      (at start (not (robot_at ?r ?from)))
      (at end (visitor_at ?v ?to))
      (at end (robot_at ?r ?to))
      (at end (robot_not_busy ?r))
    )
)
```
Con todas las acciones generales completadas podemos centrarnos en terminar las acciones própias de cada requisito.

Resolver el cubo
```sh
(:durative-action solve
  :parameters (?r - robot ?m - miscellaneus ?l - location)
  :duration (= ?duration 2)
  :condition 
    (and 
      (over all (robot_at ?r ?l))
      (over all (holding ?r ?m))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and 
      (at end (solved ?m)) 
      (at end (robot_not_busy ?r))
    )
)
```
Buscar un libro
```sh
(:durative-action search_book
  :parameters (?r - robot ?b - book ?l - bookshelf)
  :duration (= ?duration 2)
  :condition 
    (and 
      (over all (robot_at ?r ?l))
      (over all (object_at ?b ?l))
      (at start (gripper_free ?r))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and 
      (at start (not (robot_not_busy ?r)))
      (at end (book_found ?b ?l))
      (at end (robot_not_busy ?r))
    )
)
```
Mandar callar
```sh
(:durative-action shut_up
  :parameters (?r - robot ?l - location)
  :duration (= ?duration 2)
  :condition 
    (and 
      (over all (robot_at ?r ?l))
      (at start (noise_at ?l))
      (at start (robot_not_busy ?r))
    )
  :effect 
    (and 
      (at start (not (robot_not_busy ?r)))
      (at start (not (noise_at ?l)))
      (at end (quiet ?l))
      (at end (robot_not_busy ?r))
    )
)
```
### Problem.pddl
Empezamos con un diagrama en papel para organizarnos. Definimos claramente los puntos de interés y sus localizaciones.
![Diagrama](https://github.com/user-attachments/assets/7a46cea4-1326-4114-9dd0-67726dd69b9c)
Tras hacer el diagrama podemos definir los objetos del escenario.
```sh
(:objects 
  peepo - robot
  entrance origin - hall
  large_table1 large_table2 rubik_table big_table1 big_table2 - table
  bookshelf1 bookshelf5 bookshelf6 bookshelf7 - bookshelf
  book1 book2 book3 book4 - book
  rubik_cube trash - miscellaneus
  visitor1 - visitor
)
```
Con la definicíón completa, pasamos a declarar el estado inicial de la simulación.
```sh
(:init 
  (robot_at peepo origin)
  (robot_not_busy peepo)

  (gripper_free peepo)

  (object_at book1 bookshelf1)
  (object_at book2 bookshelf5)
  (object_at book3 large_table1)
  (object_at book4 large_table2)
  (object_at rubik_cube rubik_table)
  (object_at trash big_table1)
  
  (visitor_at visitor1 entrance)

  (noise_at large_table1)
  (noise_at large_table2)

  (connected [...])
)
```

## Ejercicio 3 - Acciones

Implementamos las distintas acciones en PlanSys2 como nodos de BehaviorTree.

![image](https://github.com/user-attachments/assets/3b09414c-e0a9-4f30-a178-96652c5377b9)

El resto de acciones a realizar por el robot pueden ser sintéticas, donde la acción habrá terminado después de que haya pasado un tiempo determinado.

### Composición de acciones con nodos de BT
Las acciones simples (_move_, _pick_ y _drop_) no tienen sub-acciones, las acciones que merecen atencion son estas:

1. Guide Visitor
```sh
<root BTCPP_format="4" main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <PickUp name="pick_up"/>
           <Move name="move" goal="{arg3}"/>
           <Move name="move" goal="{arg4}"/>
           <Leave name="leave"/>
       </Sequence>
    </BehaviorTree>
</root>
```
2. Move Object
```sh
<root BTCPP_format="4" main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <Pick name="pick"/>
           <Move name="move" goal="{arg3}"/>
           <Move name="move" goal="{arg4}"/>
           <Drop name="drop"/>
       </Sequence>
    </BehaviorTree>
</root>
```
3. Search Book
```sh
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Move name="move" goal="{arg3}"/>
            <Search name="Search" book="{arg2}"/>
        </Sequence>
    </BehaviorTree>
</root>
```
4. Shut Up
```sh
<root BTCPP_format="4" main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <StandUp name="stand_up"/>
           <SayShhh name="say_shhh"/>
           <StandDown name="stand_down"/>
       </Sequence>
    </BehaviorTree>
</root>
```

## Ejercicio 4 - Planner
Como nuestro planner de la práctica anterior no soportaba _durative actions_, hemos decidido a pasar a OPTIC.\
Las instrucciones de uso del plugin de optic se encuentran en el README de la carpeta plugin

## Vídeo final

Para finalizar, se debe incluir un vídeo del sistema funcionando, en el que se pueda apreciar el desarrollo realizado.

https://github.com/user-attachments/assets/5760d737-8f06-4ce7-9999-eacc96bdc299

Este es el plan que sigue:

![Screenshot from 2025-03-31 02-13-33](https://github.com/user-attachments/assets/9ee4059a-f9f0-4c63-8dfa-d2c014a5c8df)


## Presentación

Se realizará una presentación de 10 minutos del trabajo en clase y deberéis añadir también al repositorio los materiales (slides, vídeos, etc) utilizados en la presentación.
