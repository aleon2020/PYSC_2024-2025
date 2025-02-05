[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/9J2PKxcC)

# P1 - Exploración Espacial

El propósito de esta práctica es modelar con PDDL un entorno de exploración espacial, en el que un robot debe recoger muestras del entorno y analizarlas.

Las tareas principales del robot son recoger muestras (samples) como rocas o minerales para llevarlas a la base, y hacer mediciones en el suelo para analizarlas en busca de agua o indicios de vida.

## Ejemplo sencillo

Los archivos [exploration_domain_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/pddl/exploration_domain_example.pddl) y [exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/pddl/exploration_problem_example.pddl) contienen un dominio y un problema de ejemplo con la funcionalidad más básica.

En este ejemplo, se asume que el robot dispone de una base móvil con una pinza (gripper) con la capacidad de coger y soltar objetos.

Además, en el dominio propuesto asumimos que el robot se puede mover libremente de una ubicación a otra.

**[Ejercicio de calentamiento 1:](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/tree/main/Ejercicio%20de%20Calentamiento%201)** Analiza el ejemplo propuesto y ejecuta el planificador (POPF) para ver el plan que genera. Puedes cambiar la definición del problema para ver cómo cambian los planes.

*Nota: Puedes encontrar instrucciones para instalar y ejecutar POPF en el archivo [POPF_INSTRUCTIONS](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/POPF_INSTRUCTIONS.md).*

**ANÁLISIS DEL DOMINIO ([exploration_domain_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%20de%20Calentamiento%201/exploration_domain_example.pddl))**

**Tipos**

- location: Indica la localización del robot o de la muestra.

- robot: Indica qué robot está realizando las acciones.

- sample: Indica qué muestra debe buscar el robot.

- soil: Indica el tipo de suelo en el que se encuentra el robot.

**Predicados**

- robot_at: Indica la ubicación en la que se encuentra el robot.

- sample_at: Indica la ubicación en la que se encuentra la muestra.

- soil_at: Indica la ubicación en la que se encuentra el suelo que debe analizar.

- soil_analysed: Indica que el robot ya ha analizado el suelo en una ubicación concreta.

- gripper_free: Indica si el robot tiene su pinza libre.

- robot_carry: Indica si el robot está llevando una muestra.

**Acciones**

- move(robot, from, to): El robot se mueve de una ubicación a otra.

- collect(sample, location, robot): El robot coge una muestra en una ubicación concreta.

- drop(sample, location, robot): El robot deja una muestra en una ubicación concreta.

- analyse-soil(soil, location, robot): El robot analiza el suelo en una ubicación concreta.

**ANÁLISIS DEL PROBLEMA ([exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%20de%20Calentamiento%201/exploration_problem_example.pddl))**

**Objetos**

- Tipo robot: curiosity.

- Tipo location: base, valley y crater.

- Tipo sample: rock y mineral.

- Tipo soil: valley_soil y crater_soil.

**Estado inicial (init)**

- robot_at curiosity base: El robot curiosity se encuentra ubicado en la base.

- gripper_free curiosity: El robot curiosity no lleva nada en su pinza.

- sample_at rock crater: La roca se encuentra ubicada en el cráter.

- sample_at mineral valley: El mineral se encuentra ubicado en el valle.

- soil_at valley_soil valley: El suelo del valle se encuentra ubicado en el valle.

- soil_at crater_soil crater: El suelo del cráter se encuentra ubicado en el cráter.

**Estado objetivo (goal)**

- sample_at rock base: La roca debe terminar ubicada en la base.

- sample_at mineral base: El mineral debe terminar ubicado en la base.

- soil_analysed valley_soil: El suelo del valle debe terminar habiendo sido analizado.

- soil_analysed crater_soil: El suelo del valle debe terminar habiendo sido analizado.

**EJECUCIÓN DEL PLAN**

Este es el plan generado al ejecutar el planificador POPF:

```sh
ros2 run popf popf exploration_domain_example.pddl exploration_problem_example.pddl 
```

```sh
0.000: (move curiosity base crater)  [0.001]
0.001: (analyse-soil crater_soil crater curiosity)  [0.001]
0.001: (collect rock crater curiosity)  [0.001]
0.002: (move curiosity crater base)  [0.001]
0.003: (drop rock base curiosity)  [0.001]
0.004: (move curiosity base valley)  [0.001]
0.005: (collect mineral valley curiosity)  [0.001]
0.005: (analyse-soil valley_soil valley curiosity)  [0.001]
0.006: (move curiosity valley base)  [0.001]
0.007: (drop mineral base curiosity)  [0.001]
```

Como se puede observar, el robot curiosity se mueve de la base al cráter y una vez ha llegado al cráter, realiza un análisis del suelo del cráter.

Una vez ha analizado el suelo, el robot curiosity coge una roca ubicada en el cráter, regresa a la base con la roca y una vez ha llegado a la base, deja la roca que había recogido.

Después, el robot curiosity se mueve de la mesa al valle, coge un mineral ubicado en el valle y a continuación, realiza un análisis del suelo del valle.

Y por último, el robot curiosity regresa a la base desde el valle, y una vez ha llegado a la base, deja el mineral que había recogido.

**[Ejercicio de calentamiento 2:](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/tree/main/Ejercicio%20de%20Calentamiento%202)** Elimina o comenta la línea 14 del archivo [exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%20de%20Calentamiento%202/exploration_problem_example.pddl), de tal forma que el robot ya no se encuentre inicialmente en la base:

```pddl
; (robot_at curiosity base)
```

Ahora vuelve a ejecutar el planificador y comprueba qué ocurre. ¿Por qué hay un error en la planificación? ¿Crees que sería posible que se mueva el robot sin saber su ubicación inicial?

Este es el plan generado al ejecutar el planificador POPF:

```sh
ros2 run popf popf exploration_domain_example.pddl exploration_problem_example.pddl 
```

```sh
[ros2run]: Process exited with failure 1
```

Como se puede observar, si se elimina la línea 14 del fichero [exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%20de%20Calentamiento%202/exploration_problem_example.pddl), se elimina toda la información referente a la ubicación inicial del robot, lo que provoca que el planificador no pueda llevar a cabo acciones posteriores.

Además, esto se debe a que la acción move tiene la siguiente precondición:

```pddl
:precondition
  (and 
    (robot_at ?r ?from)
  )
```

Esto significa que el planificador POPF debe saber en qué ubicación se encuentra el robot antes de que éste comience a moverse.

En resumen, si no se define la ubicación inicial del robot, no se estaría cumpliendo la precondición mencionada anteriormente, lo que haría que el planificador no pudiese generar un plan válido.

**[Ejercicio de calentamiento 3:](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/tree/main/Ejercicio%20de%20Calentamiento%203)** Modifica el ejemplo para añadir más muestras (de cualquier tipo) y alguna ubicación más. Después, incluye un segundo robot que ayude a Curiosity con la exploración.

Para modificar el ejemplo, he realizado los siguientes cambios en el fichero [exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%20de%20Calentamiento%203/exploration_problem_example.pddl):

```pddl
(define (problem exploration_example_problem)
(:domain exploration_example_domain)

(:objects
  ; rover: Nuevo objeto de tipo robot.
  curiosity rover - robot
  ; crater, pit, dune y volcano: Nuevos objetos de tipo location.
  base valley crater pit dune volcano - location
  ; crystal, water y lava: Nuevos objetos de tipo sample.
  rock mineral crystal water lava - sample
  valley_soil crater_soil - soil
)

(:init
  (robot_at curiosity base)
  ; robot_at rover base: El robot rover se encuentra ubicado en la base.
  (robot_at rover base)
  (gripper_free curiosity)
  ; gripper_free curiosity: El robot rover no lleva nada en su pinza.
  (gripper_free rover)
  (sample_at rock crater)
  (sample_at mineral valley)
  ; sample_at lava volcano: La lava se encuentra ubicada en el volcán.
  (sample_at lava volcano)
  ; sample_at crystal pit: El cristal se encuentra ubicado en la fosa.
  (sample_at crystal pit)
  ; sample_at water dune: El agua se encuentra ubicada en la duna.
  (sample_at water dune)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
)

(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    ; sample_at lava base: La lava debe terminar ubicada en la base.
    (sample_at lava base)
    ; sample_at crystal base: El cristal debe terminar ubicado en la base.
    (sample_at crystal base)
    ; sample_at water base: El agua debe terminar ubicada en la base.
    (sample_at water base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
  )
)

)
```

Responde a las siguientes preguntas:

1. ¿Es necesario modificar el dominio, el problema, o ambos?

En este caso sólo es necesario modificar el problema ([exploration_problem_example.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%20de%20Calentamiento%203/exploration_problem_example.pddl)) para añadir los nuevos objetos mencionados anteriormente.

Por otro lado, el dominio admite tener varios robots ya que todas las acciones incluyen el parámetro ?r - robot, lo que permite utilizar varios robots sin tener que modificar nada del dominio.

Sin embargo, si se pidiera que los robots tuviesen que cooperar e interactuar entre ellos, si que se tendría que modificar el dominio para añadir nuevos tipos, predicados, acciones, etc.

2. ¿Crees que se realizan las tareas más rápido al tener más robots?

En este caso y por lo general, tener más de un robot ayuda a reducir el tiempo de ejecución del planificador, ya que cada robot puede moverse a una ubicación distinta y realizar acciones en cada una de ellas de forma simultánea.

Sin embargo, si el plan está mal estructurado o los robots están contínuamente intentando realizar una acción que ya se había asignado a otro robot, la planificación puede llegar a empeorar.

Este es el plan generado al ejecutar el planificador POPF:

```sh
ros2 run popf popf exploration_domain_example.pddl exploration_problem_example.pddl 
```

```sh
0.000: (move curiosity base crater)  [0.001]
0.000: (move rover base valley)  [0.001]
0.001: (collect rock crater curiosity)  [0.001]
0.001: (analyse-soil crater_soil crater curiosity)  [0.001]
0.001: (analyse-soil valley_soil valley rover)  [0.001]
0.002: (move curiosity crater base)  [0.001]
0.002: (move rover valley dune)  [0.001]
0.003: (drop rock base curiosity)  [0.001]
0.003: (collect water dune rover)  [0.001]
0.004: (move rover dune base)  [0.001]
0.004: (move curiosity base pit)  [0.001]
0.005: (drop water base rover)  [0.001]
0.005: (collect crystal pit curiosity)  [0.001]
0.006: (move curiosity pit base)  [0.001]
0.006: (move rover base valley)  [0.001]
0.007: (drop crystal base curiosity)  [0.001]
0.007: (collect mineral valley rover)  [0.001]
0.008: (move rover valley base)  [0.001]
0.008: (move curiosity base volcano)  [0.001]
0.009: (drop mineral base rover)  [0.001]
0.009: (collect lava volcano curiosity)  [0.001]
0.010: (move curiosity volcano base)  [0.001]
0.011: (drop lava base curiosity)  [0.001]
```

## Ejercicio 1

Nos hemos dado cuenta de que no es posible hacer mediciones del suelo mientras el robot está cargando con una muestra en el gripper. Por este motivo, se pide modificar el dominio PDDL para modelar esta restricción. Crea unos ficheros nuevo llamados [exploration_domain.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%201/exploration_domain.pddl) y [exploration_problem.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%201/exploration_problem.pddl) e indica qué cambios has tenido que realizar:

Para implementar esta nueva funcionalidad, solo he tenido que añadir una nueva precondición a la acción analyse-soil en el fichero [exploration_domain.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%201/exploration_domain.pddl). 

Por otro lado, en el fichero del problema ([exploration_problem.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%201/exploration_problem.pddl)) no es necesario realizar ningún cambio:

```pddl
(:action analyse-soil
  :parameters (?s - soil ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (soil_at ?s ?l)
      ; Nueva precondición
      ; La pinza del robot debe estar libre.
      (gripper_free ?r)
    )
  :effect 
    (soil_analysed ?s)
)
```

Además, analiza la salida del plan generado con respecto al ejemplo y comenta qué diferencias hay:

**Antes de la modificación**

```sh
ros2 run popf popf exploration_domain_example.pddl exploration_problem_example.pddl 
```

```sh
0.000: (move curiosity base crater)  [0.001]
0.001: (analyse-soil crater_soil crater curiosity)  [0.001]
0.001: (collect rock crater curiosity)  [0.001]
0.002: (move curiosity crater base)  [0.001]
0.003: (drop rock base curiosity)  [0.001]
0.004: (move curiosity base valley)  [0.001]
0.005: (collect mineral valley curiosity)  [0.001]
0.005: (analyse-soil valley_soil valley curiosity)  [0.001]
0.006: (move curiosity valley base)  [0.001]
0.007: (drop mineral base curiosity)  [0.001]
```

**Después de la modificación**

```sh
ros2 run popf popf exploration_domain.pddl exploration_problem.pddl 
```

```sh
0.000: (move curiosity base crater)  [0.001]
0.001: (analyse-soil crater_soil crater curiosity)  [0.001]
0.002: (collect rock crater curiosity)  [0.001]
0.003: (move curiosity crater base)  [0.001]
0.004: (drop rock base curiosity)  [0.001]
0.005: (move curiosity base valley)  [0.001]
0.006: (collect mineral valley curiosity)  [0.001]
0.007: (move curiosity valley base)  [0.001]
0.008: (drop mineral base curiosity)  [0.001]
0.009: (move curiosity base valley)  [0.001]
0.010: (analyse-soil valley_soil valley curiosity)  [0.001]
```

Como se puede observar, antes de implementar esta modificación, el robot podía recoger muestras y analizar el suelo en cualquier orden. Ahora, el robot se asegura de que el gripper del robot está libre antes de analizar el suelo.

En resumen, gracias a este cambio el robot puede analizar el suelo antes de recoger muestras, ya que la acción analyse-soil no se ejecuta si el gripper del robot no está libre. Esto hace que el orden de las acciones cambie, dando prioridad al análisis del suelo respecto a la manipulación de muestras.

## Ejercicio 2

Como hacer mediciones en el suelo es costoso, desde la misión se ha decidido que únicamente se va a disponer de un taladro en la base, el cual podrá ser utilizado por cualquier robot. Crea dos ficheros nuevos ([exploration_domain_drill.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%202/exploration_domain_drill.pddl) y [exploration_problem_drill.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%202/exploration_problem_drill.pddl)) con la nueva implementación.

Deberás añadir acciones y predicados nuevos para la gestión del taladro. Un robot podrá llevarse el taladro si está en la misma ubicación que el taladro y si el taladro está libre. Por otro lado, para poder realizar la acción de analizar el suelo, el robot debe llevar el taladro en ese momento.

Ejecuta el planificador con el nuevo dominio / problema y copia aquí la salida. ¿Es el resultado como esperabas?

Para poder implementar esta nueva funcionalidad he realizado las siguientes modificaciones en los ficheros iniciales, los cuales se pasan a llamar [exploration_domain_drill.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%202/exploration_domain_drill.pddl) y [exploration_problem_drill.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%202/exploration_problem_drill.pddl):

**Fichero [exploration_domain_drill.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%202/exploration_domain_drill.pddl)**

```pddl
(define (domain exploration_example_domain)
(:requirements :strips :typing)

(:types
  location
  robot
  sample
  soil
  ; Nuevo tipo
  drill
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (sample_at ?s - sample ?l - location)
  (soil_at ?s - soil ?l - location)
  (soil_analysed ?sl - soil)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?s - sample)
  ; Nuevos predicados
  ; robot_at: Indica la ubicación en la que se encuentra el taladro.
  (drill_at ?d - drill ?l - location)
  ; drill_free: Indica si el taladro está libre.
  (drill_free ?d - drill)
  ; robot_has_drill: Indica si el robot está llevando el taladro.
  (robot_has_drill ?r - robot ?d - drill)
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:action move
  :parameters (?r - robot ?from ?to - location)
  :precondition
    (and 
      (robot_at ?r ?from)
    )
  :effect
    (and
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
)

; Collect action. The robot collects a sample at a location.
; Both the robot and the sample must be in that location.
; The robot's gripper must be free (the robot can only hold 1 sample).
; Consequences:
;   - The sample is no longer at the given location.
;   - The robot is now carrying the sampple and its gripper is not free.
(:action collect
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition 
    (and
      (sample_at ?s ?l)
      (robot_at ?r ?l)
      (gripper_free ?r)
    )
  :effect
    (and
      (robot_carry ?r ?s)
      (not (sample_at ?s ?l))
      (not (gripper_free ?r))
    )
)

; Drop-off action. The robot drops a sample at a location.
; The robot must be in that location and must be carrying that sample.
; Consequences:
;   - The sample is now at the given location.
;   - The robot is no longer carrying the sample and its gripper is free.
(:action drop
  :parameters (?s - sample ?l - location ?r - robot)
  :precondition
    (and 
      (robot_at ?r ?l)
      (robot_carry ?r ?s)
    )
  :effect 
    (and 
      (sample_at ?s ?l)
      (gripper_free ?r)
      (not (robot_carry ?r ?s))
    )
)

; Analyse Soil action. The robot samples and analyses the soil at a location.
; The robot must be in that location.
; Consequences:
;   - The soil is now analysed.
(:action analyse-soil
  ; Nuevo parámetro ?d - drill
  :parameters (?s - soil ?l - location ?r - robot ?d - drill)
  :precondition
    (and 
      (robot_at ?r ?l)
      (soil_at ?s ?l)
      (gripper_free ?r)
      ; Nueva precondición
      ; El robot debe llevar el taladro para poder analizar el suelo.
      (robot_has_drill ?r ?d)
    )
  :effect
    (and
      (soil_analysed ?s)
    )
)

; Nueva acción collect-drill
; Collect Drill action. The robot picks up the drill at a location.
; The robot must be in that location, the drill must be there, and it must be free.
; Consequences:
;   - The robot now has the drill.
;   - The drill is no longer free.
;   - The drill is no longer at the given location.
(:action collect-drill
  :parameters (?r - robot ?d - drill ?l - location)
  :precondition
    (and
      (robot_at ?r ?l)
      (drill_at ?d ?l)
      (drill_free ?d)
    )
  :effect
    (and
      (robot_has_drill ?r ?d)
      (not (drill_free ?d))
      (not (drill_at ?d ?l))
    )
)

; Nueva acción drop-drill
; Drop Drill action. The robot drops the drill at a location.
; The robot must be in that location and must be carrying the drill.
; Consequences:
;   - The drill is now at the given location.
;   - The drill is free and available for use.
;   - The robot is no longer carrying the drill.
(:action drop-drill
  :parameters (?r - robot ?d - drill ?l - location)
  :precondition
    (and
      (robot_at ?r ?l)
      (robot_has_drill ?r ?d)
    )
  :effect
    (and
      (drill_at ?d ?l)
      (drill_free ?d)
      (not (robot_has_drill ?r ?d))
    )
)

)
```

**Fichero [exploration_problem_drill.pddl](https://github.com/Docencia-fmrico/p1-space-exploration-aleon2020/blob/main/Ejercicio%202/exploration_problem_drill.pddl)**

```pddl
(define (problem exploration_example_problem)
(:domain exploration_example_domain)

(:objects
  curiosity - robot
  base valley crater - location
  rock mineral - sample
  valley_soil crater_soil - soil
  ; drill_tool: Nuevo objeto de tipo drill.
  drill_tool - drill
)

(:init
  (robot_at curiosity base)
  (gripper_free curiosity)
  (sample_at rock crater)
  (sample_at mineral valley)
  (soil_at valley_soil valley)
  (soil_at crater_soil crater)
  ; drill_at drill_tool base: El taladro se encuentra ubicado en la base.
  (drill_at drill_tool base)
  ; drill_free drill_tool: El taladro no ha sido cogido por ningún robot.
  (drill_free drill_tool)
)

(:goal
  (and
    (sample_at rock base)
    (sample_at mineral base)
    (soil_analysed valley_soil)
    (soil_analysed crater_soil)
    ; drill_at dril_tool base: El taladro debe terminar ubicado en la base.
    (drill_at drill_tool base)
  )
)

)
```

A continuación se muestra la ejecución del planificador una vez añadidos todos los cambios necesarios para poder implementar esta nueva funcionalidad:

```sh
ros2 run popf popf exploration_domain_drill.pddl exploration_problem_drill.pddl 
```

```sh
0.000: (move curiosity base crater)  [0.001]
0.001: (collect rock crater curiosity)  [0.001]
0.002: (move curiosity crater base)  [0.001]
0.003: (drop rock base curiosity)  [0.001]
0.004: (move curiosity base valley)  [0.001]
0.005: (collect mineral valley curiosity)  [0.001]
0.006: (move curiosity valley base)  [0.001]
0.007: (drop mineral base curiosity)  [0.001]
0.007: (collect-drill curiosity drill_tool base)  [0.001]
0.008: (move curiosity base crater)  [0.001]
0.009: (analyse-soil crater_soil crater curiosity drill_tool)  [0.001]
0.010: (move curiosity crater base)  [0.001]
0.011: (move curiosity base valley)  [0.001]
0.012: (analyse-soil valley_soil valley curiosity drill_tool)  [0.001]
0.013: (move curiosity valley base)  [0.001]
0.014: (drop-drill curiosity drill_tool base)  [0.001]
```

Como se puede observar, se obtiene el resultado esperado, ya que el robot no realiza un análisis del suelo hasta que no lleva el taladro.
