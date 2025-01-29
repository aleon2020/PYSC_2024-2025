[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/9J2PKxcC)
# P1 - Exploración Espacial

El propósito de esta práctica es modelar con PDDL un entorno de exploración expacial, en el que un robot debe recoger muestras del entorno y analizarlas.

Las tareas principales del robot son recoger muestras (samples) como rocas o minerales para llevarlas a la base, y hacer mediciones en el suelo para analizarlas en busca de agua o indicios de vida.

## Ejemplo sencillo
Los archivos [exploration_domain_example](pddl/exploration_domain_example.pddl) y [exploration_problem_example](pddl/exploration_problem_example.pddl) contienen un dominio y un problema de ejemplo con la funcionalidad más básica.

En este ejemplo, se asume que el robot dispone de una base móvil con una pinza (gripper) con la capacidad de coger y soltar objetos.

Además, en el dominio propuesto asumimos que el robot se puede mover libremente de una ubicación a otra.


**Ejercicio de calentamiento 1:** Analiza el ejemplo propuesto y ejecuta el planificador (POPF) para ver el plan que genera. Puedes cambiar la definición del problema para ver cómo cambian los planes.

*Nota: Puedes encontrar instrucciones para instalar y ejecutar POPF en el archivo [POPF_INSTRUCTIONS](POPF_INSTRUCTIONS.md).*

**Ejercicio de calentamiento 2:** Elimina o comenta la línea 14 del archivo [exploration_problem_example](pddl/exploration_problem_example.pddl#L14), de tal forma que el robot ya no se encuentre inicialmente en la base:

```pddl
; (robot_at curiosity base)
```

Ahora vuelve a ejecutar el planificador y comprueba qué ocurre. ¿Por qué hay un error en la planificación? ¿Crees que sería posible que se mueva el robot sin saber su ubicación inicial?

*[Respuesta]*

**Ejercicio de calentamiento 3:** Modifica el ejemplo para añadir más muestras (de cualquier tipo) y alguna ubicación más. Después, incluye un segundo robot que ayude a Curiosity con la exploración.

Responde a las siguientes preguntas:

1. ¿Es necesario modificar el dominio, el problema, o ambos?

*[Respuesta]*

2. ¿Crees que se realizan las tareas más rápido al tener más robots?

*[Respuesta]*

## Ejercicio 1
Nos hemos dado cuenta de que no es posible hacer mediciones del suelo mientras el robot está cargando con una muestra en el gripper. Por este motivo, se pide modificar el dominio PDDL para modelar esta restricción. Crea unos ficheros nuevo llamados [exploration_domain.pddl](pddl/exploration_domain.pddl) y [exploration_problem.pddl](pddl/exploration_problem.pddl) e indica qué cambios has tenido que realizar:

*[Respuesta]*


Además, analiza la salida del plan generado con respecto al ejemplo y comenta qué diferencias hay:

*[Respuesta]*


## Ejercicio 2
Como hacer mediciones en el suelo es costoso, desde la misión se ha decidido que únicamente se va a disponer de un taladro en la base, el cual podrá ser utilizado por cualquier robot. Crea dos ficheros nuevos ([exploration_domain_drill.pddl](pddl/exploration_domain_drill.pddl) y [exploration_problem_drill.pddl](pddl/exploration_problem_drill.pddl)) con la nueva implementación.

Deberás añadir acciones y predicados nuevos para la gestión del taladro. Un robot podrá llevarse el taladro si está en la misma ubicación que el taladro y si el taladro está libre. Por otro lado, para poder realizar la acción de analizar el suelo, el robot debe llevar el taladro en ese momento.

Ejecuta el planificador con el nuevo dominio / problema y copia aquí la salida. ¿Es el resultado como esperabas?

*[Respuesta]*
