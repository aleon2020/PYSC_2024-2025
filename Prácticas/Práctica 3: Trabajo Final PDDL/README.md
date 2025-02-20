[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/waTmHlUU)

# Trabajo Final PDDL

El trabajo final del bloque de PDDL consistirá en una comparación de distintos planificadores, analizando qué features de PDDL están soportadas por cada uno.

*Nota:* Este trabajo se realizará por grupos de 4 personas. Todas las personas del grupo debéis aceptar la tarea en Github Classroom y entrar en el repositorio creado automáticamente para el grupo.

## Planificadores

Los planificadores a comparar serán POPF, OPTIC y uno más a elección del grupo. Indicad aquí cuál es el planificador adicional escogido, cuál es la web oficial, dónde se encuentra su documentación y cuáles son los pasos para poder instalarlo y ejecutarlo:

*[Respuesta: Información de planificador escogido]*

El planificador escogido en nuestro caso ha sido **Marvin**, un planificador de búsqueda heurística de encadenamiento directo independiente del dominio (domain-independent forward-chaining heuristic-search planner) que participó en el Cuarto Concurso Internacional de Planificación. 

La página oficial de este planificador se puede encontrar en el [siguiente enlace](https://nms.kcl.ac.uk/planning/software/marvin.html).

La instalación de Marvin se puede llevar a cabo de dos formas, siguiendo los pasos que se especifican en la guía, o mediante un fichero ejecutable, siendo esta última la más sencilla de llevar a cabo.

En primer lugar, descarga el fichero comprimido que contiene el ejecutable a través del [siguiente enlace](https://sourceforge.net/projects/tsgp/files/Marvin/marvin2-x86-static.gz/download).

Un vez descargado, descomprime el archivo 'marvin2-x86-static.gz' ejecutando en la terminal el siguiente comando:

```sh
gunzip marvin2-x86-static.gz
```

Esto eliminará el archivo 'marvin2-x86-static.gz' y dejará únicamente el archivo descomprimido, en este caso, el fichero ejecutable 'marvin2-x86-static'.

Una vez hecho esto, renombra el fichero ejecutable 'marvin2-x86-static' a 'marvin' para tener una mayor comodidad a la hora de ejecutar el planificador:

```sh
mv marvin2-x86-static marvin
```

**IMPORTANTE**: Otorga permisos de ejecución al fichero ejecutable 'marvin' para poder ejecutar el planificador correctamente.

```sh
chmod +x marvin
```

Después, mueve el fichero ejecutable al mismo directorio en el que se encuentren los ficheros PDDL a ejecutar:

```sh
mv ~/Descargas/marvin ~/path/to/pddl/files/
```

Y por último, para probar el planificador, ejecuta en la terminal el siguiente comando:

```sh
./marvin ~/<path/to/domain.pddl> ~/<path/to/problem.pddl>
```

## Estudio de compatibilidad con features de PDDL

A continuación, seleccionad un subset de features que os parezcan interesantes del lenguaje. Por ejemplo: `durative-actions`, `existential-preconditions`, `negative-precontitions`, etc. Después, verificad si están soportadas por cada uno de los planificadores.

Para demostrar el soporte de cada feature, es necesario hacer dos cosas:

1. Comprobar en la documentación (si la hubiese) si el planificador soporta dicha feature.

2. Crear un ejemplo mínimo, con un dominio y un problema sencillos en el que se utilice dicha feature para demostrarlo, comprobando el comportamiento de cada uno de los planificadores.

*[Respuesta: Análisis features]*

Para ejecutar cada uno de los ejemplos realizados en este trabajo, se deben ejecutar los siguientes comandos en la terminal, dependiendo del planificador con el que se quiera ejecutar dicho ejemplo, todos ellos dentro de la carpeta [Ejemplos/](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos).

```sh
cd Ejemplos/
```

**PLANIFICADOR POPF**

```sh
ros2 run popf popf ~/<path/to/domain.pddl> ~/<path/to/problem.pddl>
```

**PLANIFICADOR OPTIC**

```sh
ros2 run optic_planner optic_planner ~/<path/to/domain.pddl> ~/<path/to/problem.pddl>
```

**PLANIFICADOR MARVIN**

```sh
./marvin ~/<path/to/domain.pddl> ~/<path/to/problem.pddl>
```

[:conditional-effects](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/conditional-effects)

* **POPF**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que los conditional-effects no están soportados por POPF.

* **OPTIC**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que los conditional-effects no están soportados por OPTIC.

* **MARVIN**: SI está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente confirmando que los conditional-effects sí están soportados por MARVIN.

[:continuous-effects](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/continuous-effects)

* **POPF**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que los continuous-effects sí están soportados por POPF.

* **OPTIC**: De acuerdo a la documentación, SÍ está soportada. Sin embargo, al ejecutar el plan que utiliza esta feature, se muestra un mensaje de error y no es capaz de generar un plan. Por tanto, los continuous-effects no están soportados por OPTIC, aunque en la documentación se diga lo contrario.

* **MARVIN**: NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que los continuous-effects no están soportados por MARVIN.

[:disjunctive-preconditions](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/disjunctive-preconditions)

* **POPF**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que las disjunctive-preconditions no están soportadas por POPF.

* **OPTIC**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que las disjunctive-preconditions no están soportadas por OPTIC.

* **MARVIN**: SI está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente confirmando que los disjuntive-preconditions sí están soportados por MARVIN.

[:durative-actions](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/durative-actions)

* **POPF**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que las durative-actions sí están soportadas por POPF.

* **OPTIC**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que las durative-actions sí están soportadas por OPTIC.

* **MARVIN**: Trata a todas las acciones como instantaneas, y al procesar una acción con duración devuelve un "segmentation fault". No hay advertencia de "ADL warning" cuando salta el error.

[:existential-preconditions](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/existential-preconditions)

* **POPF**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que las existential-preconditions no están soportadas por POPF.

* **OPTIC**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que las existential-preconditions no están soportadas por OPTIC.

* **MARVIN**: SI está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente confirmando que los existential-preconditions sí están soportados por MARVIN.

[:negative-preconditions](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/negative-preconditions)

* **POPF**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que las negative-preconditions no están soportadas por POPF.

* **OPTIC**: De acuerdo a la documentación, NO está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un "ADL warning", confirmando que las negative-preconditions no están soportadas por OPTIC.

* **MARVIN**: SI está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente confirmando que los negative-preconditions sí están soportados por MARVIN.

[:numeric-fluents](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/numeric-fluents)

* **POPF**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que los numeric-fluents sí están soportados por POPF.

* **OPTIC**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que los numeric-fluents sí están soportados por OPTIC.

* **MARVIN**: Solo acepta operaciones lógicas (Booleans 0-1) pero no operaciones aritméticas. Trata todos los valores como 0-1, y si usas otros valores, en vez de dar error, te devuelve un planning erroneo.

[:universal-preconditions](https://github.com/Docencia-fmrico/trabajo-pddl-envidio33/tree/main/Ejemplos/universal-preconditions)

* **POPF**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que los universal-preconditions sí están soportadas por POPF.

* **OPTIC**: De acuerdo a la documentación, SÍ está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente, confirmando que los universal-preconditions sí están soportadas por OPTIC.

* **MARVIN**: SI está soportada. Y en efecto, al ejecutar el plan que utiliza esta feature, se muestra un plan coherente confirmando que los universal-preconditions sí están soportados por MARVIN.

Finalmente, generad una tabla en la que se resuma la compatibilidad de cada planificador, con las features analizadas por filas y los planificadores por columnas. Podéis encontrar información de cómo generar tablas en formato Markdown de Github [aquí](https://docs.github.com/es/get-started/writing-on-github/working-with-advanced-formatting/organizing-information-with-tables).

*[Respuesta: Tabla comparativa]*

✅: SOPORTADO

❌: NO SOPORTADO

⚠️: ERROR

| FEATURE                   | POPF         | OPTIC        | MARVIN       |
| ------------------------- | ------------ | ------------ | -------------|
| conditional-effects       | ❌ | ❌ | ✅ |
| continuous-effects        | ✅ | ⚠️ | ❌ |
| disjunctive-preconditions | ❌ | ❌ | ✅ |
| durative-actions          | ✅ | ✅ | ⚠️ |
| existential-preconditions | ❌ | ❌ | ✅ |
| negative-preconditions    | ❌ | ❌ | ✅ |
| numeric-fluents           | ✅ | ✅ | ✅ |
| universal-preconditions   | ✅ | ✅ | ✅ |

## Entrega del trabajo

El trabajo se presentará en clase el próximo miércoles 19 de febrero. Además, en este repositorio deberéis añadir los ficheros PDDL para los ejemplos generados, debidamente referenciados en el README.

Es recomendable preparar diapositivas para la presentación, las cuales deberéis añadir también al repositorio.
