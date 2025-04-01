# Plugin escogido y detalles de cómo poder usarlo en PlanSys2

Se ha utilizado un plugin para OPTIC creado a partir del plugin de POPF.

La guía de instalación de OPTIC se muestra más en detalle en el siguiente enlace:

[https://github.com/sscpac/optic_planner](https://github.com/sscpac/optic_planner)

**IMPORTANTE**: Se debe lanzar OPTIC usando el flag '-b' para desactivar BFS y así conseguir que el plan se cree a tiempo.

Para usar el plugin de OPTIC se deben seguir los siguientes pasos:

**PASO 1**: Mueve la carpeta `plugin/plansys2_optic_plan_solver` que se encuentra en este paquete a la carpeta `ros2_planning_system`.

**PASO 2**: La estructura de la carpeta `ros2_planning_system` debe ser la mostrada a continuación.

```sh
> plansys2_bringup
> plansys2_bt_actions
> plansys2_core
> plansys2_docs
> plansys2_domain_expert
> plansys2_executor
> plansys2_lifecycle_manager
> plansys2_msgs
> plansys2_optic_plan_solver
> plansys2_pddl_parser
> plansys2_planner
> plansys2_popf_plan_solver
> plansys2_problem_expert
> plansys2_support_py
> plansys2_terminal
> plansys2_tests
> plansys2_tools
.gitignore
CODE_OF_CONDUCT.md
codecov.yaml
CONTRIBUTING.md
dependency_repos.repos
LICENSE
```

**PASO 3**: Por último, se debe modificar el fichero de configuración `ros2_planning_system/plansys2_bringup/params/plansys2_params.yaml` de forma que su estructura sea la mostrada a continuación.

```sh
planner:
  ros__parameters:
    plan_solver_plugins: ["OPTIC"]
    POPF:
      plugin: "plansys2/POPFPlanSolver"
    TFD:
      plugin: "plansys2/TFDPlanSolver"
    OPTIC:
      plugin: "plansys2/OPTICPlanSolver"
executor:
  ros__parameters:
    bt_builder_plugin: "SimpleBTBuilder"
```