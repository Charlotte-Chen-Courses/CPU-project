# FiatLux Out of Order CPU architecture

## Module Hierarchy
In order to facilitate the implementation of the CPU, following structure diagram is provided to illustrate the planning hierarchy
and related wire connection.

Before this Friday, we will need to implement rs_unit below.

- **pipline** *(the top-level design)*
  - **stage_if**: fetch instruction from memory
  - **IF/DIS reg**: cross satge register
  - **stage_dis**: decode instruction
    - **decoder**: decode instructions
    - **reg**: get register value
  - **DIS/RM reg**: cross satge register
  - **stage_rm**: rename register to eliminate fake dependency
  - **RM/RSB reg**: cross satge register
  - **stage_rsb**: reservation station and reorder buffer
    - **rs_unit**: reservation station
    - **rob_unit**: reorder buffer
    - **mp_unit**: map table
    - **cdb_unit**: CDB broadcast unit
  - **RSB/EX reg**: cross satge register
  - **stage_ex**: execute instructions in parallel
  - **TO BE DONE**
