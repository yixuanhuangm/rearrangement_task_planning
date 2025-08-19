# Task Planning for Rearrangement Tasks

This repository focuses on **task planning**, with a particular emphasis on **rearrangement tasks**. We believe that understanding and solving rearrangement tasks is fundamental, as many forms of actions and operations can be derived from given action sequences. Studying rearrangement tasks can therefore provide insights for more efficient planning across a wide range of task types.

## Background

This project is inspired by the work of **Professor Jingjin Yu** at Rutgers University and his PhD students. Their research provides a foundation for understanding task planning and rearrangement in structured environments. Our implementation builds upon these ideas with a focus on simple, visualizable tasks.

## Features

* Focused on **rearrangement tasks**, providing a foundation for broader task planning research.
* Supports **visualization of objects and their motions**, making planning processes interpretable.
* Includes **simple planning algorithms** as a starting point for experimentation.
* Modular design, allowing easy extension for more complex planning strategies.

## Modules

* **src/disk.py**: Defines the `Disk` class and its properties (position, radius, etc.).
* **src/generation.py**: Functions to generate disks with specified positions and goals.
* **src/utils.py**: Utility functions, e.g., for plotting or checking overlaps.
* **src/visualization.py**: Functions for static visualization of disks.
* **src/planner.py**: Constructs dependency graphs and implements MRB (Minimum Remaining Buffer) scheduling logic.
* **src/animation.py**: Functions to animate disk movements.
* **examples/run_mrb_demo.py**: Demonstrates the planning and animation of disk rearrangements.

<!-- ## Experimental Tasks -->

## Task Status

- [X] Naive traversal algorithm implemented.
- [ ] Depth-First Search (DFS) based planning.
- [ ] Brute-force search algorithm.
- [ ] Extend planning algorithms to handle more complex object interactions and constraints.
- [ ] Optimize visualization and motion planning for larger scenes.

<!-- 
| Task                         | Goal                                     | Method                                                | Expected Outcome                                                 | Status      |
| ---------------------------- | ---------------------------------------- | ----------------------------------------------------- | ---------------------------------------------------------------- | ----------- |
| Naive Traversal Algorithm    | Provide a baseline rearrangement planner | Implement simple traversal over object sequences      | Can compute feasible rearrangement sequences for small scenarios | ✅ Completed |
| DFS-based Planning           | Explore deeper search strategies         | Implement depth-first search on task dependency graph | More efficient and flexible planning for complex rearrangements  | ⬜ To Do     |
| Brute-force Search           | Benchmark exhaustive planning            | Enumerate all possible object arrangements            | Identify optimal solutions and compare with heuristic methods    | ⬜ To Do     |
| Complex Interaction Handling | Extend to objects with constraints       | Introduce interactions such as collision avoidance    | More realistic rearrangement planning in constrained spaces      | ⬜ To Do     |
| Visualization Optimization   | Improve interpretability and performance | Optimize animation and plotting                       | Smooth and scalable visualization for larger scenes              | ⬜ To Do     | -->

## File Structure (simplified)

```
rearrangement_task_planning/
│
├── README.md
├── LICENSE
├── requirements.txt
│
├── src/
│   ├── __init__.py
│   ├── disk.py
│   ├── generation.py
│   ├── utils.py
│   ├── visualization.py
│   ├── mrb_logic.py
│   └── animation.py
│
├── examples/
│   └── run_mrb_demo.py
│
└── docs/
```

## Installation

```bash
git clone git@github.com:yixuanhuangm/rearrangement_task_planning.git
cd rearrangement_task_planning
pip install -r requirements.txt
```

## Usage

Run the demo:

```bash
python -m examples.run_mrb_demo
```
