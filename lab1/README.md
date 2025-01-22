# Lab 1: SLAM for Karel in 1D

## Problem Statement
In this lab, we consider the robot Karel, which operates in a 1D space. Karel can move forward and backward in this 1D world. The environment contains several beepers that respond when Karel pings within a certain range of distances. Karel is equipped with:

1. **An odometer** – Provides measurements of its own motion.
2. **A beeper sensor** – Detects beepers responding to Karel's pings and estimates their distance.

---

## Tasks

### Task 1: Initial Setup
1. Download the Matlab skeleton code `karel_lab1.m` from Moodle.
2. Run the program to observe Karel performing simple odometry.

### Task 2: Completing Missing Algorithm Parts
- Implement the missing functions:
  - `update_map`
  - `add_new_features`
- Guidelines:
  - Approximately 10 lines of code per function.
  - Utilize Kalman filter equations for accurate implementation.
  - **Tips:**
    1. Implement and test `add_new_features` first.
    2. Experiment with the program to determine how large a map can be created efficiently.
    3. Avoid using `for` loops; prefer vectorized matrix operations in Matlab.

### Task 3: Sequential Map Joining
- Implement the Sequential Map Joining technique to construct `n` sequential maps and merge them into a full map.
- Approximate implementation effort: 20 lines of code.
- **Tips:**
  1. Use linear combinations of Gaussian vectors.
  2. Start with two maps and attempt to merge them.
  3. A single `for` loop can be used for iterating and generating maps.

### Task 4: Computational Cost Analysis
- Compare the computational costs of:
  - Building the full map directly.
  - Using sequential map joining.
- **Tips:**
  1. Utilize Matlab's elapsed time functions (already present in the skeleton code).
  2. Leverage sparse matrices to optimize computational efficiency.

### Task 5 (Optional): Additional Exercises
- For higher grades, solve additional exercises.
- Implementing Exercise 11 will significantly enhance your grade.

---

## Results and Submission
- Complete as many tasks as possible within the available time.
- Achieving Task 4 ensures a maximum grade of **8.0**, with additional tasks providing extra credit.
- Prepare and submit a comprehensive, well-structured report by **February 10**.
  - Ensure clarity and conciseness, avoiding unnecessary details.

---

## Requirements
- **Software:** Matlab
- **Knowledge prerequisites:**
  - Kalman filters
  - Linear algebra operations
  - Matlab matrix and vector operations

---

## Notes
- Follow best coding practices.
- Make use of provided hints to optimize performance.
- Seek clarification from instructors if needed.

