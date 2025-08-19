### Naive Traversal Algorithm

The first baseline implemented in this repository is a **naive traversal algorithm** for disk rearrangement. The algorithm proceeds as follows:

1. **Sequential traversal:** Iterate over all objects that have not yet reached their goal positions.
2. **Direct placement:** If an object can be moved directly to its goal without conflicts, move it immediately.
3. **Buffer placement:** If the goal position is blocked, move the object to a designated buffer area.
4. **Buffer check:** After each move, check all objects currently in the buffer. If any object can now be moved to its goal, execute the move.
5. **Iteration:** Once the buffer is cleared as much as possible, continue with the remaining objects that have not yet been processed.

In this scheme, each object is guaranteed to move at most **two times**: once to the buffer (if necessary), and once to its goal. This ensures a simple yet effective baseline for rearrangement tasks, serving as the foundation for comparison with more advanced planning algorithms such as DFS or brute-force search.
