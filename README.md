# 2D grid map A*

A Python implementation of the [A* algorithm][1] in a 2D Occupancy Grid Map, based on [Claus Brenner's Path Planning lectures][2].

## Requirements
The implementation runs on both Python 2 and 3. Additionally, it requires the following python packages (available via pip):
- numpy
- pypng
- matplotlib


## Examples
Two examples are given for both binary and occupancy grid maps, each one with different allowed movements (4-connectivity and 8-connectivity respectively). Note that the examples are assuming that the directory containing the provided modules (i.e. `gridmap.py`, `a_star.py`, `utils.py`) is on `sys.path`.


[1]: https://en.wikipedia.org/wiki/A*_search_algorithm
[2]: https://www.youtube.com/playlist?list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm
