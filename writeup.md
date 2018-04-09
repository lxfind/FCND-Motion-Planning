## Project: 3D Motion Planning

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

There are two major functions that I touched in planning_utils.py. One called "find_path_using_grid" that looks for a path in the grid; while another called "find_path_using_graph" which looks for a path by constructing a graph in the grid. It demonstrates that using a graph is both much faster to run and more efficient (much less waypoints) comparing to the naive grid based approach.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that does the following things:
- Create a grid in which each cell is either 0 or 1. 1 indicates obstacles.
- Given a start point and a goal point in the grid, it implements A Star algorithm to find a path. The algorithm only allows for actions to move up, down, left and right. Diagonal actions are not supported yet in the basic implementation.
- It then returns the path and send it as waypoints.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I read the first line from colliders.csv and extract the value of lat0 and lon0 using split. After that I set it as home position.

#### 2. Set your current local position
I called global_to_local.

#### 3. Set grid start position from local position
I just use the current local position as the start position. To convert, I offset it by north_offset and east_offset. (Look for assignment to grid_start).

#### 4. Set grid goal position from geodetic coords
To make it interesting, I randomly generate a goal position (and make sure it's not in an obstacle).

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
In "find_path_using_grid" function in planning_utils.py, I added 4 extra valid actions that move diagonal.

I also reimplemented an algorithm using a graph, in "find_path_using_graph". In this algorithm, I sample a list of random states in the grid, and then connect them to construct a graph. I then use A* to find a path in the graph.

#### 6. Cull waypoints
I use collinearity test to prune waypoints in "find_path_using_grid". But I din't do this in "find_path_using_graph" because there is not much benefit.

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

Not sure how to take the drone dynamics into the planning model. Would be happy to see an solution that does.
