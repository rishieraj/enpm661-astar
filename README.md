# ENPM661: Project-3 Phase-1
Implementation and visualization of **A-Star Algorithm**.

## 1. Team:

|     UID     |  Directory ID  |            Name            |
|    :---:    | :------------: |           :----:           |
|  120425554  |     rraj27     |         Rishie Raj         |
|  120305085  |      uthu      |  Uthappa Suresh Madettira  |

## 2. Description:
The goal of the project is to implement A-Star Algorithm to find the shortest path between user defined start and goal points for a rigid robot in a given map.

## 3. Contents

 - `a_star_rishie_uthappa.py` : python script containing the code for the project.

 - `a_star_rishie_uthappa.mp4` : video file containing the visualization. The visualization has been shown for start: (50, 400) & goal: (400, 100).

 - `README.md` : markdown file containing instructions and details.

## 4. How to Use:
Please follow the following steps to implement the code in a local system:

 - Download the .py file that was provided as part of the submission or retrieve it from the following GitHub repo: https://github.com/rishieraj/enpm661-dijkstra.git .

 - Once retrieved, the .py file can be executed directly. Upon execution, the user will be prompted to enter the following:

    - Clearance between robot and obstacles.
    - Radius of the robot
    - Start point and orientation $(X,\;Y, \;\theta)$
    - Goal point and orientation $(X,\;Y, \;\theta)$
    - Step size of the robot
 
 **Note:** The coordinates need to be added as per the origin described in the project documentation.

 - Once the start and goal points are added, the program will explore the nodes based on the Total Cost = Cost2Come + Cost2Goal.

 - Once the goal is reached, the program prints *Goal Reached!* along with the computation time. It then goes on to perform the visualization of the node exploration and optimal path between start and goal.

## 5. Libraries/Dependencies Used:
The following libraries have been used in the program:

 - Numpy ( `numpy` ): used for array operations and mathematical calculations.

 - Queue ( `queue.PriorityQueue` ): used for handling nodes in Dijkstra Algorithm due to easy popping of lowest order element.

 - Time ( `time` ): used to calculate computation time.

 - OpenCV ( `opencv` ): used for display and visualization purposes. 
