// DFS Maze Solver Core Logic for ESP32
struct Node {
    int id;
    bool visited;
    bool triedLeft;
    bool triedForward;
    bool triedRight;
};

void solveMazeDFS() {
    while (!atGoal()) {
        int currentJunction = getCurrentJunctionSensorState();
        
        // 1. Check for Dead End
        if (isDeadEnd()) {
            turnAround(); // 180-degree flip
            backtrackToPreviousNode();
            continue;
        }
        
        // 2. Explore Unvisited Paths (Priority: Left -> Forward -> Right)
        if (hasUnvisitedLeftPath() && !currentData.triedLeft) {
            currentData.triedLeft = true;
            turnLeft();
            moveForwardUntilNextJunction();
        } 
        else if (hasUnvisitedForwardPath() && !currentData.triedForward) {
            currentData.triedForward = true;
            moveForwardUntilNextJunction();
        } 
        else if (hasUnvisitedRightPath() && !currentData.triedRight) {
            currentData.triedRight = true;
            turnRight();
            moveForwardUntilNextJunction();
        } 
        else {
            // All paths from this node are explored; backtrack
            markNodeAsFullyExplored();
            backtrackToParentNode();
        }
    }
    stopRobotAtGoal();
    generateOptimalPathMap();
}