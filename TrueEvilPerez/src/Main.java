import java.util.*;

public class Main {

    //map that stores movement cost for each terrain type
    static Map<Character, Integer> terrainCost = new HashMap<>();

    static {

        //plains
        terrainCost.put('P', 2);

        //forest
        terrainCost.put('F', 3);

        //swamp
        terrainCost.put('S', 5);

        //mountain is infinite
        terrainCost.put('M', Integer.MAX_VALUE);

        //goal tile is treated like plains
        terrainCost.put('G', 2);

        //start tile is also like plains, used 'K' instead of 'S' to avoid conflict with swamp
        terrainCost.put('K', 2);
    }

    //directions for moving: up, down, left, right
    static int[] dx = {-1, 1, 0, 0};
    static int[] dy = {0, 0, -1, 1};

    //node class to represent each position in the grid
    static class Node {

        //coordinates on grid
        int x, y;

        //cost from the start to this node
        int cost;


        //estimated cost from this node to goal, heuristic
        int estCost;

        //to trace back the path later
        Node parent;

        Node(int x, int y, int cost, int estCost, Node parent) {

            this.x = x;

            this.y = y;

            this.cost = cost;
            this.estCost = estCost;
            this.parent = parent;
        }

    }

    public static void main(String[] args) {
        //grid map, K = start, G = goal, rest are terrain types
        char[][] grid = {
                {'K', 'P', 'F', 'F', 'P'},
                {'P', 'M', 'F', 'F', 'F'},
                {'P', 'P', 'S', 'P', 'G'}
        };

        //start timing
        long startTime = System.currentTimeMillis();

        //locate starting and goal positions
        int[] start = find(grid, 'K');
        int[] goal = find(grid, 'G');

        //priorityqueue for open list, sorted by total estimated cost
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingInt(n -> n.cost + n.estCost));

        //tracks visited nodes
        boolean[][] visited = new boolean[grid.length][grid[0].length];

        //add start node to open list
        Node startNode = new Node(start[0], start[1], 0, heuristic(start[0], start[1], goal[0], goal[1]), null);

        open.add(startNode);

        Node endNode = null;
        int nodesVisited = 0;

        //main A* search loop
        while (!open.isEmpty()) {

            //get node with lowest cost + heuristic
            Node current = open.poll();

            if (visited[current.x][current.y]) continue;

            visited[current.x][current.y] = true;
            nodesVisited++;

            //If goal is reached, break out
            if (current.x == goal[0] && current.y == goal[1]) {

                endNode = current;

                break;
            }

            //Explore neighbor nodes
            for (int i = 0; i < 4; i++) {

                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                //make sure we are inside grid and not visiting same node again
                if (inBounds(grid, nx, ny) && !visited[nx][ny]) {
                    char tile = grid[nx][ny];

                    int tileCost = terrainCost.getOrDefault(tile, Integer.MAX_VALUE);

                    if (tileCost == Integer.MAX_VALUE) continue; // Skip mountains

                    int newCost = current.cost + tileCost;

                    int estCost = heuristic(nx, ny, goal[0], goal[1]);

                    //add neighbor to open list
                    Node neighbor = new Node(nx, ny, newCost, estCost, current);

                    open.add(neighbor);

                }
            }
        }

        //reconstruct and print path if we found one
        if (endNode != null) {

            List<String> path = new ArrayList<>();

            int totalCost = 0;

            Node curr = endNode;

            //walk backward from goal to start using parent pointers
            while (curr != null) {

                path.add(0, "(" + curr.x + "," + curr.y + ")");

                if (curr.parent != null) // Don't count the start tile's cost

                    totalCost += terrainCost.get(grid[curr.x][curr.y]);

                curr = curr.parent;

            }

            //output results
            System.out.println("Nodes visited: " + nodesVisited);
            System.out.println("Shortest path length: " + path.size());
            System.out.println("Shortest path: " + String.join(" -> ", path));
            System.out.println("Total cost: " + totalCost);
            System.out.println("Execution time: " + (System.currentTimeMillis() - startTime) + "ms");

        } else {

            System.out.println("No path found.");

        }
    }

    //heuristic function from Part 1a: Manhattan distance × 2, 2 is cheapest terrain cost
    public static int heuristic(int x1, int y1, int x2, int y2) {

        //part 1a, our heuristic
        return 2 * (Math.abs(x1 - x2) + Math.abs(y1 - y2));

    }

    //helper method to make sure we don't go out of grid bounds
    public static boolean inBounds(char[][] grid, int x, int y) {

        return x >= 0 && y >= 0 && x < grid.length && y < grid[0].length;

    }

    //finds coordinates of a specific character, such as 'K' for start, 'G' for goal
    public static int[] find(char[][] grid, char target) {

        for (int i = 0; i < grid.length; i++)

            for (int j = 0; j < grid[0].length; j++)

                if (grid[i][j] == target)

                    return new int[]{i, j};

        return null;
    }
}

