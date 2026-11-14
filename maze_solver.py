import heapq

class Node:
    """Represents a single cell/node in the grid maze."""
    def __init__(self, position, parent=None):
        self.position = position  # (row, col)
        self.parent = parent      # Parent node to trace the path
        self.g = 0                # Cost from start
        self.h = 0                # Heuristic cost to goal
        self.f = 0                # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    """Calculates the Manhattan distance heuristic."""
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def a_star_search(maze, start, goal):
    """Performs A* search to find the shortest path from start to goal."""
    rows = len(maze)
    cols = len(maze[0])

    start_node = Node(start, None)
    goal_node = Node(goal, None)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)
    open_dict = {start: start_node}

    while open_list:
        current_node = heapq.heappop(open_list)
        open_dict.pop(current_node.position, None)

        if current_node.position == goal_node.position:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        closed_set.add(current_node.position)

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dx, dy in neighbors:
            neighbor_pos = (current_node.position[0] + dx, current_node.position[1] + dy)

            if not (0 <= neighbor_pos[0] < rows and 0 <= neighbor_pos[1] < cols):
                continue

            if maze[neighbor_pos[0]][neighbor_pos[1]] == 1 or neighbor_pos in closed_set:
                continue

            g_cost = current_node.g + 1
            h_cost = heuristic(neighbor_pos, goal_node.position)
            f_cost = g_cost + h_cost

            if neighbor_pos in open_dict and open_dict[neighbor_pos].g <= g_cost:
                continue

            neighbor_node = Node(neighbor_pos, current_node)
            neighbor_node.g = g_cost
            neighbor_node.h = h_cost
            neighbor_node.f = f_cost

            heapq.heappush(open_list, neighbor_node)
            open_dict[neighbor_pos] = neighbor_node

    return None

def print_maze_with_path(maze, path, start, goal):
    """Prints a console visualization of the maze and solved path."""
    display_grid = [row[:] for row in maze]
    
    if path:
        for pos in path:
            if pos != start and pos != goal:
                display_grid[pos[0]][pos[1]] = '.'

    display_grid[start[0]][start[1]] = 'S'
    display_grid[goal[0]][goal[1]] = 'G'

    print("\nMaze Legend: S = Start, G = Goal, # = Wall, . = Path, (space) = Open")
    for r in display_grid:
        row_str = ""
        for cell in r:
            if cell == 1:
                row_str += "# "
            elif cell == 0:
                row_str += "  "
            else:
                row_str += f"{cell} "
        print(row_str)

if __name__ == "__main__":
    print("=== A* Maze Solver (Interactive) ===")
    rows = int(input("Enter number of rows: "))
    cols = int(input("Enter number of columns: "))

    print(f"\nEnter the maze row by row ({rows} rows).")
    print("Use 0 for open path and 1 for wall, separated by spaces (e.g., 0 0 1 0):")
    
    maze = []
    for i in range(rows):
        while True:
            row_input = list(map(int, input(f"Row {i} values: ").strip().split()))
            if len(row_input) == cols:
                maze.append(row_input)
                break
            print(f"Error: Expected {cols} values. Try again.")

    print("\nEnter coordinates (0-indexed row and column).")
    start_r = int(input("Start Row: "))
    start_c = int(input("Start Col: "))
    start_pos = (start_r, start_c)

    goal_r = int(input("Goal Row: "))
    goal_c = int(input("Goal Col: "))
    goal_pos = (goal_r, goal_c)

    print("\nRunning A* Search...")
    path = a_star_search(maze, start_pos, goal_pos)

    if path:
        print(f"\nPath found successfully! Total steps: {len(path) - 1}")
        print(f"Coordinates path: {path}")
        print_maze_with_path(maze, path, start_pos, goal_pos)
    else:
        print("\nDestination is unreachable! No valid path exists.")
