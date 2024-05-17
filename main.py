from colorama import Back
import heapq
        

class Coords:
    """Operations with coordinates (indexes - i,j)"""
    
    def distance(i1, j1, i2, j2):
        """
        Return the distance between two points, (i1,j1) and (i2,j2)
        
        Args:
            i1, j1: Coordinates of point 1.
            i2, j2: Coordinates of point 2.
            
        Returns:
            Distance between points.
        """
        return pow(pow(i1 - i2, 2) + pow(j1 - j2, 2), 1/2)

    def is_valid(i, j, w, h):
        """
        Return True if '0 <= i < h' and '0 <= j < w' for a coordinate of (i,j)
        
        Args:
            i, j: Coordinates of point.
            w, h: Width and height of a grid.
            
        Returns:
            Bool.
        """
        return 0 <= i < h and 0 <= j < w

    def neighbors(i, j, w, h):
        """
        Return valid neighbors of (i,j), 8 other coordinates that forms an 3x3 are with it.
        'valid' means, for each neighbor it must be '0 <= n_i < h' and '0 <= n_j < w', when (n_i,n_j)
        is neighbor coordinate.
        
        Args:
            i, j: Coordinates of point.
            w, h: Width and height of a grid.
            
        Returns:
            A list of valid neighbors.
        """
        possible_neighbors = (
            (i+1, j), (i+1, j+1),
            (i-1, j), (i-1, j-1),
            (i, j+1), (i+1, j-1),
            (i, j-1), (i-1, j+1),
        )
        return [(ni, nj) for ni, nj in possible_neighbors if Coords.is_valid(ni, nj, w, h)]
        

class Cell:
    """Single unit in the Maze."""
    
    def __init__(self, f, g, h, i, j, v, parent_cell, is_open=False):
        """
        Create a new Cell instance. 
        
        Args:
            i, j: Coordinates of Cell.
            v: Value that the Cell holds.
            h: Holds the heuristic distance from Cell to goal.
            g: Holds the distance from start to Cell.
            f: g + h.
            parent_cell: Cell that comes beBack in the path.    
            is_open: Is the Cell open for evaluation?
            is_open: Is the Cell visited?
        """
        self.f, self.g, self.h = f, g, h
        self.i, self.j = i, j
        self.v = v
        self.parent = parent_cell
        self.is_open = is_open
        
    def __repr__(self):
        if self.parent is None:
            return f"{self.i},{self.j} - NoParent"
        return f"{self.i},{self.j} - {self.parent.i},{self.parent.j}"
    
    def __lt__(self, cell):
        return self.f < cell.f


class Maze:
    """Maze that will be loaded from a file."""
    
    def __init__(self, path):
        """
        Load and create a Maze instance from a file.
        
        Args:
            path: Path to maze file.
        """
        
        # Read the maze file and save characters in a grid (2D-list) called 'maze_raw'. 
        maze_raw = []
        with open(path) as fp:
            for i, row in enumerate(fp.readlines()):
                maze_raw.append(list())
                for j, c in enumerate(row):
                    
                    # Save start and goal cells locations, while reading the file.
                    if c == 'S':
                        self.start_i, self.start_j = i, j
                    elif c == 'G':
                        self.goal_i, self.goal_j = i, j
                    if c != '\n':
                        maze_raw[i].append(c)
        
        # Save maze width and height.
        self.w, self.h = j, i+1
        print(self.w, self.h)
        
        # List of permutation of indexes from i=(0-h) and j=(0-w).
        self.indexes = ((i, j) for i in range(self.h) for j in range(self.w))
        
        # While using 'maze_raw', create 'self.maze', a dict representation of maze with Cells. 
        self.maze = {}
        for i, j in self.indexes:
            # '-1' represents infinity. 'maze_raw[i][j]' is value of Cell. Currently, Cells have no parents.
            self.maze[(i, j)] = Cell(-1, -1, -1, i, j, maze_raw[i][j], None)
        
        # Find route from start to goal. Calculate number of coins.
        self.path = self.a_star_pathfind()
        self.collected_coins = sum(int(cell.v) for cell in self.path if cell.v.isnumeric())
        
    
    def a_star_pathfind(self):
        """
        Use A* to find a path from start to goal Cell.
        
        Returns:
            A list of Cells that forms the path from start to goal. If there
            is no possible path, an empty list.
        """
        
        # open_list: List of Cells that are not visited (evaluated) yet.
        open_list = []
        
        # Add starting Cell to open_list with g = 0 (as its distance to start is 0)
        start_cell = self.maze[(self.start_i, self.start_j)]
        start_cell.g = 0
        heapq.heappush(open_list, start_cell)
        
        # While open_list is not empty, so not all possible Cells are evaluated.
        while open_list:
            
            # Find the Cell with minimum f (total distance to goal) and remove it from open_list.
            # It will be called the 'current' Cell.
            current = heapq.heappop(open_list)
            
            # Look at current Cell's valid neighbors.
            for ni, nj in Coords.neighbors(current.i, current.j, self.w, self.h):
                neighbor_cell = self.maze[(ni, nj)]
                
                # Is neighbor goal? Then finish the search and return the path.
                if ni == self.goal_i and nj == self.goal_j:
                    
                    # Use parent's of Cells' to generate the path from goal back to start.
                    cell = neighbor_cell
                    cell.parent = current
                    path = []
                    
                    # Starting Cell's parent is None, stop if this is the case.
                    while cell is not None:
                        path.append(cell)
                        cell = cell.parent
                    return list(reversed(path))
                
                # If neighbor is wall, skip it.
                elif self.maze[(ni, nj)].v == 'X':
                    continue
                
                # Generate f, g, h values of the neighbor Cell, based on the current path.
                # There might be more than one way to reach a Cell, some are farther away from
                # the goal.
                # h is heuristic distance to goal, while g is distance of path from start.
                # f is sum of both, so f = g + h. 
                # In this case, checking if new f value is shorter or not.
                else:
                    g = current.g + Coords.distance(ni, nj, current.i, current.j)
                    h = Coords.distance(ni, nj, self.goal_i, self.goal_j)
                    f = g + h
                    
                    # First time the neighbor Cell is visited.
                    if not neighbor_cell.is_open:
                        neighbor_cell.f, neighbor_cell.g, neighbor_cell.h = f, g, h
                        neighbor_cell.parent = current
                        neighbor_cell.is_open = True
                        heapq.heappush(open_list, neighbor_cell)
                    
                    # Found a shorter path to the neighbor Cell.
                    # As Cells start with f=-1, this condition will not met until they get open.
                    elif f < neighbor_cell.f: 
                        neighbor_cell.f = f
                        neighbor_cell.parent = current
                        
            current.is_open = True
        
        # open_list got empty but goal haven't found. Return an empty list as path.
        return []
    
    
    def print(self):
        """Prints the maze and the path, colored and formatted."""
        s = ""
        for i in range(self.h):
            s += "\n"
            for j in range(self.w):
                c = self.maze[(i, j)].v
                
                if c == 'X':
                    s += Back.RED + "  " + Back.RESET
                elif c == 'S':
                    s += Back.YELLOW + "  " + Back.RESET
                elif c == 'G':
                    s += Back.GREEN + "  " + Back.RESET
                elif self.maze[(i, j)] in self.path:
                    s += Back.BLUE + "  " + Back.RESET
                else:
                    s += Back.WHITE + "  " + Back.RESET
        print(s)
        print(f"Collected {self.collected_coins} coins.")
            
    
if __name__ == '__main__':
    
    maze_path = input("Enter path for the maze: ")
    Maze(maze_path).print()
