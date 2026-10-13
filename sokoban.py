# TODO: Chep file sokoban.py tu framework giang vien cung cap; khong sua file framework.
'''Sokoban routines.

    A) Class SokobanState

    A specializion of the StateSpace Class that is tailored to the game of Sokoban.

    B) class Direction

    An encoding of the directions of movement that are possible for robots in Sokoban.

    Code also contains 20 public practice problems.
'''

from state import StateSpace

class SokobanState(StateSpace):
    """State Sokoban gồm robot, hộp, ô đích và tường."""

    def __init__(self, action, gval, parent, width, height, robots, boxes, storage, obstacles):
        """Khởi tạo state với vị trí và bản đồ; O(1), không sao chép dữ liệu."""
        StateSpace.__init__(self, action, gval, parent)
        self.width = width
        self.height = height
        self.robots = robots
        self.boxes = boxes
        self.storage = storage
        self.obstacles = obstacles    

    def successors(self):
        """Sinh các bước đi/đẩy hợp lệ; O(R(R+B)) thời gian, O(S(R+B)) bộ nhớ."""
        successors = []
        transition_cost = 1

        for robot in range(0, len(self.robots)):
          for direction in (UP, RIGHT, DOWN, LEFT):
              new_location = direction.move(self.robots[robot])
              new_robots = list(self.robots);
              new_robots.remove(self.robots[robot])
              new_robots = tuple(new_robots)
              new_boxes = set(self.boxes)
              
              if new_location[0] < 0 or new_location[0] >= self.width:
                  continue
              if new_location[1] < 0 or new_location[1] >= self.height:
                  continue
              if new_location in self.obstacles:
                  continue
              if new_location in new_robots:
                  continue
              
              if new_location in self.boxes:
                  new_box_location = direction.move(new_location)
                  
                  if new_box_location[0] < 0 or new_box_location[0] >= self.width:
                      continue
                  if new_box_location[1] < 0 or new_box_location[1] >= self.height:
                      continue
                  if new_box_location in self.obstacles:
                      continue
                  if new_box_location in new_robots:
                      continue
                  if new_box_location in new_boxes:
                      continue
                  
                  new_boxes.remove(new_location)
                  new_boxes.add(new_box_location)
              
              new_robots = list(self.robots)
              new_robots[robot] = new_location
              new_robots = tuple(new_robots)

              new_state = SokobanState(str(robot) + " " + direction.name, self.gval + transition_cost, self, self.width, self.height, new_robots, frozenset(new_boxes), self.storage, self.obstacles)
              successors.append(new_state)

        return successors

    def hashable_state(self):
        """Trả khóa cấu hình từ vị trí robot và hộp; tạo tuple O(1)."""
        return (self.robots, self.boxes)

    def state_string(self):
        """Dựng bản đồ dạng text; O(WH+R+B+G+T), với W,H là kích thước bàn."""
        map = []
        for y in range(0, self.height):
            row = []
            for x in range(0, self.width):
                row += [' ']
            map += [row]
        
        for storage_point in self.storage:
            map[storage_point[1]][storage_point[0]] = '.'
        for obstacle in self.obstacles:
            map[obstacle[1]][obstacle[0]] = '#'
        for i, robot in enumerate(self.robots):
            if robot in self.storage:
                map[robot[1]][robot[0]] = chr(ord('A') + i)
            else:
                map[robot[1]][robot[0]] = chr(ord('a') + i)
        for box in self.boxes:
            if box in self.storage:
                map[box[1]][box[0]] = '*'
            else:
                map[box[1]][box[0]] = '$'
        
        for y in range(0, self.height):
            map[y] = ['#'] + map[y]
            map[y] = map[y] + ['#']
        map = ['#' * (self.width + 2)] + map
        map = map + ['#' * (self.width + 2)]

        s = ''
        for row in map:
            for char in row:
                s += char
            s += '\n'

        return s        

    def print_state(self):
        """In action gần nhất và bản đồ text của state."""
        print("ACTION was " + self.action)      
        print(self.state_string())


def sokoban_goal_state(state):
  """Kiểm tra mọi hộp đã ở trên ô đích; O(B), với B là số hộp."""
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

'''
Sokoban Problem Set, for testing
'''
PROBLEMS = (
    SokobanState("START", 0, None, 5, 5, # dimensions
                 ((2, 1), (2, 3)), #robots
                 frozenset(((1, 1), (1, 3), (3, 1), (3, 3))), #boxes
                 frozenset(((0, 0), (0, 4), (4, 0), (4, 4))), #storage
                 frozenset(((1, 0), (2, 0), (3, 0), (1, 4), (2, 4), (3, 4))) #obstacles
                 ),
    SokobanState("START", 0, None, 5, 5, # dimensions
                 ((2, 1), (2, 3)), #robots
                 frozenset(((1, 1),)), #boxes
                 frozenset(((0, 0),)), #storage
                 frozenset(((1, 0), (2, 0), (3, 0), (1, 4), (2, 4), (3, 4))) #obstacles
                 ),
    SokobanState("START", 0, None, 5, 5, # dimensions
                 ((2, 2),), #robots
                 frozenset(((1, 1), (1, 3), (3, 1), (3, 3))), #boxes
                 frozenset(((0, 0), (0, 4), (4, 0), (4, 4))), #storage
                 frozenset(((1, 0), (2, 0), (3, 0), (1, 4), (2, 4), (3, 4))) #obstacles
                 ),
    SokobanState("START", 0, None, 6, 4, # dimensions
                 ((2, 1), (2, 2)), #robots
                 frozenset(((1, 1), (4, 2))), #boxes
                 frozenset(((2, 1), (2, 2))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 4, # dimensions
                 ((2, 1), (2, 2)), #robots
                 frozenset(((4, 2),)), #boxes
                 frozenset(((2, 1),)), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 5, 5, # dimensions
                 ((4, 0), (4, 4)), #robots
                 frozenset(((3, 1), (3, 2), (3, 3))), #boxes
                 frozenset(((0, 0), (0, 2), (0, 4))), #storage
                 frozenset(((2, 0), (2, 1), (2, 3), (2, 4))) #obstacles
                 ),
    SokobanState("START", 0, None, 5, 5, # dimensions
                 ((4, 0), (4, 4)), #robots
                 frozenset(((3, 1), (3, 2))), #boxes
                 frozenset(((0, 0), (0, 2))), #storage
                 frozenset(((2, 0), (2, 1), (2, 3), (2, 4))) #obstacles
                 ),
    SokobanState("START", 0, None, 5, 5, # dimensions
                 ((4, 0),), #robots
                 frozenset(((3, 1), (3, 2), (3, 3))), #boxes
                 frozenset(((0, 0), (0, 2), (0, 4))), #storage
                 frozenset(((2, 0), (2, 1), (2, 3), (2, 4))) #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((0, 0), (0, 2), (0, 4)), #robots
                 frozenset(((1, 0), (1, 2), (1, 4))), #boxes
                 frozenset(((5, 0), (5, 2), (0, 5))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((0, 0), (0, 2), (0, 4), (5, 5)), #robots
                 frozenset(((1, 0), (4, 1), (1, 2), (4, 3), (1, 4), (4, 5))), #boxes
                 frozenset(((5, 0), (0, 1), (5, 2), (0, 3), (5, 4), (0, 5))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((5, 5), (5, 4), (4, 5)), #robots
                 frozenset(((3, 1), (2, 2), (1, 4), (3, 4))), #boxes
                 frozenset(((0, 0), (0, 1), (1, 0), (1, 1))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((5, 5), (5, 4), (4, 5)), #robots
                 frozenset(((3, 1), (1, 4), (3, 4))), #boxes
                 frozenset(((0, 0), (0, 1), (1, 0))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((5, 5), (5, 4), (4, 5)), #robots
                 frozenset(((3, 1), (2, 2), (1, 4))), #boxes
                 frozenset(((0, 0), (0, 1), (1, 0))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((5, 5), (5, 4)), #robots
                 frozenset(((3, 1), (1, 4), (3, 4))), #boxes
                 frozenset(((0, 0), (0, 1), (1, 0))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 6, 6, # dimensions
                 ((5, 5), (5, 4)), #robots
                 frozenset(((3, 1), (2, 2), (1, 4))), #boxes
                 frozenset(((0, 0), (0, 1), (1, 0))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 8, 8, # dimensions
                 ((0, 5), (1, 6), (2, 7)), #robots
                 frozenset(((5, 6), (4, 5), (6, 2), (5, 2), (4, 6))), #boxes
                 frozenset(((0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 8, 8, # dimensions
                 ((0, 5), (1, 6), (2, 7)), #robots
                 frozenset(((6, 2), (5, 6), (4, 4), (6, 3))), #boxes
                 frozenset(((0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 8, 8, # dimensions
                 ((0, 5), (1, 6), (2, 7)), #robots
                 frozenset(((5, 4), (5, 5), (6, 3), (4, 2), (6, 5), (5, 3))), #boxes
                 frozenset(((0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 8, 8, # dimensions
                 ((0, 5), (1, 6), (2, 7)), #robots
                 frozenset(((6, 6), (5, 6), (6, 2), (4, 3), (5, 1), (6, 5))), #boxes
                 frozenset(((0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2))), #storage
                 frozenset() #obstacles
                 ),
    SokobanState("START", 0, None, 8, 8, # dimensions
                 ((0, 5), (1, 6), (2, 7)), #robots
                 frozenset(((6, 6), (4, 5), (4, 1), (4, 3), (5, 2), (5, 3))), #boxes
                 frozenset(((0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2))), #storage
                 frozenset() #obstacles
                 )
    )


'''
Sokoban Directions: encodes directions of movement that are possible for each robot.
'''
class Direction():
    """Hướng di chuyển được xác định bởi tên và độ lệch tọa độ."""
    
    def __init__(self, name, delta):
        """Lưu tên hướng và độ lệch tọa độ; O(1)."""
        self.name = name
        self.delta = delta
    
    def __hash__(self):
        """Băm tên hướng để dùng làm khóa; O(1) kỳ vọng."""
        return hash(self.name)
    
    def __str__(self):
        """Trả tên hướng để hiển thị; O(1)."""
        return str(self.name)
    
    def __repr__(self):
        """Dùng cùng biểu diễn với str()."""
        return self.__str__()
    
    def move(self, location):
        """Cộng độ lệch hướng vào tọa độ; O(1)."""
        return (location[0] + self.delta[0], location[1] + self.delta[1])


#Global Directions
UP = Direction("up", (0, -1))
RIGHT = Direction("right", (1, 0))
DOWN = Direction("down", (0, 1))
LEFT = Direction("left", (-1, 0))



  
