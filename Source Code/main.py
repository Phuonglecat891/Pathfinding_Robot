import matplotlib.pyplot as plt
import time
from Map import Map
import tkinter as tk
import os
import keyboard
import random

class State:
    def __init__(self, goal, parent=None, position=(int, int), g=100):
        # init unopened state
        self.checked = False
        self.parent = parent

        # current coordinates
        self.position = position

        # heuristic values
        self.g = g
        self.h = abs(self.position[0] - goal[0]) + \
            abs(self.position[1] - goal[1])
        self.f = self.g + self.h

    def __eq__(self, other):
        return (self.position == other.position)

def insert_node(open_list, node):
    if len(open_list) == 0:
        open_list.append(node)
    else:
        open_list.append(node)
        i = len(open_list) - 1
        while i > 0 and open_list[i].f < open_list[i-1].f:
            temp = open_list[i]
            open_list[i] = open_list[i-1]
            open_list[i-1] = temp
            i -= 1

def getChildren(pos: (int, int), Map: Map, parentG):
    result = []
    if pos[0] > 0 and pos[0] < Map._map.__len__()-1:
        result.append(State((pos[0]-1, pos[1]), Map.goal, 0))
        result.append(State((pos[0]+1, pos[1]), Map.goal, 0))
    elif pos[0] == 0:
        result.append(State((pos[0]+1, pos[1]), Map.goal, 0))
    elif pos[0] == Map._map.__len__()-1:
        result.append(State((pos[0]-1, pos[1]), Map.goal, 0))

def AStar_Search(Start, Goal, Map, checked_list):
    if Map.isBlocked(Start[0], Start[1]):
        return [], 0
    if Map.isBlocked(Goal[0], Goal[1]):
        return [], 0

    # Start node, Goal node
    start_state = State(Goal, None, Start, 0)
    goal_state = State(Goal, None, Goal, 0)

    # list of opened and closed points
    open_list = []
    closed_list = []

    # add start to open list
    open_list.append(start_state)

    while len(open_list) > 0:
        # Get current node
        current_node = open_list[0]
        open_list.pop(0)
        closed_list.append(current_node)

        # Check if current node is goal
        if current_node == goal_state:
            # Back track to get path
            parent = current_node.parent
            if parent is None:
                return [], 0
            else:
                path = []
                path.append(goal_state.position)
                while parent != start_state:
                    path.append(parent.position)
                    parent = parent.parent
                return path[::-1], current_node.g

        # Check if is neighbors of current node, append it to a list
        neighbors_index1 = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        neighbors_index2 = [(1, 1), (-1, -1), (-1, 1), (1, -1)]
        neighbors = []
        for i in neighbors_index1:
            pos = (current_node.position[0] + i[0],
                   current_node.position[1] + i[1])
            # Check walkable
            if Map.isBlocked(pos[0], pos[1]):
                continue
            if pos not in checked_list:
                new_node = State(Goal, current_node, pos, current_node.g + 1)
                neighbors.append(new_node)
        for i in neighbors_index2:
            pos = (current_node.position[0] + i[0],
                   current_node.position[1] + i[1])
            # Check walkable
            if Map.isBlocked(pos[0], pos[1]):
                continue
            if Map.isBlocked(pos[0], current_node.position[1]) \
                and Map.isBlocked(current_node.position[0], pos[1]):
                continue
            if pos not in checked_list:
                new_node = State(Goal, current_node, pos, current_node.g + 1.5)
                neighbors.append(new_node)

        # Check neighbor list
        for child in neighbors:
            # Check if is in closed list
            check = 1  # still can be add to open list
            for i in closed_list:
                if child.__eq__(i):
                    check = 0
                    continue
            # Check if child is in open list
            for index, item in enumerate(open_list):
                if item.__eq__(child):
                    check = 0
                    if child.f < item.f:  # child is optimal
                        open_list.pop(index)
                        insert_node(open_list, child)
                        continue
                    else:
                        continue
            if check:
                insert_node(open_list, child)
    # can not find any path
    return [], 0

def Greedy_DFS_Recursive(checked_List, start_node, current_node, goal_node, Map, cost):
    if Map.isBlocked(start_node.position[0], start_node.position[1]):
        return [], 0
    if current_node == goal_node:
        parent = current_node.parent
        path = [goal_node.position]
        while parent != start_node:
            path.append(parent.position)
            parent = parent.parent
        # Optimize
        index=0
        path.append(start_node.position)
        while index < len(path) - 2:
            if path[index + 1][0] != path[index][0] and path[index + 1][1] != path[index][1]:
                if (path[index+1][0], path[index][1]) in path and (path[index][0], path[index+1][1]) in path:
                    x,y=(path[index+1][0], path[index][1]),(path[index][0], path[index+1][1])
                    j = index + 1
                    while path[j]!=x and path[j]!=y:
                        path.pop(j)
            index+=1
        cost = len(path) - 1
        for i in range(len(path) - 1):
            if path[i + 1][0] != path[i][0] and path[i + 1][1] != path[i][1]:
                cost+=0.5
        for i in range(len(path)):
            if path[i] == start_node.position:
                path.pop(i)
        return path, cost
    else:
        checked_List.append(current_node.position)
        # Check if is neighbors of current node, append it to a list
        neighbors_index1 = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        neighbors_index2 = [(1, 1), (-1, -1), (-1, 1), (1, -1)]
        neighbors = []
        for i in neighbors_index1:
            pos = (current_node.position[0] + i[0],
                   current_node.position[1] + i[1])
            # Check walkable
            if Map.isBlocked(pos[0], pos[1]):
                continue
            if pos not in checked_List:
                new_node = State(goal_node.position, current_node, pos, 0)
                insert_node(neighbors, new_node)
        for i in neighbors_index2:
            pos = (current_node.position[0] + i[0],
                   current_node.position[1] + i[1])
            # Check walkable
            if Map.isBlocked(pos[0], pos[1]):
                continue
            if Map.isBlocked(pos[0], current_node.position[1]) and Map.isBlocked(current_node.position[0], pos[1]):
                continue
            if pos not in checked_List:
                new_node = State(goal_node.position, current_node, pos, 0)
                insert_node(neighbors, new_node)
        # Check neighbor list
        for i in neighbors:
            if abs(i.position[0] - current_node.position[0]) + abs(i.position[1] - current_node.position[1]) == 2:
                path, distance = Greedy_DFS_Recursive(checked_List, start_node, i, goal_node, Map, cost + 1.5)
            else:
                path, distance = Greedy_DFS_Recursive(checked_List, start_node, i, goal_node, Map, cost + 1)
            if path != []:
                return path, distance
        return [], 0

def Greedy_DFS_Search(Start, Goal, Map, checked_List=[]):
    if Map.isBlocked(Start[0], Start[1]):
        return [], 0
    if Map.isBlocked(Goal[0], Goal[1]):
        return [], 0
    check_list = checked_List.copy()
    start_state = State(Goal, None, Start, 0)
    goal_state = State(Goal, None, Goal, 0)
    cost = 0
    path, cost = Greedy_DFS_Recursive(check_list, start_state, start_state, goal_state, Map, cost)
    return path[::-1], cost

class Node_Dijkstra:
    def __init__(self, parent=None, position=(int, int), distance=1000):
        #
        self.parent = parent
        self.position = position
        self.f = distance

    def __eq__(self, other):
        return (self.position == other.position)

def Dijkstra_Search(Start, Goal, Map, checked_list):
    if Map.isBlocked(Start[0], Start[1]):
        return [], 0
    if Map.isBlocked(Goal[0], Goal[1]):
        return [], 0

    # Create Start and Goal Node
    start_node = Node_Dijkstra(None, Start, 0)
    goal_node = Node_Dijkstra(None, Goal)

    # Create a List of unexplored node
    unexplored_list = []
    explored_list = []

    # Set all node to unexplored
    for i in range(len(Map._map)):
        for j in range(len(Map._map[i])):
            i_node = Node_Dijkstra(None, (i, j))
            if Map.isBlocked(i, j):
                continue
            insert_node(unexplored_list, i_node)
    for index, item in enumerate(unexplored_list):
        if item == start_node:
            unexplored_list.pop(index)
            break
    insert_node(unexplored_list, start_node)

    # Loop until the unexplored set is empty
    while len(unexplored_list) > 0:
        current_node = unexplored_list[0]

        # Remove current node from the unexplored set
        unexplored_list.pop(0)
        explored_list.append(current_node)

        # Check if current Node is Goal
        if current_node == goal_node:
            # Back track to get path
            parent = current_node.parent
            if parent is None:
                return [], 0
            else:
                path = []
                path.append(goal_node.position)
                while parent != start_node:
                    path.append(parent.position)
                    parent = parent.parent
                return path[::-1], current_node.f

        # Check if neighbors of current node, append it to a list
        neighbors_index1 = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        neighbors_index2 = [(1, 1), (-1, -1), (-1, 1), (1, -1)]
        neighbors = []
        for i in neighbors_index1:
            pos = (current_node.position[0] + i[0],
                   current_node.position[1] + i[1])
            # Check walkable
            if Map.isBlocked(pos[0], pos[1]):
                continue
            if pos not in checked_list:
                dist = current_node.f + 1
                new_node = Node_Dijkstra(current_node, pos, dist)
                neighbors.append(new_node)
        for i in neighbors_index2:
            pos = (current_node.position[0] + i[0],
                   current_node.position[1] + i[1])
            # Check walkable
            if Map.isBlocked(pos[0], pos[1]):
                continue
            if Map.isBlocked(pos[0], current_node.position[1]) and Map.isBlocked(current_node.position[0], pos[1]):
                continue
            if pos not in checked_list:
                dist = current_node.f + 1.5
                new_node = Node_Dijkstra(current_node, pos, dist)
                neighbors.append(new_node)

        # Check neighbor list
        for child in neighbors:
            # Check if is in explored list
            for i in explored_list:
                if child == i:
                    continue

            # Check if distance is better
            for index, item in enumerate(unexplored_list):
                if item == child:
                    if item.f > child.f:
                        unexplored_list.pop(index)
                        insert_node(unexplored_list, child)
    # Can not find any path
    return [], 0

def Read_Map(file_name):
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Open and read file content by line
    f = open(current_dir + '\\' + file_name, 'r')
    file_content = f.readlines()
    f.close()

    # Get number of rows and columns
    col, row = int(file_content[0].split(',')[0]) + \
        1, int(file_content[0].split(',')[1]) + 1

    # Get start and goal positions
    points_content = file_content[1].split(',')
    start = (row - int(points_content[1]) - 1, int(points_content[0]))
    goal = (row - int(points_content[3]) - 1, int(points_content[2]))

    # Check if there are pickup points
    pickup_points = []
    if points_content.__len__() > 4:
        i = 4
        while i < points_content.__len__():
            pickup_points.append(
                (row - int(points_content[i+1]) - 1, int(points_content[i])))
            i += 2

    # Initialize new map
    new_map = Map(row, col, start, goal)
    new_map.addPickupPoint(pickup_points)

    # Remove extra lines at the end
    while file_content[file_content.__len__()-1] == '\n':
        file_content.remove('\n')

    # Get obstacles
    N = int(file_content[2])
    if N == 0:
        return new_map
    obstacle_points = []
    for index in range(N):
        obstacles_content = file_content[3+index].split(',')
        obstacle_vertex = []

        i = 0
        while i < obstacles_content.__len__():
            obstacle_vertex.append((row - int(obstacles_content[i+1]) - 1, int(obstacles_content[i])))
            i += 2

        obstacle_points += obstacle_vertex
        for j in range(obstacle_vertex.__len__()):
            if j == obstacle_vertex.__len__()-1:
                obstacle_points += linePoints(
                    obstacle_vertex[j], obstacle_vertex[0])
            else:
                obstacle_points += linePoints(
                    obstacle_vertex[j], obstacle_vertex[j+1])
    # Remove duplicates
    obstacle_points = list(dict.fromkeys(obstacle_points))
    new_map.addObstacle(obstacle_points)

    return new_map

def lineLow(start: (int, int), end: (int, int)):
    points = []
    dX = end[0] - start[0]
    dY = end[1] - start[1]
    yi = 1
    if dY < 0:
        yi = -1
        dY = -dY
    D = 2*dY - dX
    y = start[1]

    for x in range(start[0], end[0]):
        points.append((x, y))
        if D > 0:
            y += yi
            D -= 2*dX
        D += 2*dY

    return points

def lineHigh(start: (int, int), end: (int, int)):
    points = []
    dX = end[0] - start[0]
    dY = end[1] - start[1]
    xi = 1
    if dX < 0:
        xi = -1
        dX = -dX
    D = 2*dX - dY
    x = start[0]

    for y in range(start[1], end[1]):
        points.append((x, y))
        if D > 0:
            x += xi
            D -= 2*dY
        D += 2*dX

    return points

def linePoints(start: (int, int), end: (int, int)):
    if abs(end[1] - start[1]) < abs(end[0] - start[0]):
        if start[0] > end[0]:
            return lineLow(end, start)
        else:
            return lineLow(start, end)
    else:
        if start[1] > end[1]:
            return lineHigh(end, start)
        else:
            return lineHigh(start, end)

# Out put a message
def dialog(message):
        root1=tk.Tk()
        ourMessage = message
        messageVar = tk.Message(root1, text = ourMessage,width = '25c') 
        messageVar.config(bg='lightpink',font=("Arial", 30, "bold"),justify='center', fg = 'purple') 
        messageVar.pack()
        root1.mainloop()

def Func(func, file_name):
    Map1 = Read_Map(file_name)
    pickUpPoint, Points = [], []
    start, goal = Map1.start, Map1.goal
    Points.append(start)
    pickUpPoint = list.copy(Map1.pickupPoints)
    pickUpPoint1 = pickUpPoint.copy()

    while len(pickUpPoint) > 0:
        index_min = 0
        min = abs(pickUpPoint[0][0] - start[0]) + \
            abs(pickUpPoint[0][1] - start[1])
        for i in range(1, len(pickUpPoint)):
            if (abs(pickUpPoint[i][0] - start[0]) + abs(pickUpPoint[i][1] - start[1])) < min:
                min = abs(pickUpPoint[i][0] - start[0]) + \
                    abs(pickUpPoint[i][1] - start[1])
                index_min = i
        start = pickUpPoint.pop(index_min)
        Points.append(start)

    Points.append(goal)
    path = []
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.show()
    plt.imshow(Map1._map)
    fig.canvas.draw()
    fig.canvas.draw()
    time.sleep(0.1)
    path_save, cost_save, cost = [], 0, 0
    for i in range(len(Points) - 1):
        check_list = []
        if Points[i + 1] != goal:
            check_list.append(goal)
        path, cost = func(Points[i], Points[i+1], Map1, check_list)
        path_save.append(path.copy())
        cost_save += cost
        if path == []:
            time.sleep(0.1)
            break
        for j in range(len(path)):
            Map1.deStartAndPickup([Points[i]])
            Map1.addPath([path[j]])
            plt.imshow(Map1._map)
            fig.canvas.draw()
            time.sleep(0.02)
            ax.clear()
            Map1.dePath([path[j]])

    # Show the whole way
    if path == []:
        dialog('Không tìm thấy đường thầy ơi!!!')
    else:
        for i in path_save:
            Map1.addPath(i)
        Map1.addPickupPoint(pickUpPoint1)
        Map1.addStart(Points[0])
        Map1.addGoal(Points[len(Points) - 1])
        plt.imshow(Map1._map)
        fig.canvas.draw()
        time.sleep(0.1)
        dialog('Tìm đường xong rồi thầy ơi, chi phí là ' + str(cost_save))

def funcMovingObject(func, file_name):
    random.seed()
    Map1 = Read_Map(file_name)
    start, goal = Map1.start, Map1.goal

    fig = plt.subplots(figsize=(10, 5))[0]
    fig.show()
    plt.imshow(Map1._map)
    fig.canvas.draw()
    fig.canvas.draw()
    time.sleep(0.1)
    check_list, path = [], []
    while not keyboard.is_pressed('esc'):
        rand = random.randrange(4)
        Map1.dePath(path)
        if rand==0:
            Map1.moveLeftObstacle()
        elif rand == 1:
            Map1.moveRightObstacle()
        elif rand == 2:
            Map1.moveUpObstacle()
        else:
            Map1.moveDownObstacle()
        path = func(start, goal, Map1, check_list)[0]
        Map1.addPath(path)
        Map1.addStart(start)
        Map1.addGoal(goal)
        plt.imshow(Map1._map)
        fig.canvas.draw()
        time.sleep(0.1)
    dialog('Chạy xong rồi thầy ơi!!!')

if __name__ == '__main__':
    file_name = ''
    root = tk.Tk()
    root.title('Menu')
    menu = tk.Menu(root)
    root.config(background = 'pink', menu=menu, heigh = '3c', width = '6c')

    GreedyMenu = tk.Menu(menu)
    def Greedy1():
        Func(Greedy_DFS_Search, "map1.txt")
    def Greedy2():
        Func(Greedy_DFS_Search, "map2.txt")
    def Greedy3():
        Func(Greedy_DFS_Search, "map3.txt")
    def Greedy4():
        funcMovingObject(Greedy_DFS_Search, "map3.txt")
    menu.add_cascade(label='GREEDY', menu=GreedyMenu)
    GreedyMenu.add_command(label='Map 1', command=Greedy1)
    GreedyMenu.add_command(label='Map 2', command=Greedy2)
    GreedyMenu.add_command(label='Map 3', command=Greedy3)
    GreedyMenu.add_command(label='Moving Object', command=Greedy4)

    def Astar1():
        Func(AStar_Search, "map1.txt")
    def Astar2():
        Func(AStar_Search, "map2.txt")
    def Astar3():
        Func(AStar_Search, "map3.txt")
    def Astar4():
        funcMovingObject(AStar_Search, "map3.txt")
    AstarMenu = tk.Menu(menu)
    menu.add_cascade(label='ASTAR', menu=AstarMenu, )
    AstarMenu.add_command(label='Map 1', command=Astar1)
    AstarMenu.add_command(label='Map 2', command=Astar2)
    AstarMenu.add_command(label='Map 3', command=Astar3)
    AstarMenu.add_command(label='Moving Object', command=Astar4)

    def Dijkstra1():
        Func(Dijkstra_Search, "map1.txt")
    def Dijkstra2():
        Func(Dijkstra_Search, "map2.txt")
    def Dijkstra3():
        Func(Dijkstra_Search, "map3.txt")
    def Dijkstra4():
        funcMovingObject(Dijkstra_Search, "map3.txt")
    DijkstraMenu = tk.Menu(menu)
    menu.add_cascade(label='DIJKSTRA', menu=DijkstraMenu)
    DijkstraMenu.add_command(label='Map 1', command=Dijkstra1)
    DijkstraMenu.add_command(label='Map 2', command=Dijkstra2)
    DijkstraMenu.add_command(label='Map 3', command=Dijkstra3)
    DijkstraMenu.add_command(label='Moving Object', command=Dijkstra4)

    root.mainloop()
