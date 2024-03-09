
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import numpy as np
import math



class World:
    def __init__(self):
        self.obstacle_x = 70
        self.obstacle_y  = 90
        self.obstacle_width = 60
        self.obstacle_height = 60
        self.vehicle1_x = 30
        self.vehicle1_y = 10
        self.vehicle2_x = 120
        self.vehicle2_y = 10
        self.robot_x = 0
        self.robot_y = 180
        self.robot_theta = 0
        self.vehicle_width = 30
        self.vehicle_height = 20
        self.padding = 5
        self.robot_start = [self.robot_x + 10, self.robot_y + self.vehicle_height/2,0]
        self.robot_goal = [self.vehicle1_x + self.vehicle_width + 30, 10 + self.vehicle_height/2,0]
        self.robot_wheelRadius = 1
        self.robot_wheelBase = 10
        self.vel_L = [5,2,1,0,-1,-2,-5]
        self.vel_R = [5,2,1,0,-1,-2,-5]
        self.park_point = False
        self.robot_boundary = [[self.robot_x,self.robot_y,1],[self.robot_x+self.vehicle_width,self.robot_y,1],[self.robot_x+self.vehicle_width,self.robot_y+self.vehicle_height,1],[self.robot_x,self.robot_y+self.vehicle_height,1]]
        self.obstacle = [[self.obstacle_x-self.padding,self.obstacle_y-self.padding],[self.obstacle_x+self.obstacle_width+self.padding,self.obstacle_y-self.padding],[self.obstacle_x+self.obstacle_width+self.padding,self.obstacle_y+self.obstacle_height+self.padding],[self.obstacle_x-self.padding,self.obstacle_y+self.obstacle_height+self.padding]]
        self.vehicle1 = [[self.vehicle1_x-self.padding,self.vehicle1_y-self.padding],[self.vehicle1_x+self.vehicle_width+self.padding,self.vehicle1_y-self.padding],[self.vehicle1_x+self.vehicle_width+self.padding,self.vehicle1_y+self.vehicle_height+self.padding],[self.vehicle1_x-self.padding,self.vehicle1_y+self.vehicle_height+self.padding]]
        self.vehicle2 = [[self.vehicle2_x-self.padding,self.vehicle2_y-self.padding],[self.vehicle2_x+self.vehicle_width+self.padding,self.vehicle2_y-self.padding],[self.vehicle2_x+self.vehicle_width+self.padding,self.vehicle2_y+self.vehicle_height+self.padding],[self.vehicle2_x-self.padding,self.vehicle2_y+self.vehicle_height+self.padding]]
        self.robot_bound = [[-10,10,10,-10],[-10,-10,10,10],[1,1,1,1]]


    def world_(self,x,y,theta):
        fig = plt.figure("Animation")
        ax = fig.add_subplot(111)
        obstacle_draw = Rectangle((self.obstacle_x, self.obstacle_y),self.obstacle_width,self.obstacle_height,color= "black")
        vehicle1_draw = Rectangle((self.vehicle1_x,self.vehicle1_y),self.vehicle_width,self.vehicle_height,color='blue')
        vehicle2_draw = Rectangle((self.vehicle2_x,self.vehicle2_y),self.vehicle_width,self.vehicle_height,color='blue')
        robot_draw = Rectangle((self.robot_x,self.robot_y),self.vehicle_width,self.vehicle_height,color='pink')
        parking_position = Rectangle((self.vehicle1_x+self.vehicle_width+10,5),self.vehicle_width+10,self.vehicle_height+10,fc='none',ec='g',lw=2)
        ax.add_patch(robot_draw)
        ax.add_patch(vehicle1_draw)
        ax.add_patch(vehicle2_draw)
        ax.add_patch(obstacle_draw)
        ax.add_patch(parking_position)
        plt.plot(x,y,"sk")
        boundary = self.get_boundary(x,y,theta)
        X = []
        Y = []
        for x,y in boundary:
            X.append(x)
            Y.append(y)
        plt.plot(X,Y)
        plt.xlim([0,200])
        plt.ylim([-20,200])
        return

    def get_boundary(self,x,y,theta):
        theta_new = (theta - self.robot_start[2])*(math.pi/180)
        homo_matrix = [[math.cos(theta_new),-math.sin(theta_new),x],[math.sin(theta_new),math.cos(theta_new),y]]
        matrix = np.dot(homo_matrix,self.robot_bound)
        updated_boundary =  [[matrix[0][0],matrix[1][0]],[matrix[0][1],matrix[1][1]],[matrix[0][2],matrix[1][2]],[matrix[0][3],matrix[1][3]]]
        return updated_boundary

    def get_neighbours(self,x,y,theta):
        neighbours = []
        for vr in self.vel_R:
            for vl in self.vel_L:
                if vl == 0 and vr == 0:
                    continue
                vel = (vl+vr)/2
                x_dot = vel * math.cos(theta*(math.pi/180))
                y_dot = vel * math.sin(theta*(math.pi/180))
                theta_dot = (self.robot_wheelRadius/(2*self.robot_wheelBase)) * (vl-vr)*(180/math.pi)
                if(self.valid_point(x+x_dot,y+y_dot,theta+theta_dot)):
                    neighbours.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,vl,vr])
        return neighbours
    
    def valid_point(self,x,y,theta):
        boundary = self.get_boundary(x,y,theta)
        if x < 1 or y < self.vehicle_height or x > 200 - self.vehicle_width or y > 200 - self.vehicle_height/2.0:
            return False
        collision_check1 = self.polygons_intersection_checker(boundary,self.obstacle)
        collision_check2 = self.polygons_intersection_checker(boundary,self.vehicle1)
        collision_check3 = self.polygons_intersection_checker(boundary,self.vehicle2)
        if (collision_check1 or collision_check2 or collision_check3):
            return False
        return True
    
    def polygons_intersection_checker(self,polygonA, polygonB):
        polygons = [polygonA,polygonB]
        minA, minB, maxA, maxB, projected, i, j, k = None, None, None, None, None, None, None, None
        for i in range(len(polygons)):
            polygon = polygons[i]
            for j in range(len(polygon)):
                vertice_1 = j
                vertice_2 = (vertice_1 + 1) % len(polygon)
                p1 = polygon[vertice_1]
                p2 = polygon[vertice_2]
                normal = {'x':p2[1] - p1[1],'y':p1[0]-p2[0]}
                minA, maxA = None, None 
                for k in range(len(polygonA)):
                    projected = normal['x'] * polygonA[k][0] + normal['y']*polygonA[k][1]
                    if (minA is None) or (projected < minA):
                        minA = projected
                    if (maxA is None) or (projected > maxA):
                        maxA = projected

                minB, maxB = None, None 
                for k in range(len(polygonB)):
                    projected = normal['x'] * polygonB[k][0] + normal['y']*polygonB[k][1]
                    if (minB is None) or (projected < minB):
                        minB = projected
                    if (maxB is None) or (projected > maxB):
                        maxB = projected
                if (maxA < minB) or (maxB < minA):
                    return False
        return True
                
    def cost_(self,x1,y1,x2,y2):
        distance = math.sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
        return distance
    
    def calc_heuristic(self,x,y,theta,vl,vr):
        _theta = 0
        theta = (theta+360)%360
        vel = (vl+vr)/2
        reverse_penalty = 0
        parking_penalty = 0
        distance = math.sqrt((pow(self.robot_goal[0]-x,2)+pow(self.robot_goal[1]-y,2)))

        if self.straight_available(x,y) and not(x > self.robot_goal[0]-1 and y > self.robot_goal[1]-1 and x <self.robot_goal[0]+1 and y <self.robot_goal[1]+1):
            _theta = abs((360 + (math.atan2(y-self.robot_goal[1],x-self.robot_goal[0]))*(180/math.pi))%360 - theta+180)
        else:
            _theta = (self.robot_goal[2]-theta+360)%360
        if vel < 0:
            reverse_penalty = 1
        
        if (x>self.robot_goal[0]-1 and y>self.robot_goal[1]-1 and x < self.robot_goal[0]+1 and y < self.robot_goal[1]+1):
            self.park_point = True
        
        if self.park_point and not(x>self.robot_goal[0]-1 and y>self.robot_goal[1]-1 and x < self.robot_goal[0]+1 and y < self.robot_goal[1]+1) and self.vel_L == self.vel_R:
            parking_penalty = 180

        heuristic = distance + _theta + reverse_penalty + parking_penalty
        return heuristic

    def straight_available(self,x,y):
        boundary_line = [[x,y],[self.robot_goal[0],self.robot_goal[1]],[self.robot_goal[0]+1,self.robot_goal[1]],[x+1,y]]
        if (self.polygons_intersection_checker(boundary_line,self.obstacle)) or (self.polygons_intersection_checker(boundary_line,self.vehicle1)):
            return False
        return True

    def priority(self,queue):
        min = math.inf
        index = 0
        for i in range(len(queue)):
            _,value,_,_ = queue[i]
            if value < min:
                min = value
                index = i
        return index
    
    def visited_check(self,curr_node,visited):
        for x,y,theta in visited:
            if curr_node[0] == x and curr_node[1] == y and curr_node[2] == theta:
                return True
        return False

    def A_star(self):
        queue = []
        visited = []
        start = self.robot_start
        f = 0
        g = 0
        path = [start]
        queue.append((start,f,g,path))
        while len(queue) > 0:
            index = self.priority(queue)
            (shortest,_,g_value,path) = queue[index]
            queue.pop(index)
            if not (self.visited_check((round(shortest[0]),round(shortest[1]),round(shortest[2])),visited)):
                visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2])])
                if shortest[2] > 180:
                    angle = 360 - shortest[2]
                else:
                    angle = shortest[2]
                
                if round(shortest[0]) <= self.robot_goal[0]+1 and round(shortest[0]) >= self.robot_goal[0]-1 and round(shortest[1]) <= self.robot_goal[1]+1 and round(shortest[1]) >= self.robot_goal[1]-1 and angle <= self.robot_goal[2]+5 and angle >= self.robot_goal[2]-5:
                    return path
                neighbours = self.get_neighbours(shortest[0],shortest[1],shortest[2])
                for neighbour in neighbours:
                    vl = neighbour[3]
                    vr = neighbour[4]
                    t_g = g_value + (0.1*self.cost_(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                    t_f = t_g +(0.9*self.calc_heuristic(neighbour[0],neighbour[1],neighbour[2],vl,vr))
                    queue.append((neighbour,t_f,t_g,path + [neighbour]))
        return path

if __name__ == "__main__":
    diff_drive_world = World()
    trajectory = diff_drive_world.A_star()
    print("Reached")
    print(trajectory[-1])
    for point in trajectory:
        plt.cla()
        diff_drive_world.world_(point[0],point[1],point[2])
        plt.pause(0.00001)



plt.figure("Axle Path")
plt.xlim([0,200])
plt.ylim([-20,200])
for point in trajectory:
    plt.scatter(point[0],point[1],color="black",s=1)

plt.show()

        



