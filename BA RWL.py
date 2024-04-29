"""""""""""""""""""""""
  FULL INITIALISATION
"""""""""""""""""""""""
#import packages
import time
import random
import math
import matplotlib.pyplot as plt
import pandas as pd
import os

start=time.time()

#Initialise robot, station, bee and global arrays
class Robot:
    def __init__(self, max_radius, min_radius, x, y):
        self.max_radius = max_radius
        self.min_radius = min_radius
        self.x = x
        self.y = y

global robot
robot = Robot(911,236,0,0)

class Station:
    def __init__(self, x, y, x1, y1, angle):
      self.x = x            #x-coordinate for station access point
      self.y = y            #y-coordinate for station access point
      self.x1 = x1          #x coordinate bottom left corner of station
      self.y1 = y1          #y coordinate bottom left corner of station
      self.angle = angle

global stationsarray
stationsarray = []

class Bee:
  #initialisation
  def __init__(self, cost, stations):
    self.cost = cost
    self.stations = stations

global bees
bees=[]
global selected_bees
selected_sites = []
global elite_bees
elite_sites = []
global best_bee
best_bee = []

global angle
#      0deg   90deg    180deg       270deg
angle = [0, math.pi/2, math.pi, (3/2)*math.pi]

#BA Parameters
numStations = 5                            #number of stations per bee
maxIt = 10000                               #maximum number of iterations for optimisation
Scout_Bees = 10                             #n - number of scout bees
SelectedSites = 4                           #m - number of sites
SelectedSite_Bees = 2                       #nsp - number of bees for selected sites
EliteSites = 0                              #e is- number of elite sites
EliteSite_Bees = 0                          #nep - number of bees for eite sites
GlobalScout_Bees = 2                        #n-m bees for global search

from typing_extensions import TypeVarTuple
#distance
def calc_distance(machinex, machiney):
  distance=0
  distance = math.sqrt((machinex)**2 + (machiney)**2)
  return distance


#cost
def calc_cost(slist):
  #calculates sum of all station distances
  total_dist=0
  for station in slist:
    total_dist = (total_dist + calc_distance(station.x, station.y))
  
  #cost is the normalised value of the cumulative distance of all stations from the centre within the constraints of the possible
  cost = (total_dist-((robot.min_radius + 125)*numStations))/((robot.max_radius*numStations) - ((robot.min_radius + 125)*numStations))              #low cost is desirable
  return cost

#random generation/initialisation
def global_search():
  s=0

  while s < numStations:
    #random angle about robot origin
    robot_ref_angle = random.uniform(0,1)*2*math.pi
    #random length from robot origin
    robot_ref_length = random.randint(robot.min_radius + 125, robot.max_radius)
    #generate station access coordiantes using polar coordinates
    rand_x = robot.x + int(robot_ref_length * math.cos(robot_ref_angle))
    rand_y = robot.y + int(robot_ref_length * math.sin(robot_ref_angle))
    #random station angle
    randangle = angle[random.randint(0,3)]
    #generate x1, y1 using station angle, x, y
    rand_x1, rand_y1 = generate_x1_y1(randangle, rand_x, rand_y)
    stationsarray.append(Station(rand_x, rand_y, rand_x1, rand_y1, randangle))
    s=s+1

    #Uncomment to see new layout after each new station added
    """
    """"PLOTS""""
    #Create subplots
    fig, ax = plt.subplots()
    #Plot robot max. reach circle
    max_circle = plt.Circle((0,0), robot.max_radius, color='gold')
    ax.add_artist(max_circle)
    #Plot robot circle
    robot_circle = plt.Circle((0,0), robot.min_radius, color='gray')
    robot_border = plt.Circle((0,0), robot.min_radius, edgecolor='black')
    ax.add_artist(robot_border)
    ax.add_artist(robot_circle)
    #Plot stations
    for station in stationsarray:
      len_hei = generate_points(station.angle, station.x1, station.y1)
      machine = plt.Rectangle((station.x1, station.y1), len_hei[0], len_hei[1], fill=False)
      #Access point
      plt.plot(station.x, station.y, color='green', marker='o', markersize=4)
      ax.add_artist(machine)

    #Set axis limits
    ax.set_xlim(-2200, 2200)
    ax.set_ylim(-2200, 2200)
    #ax.axis('off')
    ax.set_aspect('equal')
    plt.show()
    for station in stationsarray:
      print("Station", stationsarray.index(station)+1,":")
      print("X:",station.x)
      print("Y:",station.y)
      print("X1:",station.x1)
      print("Y1:",station.y1)
      print("Angle:",station.angle*(180/math.pi),"degrees\n")
    """

    #removes last station if collision detected
    if collision_checker(stationsarray,s):
      #print("POP")
      stationsarray.pop()
      s=s-1

#Uses station.x, station.y, station.angle to generate station.x1 and station.y1
def generate_x1_y1(ang, x, y):
  x_1 = 0
  y_1 = 0
  srt=200
  lng=1500
  if ang == 0 or ang == math.pi/2:  #0deg or 90deg
    x_1 = x-srt/2
    y_1 = y-srt/2
  elif ang == math.pi:              #180deg
    x_1 = x+(srt/2)-lng
    y_1 = y-srt/2
  elif ang == 3*math.pi/2:          #270deg
    x_1 = x-srt/2
    y_1 = y+(srt/2)-lng

  return x_1, y_1

#Rotates a station by 90deg
def rotate_station(b,stn):

  collide = True
  ox1 = stn.x1
  oy1 = stn.y1
  oangle = stn.angle
  c=0

  while collide:
    if c==4:                                                      #breaks loop after 4th iteration because station cannot be rotated
      break
    stn.angle = stn.angle + math.pi/2

    if stn.angle == 2*math.pi:
      stn.angle = 0

    stn.x1, stn.y1 = generate_x1_y1(stn.angle, stn.x, stn.y)

    if collision_checker(b.stations,b.stations.index(stn)):
        #print("Collision Detected")
        stn.x1 = ox1
        stn.y1 = oy1
        stn.angle = oangle
        c=c+1
    else:
        collide = False
        break

  return stn


#Uses x1, y1, angle to identify length and height
def generate_points(a, x1, y1):
  length=0
  height=0
  if a == 0 or a == math.pi:
    length = 1500
    height = 200
  elif a == math.pi/2 or a == 3*math.pi/2:
    length = 200
    height = 1500

  return length, height

def move_station(b,station):
  move = 0
  j=0
  ox = station.x
  oy = station.y
  switch = True
  #loop will continue to run if collision detected
  while switch:

    #runs loop twice so both x and y change
    while j != 2:
      #checks if x coordinate has changed
      if ox == station.x:
        move = random.randint(-10,10)
        #ignores zero value until chosen
        if move != 0:
          station.x = station.x + move
          j=j+1

      #checks if y coordinate has changed
      if oy == station.y:
        move = random.randint(-10,10)
        #prevents zero movement
        if move != 0:
          station.y = station.y + move
          j=j+1

    #recassign x1 and y1 values
    station.x1, station.y1 = generate_x1_y1(station.angle, station.x, station.y)

    #if station collides, revert back to input x and y and move station again
    if collision_checker(b.stations,b.stations.index(station)):
      #print("Collision Detected")
      station.x = ox
      station.y = oy
    else:
      switch = False

  return station

def local_search(B,BLIST):
  LS = random.uniform(0,1)
  newstationlist = []
  #rotate
  if LS < 0.9:
    
    #rotates 1 random station in every bee
    i=random.randint(0,len(B.stations)-1) #index to choose stations
    newstation = rotate_station(B,B.stations[i])
    B.stations.pop(i)
    B.stations.append(newstation)
    for station in B.stations:
      newstationlist.append(station)


    #Uncomment to plot nwe station layout after rotate
    """
    #Create subplots
    fig, ax = plt.subplots()
    #Plot robot max. reach circle
    max_circle = plt.Circle((0,0), robot.max_radius, color='gold')
    ax.add_artist(max_circle)
    #Plot robot circle
    robot_circle = plt.Circle((0,0), robot.min_radius, color='gray')
    robot_border = plt.Circle((0,0), robot.min_radius, edgecolor='black')
    ax.add_artist(robot_border)
    ax.add_artist(robot_circle)
    #Plot stations
    for station in B.stations:
      len_hei = generate_points(station.angle, station.x1, station.y1)
      machine = plt.Rectangle((station.x1, station.y1), len_hei[0], len_hei[1], fill=False)
      #Robot access point
      plt.plot(station.x, station.y, color='green', marker='o', markersize=4)
      ax.add_artist(machine)

    #Set axis limits
    ax.set_xlim(-2200, 2200)
    ax.set_ylim(-2200, 2200)
    #ax.axis('off')
    ax.set_aspect('equal')
    plt.show()
    for station in B.stations:
      print("Station", B.stations.index(station)+1,":")
      print("X:",station.x)
      print("Y:",station.y)
      print("X1:",station.x1)
      print("Y1:",station.y1)
      print("Angle:",station.angle*(180/math.pi),"degrees\n")
      """
  #move
  elif LS >= 0.9:
    #moves each station by a random non-zero x and y value
    for station in B.stations:
      newstationlist.append(move_station(B,station))
    
    #Uncomment to plot nwe station layout after move
    """
    #Create subplots
    fig, ax = plt.subplots()
    #Plot robot max. reach circle
    max_circle = plt.Circle((0,0), robot.max_radius, color='gold')
    ax.add_artist(max_circle)
    #Plot robot circle
    robot_circle = plt.Circle((0,0), robot.min_radius, color='gray')
    robot_border = plt.Circle((0,0), robot.min_radius, edgecolor='black')
    ax.add_artist(robot_border)
    ax.add_artist(robot_circle)
    #Plot stations
    for station in B.stations:
      len_hei = generate_points(station.angle, station.x1, station.y1)
      machine = plt.Rectangle((station.x1, station.y1), len_hei[0], len_hei[1], fill=False)
      #Robot access point
      plt.plot(station.x, station.y, color='green', marker='o', markersize=4)
      ax.add_artist(machine)

    #Set axis limits
    ax.set_xlim(-2200, 2200)
    ax.set_ylim(-2200, 2200)
    #ax.axis('off')
    ax.set_aspect('equal')
    plt.show()
    for station in B.stations:
      print("Station", B.stations.index(station)+1,":")
      print("X:",station.x)
      print("Y:",station.y)
      print("X1:",station.x1)
      print("Y1:",station.y1)
      print("Angle:",station.angle*(180/math.pi),"degrees\n")
    """

  return newstationlist


#checks robot-to-station and station-to-station collisions
def collision_checker(stationlist,stationnum):
  #array index
  i=stationnum-1

  #Robot collision detection
  while i < len(stationlist):
    len_hei_1 = generate_points(stationlist[i].angle, stationlist[i].x1, stationlist[i].y1)
    if (stationlist[i].x1 < -robot.min_radius) or ((-robot.min_radius < stationlist[i].x1) and (stationlist[i].x1 < robot.min_radius)):
      if (stationlist[i].y1 < -robot.min_radius) or ((-robot.min_radius < stationlist[i].y1) and (stationlist[i].y1 < robot.min_radius)):
        if (robot.min_radius < stationlist[i].x1+len_hei_1[0]) or ((-robot.min_radius < stationlist[i].x1+len_hei_1[0]) and (stationlist[i].x1+len_hei_1[0] < robot.min_radius)):
          if (robot.min_radius < stationlist[i].y1 + len_hei_1[1]) or ((-robot.min_radius < stationlist[i].y1 + len_hei_1[1]) and (stationlist[i].y1 + len_hei_1[1] < robot.min_radius)):
            return True
    i=i+1

  #once 2 stations have been added, machine collision checker will start
  if i > 1:
    #index 1
    i=0
    while i < len(stationlist):
      #index 2
      j=0
      len_hei_1 = generate_points(stationlist[i].angle, stationlist[i].x1, stationlist[i].y1)
#
      while j < len(stationlist):
        len_hei_2 = generate_points(stationlist[j].angle, stationlist[j].x1, stationlist[j].y1)

        #prevents station being compared against itself
        if (stationlist[i] != stationlist[j]):

          #Mid-Left & Mid-Right, Top Right & Bottom Left
          if ((stationlist[i].x1 < stationlist[j].x1) and (stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0])):
            if (stationlist[j].x1+len_hei_2[0] > stationlist[i].x1 + len_hei_1[0]):
              if ((stationlist[i].y1 <= stationlist[j].y1) and (stationlist[j].y1 < stationlist[i].y1+len_hei_1[1])):
                if ((stationlist[i].y1+len_hei_1[1] < stationlist[j].y1 + len_hei_2[1]) or ((stationlist[i].y1 < stationlist[j].y1 + len_hei_2[1]) and (stationlist[j].y1 + len_hei_2[1] < stationlist[i].y1+len_hei_1[1]))):
                  return True

          #Bottom-Right & Top-Left
          if ((stationlist[i].x1 < stationlist[j].x1) and (stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0])):
            if (stationlist[j].x1+len_hei_2[0] > stationlist[i].x1 + len_hei_1[0]):
              if ((stationlist[j].y1 < stationlist[i].y1+len_hei_1[1])):
                if ((stationlist[i].y1 < stationlist[j].y1 + len_hei_2[1]) and (stationlist[j].y1 + len_hei_2[1] < stationlist[i].y1+len_hei_1[1])):
                  return True

          #Bottom & Top
          if ((stationlist[i].x1 <= stationlist[j].x1) and (stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0])):
            if (stationlist[j].x1 + len_hei_2[0] <= stationlist[i].x1 + len_hei_1[0]):
              if (stationlist[j].y1 <= stationlist[i].y1):
                if ((stationlist[i].y1 < stationlist[j].y1 + len_hei_2[1]) and (stationlist[j].y1 + len_hei_2[1] <= stationlist[i].y1+len_hei_1[1])):
                  return True

          #Perpendicular Crossed
          if ((stationlist[i].x1 <= stationlist[j].x1) and (stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0])):
            if ((stationlist[j].y1 <= stationlist[i].y1) and (stationlist[i].y1 < stationlist[j].y1 + len_hei_2[1])):
              if ((stationlist[i].x1 < stationlist[j].x1 + len_hei_2[0]) and (stationlist[j].x1 + len_hei_2[0] <= stationlist[i].x1 + len_hei_1[0])):
                if ((stationlist[j].y1 < stationlist[i].y1+len_hei_1[1]) and (stationlist[i].y1+len_hei_1[1] <= stationlist[j].y1 + len_hei_2[1])):
                  return True

          if ((stationlist[j].x1 < stationlist[i].x1) and (stationlist[i].x1 < stationlist[j].x1 + len_hei_2[0])):
            if ((stationlist[j].y1 < stationlist[i].y1) and (stationlist[i].y1 < stationlist[j].y1 + len_hei_2[1])):
              #print(i+1," ", j+1, " MACHINE COLLISION B")
              return True

          if ((stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0]) and (stationlist[i].x1+len_hei_1[0] < stationlist[j].x1 + len_hei_2[0])):
            if ((stationlist[j].y1 < stationlist[i].y1) and (stationlist[i].y1 < stationlist[j].y1 + len_hei_2[1])):
              #print(i+1," ", j+1, "MACHINE COLLISION C")
              return True

          if (stationlist[i].x1 < stationlist[j].x1) or (stationlist[i].x1 <= stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0]):
            if (stationlist[j].x1 < stationlist[i].x1 + len_hei_1[0] < stationlist[j].x1 + len_hei_2[0]):
              if (stationlist[j].y1 <= stationlist[i].y1) or (stationlist[i].y1 < stationlist[j].y1 < stationlist[i].y1 + len_hei_1[1]):
                if (stationlist[i].y1 + len_hei_1[1] <= stationlist[j].y1 + len_hei_2[1]): #or (stationlist[j].y1 + len_hei_2[1] < stationlist[i].y1 + len_hei_1[1])
                  #print(i+1," ", j+1, "MACHINE COLLISION D")
                  return True

        j=j+1
      i=i+1
  return False

#Plots each bee with a given list of bees
def beeplot(beelist):
  for bee in beelist:
    #PRINTS bee and cost
    print("Cost:", bee.cost,"\n")
    print("Fitness:", 1/bee.cost,"\n")
    #Create subplots
    fig, ax = plt.subplots()
    #Plot robot max. reach circle
    max_circle = plt.Circle((0,0), robot.max_radius, color='gold')
    ax.add_artist(max_circle)
    #Plot robot circle
    robot_circle = plt.Circle((0,0), robot.min_radius, color='gray')
    robot_border = plt.Circle((0,0), robot.min_radius, edgecolor='black')
    ax.add_artist(robot_border)
    ax.add_artist(robot_circle)
    #Plot stations
    for station in bee.stations:
      len_hei = generate_points(station.angle, station.x1, station.y1)
      machine = plt.Rectangle((station.x1, station.y1), len_hei[0], len_hei[1], fill=False)
      #Robot access point
      plt.plot(station.x, station.y, color='green', marker='o', markersize=4)
      #Each vertex point
      """
      plt.plot(station.x1, station.y1, color='blue', marker='o', markersize=3)
      plt.plot(station.x1, station.y1+len_hei[1], color='blue', marker='o', markersize=3)
      plt.plot(station.x1+len_hei[0], station.y1, color='blue', marker='o', markersize=3)
      plt.plot(station.x1+len_hei[0], station.y1+len_hei[1], color='blue', marker='o', markersize=3)
      """
      ax.add_artist(machine)

    #Set axis limits
    ax.set_xlim(-2200, 2200)
    ax.set_ylim(-2200, 2200)
    #ax.axis('off')
    ax.set_aspect('equal')
    plt.title(f'Best Robot Layout')
    plt.show()
    
    #Prints all station value for each bee
    for station in bee.stations:
      print("Station", bee.stations.index(station)+1,":")
      print("X:",station.x)
      print("Y:",station.y)
      print("X1:",station.x1)
      print("Y1:",station.y1)
      print("Angle:",station.angle*(180/math.pi),"degrees\n")

#Uncomment to export:
#best bee cost saved at each iteration
#bee cost calculated at each iteration
#No. of scout bees achieving convergence in each current group
"""
folderpath = " INSERT DESIRED FOLDER PATH HERE "
filename=' INSERT DESIRED FILE NAME HERE '
datafilepath = os.path.join(folderpath,filename)
def csvwrite(datafilepath, data):

  df = pd.DataFrame(data)

  with open(datafilepath, 'w') as file:
    file.write('TITLE = "INSERT DESIRED TITLE HERE"\n')
    file.write('VARIABLES = "Iteration"\n,"Best Bee Cost"\n,"Current Bee Cost"\n, "No. of Scout Bees Achieving Convergence in Current Group"\n')
    df.to_csv(file, index=False, header=False, sep=",")
"""


"""""""""""""""""""""
    BEES PROGRAM
"""""""""""""""""""""
stations=[]
bees=[]
data=[]
best_bee=[]
n=0
it=0
newbeecost=0

#Random station initialisation of each bee
while n < Scout_Bees:
  global_search()
  newbeecost = calc_cost(stationsarray)
  bees.append(Bee(newbeecost,stationsarray)) #Bee created using cost and stations
  stationsarray=[]
  n=n+1

#sorts bees according to cost attribute
bees.sort(key=lambda x: x.cost, reverse=False)

#counts number of times best cost value has been reached
bestbeecount=0
while it < maxIt:
  # Selected sites, m
  m = 0
  selected_sites = []
  while m<SelectedSites:                                                #for each site, find fittest bee from list 'bees'
    selected_sites.append(Bee(bees[m].cost, bees[m].stations))          #add bee with best cost value to a list called selected_bees
    m=m+1

  # Elite sites, e
  e = 0
  elite_sites = []
  while e<EliteSites:
    elite_sites.append(Bee(selected_sites[e].cost, selected_sites[e].stations))  
    e=e+1
  
  #create a list of elitebees
  elite_bees = []
  es=0
  while es<EliteSites:
    eb=0
    while eb<EliteSite_Bees:
      elite_bees.append(Bee(elite_sites[es].cost, elite_sites[es].stations))
      eb=eb+1
    es=es+1

  #create a list of selectedsitebees (non-elite)
  selectedsite_bees = []
  ss=0
  while ss<(SelectedSites):
    sb=0
    while sb<SelectedSite_Bees:
      selectedsite_bees.append(Bee(selected_sites[ss].cost, selected_sites[ss].stations))
      sb=sb+1
    ss=ss+1

  bees = []                                                             #reinitialise bees[]
  newstations = []                                                      #array to store each bee's new arrangement of stations after local search
  oldstations = []                                                      #array to store each bee's original arrangement of stations before local search
  
  #Elite Site local search
  for bee in elite_bees:
    #if conducting test with no elite bees, loop breaks
    if elite_bees == []:
      break
    oldbeecost=0
    oldstations=bee.stations
    oldbeecost=bee.cost
    newstations = local_search(bee,elite_bees)                               #localsearch returns list of stations
    newbeecost = calc_cost(newstations)
    if newbeecost > oldbeecost:
      bees.append(Bee(newbeecost,newstations))                            #new bee created using new cost and new list of stations
    else:
      bees.append(Bee(oldbeecost,oldstations))
    newstations = []

  #Selected Site local search
  for bee in selectedsite_bees:
    oldbeecost=0
    oldstations=bee.stations
    oldbeecost=bee.cost
    newstations = local_search(bee,selectedsite_bees)
    newbeecost = calc_cost(bee.stations)
    if newbeecost > oldbeecost:
      bees.append(Bee(newbeecost,newstations))                            #new bee created using new cost and new list of stations
    else:
      bees.append(Bee(oldbeecost,oldstations))
    newstations = []
  
  #global search
  gs=0
  stationsarray=[]
  while gs < GlobalScout_Bees:
    global_search()
    newbeecost = calc_cost(stationsarray)
    bees.append(Bee(newbeecost,stationsarray))
    stationsarray=[]
    gs=gs+1

  print("ITERATION NO. ",it+1)

  #sorts bees by cost attribute
  bees.sort(key=lambda x: x.cost, reverse=False)
  
  #Stores best bee found in entire run here
  if best_bee==[]:
    bestbeestations=[]
    #Deep copy of attribute values to form new objects, independent of the original
    for station in bees[0].stations:
      bestbeestations.append(Station(station.x,station.y,station.x1,station.y1,station.angle))
    best_bee.append(Bee(calc_cost(bestbeestations),bestbeestations))
    
    iterationfound=it+1   #store the iteration that it was found
    bestbeecount=1        #count how many times solution was found

  elif best_bee[0].cost > bees[0].cost:   #if best bee is worse than new bee, replace it
    iterationfound=it+1
    best_bee.pop()
    bestbeestations=[]
    for station in bees[0].stations:
      bestbeestations.append(Station(station.x,station.y,station.x1,station.y1,station.angle))
    best_bee.append(Bee(calc_cost(bestbeestations),bestbeestations))
    print("\nNEW Best Bee Found with Cost: ", best_bee[0].cost)
    bestbeecount=1
  
  elif best_bee[0].cost == bees[0].cost:  #if optimum bee was found again +1 to counter
    #print("mark9")
    bestbeecount=bestbeecount+1
  
  #Count number of bees that achieved lowest cost within the same generation
  numbees=0
  for bee in bees:
    if bee.cost==bees[0].cost:
      numbees=numbees+1
  
  actit=it+1

  #Stores variables to append in array
  """
  datatoappend = [actit, best_bee[0].cost, bees[0].cost, numbees]
  data.append(datatoappend)
  """
  
  it=it+1

#Uncomment to call  CSV write function
"""
csvwrite(datafilepath,data)
"""

runtime=(time.time() - start)

#Runtime stop
print("Runtime: ",runtime)
#Iteration at which best solution was found
print("Best Bee found on iteration: ", iterationfound)
#How many times the best solution was found
print("Best Bee found: ",bestbeecount, "times")

beeplot(best_bee)