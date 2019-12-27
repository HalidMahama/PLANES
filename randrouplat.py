import os
import sys
import ccparams as cc
import random
import time
import planers

from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running, validate_params, retrieve_vehicles, \
    filter_cacc_vehicles, get_dist_to_POI

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci

# Length of vehicles
LENGTH = 4
# inter-vehicluar gap
DISTANCE = 5
# cruise speed
SPEED = 35
PSPEED = 48

# PLANE STATES
# If the plane state is 0: there are no active planes to control
# If the plane state is 1: platoons are formed and lane changes are forbiden
# If the plane state is 2: vehicles are acc controlled and are able to change lanes
NON = 0
PLATOONING = 1
NONPLATOONING = 2

# VEHICLE STATES
# If the vehicle state is 0: It is platooning and only controlled by plane logic
# If the vehicle state is 1: It is in a nonplatooning lane, acc controlled and can change lanes
# If the vehicle state is 2: It is a free agent, Driver controlled and obeys no plane logic
IDLE = 0
MANEUVERING = 1
FREE_AGENT = 2

N_VEHICLES = 24
N_VEHICLES_GEN = 24
SOURCES = ["p0", "s0"]
pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
PLAT_EDGES =  ["p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19"]
ARR_EDGES = ["e0", "exit0", "e1", "exit1", "e2", "exit2", "e3", "exit3"]
ADD_PLAT_STEP = 700


# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "D", "-c", "cfg/freeway.sumo.cfg"]


def add_vehicles(n, batch_num, platoon_len, fromEdge, real_engine):
    # route param: for each source edge there are four possible routes
    start_from = n*batch_num
    end_at = start_from + platoon_len
    
    index = fromEdge.split("e")[1]
    if index == "0":
        exitEdges =['exit0', 'exit1', 'exit2']
    elif index=="1":
        exitEdges =['exit1', 'exit2', 'exit3']
    elif index == "2":
        exitEdges =['exit2', 'exit3', 'exit1']
    else: # index = "3"
        exitEdges =['exit3', 'exit1', 'exit2']

    for i in range(start_from, end_at):
        lane = 1
        vid = "v.%d" % i
        toEdge= exitEdges[0]
        route = "route_"+index+"_"+str(lane-1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

    start_from = start_from + platoon_len
    end_at = start_from + platoon_len

    for i in range(start_from, end_at):
        lane = 2
        vid = "v.%d" % i
        toEdge= exitEdges[1]
        route = "route_"+index+"_"+str(lane-1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

    start_from = start_from + platoon_len
    end_at = start_from + platoon_len

    for i in range(start_from, end_at):
        lane = 3
        vid = "v.%d" % i
        toEdge= exitEdges[2]
        route = route = "route_"+index+"_"+str(lane-1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)



def lane_gen(lanes_per_edge=4): # true num of edges 19, manipulating

    edges = ["source0", "s0", "source1", "s1", "source2", "s2" , "source3", "s3", "p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19" ]
    lanes =[]
    for edge in edges:
        for lane in range(1,lanes_per_edge):
            laneID = edge + "_" + str(lane)
            lanes.append(laneID)
    return lanes


def route_assigner(SOURCES):
    routes = ["route0", "route1"]
    for edge in SOURCES:
        vehicles = traci.edge.getLastStepVehicleIDs(edge)
        #print("The vehs at source are : {}".format(vehicles))
        for vehicle in vehicles:
            route = routes[random.randint(0, 1)]
            #print("veh {} route is {}".format(vehicle, route))
            traci.vehicle.setRouteID(vehicle, route)

def proute_assigner(SOURCES):
    routes = ["route0", "route1", "route2", "route3"]
    for edge in SOURCES:
        vehicles = traci.edge.getLastStepVehicleIDs(edge)
        #print("The vehs at source are : {}".format(vehicles))
        for vehicle in vehicles:
            route = routes[int(traci.vehicle.getLaneID(vehicle).split("_")[1]) - 1]
            traci.vehicle.setRouteID(vehicle, route)
            #print(" The best lanes for {} is {} ".format(vehicle, route))

def sorted_planes(lane_vehicles, lane):
    planes=[]
    #routes =["route0", "route1", "route2", "route3"]
    primary_plane = []
    secondary_plane = []
    leader_route = traci.vehicle.getRouteID(lane_vehicles[0])
   
    for vehicle in lane_vehicles:
        if traci.vehicle.getRouteID(vehicle) == leader_route and get_distance(vehicle, lane_vehicles[0]) < (240 + 200): # 200 + length of plat
            primary_plane.append(vehicle)
        else:
            
            secondary_plane.append(vehicle)

    ps_planes = [primary_plane, secondary_plane]
    all_planes = []

    for item in ps_planes:
        #print("ps item {}".format(item))
        item_planes_with_empties = [(item[N_VEHICLES_GEN*i: N_VEHICLES_GEN*i + N_VEHICLES_GEN]) for i in range(N_VEHICLES_GEN)]
        item_planes = [plane for plane in item_planes_with_empties if plane != []]
        #print("item_plane:{}".format(item_planes))

        for plane in item_planes:
            #print("plane:{}".format(plane))
            planes.append(planers.Plane(
                            lane, plane , lane_spacing=4, lane_speed=SPEED, platoonable=False))
    return planes

def set_arrived_free(ARR_EDGES):
    for edge in ARR_EDGES:
        vehicles = traci.edge.getLastStepVehicleIDs(edge)
        for vehicle in vehicles:
            set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
            set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, SPEED + 25)
            set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 1)



def main(real_engine, setter=None, demo_mode= False):

    start_sumo("cfg/freeway.sumo.cfg", False)
    step = 0
    batch_num = 0
    veh_of_interest = "v.40"
    source_edges = ['source0', 'source1', 'source2', 'source3']
    edge_filter, vtype_filter = validate_params(
        edge_filter=PLAT_EDGES, vtype_filter=["vtypeauto"])
    pstate = NON

    while running(demo_mode, step, 3800): ##Use multiples 700 to reflect last vehicle of farthest traveling platoon arrival time of each batch

        if demo_mode and step == 3800: 
            start_sumo("cfg/freeway.sumo.cfg", False)
            step = 0



        #print(traci.simulation.findRoute('source0', 'exit1')[3])

        if pstate == NON:
            lanes = lane_gen()
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[0], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3 ### Start from here
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[1], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[2], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[3], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3
            traci.gui.setZoom("View #0", 4500)
            #traci.gui.setZoom("View #1", 4500)
            topology = {}
            vstate = IDLE
            pstate = PLATOONING
            genStep = step
            print("Gen Step is : {}".format(genStep))
            print("pstate at gen is : {}".format(pstate))
        if pstate == PLATOONING and step == genStep + 1:
            veh_of_interest = traci.lane.getLastStepVehicleIDs('source0_3')[::-1][0]
            print("veh of interest is: {}".format(veh_of_interest))


        if pstate == PLATOONING:
            traci.simulationStep()

        if step > genStep + 10 and pstate == PLATOONING:
            print("veh of int is:{}".format(veh_of_interest))
            if veh_of_interest in traci.edge.getLastStepVehicleIDs("p12"):#traci.vehicle.getLaneID(veh_of_interest).split('_')[1] == "0":
                print("veh of interest at set location {}!!!!!!!!!!!".format(traci.vehicle.getLaneID(veh_of_interest)))
                pstate = NON
             
            for lane in lanes:

                if traci.lane.getLastStepVehicleIDs(lane)==[]:
                    continue
                
                #print("Checking lane {} for vehicles!!!".format(lane))
                lane_vehicles = traci.lane.getLastStepVehicleIDs(lane)[::-1]
                #print("Lane vehicles are : {}".format(lane_vehicles))
                planes = sorted_planes(lane_vehicles, lane)
                #pstate = PLATOONING
                #print("sorted planes are {}".format(planes))
                #poi_index = "0"
                #topologies =[]
                for plane in planes:
                    topology = plane.topo_contsructor()
                    #topologies.append(topology)
                    #print("Topology at topoConstructor is : {}".format(topology))
                    topology = plane.pla_speed_spacing(topology)
                    #print("Topology at platSpeedSpacing is : {}".format(topology))

                    communicate(topology)
                    plane.set_arrived_free() # Consider putting in 
                    if plane.near_flag():
                        flag_n_poi_index = plane.look_for_flags(pois, step)
                        if flag_n_poi_index[0] == True :

                            plane.move_to_next_best_lane(step, flag_n_poi_index)
                        #topology = plane.pla_speed_spacing(topology)
                        #for topology in topologies:
                          

                traci.simulationStep()
                          
   
        #set_arrived_free(ARR_EDGES)
        print("step is : {}".format(step))
        print("Current time is :{}".format(traci.simulation.getCurrentTime()))
        #print("vstate is : {}".format(vstate))
        print("pstate is : {}".format(pstate))

        step += 1
        
            



    traci.close()

if __name__ == "__main__":
    main(True, True)
