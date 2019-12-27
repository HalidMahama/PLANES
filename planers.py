import os
import sys
import ccparams as cc
import utils
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci



class Plane:
    """
    Planes for platoonable lanes, this class manages the lanes of the edges in a highway, every lane in the highway
    formats vehicles into conforming with its speed and spacing requirements thereby forcing the vehicles to move 
    platoons.
    """
    #Lane length
    DISTANCE = 5
    VLENGTH = 4
    N_VEHICLES = 24
    SPEED = 35
    PSPEED = 48

    NON = 0
    SWITCHED = 1
    PASSING_FLAG = 2
    FLAG_PASSED = 3
    
    pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
    time_flags_found = [[0],[0],[0],[0]]



    def __init__(self,laneID, vehicles, lane_spacing, lane_speed, platoonable = False):
        """
        There are four lanes per edge in the setup for the simulation, each plane is a lane on a particular edge 
        of the highway. It has an ID, fixed length, the vehicles that are on it at each timestep, the max number of vehicle in a platoons, its allowed speed 
        and spacing requirements and whether or not it's platoonable
        
        laneID : the ID of the lane
        lane_length : the length of the lane
        platnum: the max number of vehicles in each platoonable
        lane_spacing: the min spacing requirement for the lane
        lane_speed: the travel speed of the lane
        platoonable: whether or not the lane allows platooning of vehicles

        """
        self.states = [self.NON]
        self._ID = laneID
        self._members = vehicles # all vehicles of the plane at each timestep
        self._father_vehicle = vehicles[0] # the leader of the first platoon
        self._children_vehicles = [vehicle for vehicle in self._members if self._members.index(vehicle) % 24 == 0 and self._members.index(vehicle) != 0] # leaders of subsequent vehicles
        self._grandchildren_vehicles =  [vehicle for vehicle in self._members if self._members.index(vehicle) % 24 != 0 and self._members.index(vehicle) != 0]# followers of platoon leaders


    def plane_members(self):
        members = self._members

        return members

    def plane_leader(self):
        leader = self._father_vehicle
        return leader

    def plane_subleaders(self):
        subleaders = self._children_vehicles
        return subleaders

    def plane_followers(self):
        followers = self._grandchildren_vehicles
        return followers

    def veh_pos_pairs(self, lane):

        lanemembership = {self._ID: self.plane_followers()}
        locdic = {}
        sortedlocdict={}
        i=0
        for vehicle in self._members:
            #if vehicle != self.plane_members()[0]:
            loc = traci.vehicle.getLanePosition(vehicle)
            locdic.update({vehicle:loc})
            sortedlocdict = sorted(locdic.items(), key=lambda kv: kv[1])
        #print("sortedlocdic is of {} is: {}".format(lane, sortedlocdict))
                

        return sortedlocdict

    def topo_contsructor(self):
        sortd = self.veh_pos_pairs(self._ID)
        topology = {}
        for item in sortd:
            current_veh = item[0]
            if current_veh == self._father_vehicle:
                print("father vehicle of veh {} is {}".format(current_veh, self._father_vehicle)) 
                topology.update({current_veh: {"front": current_veh, "leader": self._father_vehicle}})
            else:
                lane_vehicles = traci.lane.getLastStepVehicleIDs(traci.vehicle.getLaneID(current_veh))[::-1]# change to vehicles
                index = lane_vehicles.index(current_veh)-1
                preceeding_veh = lane_vehicles[index]#"v.%d" % (int(current_veh.split(".")[1]) - 1) 
                ## PRIMARY & SECONDARY FORMATION CONTROL
                ## To use secondary Platoon Formation uncomment this section of code and comment the last section
                """if self._members.index(current_veh)<= self.N_VEHICLES/2: # Will always evaluate to true if plat vehs == num vehs per lane
                    #print("Current Veh is : {}".format(current_veh))
                    #print("Preceeding Veh is {}".format(preceeding_veh))
                    #print(utils.get_distance(current_veh, preceeding_veh))
                    topology.update({current_veh: {"front": preceeding_veh, "leader": self._father_vehicle}})

                else: #Topology for seconadry platoon goes here
                    print("father veh and Child Vehs are:{} and {}".format(self._father_vehicle, self._children_vehicles))
                    if current_veh in self._children_vehicles:
                        second_leader = current_veh
                        topology.update({current_veh: {"front": preceeding_veh, "leader": second_leader}})
                    else: 

                        second_leader = [vehicle for vehicle in self._children_vehicles if traci.vehicle.getRouteID(vehicle) == traci.vehicle.getRouteID(current_veh)][0]
                        topology.update({current_veh: {"front": preceeding_veh, "leader": second_leader}})
        print("Topocon {}".format(topology))"""
        ### PRIMARY ONLY 
        ## To use only primary platoon formation Uncomment this section and comment the code secondary part
                if utils.get_distance(current_veh, preceeding_veh) < 100: # Will always evaluate to true if plat vehs == num vehs per lane
                    #print("Current Veh is : {}".format(current_veh))
                    #print("Preceeding Veh is {}".format(preceeding_veh))
                    #print(utils.get_distance(current_veh, preceeding_veh))
                    topology.update({current_veh: {"front": preceeding_veh, "leader": self._father_vehicle}})

        return topology

    def pla_speed_spacing(self, topology):
        '''This funnction is designed to control primary, same route secondary and different route 
        secondary platoons, if '''
        vehicles = self._members
        lane = self._ID
        #print("vehs at speedspace: {}".format(vehicles))
        first_of_lane = traci.lane.getLastStepVehicleIDs(lane)[::-1][0]
        if vehicles[0] == first_of_lane: 
            for vehicle in vehicles:
                lane_num = traci.vehicle.getLaneID(vehicle).split("_")[1]
                # Vehicles controlled here are lane leaders
                if vehicle == vehicles[0]: 
                    #print("veh being speed adjusted is {}".format(vehicle))
                    if lane_num == "0":
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 1)

                    if lane_num == "1":
                        #print("veh being speed adjusted is {}".format(vehicle))
                        #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED + 10))
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)

                        
                    if lane_num == "2":
                        #print("veh being speed adjusted is {}".format(vehicle))
                        #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED + 15))
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)
                        
                    if lane_num == "3":
                        #print("veh being speed adjusted is {}".format(vehicle))
                        #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED +25))
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 10)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)
                elif vehicle in self._children_vehicles:
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, 10)
                else:
                    # Vehicles controlled here are lane followers of the first platoon in the lane
                    #print("veh being speed adjusted is {}".format(vehicle))
                    #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED +25)) 
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                    utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
            topology = topology
            #print("topo in speed_spacing prim :{}".format(topology))
            return topology

        ### Secondary Platoon Control

        if vehicles[0] != traci.lane.getLastStepVehicleIDs(lane)[::-1][0]: 
            index = traci.lane.getLastStepVehicleIDs(lane)[::-1].index(vehicles[0])
            last_veh_plat_ahead = traci.lane.getLastStepVehicleIDs(lane)[::-1][index -1]
            for vehicle in vehicles:
                #print("veh being speed adjusted is {}".format(vehicle))
                if vehicle == vehicles[0]: # 
                    # Vehicles controlled here are secondary platoons with same route as lane leader
                    if traci.vehicle.getRouteID(vehicle) == traci.vehicle.getRouteID(last_veh_plat_ahead) and utils.get_distance(vehicle, last_veh_plat_ahead) < 100: 
                        #topology[vehicle] = {"front": last_veh_plat_ahead, "leader": traci.lane.getLastStepVehicleIDs(lane)[::-1][0] }
                        #print("Sec_leader to:{}".format(topology[vehicle]))
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE*2)

                    else:
                        # Vehicles controlled here are temporary secondary platoon leaders in same lane but diff routes as lane leader
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                        lane_num = traci.vehicle.getLaneID(vehicle).split("_")[1]
                        if lane_num == "0":
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                            utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                            utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)

                        if lane_num == "1":
                            #print("veh being speed adjusted is {}".format(vehicle))
                            #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED + 10))
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED )
                        
                        if lane_num == "2":
                            #print("veh being speed adjusted is {}".format(vehicle))
                            #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED + 15))
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                            
                        if lane_num == "3":
                            #print("setting vehicle {} speed to {}".format(vehicle, self.PSPEED + 25))
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 10)
                else:
                    # All secondary followers control
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC) 
                    utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
            #print("topo at speed_spacing sec is {}".format(topology))
            return topology

    def near_flag(self):
        vehicles = self._members
        leader = vehicles[0]
        flag_edges = ["n3", "n8", "n13", "n18"]
        if traci.vehicle.getRoadID(leader) in flag_edges:
            return True



    def look_for_flags(self, pois, step):
        vehicles = self._members
        leader = vehicles[0]
        #print("leader at flag: {}".format(leader))
        flag_found = False
        poi_index = "0"
        vehicle_data = utils.get_par(self._members[0], cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data)
        time_to_pass = (((self.DISTANCE + self.VLENGTH)* self.N_VEHICLES)/v)
        print("Passing timer {}".format(time_to_pass))
        for poi in pois:
            if utils.get_dist_to_POI(leader, poi) < 70:
                #self.states.append(self.SWITCHING)
                print("distance to poi is {}".format(utils.get_dist_to_POI(leader, poi)))
                poi_index= int(poi.split("_")[2])
                flag_found = True
                self.time_flags_found[poi_index].append(traci.simulation.getCurrentTime()/1000)
                print("flag found at {}:".format(self.time_flags_found[poi_index][-1]))

                if self.time_flags_found[poi_index][-1] - self.time_flags_found[poi_index][-2] < time_to_pass:
                    #if self.states[-1] in [self.SWITCHED, self.PASSING_FLAG] and utils.get_dist_to_POI(leader, poi) > (self.DISTANCE+self.VLENGTH)*self.N_VEHICLES:
                    self.states.append(self.FLAG_PASSED)
                    #print("Flag Passed")
                    poi_index= int(poi.split("_")[2])
                    flag_found = False

        flag_n_poi_index = [flag_found, poi_index]
        return flag_n_poi_index


    def move_to_next_best_lane(self, step, flag_n_poi):
        time_flags_found = self.time_flags_found
        flag_found = flag_n_poi[0]
        poi_index=flag_n_poi[1]
        delay = 30
        next_xnum_steps = [time_flags_found[poi_index][-1] + index for index in range(1, delay) ]
        #print("Next ten steps are {}".format(next_ten_steps))
        prev_xnum_steps = [time_flags_found[poi_index][-1] - index for index in range(1, delay) ]
        #print("Past ten steps are {}".format(prev_ten_steps)) ##  self.states[-1] not in [self.SWITCHED, self.PASSING_FLAG] and not in next_ten_steps and step not in prev_ten_steps
        vehicle_data = utils.get_par(self._members[0], cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data) 
        time_to_pass = (((self.DISTANCE + self.VLENGTH)* self.N_VEHICLES)/v)
        print("Passing time is {}".format(time_to_pass))
        ## Move to next best lane only if it's first of this flag and don't move within passing time
        if traci.simulation.getCurrentTime() > time_flags_found[poi_index][-2] + time_to_pass and flag_found == True:#+ time_to_pass and flag_found == True:#time_flags_found[poi_index][-2] + 1 and flag_found == True:#and step >  time_flags_found[poi_index][-2]+ time_to_pass:
        
            #print("state is {} Moving to next best lane".format(self.states[-1]))
            vehicles = self._members
            print("vehs at movetonext: {}".format(vehicles))
            pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
            current_lane_num = traci.vehicle.getLaneIndex(vehicles[0])
            next_best_lane = current_lane_num - 1
            #print(next_best_lane)
            
            for vehicle in vehicles:

                utils.change_lane(vehicle, next_best_lane)

            self.states.append(self.SWITCHED)
            #print(self.states)

        else:
            self.states.append(self.PASSING_FLAG)
            #print("Passing flag state is {}".format(self.states))


    def set_arrived_free(self):
        vehicles = self._members
        leader = vehicles[0]

        if traci.vehicle.getLaneID(leader).split("_")[1] == "0":
            for vehicle in vehicles:
                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                #utils.set_par(vehicle, cc.PAR_FIXED_ACCELERATION, 1)
            #print("Vehicles set free : {}".format(vehicles))