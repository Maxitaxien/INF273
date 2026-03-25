# 0,7,4,8,1,9,6,10,3|5,2,-1|2,6,-1|4,7,-1
# 0,5,7,8,4,9,1,2,6|10,-1,3|6,-1,2|8,-1,4
from pathlib import Path
from data_handler import load_data
'''
solution = {"part1": [0, 5, 7, 8, 4, 9, 1, 2, 6],
            "part2": [10, -1, 3],
            "part3": [6, -1, 2],
            "part4": [8, -1, 4]}

'''

DATA_PATH = Path("data/test_instances/F_100.txt")
print(DATA_PATH)
solution = {"part1": [0,72,2,80,66,31,55,60,9,16,73,54,40,67,21,59,63,29,15,11,94,79,65,38,93,43,87,68,85,58,92,95,56,88,47,61,24,27,64,20,6,41,98,77,50,13,22,1,23,34,69,25,100,71,81,0],
            "part2": [70,49,83,42,75,28,33,37,91,86,32,89,19,82,44,76,3,26,62,5,48,99,4,14,18,-1,78,17,7,52,97,51,57,53,74,46,36,84,45,35,30,10,90,96,8,39,12], 
            "part3": [4,5,7,9,11,14,15,16,17,18,19,20,22,25,29,33,34,35,36,39,41,43,45,48,52,-1,3,4,5,7,9,12,17,18,20,23,27,33,38,41,43,45,47,49,50,51,52],
            "part4": [5,7,8,10,12,15,16,17,18,19,20,22,24,27,30,34,35,36,37,41,43,44,47,49,54,-1,4,5,7,8,10,13,18,20,21,27,28,34,40,43,44,47,48,50,51,52,53]
}


n, lim, truck_times, drone_times = load_data(DATA_PATH)

def calculate_total_waiting_time(solution):
        """
        Iteratively compute total arrival time (objective) for STRPD with full truck–drone synchronization.
        Each drone has its own availability timeline — it cannot start a new mission before:
        - the truck arrives at the launch node, AND
        - the drone is available from its previous return.
        """
        
        feas = True

        truck_route = solution["part1"]
        part2 = solution["part2"]
        part3 = solution["part3"]
        part4 = solution["part4"]

        # Clean separators (-1) and convert to ints
        part3_clean = [int(x) for x in part3 if x != -1]
        part4_clean = [int(x) for x in part4 if x != -1]

        # Split UAV customers by drone (using -1)
        drone_routes = []
        current = []
        for x in part2:
            if x == -1:
                drone_routes.append(current)
                current = []
            else:
                current.append(x)
        drone_routes.append(current)

        # Map each drone's flights: (cust, launch_idx, return_idx)
        drone_flights = []
        start_idx = 0
        for drone_customers in drone_routes:
            flights = []
            for c in drone_customers:
                launch_idx = part3_clean[start_idx] - 1  # 1-based → 0-based
                return_idx = part4_clean[start_idx] - 1
                flights.append((c, launch_idx, return_idx))
                start_idx += 1
            drone_flights.append(flights)

        # Initialize truck timeline
        depot_index = 0
        t_arrival = {depot_index: 0}
        t_departure = {depot_index: 0}
        total_time = 0
        drone_availability = [0] * len(drone_flights)
        
        for i in range(1, len(truck_route)):
            prev_node = truck_route[i - 1]
            curr_node = truck_route[i]
            
           
            truck_travel = truck_times[prev_node][curr_node]
            truck_arrival = t_departure[prev_node] + truck_travel
            t_arrival[curr_node] = truck_arrival
            #print("\nTruck departs from",prev_node,"at time",t_departure[prev_node],"at",prev_node,
            #      "and travels for a time of",truck_travel,"to arrive",curr_node,"at time",t_arrival[curr_node])
            
            #print("Delay in next departure due to waiting for drone landings at",curr_node,":")   

            # Check returning drones
            drone_returns = []
            for u, flights in enumerate(drone_flights):
                for (cust, launch_idx, return_idx) in flights:
                    if return_idx == i:
                        launch_node = truck_route[launch_idx]
                        return_node = truck_route[return_idx]
                        flight_out = drone_times[launch_node][cust]
                        flight_back = drone_times[cust][return_node]
                        total_flight = flight_out + flight_back
                        #print("Flight of drone",u,":",launch_node,"->",cust,"(travel time=",self.drone_times[launch_node][cust],")",
                        #      "->",return_node,"(travel time=",self.drone_times[cust][return_node],")")    
                        #print("Drone arrival time at launch node",launch_node,":",drone_availability[u],
                        #      "Truck arrival time at launch node",launch_node,":",t_arrival[launch_node])
                        
                        # Drone cannot depart before both truck and its own availability
                        possible_launch_time = t_arrival[launch_node]
                        actual_launch_time = max(possible_launch_time, drone_availability[u])
                                                     
                        drone_arrival_customer = actual_launch_time + flight_out
                        drone_return_time = actual_launch_time + total_flight
                        drone_availability[u] = drone_return_time
                        drone_returns.append(drone_return_time)
                        total_time += drone_arrival_customer
                        
                        drone_wait = max(t_arrival[curr_node]-drone_return_time,0) if curr_node != 0 else 0
                        
                        #print("Drone return time = ",drone_return_time," Truck arrivalt time = ",t_arrival[curr_node])
                        total_flight_ =  total_flight + drone_wait
                        
                        #print("Flight start time at",launch_node,":",actual_launch_time,"Drone arrival time at customer",cust,":",drone_arrival_customer,
                        #      "Drone return time at",return_node,":",drone_return_time,"flight time = ",total_flight_)
                        
                        #"""
                        if total_flight_ > lim:
                           feas = False
                           return total_time, t_arrival, t_departure, feas
                        #"""   

             #Truck waits for the latest returning drone
            if drone_returns:
                latest_drone = max(drone_returns)
                t_departure[curr_node] = max(truck_arrival, latest_drone)
            else:
                t_departure[curr_node] = truck_arrival
                
            #print("Next departure time for truck at",curr_node,"after waiting to receive all drones:",t_departure[curr_node]) 

            if curr_node != depot_index:
                total_time += truck_arrival

        # Final adjustment: convert from seconds to minutes (or 100-unit scale)
        total_time /= 100.0

        """
        print("\n=== FINAL TIMINGS ===")
        for k in t_arrival:
            print(f"Truck node {k}: arrival={t_arrival[k]:.1f}, depart={t_departure[k]:.1f}")
        print("=====================\n")
        #"""

        return total_time, t_arrival, t_departure, feas

t, a, d, f = calculate_total_waiting_time(solution)
print(t, f)