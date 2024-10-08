import json
from fastapi import FastAPI, HTTPException
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from app.utils.matrix import create_matrix
from app.utils.data_reader import create_data, create_data_model, minutes_to_time

def solve_vrp():
    """Solves the CVRP and returns a structured solution."""
    data = create_data_model()
    #Create node index to variable index mapping
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), len(data['vehicle_capacities']), data['depot']) 
    routing = pywrapcp.RoutingModel(manager)

    # Time callback
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node] + data['service_time'][
            from_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)

    # Time dimension
    time = 'Time'
    routing.AddDimension(
        time_callback_index,
        5,  # allow waiting time
        1900,  # maximum time
        False,
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    time_dimension.SetGlobalSpanCostCoefficient(10)

    # Breaks
    # [START break_constraint]
    # warning: Need a pre-travel array using the solver's index order.

    node_visit_transit = [0] * routing.Size()
    for index in range(routing.Size()):
        node = manager.IndexToNode(index)
        node_visit_transit[index] = data['service_time'][node]

    break_intervals = {}
    for v in range(manager.GetNumberOfVehicles()):
        break_intervals[v] = [
        routing.solver().FixedDurationIntervalVar(
            240,  # start min
            300,  # start max
            0,  # duration: 10 min
            True,  # optional: yes
            f'Break for vehicle {v}')
        ]

        time_dimension.SetBreakIntervalsOfVehicle(
            break_intervals[v],  # breaks
            v,  # vehicle index
            node_visit_transit)
    # [END break_constraint]

    # Add time window constraints
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    depot_idx = data['depot']
    for vehicle_id in range(len(data['vehicle_capacities'])):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][depot_idx][0],
            data['time_windows'][depot_idx][1])

    for i in range(len(data['vehicle_capacities'])):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Capacity constraint
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        data['vehicle_capacities'],
        True,
        "Capacity")

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)
   

    if solution:
        print_solution(data, manager, routing, solution)
        result = format_solution_json(data, manager, routing, solution)
        return json.dumps(result, indent=4)
    else:
        raise HTTPException(status_code=500, detail="No solution found!")

def format_solution_json(data, manager, routing, solution):
    """Formats the solution into the desired JSON structure."""
    result = {
        "routes": {
            "total_distance":0,
            "total_time":0,
            "total_load" :0,
            "vehicle_routes": []
        }
    }

    address_data = create_data()
    time_dimension = routing.GetDimensionOrDie('Time')

    # Initialize total values
    total_time = 0
    total_distance = 0
    total_load = 0

    for vehicle_id in range(len(data['vehicle_capacities'])):
        index = routing.Start(vehicle_id)
        route_orders = []
        route_distance = 0
        route_load = 0

        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            node_index = manager.IndexToNode(index)

            order_time_min = solution.Min(time_var)
            order_time_max = solution.Max(time_var)

            # Creating order details for each node in the route
            order = {
                "id": node_index,
                "order_id": node_index + 1,  # Assuming order_id is node_index + 1
                "order_weight": data['demands'][node_index],
                "latitude": address_data['addresses'][node_index][0],
                "longitude": address_data['addresses'][node_index][1],
                "arrive_time": f"{minutes_to_time(order_time_min)} - {minutes_to_time(order_time_max)}"
            }
            route_orders.append(order)
            route_load += data['demands'][node_index]

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += data['distance_matrix'][manager.IndexToNode(previous_index)][manager.IndexToNode(index)]

        # Once the route for this vehicle is processed, compute the total route time
        route_time = solution.Max(time_dimension.CumulVar(index))

        # Append the complete vehicle route to result["route"]["vehicle_routes"]
        result["routes"]["vehicle_routes"].append({
            "vehicle_id": vehicle_id,
            "no_of_orders_for_route": len(route_orders),
            "total_weight_for_route": route_load,
            "total_distance_for_route": route_distance,
            "total_time_for_route": route_time,
            "orders": route_orders
        })

        # Accumulate total distance, load, and time across all vehicles
        total_distance += route_distance
        total_load += route_load
        total_time += route_time

    # Add overall totals to the result under the "route" key
    result["routes"]["total_distance"] = total_distance
    result["routes"]["total_time"] = total_time
    result["routes"]["total_load"] = total_load

    return result




def format_solution(data, manager, routing, solution):
    """Formats the solution into a structured dictionary."""
    routes = {}
    total_distance = 0
    total_load = 0

    for vehicle_id in range(len(data['vehicle_capacities'])):
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0
        route_load = 0

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route.append(node_index)
            route_load += data['demands'][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        # Capture end point and stats for each vehicle
        routes[vehicle_id] = {
            "route": route,
            "distance": route_distance,
            "load": route_load
        }
        total_distance += route_distance
        total_load += route_load

    # Final solution data
    return {
        "routes": routes,
        "total_distance": total_distance,
        "total_load": total_load
    }

# function to print the results/solution
def print_solution(data, manager, routing, solution):
    print(f'Objective: {solution.ObjectiveValue()}')  # The objective function
    
    # Display routes
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    total_distance = 0
    total_loadLB = 0
    
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_loadLB = 0
        
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            node_index = manager.IndexToNode(index)
            
            # Display node index, time, and load in one line
            plan_output += f'{node_index} Time({solution.Min(time_var)},{solution.Max(time_var)}) Load({route_loadLB}) -> '
            
            # Accumulate the route load
            route_loadLB += data['demands'][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            
            # Accumulate distance for the current arc
            route_distance += data['distance_matrix'][manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
        
        # Final stop at the depot
        time_var = time_dimension.CumulVar(index)
        node_index = manager.IndexToNode(index)
        
        # Add the final stop details (the depot) to the plan
        plan_output += f'{node_index} Time({solution.Min(time_var)},{solution.Max(time_var)}) Load({route_loadLB})\n'
        
        # Display the total time, distance, and load for the vehicle
        plan_output += f'Time of the route: {solution.Min(time_var)} min\n'
        plan_output += f'Distance of the route: {route_distance} m\n'
        plan_output += f'Load of the route: {route_loadLB}\n'
        
        # Print the result for the current vehicle
        print(plan_output)
        
        # Update totals across all vehicles
        total_distance += route_distance
        total_loadLB += route_loadLB
        total_time += solution.Min(time_var)
    
    # Print the overall totals for all vehicles
    print(f'Total time of all routes: {total_time} min')
    print(f'Total distance of all routes: {total_distance} m')
    print(f'Total load of all routes: {total_loadLB}')