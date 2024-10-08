import json
from fastapi import FastAPI, HTTPException
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from app.utils.data_reader import create_tsp_data_model

def solve_tsp():
    """Solves the TSP and returns a structured solution."""
    data = create_tsp_data_model()
    
    # Create routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    
    # Create a distance callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Define cost of each arc (distance)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        return format_tsp_solution(data, manager, routing, solution)
    else:
        raise HTTPException(status_code=500, detail="No solution found!")

def format_tsp_solution(data, manager, routing, solution):
    """Formats the TSP solution into a structured dictionary."""
    route = []
    total_distance = 0

    index = routing.Start(0)
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        route.append(node_index)
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        total_distance += routing.GetArcCostForVehicle(previous_index, index, 0)

    route.append(manager.IndexToNode(index))  # End at the depot

    # Return solution as a dictionary
    return {
        "route": route,
        "total_distance": total_distance,
    }

def print_tsp_solution(data, manager, routing, solution):
    """Prints the TSP solution."""
    print(f'Objective: {solution.ObjectiveValue()}')  # The objective function
    
    # Initialize output and totals
    route_output = 'Route for the salesperson:\n'
    route_distance = 0

    index = routing.Start(0)
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        route_output += f'{node_index} -> '
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)

    # Add final stop at the depot
    node_index = manager.IndexToNode(index)
    route_output += f'{node_index}\n'
    route_output += f'Total distance of the route: {route_distance} meters\n'
    
    # Print the complete route and total distance
    print(route_output)