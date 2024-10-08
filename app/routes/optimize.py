from fastapi import APIRouter, HTTPException,Depends
from app.services.tsp_services import solve_tsp
from app.services.vrp_services import solve_vrp
from app.utils.data_reader import create_data, read_data_from_csv
from app.utils.matrix import create_matrix
import folium
from fastapi.responses import HTMLResponse
import os
import polyline
import requests
from sqlalchemy.orm import Session
from app.models import models
from app.database import database
from app.database.database import get_db
from typing import List
from pydantic import BaseModel
# from app.schemas.schemas import RoutesBase, VehicleRouteBase, OrderBase
from fastapi.responses import JSONResponse
import json 

router = APIRouter()

@router.get("/distance_matrix")
async def get_distance_matrix():
    try:
        data = create_data()
        time_windows = data["time_windows"]
        distance_matrix, time_matrix = create_matrix(data)
        return {"distance_matrix": distance_matrix, "time_matrix": time_matrix,"time_windows":time_windows}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error building distance matrix: {e}")


# FastAPI route to return the solution as JSON
@router.get("/solve_vrp", response_class=JSONResponse)
async def solve_vrp_endpoint():
    try:
        result = solve_vrp()
        return JSONResponse(content=json.loads(result))  # Convert string to JSON
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error solving VRP: {e}")
    
@router.get("/solve_tsp")
async def solve_tsp_endpoint():
    try:
        tsp_solution = solve_tsp()
        return {"solution": tsp_solution}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error solving TSP: {e}")


# Function to get route from Google Directions API
def get_route_from_google_directions(start, end, API_key):
    url = f"https://maps.googleapis.com/maps/api/directions/json?origin={start[0]},{start[1]}&destination={end[0]},{end[1]}&key={API_key}"
    response = requests.get(url)
    if response.status_code == 200:
        directions = response.json()
        if directions['routes']:
            polyline_points = directions['routes'][0]['overview_polyline']['points']
            return polyline.decode(polyline_points)  # Decode polyline to list of lat/lng
        else:
            return []
    else:
        raise HTTPException(status_code=500, detail=f"Google Directions API error: {response.status_code}")

@router.get("/plot_routes", response_class=HTMLResponse)
def plot_routes():

    solution_data = solve_vrp()

    if solution_data is None or not isinstance(solution_data, dict):
        raise HTTPException(status_code=500, detail="No valid VRP solution found.")

    try:
        csv_file_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'orders.csv')
        csv_file_path = os.path.normpath(csv_file_path)

        # Then in your function:
        addresses, demands, time_windows = read_data_from_csv(csv_file_path)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading data from CSV: {e}")

    depot_location = addresses[0]
    m = folium.Map(location=depot_location, zoom_start=12)

    # API Key for Google Directions
    API_key = "AIzaSyDErHVOpa4XB1Iewo2mWU6Y44I7OelMWSE"

    colors = ['red', 'blue', 'green', 'orange', 'purple', 'darkred', 'lightblue']

    # Plot the depot location marker with a larger, more distinct icon
    folium.Marker(
        location=depot_location,
        popup="Depot (Start/End)",
        icon=folium.Icon(color="black", icon="truck", prefix="fa")  # Change to any suitable FontAwesome icon
    ).add_to(m)

    # Add a larger circle around the depot for more visibility
    folium.CircleMarker(
        location=depot_location,
        radius=15,  # Larger circle to stand out
        color="black",  # Make the circle black to highlight it
        fill=True,
        fill_color="yellow",  # Use yellow to contrast with the map
        fill_opacity=0.9
    ).add_to(m)

    for vehicle_id, route_info in solution_data.items():
        vehicle_route = route_info['route']
        vehicle_loads = route_info['loads']
        vehicle_color = colors[vehicle_id % len(colors)]

        # For each vehicle's route, get the directions between stops
        for i in range(len(vehicle_route) - 1):
            start_index = vehicle_route[i]
            end_index = vehicle_route[i + 1]

            start_location = addresses[start_index]
            end_location = addresses[end_index]

            # Get the actual road route from Google Directions API
            road_route = get_route_from_google_directions(start_location, end_location, API_key)

            # Plot numbered markers for each order (excluding depot)
            if start_index != 0:
                folium.Marker(
                    location=start_location,
                    popup=f"Order {i}, Load {vehicle_loads[i]}",
                    icon=folium.DivIcon(html=f"""
                        <div style="font-size: 12px; color: white; background-color: {vehicle_color}; 
                        border-radius: 50%; text-align: center; width: 32px; height: 32px; line-height: 32px; border: 2px solid white;">
                            {i}
                        </div>
                    """)
                ).add_to(m)

            # Plot the route with polylines that follow the road
            folium.PolyLine(road_route, color=vehicle_color, weight=2.5, opacity=1).add_to(m)

        # Plot the last marker for the vehicle route (before returning to the depot)
        end_location = addresses[vehicle_route[-1]]
        if vehicle_route[-1] != 0:
            folium.Marker(
                location=end_location,
                popup=f"Order {len(vehicle_route)}, Load {vehicle_loads[-1]}",
                icon=folium.DivIcon(html=f"""
                    <div style="font-size: 12px; color: white; background-color: {vehicle_color}; 
                    border-radius: 50%; text-align: center; width: 32px; height: 32px; line-height: 32px; border: 2px solid white;">
                        {len(vehicle_route)}
                    </div>
                """)
            ).add_to(m)

        # Highlight the route back to the depot at the end of the vehicle's trip
        last_order_location = addresses[vehicle_route[-1]]
        road_route_back_to_depot = get_route_from_google_directions(last_order_location, depot_location, API_key)

        folium.PolyLine(road_route_back_to_depot, color=vehicle_color, weight=2.5, opacity=1, dash_array="5, 10").add_to(m)

    # Save the map to an HTML file
    map_file = "route_map.html"
    m.save(map_file)

    # Read and return the HTML content to be displayed in the browser
    with open(map_file, "r") as file:
        map_html = file.read()

    return HTMLResponse(content=map_html)




def save_solution_to_db(db: Session, result: dict):
    route_data = result['routes']

    # Create Route entry
    db_route = models.Routes(
        total_distance=route_data['total_distance'],
        total_time=route_data['total_time'],
        total_weight=route_data['total_load']
    )
    db.add(db_route)
    db.commit()
    db.refresh(db_route)

    # Create VehicleRoute and Order entries
    for vehicle_route_data in route_data['vehicle_routes']:
        db_vehicle_route = models.VehicleRoute(
            vehicle_id=vehicle_route_data['vehicle_id'],
            no_of_orders_for_route=vehicle_route_data['no_of_orders_for_route'],
            total_weight_for_route=vehicle_route_data['total_weight_for_route'],
            total_distance_for_route=vehicle_route_data['total_distance_for_route'],
            total_time_for_route=vehicle_route_data['total_time_for_route'],
            routes_id=db_route.id
        )
        db.add(db_vehicle_route)
        db.commit()
        db.refresh(db_vehicle_route)

        for order_data in vehicle_route_data['orders']:
            db_order = models.Orders(
                order_id=order_data['order_id'],
                order_weight=order_data['order_weight'],
                latitude=order_data['latitude'],
                longitude=order_data['longitude'],
                arrive_time=order_data['arrive_time'],
                vehicle_route_id=db_vehicle_route.id
            )
            db.add(db_order)

    db.commit()

@router.post("/solve")
def solve_vrp_endpoint(db: Session = Depends(get_db)):
    result = solve_vrp()
  # Call your solve function to get the result
    save_solution_to_db(db, json.loads(result))  # Save the result in the database
    return {"message": "Solution saved successfully", "result": json.loads(result)}