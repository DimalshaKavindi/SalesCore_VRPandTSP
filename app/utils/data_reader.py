import os
import pandas as pd
from datetime import datetime, timedelta

from app.utils.matrix import create_matrix

def time_to_minutes(time_str):
    """Convert time string (e.g., '8:00AM') to minutes since midnight."""
    dt = datetime.strptime(time_str, '%I:%M%p')
    return (dt.hour-8) * 60 + dt.minute

def minutes_to_time(minutes):
    """Convert minutes since 8:00 AM to a time string (e.g., '8:00AM')."""
    base_time = datetime.strptime('8:00AM', '%I:%M%p')
    time_after_minutes = base_time + timedelta(minutes=minutes)
    return time_after_minutes.strftime('%I:%M%p').lstrip('0')  

def read_data_from_csv(file_path):
    df = pd.read_csv(file_path)
    addresses = df[['location_latitude', 'location_longitude']].values.tolist()
    demands = df['order_weight'].tolist()
    
    # Convert time windows to hours
    time_windows = []
    for from_time, to_time in zip(df['from'], df['to']):
        from_minute = time_to_minutes(from_time)
        to_minute = time_to_minutes(to_time)
        time_windows.append((from_minute, to_minute))
    
    return addresses, demands, time_windows


def create_data():
    csv_file_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'orders.csv')
    file_path = os.path.normpath(csv_file_path)
    addresses, demands, time_windows = read_data_from_csv(file_path)
    
    data = {}
    data['API_key'] = 'AIzaSyDErHVOpa4XB1Iewo2mWU6Y44I7OelMWSE'
    data['addresses'] = addresses
    data["demands"] = demands
    data["time_windows"] = time_windows
    data["vehicle_capacities"] = [2770,2770]
    data['num_vehicles'] = 2
    data['depot'] = 0
    
    return data

def create_data_model():
    """Stores the data for the problem, including both distance and time matrices."""
    data = {}
    data_model = create_data()
    distance_matrix, time_matrix = create_matrix(data_model)
    
    data['distance_matrix'] = distance_matrix     # In meter
    data['time_matrix'] = time_matrix             # In minutes
    data["time_windows"] = data_model["time_windows"]
    data["demands"] = data_model["demands"]
    data["vehicle_capacities"] = data_model["vehicle_capacities"]
    data["num_vehicles"] = data_model["num_vehicles"]
    data["depot"] = data_model["depot"]

    # 15 min of service time
    data['service_time'] = [5] * len(data['time_matrix'])
    data['service_time'][data['depot']] = 0
    assert len(data['time_matrix']) == len(data['service_time'])
    
    return data

def create_tsp_data_model():
    """Creates data model for TSP (single vehicle)."""
    data = {}
    data_model = create_data()  # Reuse the existing function
    distance_matrix, time_matrix = create_matrix(data_model)  # Get the distance and time matrix
    
    data['distance_matrix'] = distance_matrix
    data['time_matrix'] = time_matrix
    data["time_windows"] = data_model["time_windows"]
    data['num_vehicles'] = 1  # Only one vehicle for TSP
    data['depot'] = data_model['depot']  # Start and end at the depot

    # No vehicle capacity or demand needed for TSP
    return data
