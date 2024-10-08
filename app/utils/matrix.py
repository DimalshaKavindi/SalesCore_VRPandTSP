import requests
import urllib
import json



def create_matrix(data):
    addresses = data["addresses"]
    API_key = data["API_key"]
    max_elements = 100
    num_addresses = len(addresses)
    max_rows = max_elements // num_addresses
    q, r = divmod(num_addresses, max_rows)
    dest_addresses = addresses
    distance_matrix = []
    time_matrix = []

    departure_time = "now"
    
    for i in range(q):
        origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
        response = send_request(origin_addresses, dest_addresses, API_key,departure_time)
        distance_matrix += build_distance_matrix(response)
        time_matrix += build_time_matrix(response)

    if r > 0:
        origin_addresses = addresses[q * max_rows: q * max_rows + r]
        response = send_request(origin_addresses, dest_addresses, API_key, departure_time)
        distance_matrix += build_distance_matrix(response)
        time_matrix += build_time_matrix(response)
    return distance_matrix,time_matrix

def send_request(origin_addresses, dest_addresses, API_key,departure_time):
    def build_address_str(addresses):
        address_str = ''
        for i in range(len(addresses)):
        # Each address should be formatted as 'lat,lng'
            lat, lng = addresses[i]
            address_str += f'{lat},{lng}'
            if i < len(addresses) - 1:
                address_str += '|'
        return address_str

    request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=imperial'
    origin_address_str = build_address_str(origin_addresses)
    dest_address_str = build_address_str(dest_addresses)
    
    # Add traffic model and departure time for traffic-aware travel times
    request += '&origins=' + origin_address_str
    request += '&destinations=' + dest_address_str
    request += '&key=' + API_key
    request += '&departure_time=' + departure_time  # Set to "now" or a specific timestamp
    request += '&traffic_model=best_guess'  # Use 'best_guess', 'pessimistic', or 'optimistic'
    
    jsonResult = urllib.request.urlopen(request).read()
    response = json.loads(jsonResult)
    return response

def build_distance_matrix(response):
    distance_matrix = []
    for row in response['rows']:
        row_list = [row['elements'][j]['distance']['value'] for j in range(len(row['elements']))]
        distance_matrix.append(row_list)
    return distance_matrix

def build_time_matrix(response):
    time_matrix = []
    for row in response['rows']:
        row_list = []
        for element in row['elements']:
            # Check if traffic-aware duration is available, otherwise fallback to normal duration
            if 'duration_in_traffic' in element:
                time = element['duration_in_traffic']['value']  # Use traffic-aware duration
            else:
                time = element['duration']['value']  # Fallback to standard duration if traffic data is unavailable
            
            row_list.append(time // 60)  # Convert seconds to minutes
        time_matrix.append(row_list)
    return time_matrix
