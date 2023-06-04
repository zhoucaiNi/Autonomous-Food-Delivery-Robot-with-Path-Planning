import requests
import sys
import math

# USAGE: python get_map.py "Hanover, NH"

def save_map(search_phrase):

	# api_key = 'AIzaSyAdEr6vxj6p_Fp1YwNmwpbvuEIN3VMJzfU'

	# Get the geolocation of the search_phrase
	geocode_url = f"https://maps.googleapis.com/maps/api/geocode/json?address={search_phrase}&key={api_key}"
	geocode_response = requests.get(geocode_url)
	geocode_data = geocode_response.json()
	lat, lng = geocode_data["results"][0]["geometry"]["location"].values()

	# Prepare the request URL for the static map
	base_url = 'https://maps.googleapis.com/maps/api/staticmap?'
	parameters = {
		'key': api_key,
		'size': '2048x2048',
		'maptype': 'roadmap',
		'zoom': str(16),
		'center': search_phrase,  # Use the search phrase as the center
	}
	request_url = base_url + '&'.join([f'{key}={value}' for key, value in parameters.items()])

	try:
		# Send the request to the Google Maps Static API
		response = requests.get(request_url)
		response.raise_for_status()

		# Save the map image as a PNG file
		with open('map.png', 'wb') as file:
			file.write(response.content)

		print('Map saved as map.png')
		# Calculate the rough "bounds" of the map
		top_left_lat, top_left_lng = calculate_bounds(lat, lng, 16)
		print(f'The top left corner is approximately at ({top_left_lat}, {top_left_lng})')

	except requests.exceptions.RequestException as e:
		print(f'Error: {e}')

def calculate_bounds(lat, lng, zoom):
	# Inverse calculation of latitude and longitude per pixel at this zoom level
	lat_rad = lat * math.pi / 180.0
	one_px_lat = 1 / (111.32 * 1000)  # roughly one pixel in degrees at equator
	one_px_lng = one_px_lat / math.cos(lat_rad)

	map_px_height = map_px_width = 2048  # the height and width of your map in pixels

	half_map_height_deg = (map_px_height / 2) * one_px_lat
	half_map_width_deg = (map_px_width / 2) * one_px_lng

	top_lat = lat + half_map_height_deg
	left_lng = lng - half_map_width_deg

	return top_lat, left_lng

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print('Usage: python map_saver.py <search_phrase>')
	else:
		search_phrase = sys.argv[1]
