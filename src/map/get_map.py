import requests
import sys

# USAGE: python get_map.py "Hanover, NH"

def save_map(search_phrase):

	api_key = 'AIzaSyAdEr6vxj6p_Fp1YwNmwpbvuEIN3VMJzfU'

	# Prepare the request URL
	base_url = 'https://maps.googleapis.com/maps/api/staticmap?'
	parameters = {
		'key': api_key,
		'size': '2048x2048',
		'maptype': 'roadmap',
		'zoom': str(16),
		'center': search_phrase,  # Use the search phrase as the center
		'style': "feature%3Aadministrative%7Celement%3Aall%7Cvisibility%3Aoff%7C&style=feature%3Alandscape%7Celement%3Aall%7Ccolor%3A0x000000%7C&style=feature%3Apoi%7Celement%3Aall%7Cvisibility%3Aoff%7C&style=feature%3Apoi%7Celement%3Ageometry%7Ccolor%3A0x000000%7C&style=feature%3Aroad%7Celement%3Ageometry.fill%7Ccolor%3A0xffffff%7C&style=feature%3Aroad%7Celement%3Ageometry.stroke%7Ccolor%3A0xffffff%7C&style=feature%3Aroad%7Celement%3Alabels%7Cvisibility%3Aoff%7C&style=feature%3Atransit%7Celement%3Aall%7Cvisibility%3Aoff%7C&style=feature%3Awater%7Celement%3Aall%7Ccolor%3A0x000000%7C"
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
	except requests.exceptions.RequestException as e:
		print(f'Error: {e}')

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print('Usage: python map_saver.py <search_phrase>')
	else:
		search_phrase = sys.argv[1]
		save_map(search_phrase)
