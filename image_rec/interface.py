import requests
import json

# Specify the URL of the server's image endpoint
# url = "http://localhost:5000/image"
url_image = "http://localhost:5001/image"
url_stitch = "http://localhost:5001/stitch"

# Specify the path to the image file you want to upload
# image_path = "/Users/hippoeug/Desktop/MDP/SERVER_W8/self_uploaded/time_36_L.jpg"
# image_path = "/Users/hippoeug/Desktop/MDP/SERVER_W8/self_uploaded/time_37_L.jpg"
# image_path = "/Users/hippoeug/Desktop/MDP/SERVER_W8/self_uploaded/time_38_L.jpg"
# image_path = "/Users/hippoeug/Desktop/MDP/SERVER_W8/self_uploaded/time_39_L.jpg"
image_path = "/Users/hippoeug/Desktop/MDP/SERVER_W8/self_uploaded/complicated_69_L.jpg"
# image_path = "/Users/hippoeug/Desktop/MDP/SERVER_W8/self_uploaded/time_bullseye_L.jpg"

# Read the content of the image file as bytes
with open(image_path, "rb") as f:
    image_data = f.read()

# Make the POST request to the server, FOR IMAGE
# response = requests.post(url_image, files={"file": ("time_36_L.jpg", image_data)})
response = requests.post(url_image, files={"file": ("complicated_69_L.jpg", image_data)})
# response = requests.post(url_image, files={"file": ("time_bullseye_L.jpg", image_data)})

# Make the GET request to the server, FOR STITCH
# response = requests.get(url_stitch)

# Print the server's response
try:
    # print(response.json())
    results = json.loads(response.content)
    image_id = results['image_id']

    # response_data = response.json()
    # image_id = response_data['image_id']
    print(image_id)

    if image_id == "38":
        print("RIGHT ARROW")
    elif image_id == "39":
        print("LEFT ARROW")
    else:
        print("NO ARROW")
except requests.exceptions.JSONDecodeError:
    print("Non-JSON response:", response.text)