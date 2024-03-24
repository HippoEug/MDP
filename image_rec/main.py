import time
from flask import Flask, request, jsonify
from flask_cors import CORS
from model import *

app = Flask(__name__)
CORS(app)

# model = load_model() # Needed for week 8???
model = None  # Week 9???

@app.route('/status', methods=['GET'])
def status():
    """
    This is a health check endpoint to check if the server is running
    :return: a json object with a key "result" and value "ok"
    """
    return jsonify({"result": "ok"})

@app.route('/image', methods=['POST'])
def image_predict():
    """
    This is the main endpoint for the image prediction algorithm
    :return: a json object with a key "result" and value a dictionary with keys "obstacle_id" and "image_id"
    """
    file = request.files['file']
    filename = file.filename
    print("filename: ", filename)
    
    # file.save(os.path.join('C:/Users/efoo1/Desktop/CZ3004-SC2079-MDP-ImageRecognition-main/YOLOv5 Inference Server/uploads', filename))
    file.save(os.path.join('/Users/hippoeug/Desktop/MDP/image_rec/uploads', filename))
    
    # filename format: "<timestamp>_<obstacle_id>_<signal>.jpeg"
    constituents = file.filename.split("_")
    obstacle_id = constituents[1]
    
    signal = constituents[2].strip(".jpg")
    image_id = predict_image(filename, model, signal) # Check model here

    # Return the obstacle_id and image_id
    result = {
        "obstacle_id": obstacle_id,
        "image_id": image_id
    }
    return jsonify(result)

@app.route('/stitch', methods=['GET'])
def stitch():
    """
    This is the main endpoint for the stitching command. Stitches the images using two different functions, in effect creating two stitches, just for redundancy purposes
    """
    img = stitch_image()
    img.show()
    # img2 = stitch_image_own()
    # img2.show()
    return jsonify({"result": "ok"})

if __name__ == '__main__':
    # app.run(host='0.0.0.0', port=5000, debug=True)
    app.run(host='0.0.0.0', port=5001, debug=True)
