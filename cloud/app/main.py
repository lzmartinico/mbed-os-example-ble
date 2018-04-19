from flask import Flask, request
app = Flask(__name__)

csv = open('rssi.csv', 'w')

@app.route("/", methods=["GET", "POST"])
def hello():
    if request.method == 'POST':
        content = request.get_json()
        csv.write("{},{},{},{}\n".format(content['mac'], content['rssi'], content['x'], content['y']))
        return ""
    else:
        return "Hello World!"