from flask import Flask, request
app = Flask(__name__)

rssi_csv = open('rssi.csv', 'a+')
imu_csv = open('imu.csv', 'a+')

@app.route("/demo", methods=["POST"])
def demo():
    return "TODO demo"

@app.route("/rssi", methods=["GET", "POST"])
def collect_rssi():
    if request.method == 'POST':
        content = request.get_json()
        rssi_csv.write("{},{},{},{}\n".format(content['mac'], content['rssi'], content['x'], content['y']))
        return ""
    else:
        return "Hello Bluetooth!"

@app.route("/imu", methods=["GET", "POST"])
def collect_imu():
    if request.method == 'POST':
        content = request.get_json()
        imu_csv.write(','.join(content["timestamp"] + map(str, content["acc"] + content["gyro"] + content["magn"])) )
        return ""
    else:
        return "Hello IMU!"
