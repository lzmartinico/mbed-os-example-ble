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
        for elem in content:
            rssi_csv.write("{},{},{}\n".format(elem['timestamp'], elem['mac'], elem['rssi']))
        rssi_csv.flush()
        return ""
    else:
        return "Hello Bluetooth!"

@app.route("/imu", methods=["GET", "POST"])
def collect_imu():
    if request.method == 'POST':
        content = request.get_json()
        for elem in content:
            imu_csv.write(','.join([elem["timestamp"]] + list(map(str, elem["acc"] + elem["gyro"] + elem["magn"]))) + "\n")
        imu_csv.flush()
        return ""
    else:
        return "Hello IMU!"

