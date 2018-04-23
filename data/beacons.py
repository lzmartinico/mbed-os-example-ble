import numpy as np
import csv
import plotly
import plotly.graph_objs as go
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import gaussian_filter, gaussian_filter1d
from math import asin, acos, degrees, sin, cos, pi

plotly.offline.init_notebook_mode(connected=True)
py = plotly.offline

# read data

def convert(row):
    id = row[0]
    rssi = row[1]
    x = row[2]
    y = row[3]
    return (id, int(rssi), (int(x) + 390) / 3, (int(y) + 960) / 3)
    
def read_data():
    with open('rssi_sat.csv', 'rb') as f:
        reader = csv.reader(f)
        data = map(convert, reader)

    data_for_beacon = {}
    for id in beacon_ids:
        data_for_beacon[id] = []
        
    for row in data:
        beacon = row[0]
        data_for_beacon[beacon].append(row)

    return data_for_beacon

data_for_beacon = read_data()
    
# global coordinates to pixels conversion

# pixel_origin = np.array([252, 1012])
# global_origin = np.array([55.94444938963575,-3.1869836524128914])

pixel_origin = np.array([125, 410])
global_origin = np.array([55.94455753546212,-3.1866420060396194])

# 55.94455753546212,-3.1866420060396194
# 55.944491821888334,-3.1870497018098827
# 55.94443343059064,-3.186571933329106

def get_conversion_data():
#     se1_pixel = np.array([409, 479])
#     nw10_pixel = np.array([161, 897])

#     se1_global = np.array([55.9444578385393,-3.1866151839494705])
#     nw10_global = np.array([55.94449107087541,-3.186941407620907])
    
    se1_pixel = np.array([125, 1040])
    nw10_pixel = np.array([480, 410])

    se1_global = np.array([55.944491821888334,-3.1870497018098827])
    nw10_global = np.array([55.94443343059064,-3.186571933329106])

    se1_pixel_trans = se1_pixel - pixel_origin
    nw10_pixel_trans = nw10_pixel - pixel_origin

    se1_global_trans = se1_global - global_origin
    nw10_global_trans = nw10_global - global_origin

    return ((se1_pixel_trans, nw10_pixel_trans), (se1_global_trans, nw10_global_trans))

# a=-2.62187x10^6, b=422601., c=-799717., d=-1.41637x10^6
# coor_change = np.array([[-2.67731e6, 487478.], [-1.32371e6, -1.41618e6]])
coor_change = np.array([[-2.62187e6, 422601.], [-799717., -1.41637e6]])


# convert global coordinates to pixels

beacons_global = np.array([
    [55.9444578385393,-3.1866151839494705],
    [55.94444244275808,-3.18672649562358860],
    [55.94452336441765,-3.1866540759801865],
    [55.94452261340533,-3.1867526471614838],
    [55.94448393625199,-3.1868280842900276],
    [55.94449050761571,-3.1866483762860294],
    [55.94443774892113,-3.1867992505431175],
    [55.944432116316044,-3.186904862523079],
    [55.94444938963575,-3.1869836524128914],
    [55.94449107087541,-3.186941407620907]
])

beacons_pixel = np.array(np.transpose(coor_change.dot(np.transpose(beacons_global - global_origin))) + pixel_origin, dtype=np.int16)

old_beacons_pixel = np.array([(409, 479), (404, 651), (204, 495), (165, 626), (243, 758), (304, 504), (396, 739), (356, 922), (252, 1012), (161, 897)])

# beacon id

beacon_ids = ['cd', 'd7', '17', '51', '43', 'b8', '2a', 'f8', '3d', '62',
            '3']
    
# drawing images
    
def draw_image(points, b):
    # get an image
    img = Image.open('map.png')
    fnt = ImageFont.truetype('/Users/piotr/Library/Fonts/Literation Mono Powerline.ttf', 40)
    
    d = ImageDraw.Draw(img)

    d.text((10,10), b, font=fnt, fill=(0,0,0,0))

    for (ident, beacon) in zip(beacon_ids, beacons_pixel):
        (x, y) = beacon
        d.text((x+5,y+5), ident, font=fnt, fill=(0,0,0,0))
        d.ellipse([x-2, y-2, x+2, y+2], outline=(0,255,0,0), fill=(0,255,0,0))
        
    for old in old_beacons_pixel:
        (x, y) = old
        d.ellipse([x-2, y-2, x+2, y+2], outline=(0,0,255,0), fill=(0,0,255,0))

    for point in points:
        x = point[0]
        y = point[1]
        d.ellipse([x-2, y-2, x+2, y+2], outline=(255,0,0,0), fill=(255,0,0,0))

    img.show(title=b)
    
def draw_image_for(b_id):
    points = np.array([[row[2], row[3]] for row in data_for_beacon[b_id]], dtype=np.int32)
    draw_image(points, b_id)
    
# analytics

def rssi_from_data(data):
    return np.array([row[1] for row in data], dtype=np.int16)

def pos_from_data(data):
    return np.array([[row[2], row[3]] for row in data], dtype=np.int16)

def dist_from_point(point, positions):
    diff = positions - point
    return np.linalg.norm(diff, axis=1)
    
def beacon_fun(b_id, b_pos):
    data = data_for_beacon[b_id]
    pos = pos_from_data(data)
    rssi = rssi_from_data(data)
    dist = dist_from_point(b_pos, pos)
    return (pos, rssi, dist)

def map_strongest_signal(rssi, pos, n, b_id):
    largest = np.argsort(rssi)[:n]
    points = pos[largest]
    draw_image(points, b_id)
    
def bestfit_rssi_dist(rssi, dist):
    return np.poly1d(np.polyfit(dist, rssi, 2))
    
def plot_rssi_f_dist(b_id, b_pos):
    (beacon_pos, beacon_rssi, dist) = beacon_fun(b_id, b_pos)
    f = bestfit_rssi_dist(beacon_rssi, dist)
    domain = np.linspace(0, 650)
    py.iplot({
        "data": [
            go.Scatter(x=dist, y=beacon_rssi, mode='markers'),
            go.Scatter(x=domain, y=f(domain))
        ],
        "layout": go.Layout(title=b_id)
    }) 
    
def normalize(array):
    return (array - np.mean(array)) / np.std(array)

# RSSI along different radii

endpoints = np.concatenate([
    np.stack([np.linspace(125, 480, 6), np.full(6, 1040)], axis=1), 
    np.array([[x, y] for x in [125, 480] for y in [1040 - 60, 1040 - 120]])
    ])

class Beacon(object):
    def __init__(self, b_id, pos):
        self.name = b_id
        self.pos = pos
        self.points = pos_from_data(data_for_beacon[b_id])
        self.rssi = rssi_from_data(data_for_beacon[b_id])
        self.dist = dist_from_point(pos, self.points)

# # now for each vector, get the vector from b8 or 17
# pos_b8 = beacons_pixel[5]
# pos_17 = beacons_pixel[2]

# points_b8 = pos_from_data(data_for_beacon['b8'])
# rssi_b8 = rssi_from_data(data_for_beacon['b8'])

# points_17 = pos_from_data(data_for_beacon['17'])
# rssi_17 = rssi_from_data(data_for_beacon['17'])

# dist_17 = dist_from_point(pos_17, points_17)
# dist_b8 = dist_from_point(pos_b8, points_b8)

b_17 = Beacon('17', beacons_pixel[2])
b_b8 = Beacon('b8', beacons_pixel[5])

def plot_points_on_line(points, line):
    # get an image
    img = Image.open('map.png')
    fnt = ImageFont.truetype('/Users/piotr/Library/Fonts/Literation Mono Powerline.ttf', 40)
    
    d = ImageDraw.Draw(img)

    for point in points:
        x = point[0]
        y = point[1]
        d.ellipse([x-2, y-2, x+2, y+2], outline=(255,0,0,0), fill=(255,0,0,0))
        
    for point in line:
        x = point[0]
        y = point[1]
        d.ellipse([x-2, y-2, x+2, y+2], outline=(0,255,0,0), fill=(0,255,0,0))

    img.show()

    
def dist_from_line(points, start, end):
    return np.absolute(np.cross(end-start,points-start) / np.linalg.norm(end - start))
    
    
def map_points_on_line(points, start, end):
    line = np.stack([np.linspace(start[0], end[0]), np.linspace(start[1], end[1])], axis=1)
    margin = dist_from_line(points, start, end)
    plot_points_on_line(points_b8[margin < 20], line)

    
def plot_rssi_dist(rssi, dist, title):
    f = bestfit_rssi_dist(rssi, dist)
    domain = np.linspace(0, 650)
    py.iplot({
        "data": [
            go.Scatter(x=dist, y=rssi, mode='markers'),
            go.Scatter(x=domain, y=f(domain))
        ],
        "layout": go.Layout(title=title)
    }) 
    

def plot_rssi_dist_on_line(points, rssi, start, end):
    dist = dist_from_point(points, start)
    margin = dist_from_line(points, start, end)
    plot_rssi_dist(rssi[margin < 20], dist[margin < 20], title=end)
        
    
def plot_all(points, rssi, start, beacon):
    dist = dist_from_point(points, start)
    domain = np.linspace(0, 650)
    fitlines = []
    ends = []
    for end in endpoints:
        margin = dist_from_line(points, start, end)
        line_dist = dist[margin < 20]
        line_rssi = rssi[margin < 20]
        f = bestfit_rssi_dist(line_rssi, line_dist)
#         fitlines.append(f)
#         ends.append(end)

        py.iplot({
            "data": [go.Scatter(x=line_dist, y=line_rssi, mode='markers')] + [go.Scatter(x=domain, y=f(domain))],
            "layout": go.Layout(title=repr(list(end)))
        }) 
        
#     py.iplot({
#         "data": [go.Scatter(x=dist, y=rssi, mode='markers')] + [go.Scatter(x=domain, y=f(domain), name=repr(list(end))) for (f, end) in zip(fitlines, ends)],
#         "layout": go.Layout(title=beacon)
#     }) 

def gamma(a, b, c):
    cosval = (a ** 2 + b ** 2 - c ** 2)/(2.0 * a * b)
    if cosval > 1.:
        cosval = 1.
    elif cosval < -1.:
        cosval = -1.
    return acos(cosval)

r = np.linalg.norm(np.array(pos_b8 - pos_17, dtype=np.float))
alpha = asin(v[1] / v[0])

def get_position_from_dist(r_17, r_b8):    
    beta = gamma(r_17, r, r_b8)
    return pos_17 + [r_17 * cos(alpha + beta), r_17 * sin(alpha + beta)]

def get_r_17_b8(point):
    r_17 = np.linalg.norm(point - pos_17)
    r_b8 = np.linalg.norm(point - pos_b8)
    return np.array([r_17, r_b8])

def reflect(point):
    return get_position_from_dist(*get_r_17_b8(point))
    
# now apply to our test data
# but we have them in order
# now we want to read them in a way so that we always have last reading from 17 and b8

def read_data_17_b8():
    with open('rssi_sat.csv', 'rb') as f:
        reader = csv.reader(f)
        rssi_17_b8 = []
        positions_17_b8 = []
        last_17 = -70
        last_b8 = -70
        for [b_id, rssi, x, y] in reader:
            rssi, x, y = int(rssi), (int(x) + 390.) / 3, (int(y) + 960.) / 3
            if b_id == '17' and rssi > -80:
                last_17 = rssi
                rssi_17_b8.append(np.array([last_17, last_b8], dtype=np.float))
                positions_17_b8.append(np.array([x, y], dtype=np.float))
            elif b_id == 'b8' and rssi > -80:
                last_b8 = rssi
                rssi_17_b8.append(np.array([last_17, last_b8], dtype=np.float))
                positions_17_b8.append(np.array([x, y], dtype=np.float))

    return (np.array(rssi_17_b8, dtype=np.float), np.array(positions_17_b8, dtype=np.float))

rssi_17_b8, positions_17_b8 = read_data_17_b8()

f_dist_rssi_17 = np.poly1d(np.polyfit(rssi_17, dist_17, 2))
f_dist_rssi_b8 = np.poly1d(np.polyfit(rssi_b8, dist_b8, 2))

def plot_f_dist_rssi(rssi, dist, f, title='', low=-100):
    domain = np.linspace(low, -50)
    py.iplot({
        "data": [
            go.Scatter(x=rssi, y=dist, mode='markers'),
            go.Scatter(x=domain, y=f(domain))
        ],
        "layout": go.Layout(title=title)
    })

# plot rssi as function of distance from beacon

def get_rel_dist_from_17_b8(rssi):
    return np.array([f_dist_rssi_17(rssi[0]), f_dist_rssi_b8(rssi[1])])

def get_pos_from_rssi(rssi):
    return get_position_from_dist(*get_rel_dist_from_17_b8(rssi))

def plot_predictions(rssi, positions, start, n):
    stop = start + n
    smooth_rssi = gaussian_filter1d(rssi, sigma=16, axis=0)
    estimated_position = np.apply_along_axis(get_pos_from_rssi, 1, smooth_rssi)
    
    py.iplot({
        "data": [
            go.Scatter(x=positions[start:stop,0], y=positions[start:stop,1]),
            go.Scatter(x=estimated_position[start:stop,0], y=estimated_position[start:stop,1])
        ],
        "layout": go.Layout(
            title='beacons',
            xaxis=dict(
                range=[-200, 800]
            ),
            yaxis=dict(
                range=[1050, 300]
            )
        )
    })
    
    py.iplot({
        "data": [
            go.Scatter(x=np.linspace(0, 100, n), y=positions[start:stop,0]),
            go.Scatter(x=np.linspace(0, 100, n), y=estimated_position[start:stop,0])
        ],
        "layout": go.Layout(
            title='x'
        )
    })
    
    py.iplot({
        "data": [
            go.Scatter(x=np.linspace(0, 100, n), y=positions[start:stop,1]),
            go.Scatter(x=np.linspace(0, 100, n), y=estimated_position[start:stop,1])
        ],
        "layout": go.Layout(
            title='y'
        )
    })

def rssi_changes(b_id, b_pos, start, n):
    stop = start + n
    data = data_for_beacon[b_id]
    rssi = rssi_from_data(data)
    dist = dist_from_point(b_pos, pos_from_data(data))
    normalized_rssi = normalize(rssi)[start:stop]
    normalized_dist = normalize(dist)[start:stop]
    filtered_rssi = gaussian_filter(normalized_rssi, sigma=6)
    n = len(rssi)
    datalines = ['normalized_rssi', 'normalized_dist', 'filtered_rssi']

    py.iplot({
        'data': [go.Scatter(name=data, x=np.linspace(0, 100, n), y=locals()[data]) for data in datalines],
        'layout': go.Layout(title=b_id)
    })

def plot_rssi_actual_and_predicted(actual, predicted, start, n):
    stop = start + n
    domain = np.linspace(0, 1, n)
    
    def plot(i, name):
        py.iplot({
            "data": [
                go.Scatter(x=domain, y=actual[start:stop, i], name='actual'),
                go.Scatter(x=domain, y=predicted[start:stop, i], name='predicted')
            ],
            "layout": go.Layout(
                title=name,
                yaxis=dict(
                    range=[-100, -50]
                )
            )
        })
        
    plot(0, '17')
    plot(1, 'b8')

    
plot_rssi_actual_and_predicted(rssi_17_b8, estimate_rssi(positions_17_b8), 0, 3100)

# okay now need a model of ideal RSSI = f(dist_17, dist_b8)

def f_rssi_dist_poly2d(params):
    return polyval2d(dist_from_17, dist_from_b8, np.reshape(params, (2, 3)))

def fun_17(params):
    return rssi_17_b8[:,0] - f_rssi_dist_poly2d(params)

def fun_b8(params):
    return rssi_17_b8[:,1] - f_rssi_dist_poly2d(params)

x0 = np.random.normal(size=6)
# x0 = np.zeros(6)o

pred_rssi_17 = f_rssi_dist_poly2d(least_squares(fun_17, x0).x)
pred_rssi_b8 = f_rssi_dist_poly2d(least_squares(fun_b8, x0).x)

pred_rssi_17_sv = b_17.f_rssi_dist(dist_from_17)
pred_rssi_b8_sv = b_b8.f_rssi_dist(dist_from_b8)


def plot_dist_actual_and_predicted(actual, predicted, predicted_sv, name, start, n):
    stop = start + n
    domain = np.linspace(0, 1, n)
    
    py.iplot({
        "data": [
            go.Scatter(x=domain, y=actual[start:stop], name='actual'),
            go.Scatter(x=domain, y=predicted[start:stop], name='predicted'),
            go.Scatter(x=domain, y=predicted_sv[start:stop], name='predicted single var')
        ],
        "layout": go.Layout(
            title=name,
            yaxis=dict(
                range=[-105, -50]
            )
        )
    })


plot_dist_actual_and_predicted(rssi_17_b8[:,0], pred_rssi_17, pred_rssi_17_sv, '17', 0, 3000)
plot_dist_actual_and_predicted(rssi_17_b8[:,1], pred_rssi_b8, pred_rssi_b8_sv, 'b8', 0, 3000)

# okay now get covariance of rssi

r_cov = np.cov([pred_rssi_17 - rssi_17_b8[:, 0], pred_rssi_b8 - rssi_17_b8[:, 1]])