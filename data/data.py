import numpy as np
import csv
import plotly
import plotly.graph_objs as go
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import gaussian_filter

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
