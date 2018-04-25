def sub(t1, t2):
    return (t1[0] - t2[0], t1[1] - t2[1])

def transform(beacon):
    upperleft = (280, 200)
    lowerright = (470, 1045)
    res = sub(beacon, upperleft)
    res = (res[0] / 1.15, res[1] / 1.22)
    res = sub(lowerright, res)
    return (int(res[0]), int(res[1]))

beacons = [
    (350, 890),
    (355, 680),
    (585, 870),
    (630, 710),
    (540, 550),
    (470, 860),
    (365, 573),
    (410, 350),
    (530, 240),
    (635, 380)
]

beacons = map(transform, beacons)

print(beacons)
