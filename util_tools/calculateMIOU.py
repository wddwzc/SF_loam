import math
import numpy as np

weight = [
    0.03150183342534689,    # 0 unable
    0.042607828674502385,   # 1 car
    0.00016609538710764618, # 2 bicycle
    0.0018070552978863615,  # 3 other-vehicle
    0.00039838616015114444, # 4 motorcycle
    0.0021649398241338114,  # 5 truck
    0.0003375832743104974,  # 6 person
    0.00012711105887399155, # 7 bicyclist
    0.00023746106399997357, # 8 motorcyclist
    0.19879647126983288,    # 9 road
    0.014717169549888214,   # 10 parking
    0.14392298360372,       # 11 sidewalk
    0.0039048553037472045,  # 12 other-ground
    0.1326861944777486,     # 13 building
    0.0723592229456223,     # 14 fence
    0.26681502148037506,    # 15 vegeatation
    0.006035012012626033,   # 16 trunk
    0.07814222006271769,    # 17 terrain
    0.002855498193863172,   # 18 pole
    0.0006155958086189918   # 19 traffic-sign
]

weight_simple = [
    0.04657982379652256,    # 1 car 1 3 5
    0.0009290536701327557,  # 2 bicycle 2 4 7 8
    0.0003375832743104974,  # 3 person
    0.3613414797271883,     # 4 ground 9 10 11 12
    0.1326861944777486,     # 5 building 13
    0.0723592229456223,     # 6 fence
    0.34495724154309276,    # 7 vegeatation 15 17
    0.006035012012626033,   # 8 trunk
    0.002855498193863172,   # 9 pole
    0.0006155958086189918   # 10 traffic-sign
]

# 50.3
RandLAiou = [
    0.03150183342534689,    # 0 unable
    94.0,   # 1 car
    19.8,   # 2 bicycle
    38.7,  # 3 other-vehicle
    21.4, # 4 motorcycle
    42.7,  # 5 truck
    47.5,  # 6 person
    48.8, # 7 bicyclist
    4.6, # 8 motorcyclist
    90.4,    # 9 road
    56.9,   # 10 parking
    67.9,       # 11 sidewalk
    15.5,  # 12 other-ground
    81.1,     # 13 building
    49.7,     # 14 fence
    78.3,    # 15 vegeatation
    60.3,   # 16 trunk
    59.0,    # 17 terrain
    44.2,   # 18 pole
    38.1   # 19 traffic-sign
]

# 47.4
DarkNet21iou = [
    0.03150183342534689,    # 0 unable
    85.4,   # 1 car
    26.2,   # 2 bicycle
    15.6,  # 3 other-vehicle
    26.5, # 4 motorcycle
    18.6,  # 5 truck
    31.8,  # 6 person
    33.6, # 7 bicyclist
    4.0, # 8 motorcyclist
    91.4,    # 9 road
    57.0,   # 10 parking
    74.0,       # 11 sidewalk
    26.4,  # 12 other-ground
    81.9,     # 13 building
    52.3,     # 14 fence
    77.6,    # 15 vegeatation
    48.4,   # 16 trunk
    63.6,    # 17 terrain
    36.0,   # 18 pole
    50.0   # 19 traffic-sign
]

# 55.9
squeezev3iou = [
    0.03150183342534689,    # 0 unable
    85.4,   # 1 car
    38.7,   # 2 bicycle
    33.0,  # 3 other-vehicle
    36.5, # 4 motorcycle
    29.6,  # 5 truck
    45.6,  # 6 person
    46.2, # 7 bicyclist
    20.1, # 8 motorcyclist
    91.7,    # 9 road
    63.4,   # 10 parking
    74.8,       # 11 sidewalk
    26.4,  # 12 other-ground
    89.0,     # 13 building
    59.4,     # 14 fence
    82.0,    # 15 vegeatation
    58.7,   # 16 trunk
    65.4,    # 17 terrain
    49.6,   # 18 pole
    58.9   # 19 traffic-sign
]

def averageAsIndex(index, data):
    sum = 0
    sum_w = 0
    for ind in index:
        sum += data[ind] * weight[ind]
        sum_w += weight[ind]
    return sum / sum_w

def simplifyIOU(iou):
    simple_iou = [
        averageAsIndex([1,3,5], iou),
        averageAsIndex([2,4,7,8], iou),
        iou[6],
        averageAsIndex([9,10,11,12], iou),
        iou[13],
        iou[14],
        averageAsIndex([15,17], iou),
        iou[16],
        iou[18],
        iou[19]
    ]
    return simple_iou

def getmIoU(iou):
    sum = 0
    sum_w = 0
    for w, iou in zip(weight_simple, iou):
        sum += w * iou
        sum_w += w
    # return sum / sum_w
    return sum



sRandLA = simplifyIOU(RandLAiou)
sDark21 = simplifyIOU(DarkNet21iou)
sSqueezeV3 = simplifyIOU(squeezev3iou)
print("Simple iou: ")
print(sRandLA)
print(sDark21)
print(sSqueezeV3)


sSqueezeV3_hance = [
    85.3,
    27.2,
    35.7,
    81.2,
    83.9,
    54.1,
    75.1,
    50.6,
    34.2,
    52.6
]
# sSqueezeV3_hance = [
#     86.9,
#     33.7,
#     48.7,
#     89.4,
#     85.5,
#     58.1,
#     82.9,
#     58.9,
#     51.6,
#     54.0
# ]


print("mean iou: ")
iou_array = np.array(sRandLA)
print(np.mean(iou_array))
iou_array = np.array(sDark21)
print(np.mean(iou_array))
iou_array = np.array(sSqueezeV3)
print(np.mean(iou_array))
iou_array = np.array(sSqueezeV3_hance)
print(np.mean(iou_array))


mIoURandLA = getmIoU(sRandLA)
mIoUDark21 = getmIoU(sDark21)
mIoUSquuzeV3 = getmIoU(sSqueezeV3)
mIoUV3Hance = getmIoU(sSqueezeV3_hance)
print("mIoU: ")
print(mIoURandLA)
print(mIoUDark21)
print(mIoUSquuzeV3)
print(mIoUV3Hance)





