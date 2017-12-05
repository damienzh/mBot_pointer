# global variables

R200_DEPTH_FOV_H = 59  # degree
R200_DEPTH_FOV_V = 45  # degree

R200_MAX_DISTANCE = 4000
R200_MIN_DISTANCE = 500

XTION_FOV_H = 58  # degree
XTION_FOV_V = 45  # degree

XTION_MAX_DISTANCE = 500   # mm
XTION_MIN_DISTANCE = 4000  # mm

MBOT_WHEEL_R = 40/2 + (148-139)/2
MBOT_WIDTH = 175 - 30   # total width - one track width
MBOT_LENGTH = 200
MBOT_WHEEL_DISTANCE = 120

count2dis = 0.000355
wheelbase = 0.175

SERVO_0 = 0     # degree
SERVO_90 = 65   # degree
SERVO_180 = 140 # degree
# imd_filename = 'test_data/R200/depth_image_dc4_20170408-170421.png'  # box
# imd_filename = 'test_data/R200/depth_image_dc4_20170408-171930.png'  # doorway
# imd_filename = 'test_data/R200/depth_image_dc4_20170408-171342.png'  # table
# imc_filename = 'test_data/R200/color_image20170408-170543.png'

xtion_depth = ['xtion_depth_image_1.png', 'xtion_depth_image_2.png', 'xtion_depth_image_3.png',
               'xtion_depth_image_4.png']
xtion_color = ['xtion_color_image_1.png', 'xtion_color_image_2.png', 'xtion_color_image_3.png',
               'xtion_color_image_4.png']

path = 'test_data/xtion/'