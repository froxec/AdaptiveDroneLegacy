#X-configuration
#ccw is positive
#   cw  4  🢁🢁🢁  2 ccw
#         \    /  
#          \  /
#           \/
#           /\
#          /  \
#         /    \
#   ccw 3        1 cw


motor1 = thrust + roll - pitch - yaw
motor2 = thrust + roll + pitch + yaw
motor3 = thrust - roll - pitch + yaw
motor4 = thrust - roll + pitch - yaw