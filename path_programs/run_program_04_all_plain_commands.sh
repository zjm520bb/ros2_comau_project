#!/usr/bin/env bash
set -eo pipefail

cd /home/fishros/ros2_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

send_cmd() {
  local cmd="$1"
  ros2 action send_goal \
    /arm/execute \
    arm_tcp_bridge_interfaces/action/ExecuteCommand \
    "{command: '${cmd}'}" \
    --feedback
}

# Conservative defaults for the real C4G test.
send_cmd "setTool:0,0,200,0,0,0"
send_cmd "setOrientation:0"
send_cmd "setSpeedLin:0.05"
send_cmd "setSpeedJnt:5"
send_cmd "setAcceleration:10"
send_cmd "setDeceleration:10"

# PATH 1: moveJoint
send_cmd "moveJoint:-90,0,-90,0,0,0"
send_cmd "moveJoint:-90,10,-160,0,-70,0"
send_cmd "setFlyNorm:30"
send_cmd "clearFlyQueue"
send_cmd "addFlyJoint:-89.999937,9.866636,-159.856772,0.000039,-69.856761,-0.000008"
send_cmd "addFlyJoint:-89.999748,9.466544,-159.42709,0.000156,-69.427042,-0.000031"
send_cmd "addFlyJoint:-89.999432,8.799725,-158.710953,0.000351,-68.710847,-0.00007"
send_cmd "addFlyJoint:-89.998991,7.86618,-157.708364,0.000624,-67.708174,-0.000124"
send_cmd "addFlyJoint:-89.998424,6.665909,-156.41932,0.000976,-66.419024,-0.000193"
send_cmd "addFlyJoint:-89.99773,5.19891,-154.843823,0.001405,-64.843397,-0.000279"
send_cmd "addFlyJoint:-89.996946,3.541186,-153.063494,0.00189,-63.062921,-0.000375"
send_cmd "addFlyJoint:-89.996261,2.092219,-151.507361,0.002314,-61.50666,-0.000459"
send_cmd "addFlyJoint:-89.995702,0.90998,-150.237684,0.00266,-60.236878,-0.000527"
send_cmd "addFlyJoint:-89.995269,-0.005531,-149.254462,0.002927,-59.253575,-0.000581"
send_cmd "addFlyJoint:-89.994962,-0.654314,-148.557695,0.003117,-58.55675,-0.000618"
send_cmd "addFlyJoint:-89.994782,-1.036369,-148.147384,0.003229,-58.146405,-0.00064"
send_cmd "addFlyJoint:-89.994727,-1.152306,-148.022873,0.003263,-58.021883,-0.000647"
send_cmd "executeFlyQueue"

# PATH 2: moveRelative
send_cmd "setUframe:0,2341,1300,-90,0,0"
send_cmd "moveLin:469.902265,0.148117,-0.001458,179.99707,90.001831,-0.000807"
send_cmd "moveLin:469.902305,0.148117,24.998035,179.99707,90.001831,-0.000807"
send_cmd "moveLin:367.902971,0.148117,24.998145,179.997069,90.001831,-0.000807"
send_cmd "moveLin:367.903554,0.148117,-25.005381,179.99707,90.001831,-0.000807"
send_cmd "moveLin:367.904155,20.148116,-25.005308,179.997069,90.001831,-0.000807"
send_cmd "moveLin:367.904569,20.148127,24.99461,179.997069,90.001841,-0.000807"
send_cmd "moveLin:367.90517,-19.851873,24.994702,179.99705,90.001841,-0.000814"
send_cmd "moveLin:367.905584,-19.851883,-25.005389,179.99705,90.001852,-0.000814"
send_cmd "moveLin:367.906184,0.148118,-25.005317,179.997051,90.001852,-0.000813"
send_cmd "moveLin:469.906598,0.148123,-25.005399,179.997051,90.001861,-0.000813"
send_cmd "moveLin:469.907014,0.148122,-0.009141,179.997051,90.001861,-0.000813"
send_cmd "moveLin:469.907679,0.148122,-0.009042,89.997051,90.001861,-0.000813"
send_cmd "moveLin:280.907879,0.14683,-0.00962,89.99706,90.001895,-0.000902"
send_cmd "setFlyCart:10,0,10"
send_cmd "clearFlyQueue"
send_cmd "addFlyLin:280.907969,-24.853182,-0.009635,89.99827,90.002156,89.999098"
send_cmd "addFlyLin:280.908059,-49.853194,-0.009649,89.999479,90.002417,179.999097"
send_cmd "addFlyLin:280.908149,-74.853205,-0.009664,90.000689,90.002678,269.999096"
send_cmd "addFlyLin:280.908239,-99.853217,-0.009678,90.001898,90.002939,359.999095"
send_cmd "addFlyLin:280.908239,-74.853217,-0.009678,90.001898,90.002939,269.999096"
send_cmd "addFlyLin:280.908239,-49.853217,-0.009678,90.001898,90.002939,179.999097"
send_cmd "addFlyLin:280.908239,-24.853217,-0.009678,90.001898,90.002939,89.999098"
send_cmd "addFlyLin:280.908239,0.146783,-0.009678,90.001898,90.002939,-0.000902"
send_cmd "executeFlyQueue"
send_cmd "moveLin:382.908554,0.146728,-0.009708,90.001898,90.002942,-0.000902"
send_cmd "moveLin:382.910847,0.148412,-0.011241,-179.998114,90.002914,-0.000902"

# Extra Cartesian FLY before PATH 3 in the YZ plane:
# right 30, down 60, left 60, up 60, right 30.
send_cmd "setFlyCart:10,0,10"
send_cmd "clearFlyQueue"
send_cmd "addFlyLin:382.910847,30.148412,-0.011241,-179.998114,90.002914,-0.000902"
send_cmd "addFlyLin:382.910847,30.148412,-60.011241,-179.998114,90.002914,-0.000902"
send_cmd "addFlyLin:382.910847,-29.851588,-60.011241,-179.998114,90.002914,-0.000902"
send_cmd "addFlyLin:382.910847,-29.851588,-0.011241,-179.998114,90.002914,-0.000902"
send_cmd "addFlyLin:382.910847,0.148412,-0.011241,-179.998114,90.002914,-0.000902"
send_cmd "executeFlyQueue"

# Two semicircles in the YZ plane, radius 40 mm, returning to the same start point.
send_cmd "setSpeedLin:0.05"
send_cmd "moveCircular:382.910847,40.148412,-40.011241,-179.998114,90.002914,-0.000902,382.910847,0.148412,-80.011241,-179.998114,90.002914,-0.000902"
send_cmd "setSpeedLin:0.02"
send_cmd "moveCircular:382.910847,-39.851588,-40.011241,-179.998114,90.002914,-0.000902,382.910847,0.148412,-0.011241,-179.998114,90.002914,-0.000902"

# PATH 3: moveJoint
send_cmd "moveJoint:-90,0,-80,0,0,0"
send_cmd "moveJoint:-30,0,-80,0,0,0"
send_cmd "moveJoint:-15,0,-80,0,0,0"

# PATH 4: moveLin
send_cmd "setUframe:0,2341,1300,-90,0,0"
send_cmd "moveLin:1803,2007,500,105,90,0"
