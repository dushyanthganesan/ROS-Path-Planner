/*
Deprecated publisher
Publish goal point from RViz or from terminal
From terminal, use: 
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}'
*/