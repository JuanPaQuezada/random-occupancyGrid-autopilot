import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from src.mazeMapping import Map 

class StochasticMapROS:
    def __init__(self):
        rospy.init_node('stochastic_map_node', anonymous=True)
        self.map_pub = rospy.Publisher('/stochastic_map', OccupancyGrid, queue_size=10, latch=True)
        self.wall_pub = rospy.Publisher('/maze_walls_3d', Marker, queue_size=10,latch=True)
        self.rate = rospy.Rate(1)  # 1 Hz
    def generate_and_publish_map(self,ambient_density):
        start_node = (0, 0)
        goal_node = (49, 49)
        resolution = 0.5
        wall_height = 1.0
        width = 50
        height = 50
        map_instance = Map(width=50, height=50, density_rate=ambient_density, hardness_rate=0.5, start=start_node, goal=goal_node, seed=42)
        grid = map_instance.generate_map()

        # Publish the generated map as a PoseStamped message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.width = 50
        grid_msg.info.height = 50

        #position
        grid_msg.info.origin.position.x = -12.5
        grid_msg.info.origin.position.y = -12.5
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0 #without rotation
        grid_msg.info.resolution = 0.5
        grid_msg.info.origin.orientation.w = 1.0

        # 3D 
        marker=Marker()
        marker.header.frame_id="map"
        marker.header.stamp=rospy.Time.now()
        marker.ns="stochastic_map"
        marker.id=0
        marker.type=Marker.CUBE_LIST
        marker.action=Marker.ADD
        marker.pose.orientation.w=1.0

        #size of the cube
        marker.scale.x=resolution
        marker.scale.y=resolution
        marker.scale.z=wall_height

        #color of the cube
        marker.color.r=0.2
        marker.color.g=0.6
        marker.color.b=0.9
        marker.color.a=0.9
        flat_data=[]
        for y in range(width):
            for x in range(height):
                cell=grid[y][x]
                if cell==float('inf'):
                    flat_data.append(100)  # Occupied
                    p=Point()
                    p.x=-12.5+(x+0.5)*resolution
                    p.y=-12.5+(y+0.5)*resolution
                    p.z=wall_height/2.0 
                    marker.points.append(p)
                else:
                    flat_data.append(0)  # Free
        grid_msg.data=flat_data
        #publish the map
        self.map_pub.publish(grid_msg)
        self.wall_pub.publish(marker)

    
    
if __name__ == '__main__':
    try:
        ros_node = StochasticMapROS()
        rospy.sleep(0.5)  # Wait for the publishers to initialize
        ambient_density = 0.3
        ros_node.generate_and_publish_map(ambient_density)
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

