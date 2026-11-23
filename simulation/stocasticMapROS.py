import os
import sys


ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from src.mazeMapping import Map 
from src.pathfinding_engine import a_star, visualize_path
import tf2_ros

class StochasticMapROS:
    def __init__(self):
        rospy.init_node('stochastic_map_node', anonymous=True)
        self.map_pub = rospy.Publisher('/stochastic_map', OccupancyGrid, queue_size=10, latch=True)
        self.wall_pub = rospy.Publisher('/maze_walls_3d', Marker, queue_size=10,latch=True)
        self.path_pub=rospy.Publisher('/planned_path', Path, queue_size=10, latch=True) 
        self.tf_broadcaster=tf2_ros.StaticTransformBroadcaster()
        static_tf=TransformStamped()
        static_tf.header.stamp=rospy.Time.now()
        static_tf.header.frame_id="map"
        static_tf.child_frame_id="odom"
        static_tf.transform.translation.x=-12.25
        static_tf.transform.translation.y=-12.25
        static_tf.transform.translation.z=0.0
        static_tf.transform.rotation.w=1.0
        self.tf_broadcaster.sendTransform(static_tf)
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

        route, _ = self.path_finding(grid, start_node, goal_node)
        self.publish_path(route, resolution)
    def path_finding(self, grid, start, goal):
        route = a_star(grid, start, goal)
        show_route = visualize_path(cost_map=grid, path=route, start=start, goal=goal)
        return route, show_route
    def publish_path(self, route, resolution):
        if not route:
            rospy.logwarn("No path found from start to goal.")
            return
        path_msg=Path()
        path_msg.header.stamp=rospy.Time.now()
        path_msg.header.frame_id="map"
        for (x,y) in route:
            pose=PoseStamped()
            pose.header.stamp=rospy.Time.now()
            pose.header.frame_id="map"
            pose.pose.position.x=-12.5+(x+0.5)*resolution
            pose.pose.position.y=-12.5+(y+0.5)*resolution
            pose.pose.position.z=0.05
            pose.pose.orientation.w=1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published with {} waypoints.".format(len(route)))


    
    
if __name__ == '__main__':
    try:
        ros_node = StochasticMapROS()
        rospy.sleep(0.5)  # Wait for the publishers to initialize
        ambient_density = 0.3
        ros_node.generate_and_publish_map(ambient_density)
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

