#!/usr/bin/env python
import rospy
import rospkg
import os
from sensor_msgs.msg import LaserScan

def modify_sdf_file(model_name, ellipse_params):
    # Locate the package where Gazebo models are stored
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('my_truckie')  

    # Path to the SDF file of the model
    sdf_path = os.path.join(package_path, 'models', model_name, 'model.sdf')

    # Read the original SDF file content
    with open(sdf_path, 'r') as sdf_file:
        sdf_content = sdf_file.read()

    # Add the ellipse XML code to the SDF content
    ellipse_xml = f'''
      <ellipse>
        <radius>
          <x>{ellipse_params['x_radius']}</x>
          <y>{ellipse_params['y_radius']}</y>
          <z>{ellipse_params['z_radius']}</z>
        </radius>
        <height>{ellipse_params['height']}</height>
      </ellipse>
    '''

    modified_sdf_content = sdf_content.replace('</collision>', f'</collision>{ellipse_xml}')

    # Write the modified SDF content back to the file
    with open(sdf_path, 'w') as sdf_file:
        sdf_file.write(modified_sdf_content)

def callback(msg):
    # Check if an obstacle is detected based on your criteria
    threshold = 10  # Adjust the desired threshold
    if any(distance < threshold for distance in msg.ranges):
        print('Obstacle Ahead Detected')
        # Modify the SDF file of the model to include an enclosing ellipse
        ellipse_params = {
            'x_radius': 1.0,
            'y_radius': 0.5,
            'z_radius': 0.2,
            'height': 0.01
        }
        modify_sdf_file('your_model_name', ellipse_params)  # Replace 'your_model_name' with the model name

rospy.init_node('ellipse_drawer')
sub = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, callback)
rospy.spin()