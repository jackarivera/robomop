# robomop_driver/map_manager.py

import os
import subprocess
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, Trigger_Response
from robomop_navigation.srv import ListMaps  # Ensure this service is correctly defined
from std_msgs.msg import String
from slam_toolbox.srv import SerializePoseGraph
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
from rclpy.duration import Duration

class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')

        self.get_logger().info('Initializing MapManager node...')

        # Directory to store maps
        self.map_directory = '/home/robomop/robot-maps'  # Update this path as needed

        # Ensure the map directory exists
        os.makedirs(self.map_directory, exist_ok=True)
        self.get_logger().info(f'Map directory set to: {self.map_directory}')

        # Initialize map name
        self.current_map_name = None

        # Create services
        self.create_service(Trigger, 'create_map', self.create_map_callback)
        self.create_service(Trigger, 'save_map', self.save_map_callback)
        self.create_service(Trigger, 'stop_mapping', self.stop_mapping_callback)
        self.create_service(ListMaps, 'list_maps', self.list_maps_callback)
        self.create_service(Trigger, 'load_map', self.load_map_callback)
        self.create_service(Trigger, 'start_navigation', self.start_navigation_callback)
        self.get_logger().info('Services created: /create_map, /save_map, /stop_mapping, /list_maps, /load_map, /start_navigation')

        # Create subscriber for map name
        self.map_name_subscriber = self.create_subscription(
            String,
            '/map_manager/map_name',
            self.map_name_callback,
            10
        )
        self.get_logger().info('Subscribed to /map_manager/map_name for receiving map names.')

        # Initialize process handles
        self.mapping_process = None
        self.localization_process = None
        self.navigation_process = None

        # TF2 Buffer and Listener with increased cache_time
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))  # Increased buffer duration
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for robot pose in map frame
        self.robot_pose_map_publisher = self.create_publisher(PoseStamped, '/robot_pose_map', 10)

        # Subscribe to odometry
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Subscribed to /pose and initialized TF2 listener.')

        # Create client for slam_toolbox's save_map service
        self.serialize_map_client = self.create_client(SerializePoseGraph, 'slam_toolbox/serialize_map')
        self.get_logger().info('Created client for slam_toolbox/serialize_map service.')


    def map_name_callback(self, msg):
        self.current_map_name = msg.data
        self.get_logger().info(f'Received map name: {self.current_map_name}')

    def create_map_callback(self, request, response):
        if self.mapping_process is not None and self.mapping_process.poll() is None:
            response.success = False
            response.message = 'Mapping is already running.'
            self.get_logger().warn('Create Map request failed: Mapping is already running.')
            return response

        self.get_logger().info('Create Map service called. Starting mapping process...')

        # Stop localization if running
        if self.localization_process is not None and self.localization_process.poll() is None:
            self.get_logger().info('Stopping existing localization process before starting mapping...')
            self.terminate_process('localization')

        # Start the slam_toolbox mapping process
        try:
            self.mapping_process = subprocess.Popen(
                ['ros2', 'launch', 'robomop_navigation', 'mapping.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            response.success = True
            response.message = 'Mapping process started successfully.'
            self.get_logger().info('Mapping process started successfully.')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start mapping process: {str(e)}'
            self.get_logger().error(f'Failed to start mapping process: {str(e)}')

        return response

    def save_map_callback(self, request, response):
        if not self.current_map_name:
            response.success = False
            response.message = 'Map name not set. Please set the map name before saving.'
            self.get_logger().warn('Save Map request failed: Map name not set.')
            return response

        # Check if a map with the same name already exists
        map_path_yaml = os.path.join(self.map_directory, self.current_map_name + '.yaml')
        map_path_pgm = os.path.join(self.map_directory, self.current_map_name + '.pgm')
        if os.path.exists(map_path_yaml) or os.path.exists(map_path_pgm):
            response.success = False
            response.message = f'Map "{self.current_map_name}" already exists.'
            self.get_logger().warn(f'Save Map request failed: Map "{self.current_map_name}" already exists.')
            return response

        self.get_logger().info(f'Saving map: {self.current_map_name}')

        # Get the namespace
        namespace_str = self.get_namespace()
        set_namespace = f" -r __ns:={namespace_str}" if namespace_str else ""

        if self.current_map_name != "":
            command = f"ros2 run nav2_map_server map_saver_cli -f {self.current_map_name} --ros-args -p map_subscribe_transient_local:=true{set_namespace}"
            self.get_logger().info(f"SlamToolbox: Saving map as {self.current_map_name}. Command: {command}")
        else:
            command = f"ros2 run nav2_map_server map_saver_cli --ros-args -p map_subscribe_transient_local:=true{set_namespace}"
            self.get_logger().info("SlamToolbox: Saving map in current directory.")

        try:
            # Execute the command
            rc = subprocess.call(command, shell=True, cwd=self.map_directory)
            if rc == 0:
                response.success = True
                response.message = f'Map "{self.current_map_name}" saved successfully.'
                self.get_logger().info(f'Map "{self.current_map_name}" saved successfully.')
                # Create client for slam_toolbox's deserialize_map service

                if not self.serialize_map_client.wait_for_service(timeout_sec=1.0):
                    response.success = False
                    response.message = 'slam_toolbox/serialize_map service not available.'
                    self.get_logger().warn('Load Map request failed: slam_toolbox/serialize_map service not available.')
                    return response

                # Create the request
                map_request = String()
                map_request.data = os.path.join(self.map_directory, self.current_map_name)
                serialize_map_request = SerializePoseGraph.Request()
                serialize_map_request.filename = os.path.join(self.map_directory, self.current_map_name)

                try:
                    # Call the service asynchronously with a 5-second timeout
                    future = self.serialize_map_client.call_async(serialize_map_request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                    if future.result() is None:
                        response.success = future.result().success
                        response.message = future.result().message
                        if response.success:
                            self.get_logger().info(f'Map "{self.current_map_name}" serialized successfully.')
                        else:
                            self.get_logger().warn(f'Failed to serialize map "{self.current_map_name}": {response.message}')
                    else:
                        response.success = False
                        response.message = f'Service call to slam_toolbox/serialize_map failed or timed out: {future.result()}'
                        self.get_logger().error('Service call to slam_toolbox/serialize_map failed or timed out.')
                except Exception as e:
                    response.success = False
                    response.message = f'Exception during serialize_map service call: {str(e)}'
                    self.get_logger().error(f'Exception during serialize_map service call: {str(e)}')
            else:
                response.success = False
                response.message = f'Failed to save map "{self.current_map_name}". Return code: {rc}'
                self.get_logger().error(f'Failed to save map "{self.current_map_name}". Return code: {rc}')
                
        except Exception as e:
            response.success = False
            response.message = f'Exception during map saving: {str(e)}'
            self.get_logger().error(f'Exception during map saving: {str(e)}')

        return response


    def stop_mapping_callback(self, request, response):
        if self.mapping_process is None:
            response.success = False
            response.message = 'Mapping is not running.'
            self.get_logger().warn('Stop Mapping request failed: Mapping is not running.')
            return response

        if self.mapping_process.poll() is not None:
            response.success = False
            response.message = 'Mapping process is already terminated.'
            self.get_logger().warn('Stop Mapping request failed: Mapping process is already terminated.')
            return response

        self.get_logger().info('Stop Mapping service called. Terminating mapping process...')

        try:
            self.mapping_process.terminate()
            self.mapping_process.wait(timeout=5)
            self.get_logger().info('Mapping process terminated successfully.')
            response.success = True
            response.message = 'Mapping process terminated successfully.'
            self.mapping_process = None
        except subprocess.TimeoutExpired:
            self.get_logger().error('Failed to terminate mapping process within timeout.')
            response.success = False
            response.message = 'Failed to terminate mapping process within timeout.'
        except Exception as e:
            self.get_logger().error(f'Exception while terminating mapping process: {str(e)}')
            response.success = False
            response.message = f'Exception while terminating mapping process: {str(e)}'

        return response

    def list_maps_callback(self, request, response):
        self.get_logger().info('List Maps service called.')
        try:
            # List all .yaml files in the map directory
            yaml_files = [f for f in os.listdir(self.map_directory) if f.endswith('.yaml')]
            map_names = [os.path.splitext(f)[0] for f in yaml_files]
            response.maps = map_names
            self.get_logger().info(f'Available maps: {map_names}')
        except Exception as e:
            response.maps = []
            self.get_logger().error(f'Failed to list maps: {str(e)}')
        return response

    def load_map_callback(self, request, response):
        if not self.current_map_name:
            response.success = False
            response.message = 'Map name not set. Please set the map name before loading.'
            self.get_logger().warn('Load Map request failed: Map name not set.')
            return response

        map_path_yaml = os.path.join(self.map_directory, self.current_map_name + '.yaml')
        map_path_pgm = os.path.join(self.map_directory, self.current_map_name + '.pgm')
        if not os.path.exists(map_path_yaml) or not os.path.exists(map_path_pgm):
            response.success = False
            response.message = f'Map "{self.current_map_name}" does not exist.'
            self.get_logger().warn(f'Load Map request failed: Map "{self.current_map_name}" does not exist.')
            return response

        self.get_logger().info(f'Received request to load map: {self.current_map_name}')

        # Stop mapping and localization if running
        if self.mapping_process is not None and self.mapping_process.poll() is None:
            self.get_logger().info('Stopping existing mapping process before starting localization...')
            self.terminate_process('mapping')

        if self.localization_process is not None and self.localization_process.poll() is None:
            self.get_logger().info('Stopping existing localization process before loading a new map...')
            self.terminate_process('localization')

        try:
            # Example: Start localization with the selected map
            command = [
                'ros2', 'launch', 'robomop_navigation', 'localize.launch.py',
                f'map_file_name:={os.path.join(self.map_directory, self.current_map_name)}'
            ]
            self.localization_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            response.success = True
            response.message = f'Map "{self.current_map_name}" loaded and localization started successfully.'
            self.get_logger().info(f'Map "{self.current_map_name}" loaded and localization started successfully.')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start localization: {str(e)}'
            self.get_logger().error(f'Failed to start localization: {str(e)}')

        return response



    def pose_callback(self, msg):
        try:
            # Create a PoseStamped message from odometry
            pose_odom = PoseStamped()
            pose_odom.header = msg.header
            pose_odom.pose = msg.pose.pose

            # Publish the transformed pose
            self.robot_pose_map_publisher.publish(pose_odom)
            self.get_logger().debug(f'Published robot pose in map frame: {pose_odom.pose}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get pose from topic /pose: {e}')

    def start_navigation_callback(self, request, response):
        if self.navigation_process is not None and self.navigation_process.poll() is None:
            response.success = False
            response.message = 'Navigation is already running.'
            self.get_logger().warn('Start Navigation request failed: Navigation is already running.')
            return response

        self.get_logger().info('Start Navigation service called. Launching navigation process...')
        if not self.current_map_name:
            response.success = False
            response.message = 'No map loaded. Please load a map before starting navigation.'
            self.get_logger().warn('Start Navigation request failed: No map loaded.')
            return response

        # Construct the path to the selected map's YAML file
        map_path_yaml = os.path.join(self.map_directory, self.current_map_name + '.yaml')
        if not os.path.exists(map_path_yaml):
            response.success = False
            response.message = f'Map file "{map_path_yaml}" does not exist.'
            self.get_logger().error(f'Start Navigation request failed: Map file "{map_path_yaml}" does not exist.')
            return response
        # Optionally, stop mapping and localization if they are running
        # if self.mapping_process is not None and self.mapping_process.poll() is None:
        #     self.get_logger().info('Stopping existing mapping process before starting navigation...')
        #     self.terminate_process('mapping')

        # if self.localization_process is not None and self.localization_process.poll() is None:
        #     self.get_logger().info('Stopping existing localization process before starting navigation...')
        #     self.terminate_process('localization')

        # Launch the navigation.launch.py file
        try:
            self.navigation_process = subprocess.Popen(
                ['ros2', 'launch', 'robomop_navigation', 'navigation.launch.py', f'map:={map_path_yaml}', 'use_sim_time:=false'],  # Update the package and launch file name as needed
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            response.success = True
            response.message = 'Navigation started successfully.'
            self.get_logger().info('Navigation process started successfully.')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start navigation: {str(e)}'
            self.get_logger().error(f'Failed to start navigation: {str(e)}')

        return response

    def terminate_process(self, process_type):
        """
        Terminates the specified process (mapping or localization).
        """
        process_attr = f"{process_type}_process"
        process = getattr(self, process_attr, None)

        if process is not None and process.poll() is None:
            self.get_logger().info(f'Terminating {process_type} process...')
            try:
                process.terminate()
                process.wait(timeout=5)
                self.get_logger().info(f'{process_type.capitalize()} process terminated successfully.')
                setattr(self, process_attr, None)
            except subprocess.TimeoutExpired:
                self.get_logger().error(f'Failed to terminate {process_type} process within timeout.')
            except Exception as e:
                self.get_logger().error(f'Exception while terminating {process_type} process: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    map_manager = MapManager()

    try:
        rclpy.spin(map_manager)
    except KeyboardInterrupt:
        map_manager.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Ensure that any running subprocesses are terminated on shutdown
        if map_manager.mapping_process:
            map_manager.get_logger().info('Terminating mapping process...')
            try:
                map_manager.mapping_process.terminate()
                map_manager.mapping_process.wait(timeout=5)
                map_manager.get_logger().info('Mapping process terminated.')
            except subprocess.TimeoutExpired:
                map_manager.get_logger().error('Failed to terminate mapping process within timeout.')
            except Exception as e:
                map_manager.get_logger().error(f'Exception while terminating mapping process: {str(e)}')

        # If you have a localization process, handle it similarly
        if map_manager.localization_process:
            map_manager.get_logger().info('Terminating localization process...')
            try:
                map_manager.localization_process.terminate()
                map_manager.localization_process.wait(timeout=5)
                map_manager.get_logger().info('Localization process terminated.')
            except subprocess.TimeoutExpired:
                map_manager.get_logger().error('Failed to terminate localization process within timeout.')
            except Exception as e:
                map_manager.get_logger().error(f'Exception while terminating localization process: {str(e)}')

        map_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
