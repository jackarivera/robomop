// src/components/MapView.js

import React, { useState, useEffect, useRef, useCallback } from 'react';
import ControlPage from './ControlPage';
import '../styles/MapView.css';

function MapView() {
  // State variables for controls
  const [mapList, setMapList] = useState([]);
  const [selectedMap, setSelectedMap] = useState('');
  const [mapName, setMapName] = useState('');
  const [waypoints, setWaypoints] = useState([]);

  // Refs and variables for ROS and canvas
  const rosRef = useRef(null);
  const canvasRef = useRef(null);
  const mapDataRef = useRef(null);
  const robotPoseRef = useRef({ x: 0, y: 0, theta: 0 });
  const isMountedRef = useRef(true);
  const navigateToPoseClientRef = useRef(null);

  useEffect(() => {
    // Initialize ROS connection
    rosRef.current = new window.ROSLIB.Ros({
      url: 'ws://' + window.location.hostname + ':9090',
    });

    rosRef.current.on('connection', () => {
      console.log('Connected to ROS bridge');
      fetchMapList(); // Fetch the map list upon connection
    });

    rosRef.current.on('error', (error) => {
      console.error('Error connecting to ROS bridge:', error);
    });

    rosRef.current.on('close', () => {
      console.log('Connection to ROS bridge closed');
    });

    // Subscribe to map topic
    const mapListener = new window.ROSLIB.Topic({
      ros: rosRef.current,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
    });

    mapListener.subscribe((message) => {
      if (isMountedRef.current) {
        mapDataRef.current = message;
        drawMap();
      }
    });

    // Navigation Waypoints
    navigateToPoseClientRef.current = new window.ROSLIB.ActionClient({
      ros: rosRef.current,
      serverName: '/navigate_to_pose',
      actionName: 'nav2_msgs/action/NavigateToPose', // Update based on your Nav2 version
    });

    // Subscribe to robot pose in map frame
    const robotPoseMapListener = new window.ROSLIB.Topic({
      ros: rosRef.current,
      name: '/robot_pose_map',
      messageType: 'geometry_msgs/PoseStamped',
    });

    robotPoseMapListener.subscribe((message) => {
      if (isMountedRef.current) {
        const position = message.pose.position;
        const orientation = message.pose.orientation;
        const theta = quaternionToYaw(orientation);

        console.log('Received robot pose:', { x: position.x, y: position.y, theta });

        robotPoseRef.current = {
          x: position.x,
          y: position.y,
          theta: theta,
        };

        drawMap(); // Redraw map with updated robot position
      }
    });

    return () => {
      isMountedRef.current = false;
      mapListener.unsubscribe();
      robotPoseMapListener.unsubscribe(); // Unsubscribe from the new topic
      if (navigateToPoseClientRef.current){
        navigateToPoseClientRef = null;
      }
      if (rosRef.current) {
        rosRef.current.close();
      }
    };
  }, []);

  const fetchMapList = () => {
    console.log('Fetching map list...');
    const listMapsService = new window.ROSLIB.Service({
      ros: rosRef.current,
      name: '/list_maps',
      serviceType: 'robomop_navigation/ListMaps',
    });

    const request = new window.ROSLIB.ServiceRequest({});

    listMapsService.callService(
      request,
      (result) => {
        if (result.maps) {
          setMapList(result.maps);
          console.log('Map list fetched:', result.maps);
        } else {
          console.error('Failed to fetch map list.');
          alert('Failed to fetch map list.');
        }
      },
      (error) => {
        console.error('Service call failed:', error);
        alert('Service call failed while fetching map list.');
      }
    );
  };

  const handleCreateMap = () => {
    console.log('Attempting to call /create_map service');
    const createMapService = new window.ROSLIB.Service({
      ros: rosRef.current,
      name: '/create_map',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new window.ROSLIB.ServiceRequest({});

    createMapService.callService(
      request,
      (result) => {
        if (result.success) {
          console.log(result.message);
          alert('Mapping started.');
        } else {
          console.error('Failed to start mapping:', result.message);
          alert('Failed to start mapping: ' + result.message);
        }
      },
      (error) => {
        console.error('Service call failed:', error);
        alert('Service call failed while starting mapping.');
      }
    );
  };

  const handleSaveMap = () => {
    if (!mapName.trim()) {
      alert('Please enter a map name before saving.');
      return;
    }

    console.log(`Publishing map name: ${mapName}`);
    // Publish the map name to the /map_manager/map_name topic
    const mapNamePublisher = new window.ROSLIB.Topic({
      ros: rosRef.current,
      name: '/map_manager/map_name',
      messageType: 'std_msgs/String',
    });

    const mapNameMsg = new window.ROSLIB.Message({
      data: mapName.trim(),
    });

    mapNamePublisher.publish(mapNameMsg);
    console.log('Map name published.');

    // Now call the save_map service
    console.log('Attempting to call /save_map service');
    const saveMapService = new window.ROSLIB.Service({
      ros: rosRef.current,
      name: '/save_map',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new window.ROSLIB.ServiceRequest({});

    saveMapService.callService(
      request,
      (result) => {
        if (result.success) {
          console.log(result.message);
          alert('Map saved successfully.');
          fetchMapList(); // Refresh the map list
        } else {
          console.error('Failed to save map:', result.message);
          alert('Failed to save map: ' + result.message);
        }
      },
      (error) => {
        console.error('Service call failed:', error);
        alert('Service call failed while saving map.');
      }
    );
  };

  const handleStopMapping = () => {
    console.log('Attempting to call /stop_mapping service');
    const stopMappingService = new window.ROSLIB.Service({
      ros: rosRef.current,
      name: '/stop_mapping',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new window.ROSLIB.ServiceRequest({});

    stopMappingService.callService(
      request,
      (result) => {
        if (result.success) {
          console.log(result.message);
          alert('Mapping stopped.');
        } else {
          console.error('Failed to stop mapping:', result.message);
          alert('Failed to stop mapping: ' + result.message);
        }
      },
      (error) => {
        console.error('Service call failed:', error);
        alert('Service call failed while stopping mapping.');
      }
    );
  };

  const handleChooseMap = (event) => {
    const selected = event.target.value;
    setSelectedMap(selected);
    console.log(`Selected map: ${selected}`);
    console.log(`Publishing map name: ${selected}`);
    // Publish the map name to the /map_manager/map_name topic
    const mapNamePublisher = new window.ROSLIB.Topic({
      ros: rosRef.current,
      name: '/map_manager/map_name',
      messageType: 'std_msgs/String',
    });

    const mapNameMsg = new window.ROSLIB.Message({
      data: selected.trim(),
    });

    mapNamePublisher.publish(mapNameMsg);
    console.log('Map name published.');

    if (selected) {
      console.log('Attempting to call /load_map service');
      const loadMapService = new window.ROSLIB.Service({
        ros: rosRef.current,
        name: '/load_map',
        serviceType: 'std_srvs/Trigger',
      });

      const request = new window.ROSLIB.ServiceRequest({});

      loadMapService.callService(
        request,
        (result) => {
          if (result.success) {
            console.log(result.message);
            alert('Map loaded successfully.');
          } else {
            console.error('Failed to load map:', result.message);
            alert('Failed to load map: ' + result.message);
          }
        },
        (error) => {
          console.error('Service call failed:', error);
          alert('Service call failed while loading map.');
        }
      );
    }
  };

  // New Handler for Start Navigation Button
  const handleStartNavigation = () => {
    console.log('Attempting to call /start_navigation service');
    const startNavService = new window.ROSLIB.Service({
      ros: rosRef.current,
      name: '/start_navigation',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new window.ROSLIB.ServiceRequest({});

    startNavService.callService(
      request,
      (result) => {
        if (result.success) {
          console.log(result.message);
          alert('Navigation started successfully.');
        } else {
          console.error('Failed to start navigation:', result.message);
          alert('Failed to start navigation: ' + result.message);
        }
      },
      (error) => {
        console.error('Service call failed:', error);
        alert('Service call failed while starting navigation.');
      }
    );
  };
  // New Handler for Drawing Waypoints
  const handleCanvasClick = useCallback((event) => {
    if (!mapDataRef.current) {
      alert('Map data not available.');
      return;
    }

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const clickY = event.clientY - rect.top;

    // Convert canvas coordinates to map coordinates
    const mapPoint = canvasToMapCoordinates(clickX, clickY, mapDataRef.current.info);

    // Add the waypoint to the state
    setWaypoints((prevWaypoints) => [...prevWaypoints, mapPoint]);

    // Send the waypoint to Nav2
    sendNavigationGoal(mapPoint);
  }, [mapDataRef.current, robotPoseRef.current]);

  useEffect(() => {
    const canvas = canvasRef.current;
    canvas.addEventListener('click', handleCanvasClick);

    return () => {
      canvas.removeEventListener('click', handleCanvasClick);
    };
  }, [handleCanvasClick]);

  // Function to convert canvas pixel coordinates to map coordinates
  const canvasToMapCoordinates = (canvasX, canvasY, mapInfo) => {
    const { resolution, origin, width, height } = mapInfo;

    // Calculate map origin in map coordinates
    const originX = origin.position.x;
    const originY = origin.position.y;

    // Convert canvas coordinates to map coordinates
    const mapX = originX + (canvasX * resolution);
    const mapY = originY + ((height - canvasY) * resolution); // Flip Y-axis

    return { x: mapX, y: mapY, theta: 0 }; // theta will be set by Nav2
  };

  // Function to send navigation goal to Nav2
  const sendNavigationGoal = (mapPoint) => {
    if (!navigateToPoseClientRef.current) {
      alert('Navigation ActionClient not initialized.');
      return;
    }

    // Define the goal
    const goal = new window.ROSLIB.Goal({
      actionClient: navigateToPoseClientRef.current,
      goalMessage: {
        pose: {
          header: {
            frame_id: 'map',
            stamp: {
              sec: 0,
              nanosec: 0,
            },
          },
          pose: {
            position: { x: mapPoint.x, y: mapPoint.y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }, // Default orientation
          },
        },
      },
    });

    // Define callbacks for goal status
    goal.on('status', (status) => {
      console.log('Goal status:', status);
    });

    goal.on('result', (result) => {
      console.log('Navigation result:', result);
      alert('Navigation to waypoint completed.');
    });

    // Send the goal
    goal.send();

    console.log(`Navigation goal sent to (${mapPoint.x}, ${mapPoint.y})`);
  };

  const drawMap = () => {
    const canvas = canvasRef.current;
    const context = canvas.getContext('2d');

    if (!mapDataRef.current) return;

    const { width, height, resolution, origin } = mapDataRef.current.info;
    const mapData = mapDataRef.current.data;

    // Set canvas dimensions
    canvas.width = width;
    canvas.height = height;

    const imageData = context.createImageData(width, height);

    // Draw the map
    for (let i = 0; i < mapData.length; i++) {
      const value = mapData[i];
      const idx = i * 4;

      // Map data is in row-major order, starting from bottom left
      const y = Math.floor(i / width);
      const x = i % width;
      const canvasIdx = ((height - y - 1) * width + x) * 4; // Flip Y-axis

      // Set pixel color based on occupancy value
      if (value === -1) {
        // Unknown
        imageData.data[canvasIdx] = 128;
        imageData.data[canvasIdx + 1] = 128;
        imageData.data[canvasIdx + 2] = 128;
        imageData.data[canvasIdx + 3] = 255;
      } else if (value === 0) {
        // Free space
        imageData.data[canvasIdx] = 255;
        imageData.data[canvasIdx + 1] = 255;
        imageData.data[canvasIdx + 2] = 255;
        imageData.data[canvasIdx + 3] = 255;
      } else {
        // Occupied space
        imageData.data[canvasIdx] = 0;
        imageData.data[canvasIdx + 1] = 0;
        imageData.data[canvasIdx + 2] = 0;
        imageData.data[canvasIdx + 3] = 255;
      }
    }

    // Clear the canvas
    context.clearRect(0, 0, canvas.width, canvas.height);

    // Put map image data onto canvas
    context.putImageData(imageData, 0, 0);

    // Draw the robot position
    drawRobotPosition(context, width, height, resolution, origin);

    // Draw Waypoints
    drawWaypoints(context, width, height, resolution, origin);
  };

  const drawRobotPosition = (context, width, height, resolution, origin) => {
    const robotPose = robotPoseRef.current;
    console.log('Drawing robot position:', robotPose);

    // Convert robot coordinates to map pixels
    const mapX = (robotPose.x - origin.position.x) / resolution;
    const mapY = height - (robotPose.y - origin.position.y) / resolution;

    // Draw robot marker
    context.fillStyle = 'red';
    context.beginPath();
    context.arc(mapX, mapY, 5, 0, 2 * Math.PI);
    context.fill();

    // Draw heading indicator
    context.strokeStyle = 'red';
    context.beginPath();
    context.moveTo(mapX, mapY);
    const headingX = mapX + 10 * Math.cos(-robotPose.theta);
    const headingY = mapY + 10 * Math.sin(-robotPose.theta);
    context.lineTo(headingX, headingY);
    context.stroke();
  };
  const drawWaypoints = (context, width, height, resolution, origin) => {
    waypoints.forEach((wp, index) => {
      // Convert map coordinates to canvas pixels
      const canvasX = (wp.x - origin.position.x) / resolution;
      const canvasY = height - (wp.y - origin.position.y) / resolution;

      // Draw waypoint marker
      context.fillStyle = 'blue';
      context.beginPath();
      context.arc(canvasX, canvasY, 5, 0, 2 * Math.PI);
      context.fill();

      // Optionally, label the waypoint
      context.fillStyle = 'white';
      context.font = '12px Arial';
      context.fillText(`WP${index + 1}`, canvasX + 6, canvasY - 6);
    });
  };
  const quaternionToYaw = (orientation) => {
    const { x, y, z, w } = orientation;
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    return Math.atan2(siny_cosp, cosy_cosp);
  };

  return (
    <div className="map-view">
      <div className="controls-bar">
        <button onClick={handleCreateMap}>Create Map</button>

        <input
          type="text"
          placeholder="Enter Map Name"
          value={mapName}
          onChange={(e) => setMapName(e.target.value)}
        />

        <button onClick={handleSaveMap}>Save Map</button>

        <button onClick={handleStopMapping}>Stop Mapping</button>

        <select value={selectedMap} onChange={handleChooseMap}>
          <option value="">Choose Map</option>
          {mapList.map((map, index) => (
            <option key={index} value={map}>
              {map}
            </option>
          ))}
        </select>
      </div>
      <button onClick={handleStartNavigation}>Start Navigation</button>
      <button onClick={() => setWaypoints([])}>Clear Waypoints</button>
      <canvas ref={canvasRef} style={{ border: '1px solid #ccc' }} />
      <ControlPage ros={rosRef.current} />
    </div>
  );
}

export default MapView;
