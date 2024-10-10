import React, { useEffect, useRef, useState, useCallback } from 'react';
import ROSLIB from 'roslib';
import { Joystick } from 'react-joystick-component';
import '../styles/ControlPage.css';

function ControlPage() {
  const ros = useRef(null);
  const cmdVel = useRef(null);
  const serialSubscriber = useRef(null);
  const publishInterval = useRef(null);

  // Use refs for control inputs to avoid re-renders
  const linearXRef = useRef(0);
  const angularZRef = useRef(0);
  const brushMotorPercentRef = useRef(0);
  const pumpMotorOnRef = useRef(false);

  // State variables for UI elements
  const [brushMotorPercent, setBrushMotorPercent] = useState(0);
  const [pumpMotorOn, setPumpMotorOn] = useState(false);

  const [serialLogs, setSerialLogs] = useState(''); // For /serial_in messages

  // Function to publish cmd_vel message
  const publishCmdVel = useCallback(() => {
    if (cmdVel.current) {
      // Clamp values to their respective ranges
      const clampedLinearX = Math.max(-100, Math.min(100, linearXRef.current));
      const clampedAngularZ = Math.max(-500, Math.min(500, angularZRef.current));
      const clampedLinearY = Math.max(-50, Math.min(50, brushMotorPercentRef.current));

      const twist = new ROSLIB.Message({
        linear: {
          x: clampedLinearX,
          y: clampedLinearY,
          z: 0,
        },
        angular: {
          x: pumpMotorOnRef.current ? 1 : 0,
          y: 0,
          z: clampedAngularZ,
        },
      });

      // Publish the message
      cmdVel.current.publish(twist);
    }
  }, []);

  // Initialize ROS connection and set up subscriptions
  useEffect(() => {
    // Use window.location.hostname to allow access from other devices
    ros.current = new ROSLIB.Ros({
      url: 'ws://' + window.location.hostname + ':9090',
    });

    ros.current.on('connection', () => {
      console.log('Connected to ROS bridge');
    });

    ros.current.on('error', (error) => {
      console.error('Error connecting to ROS bridge:', error);
    });

    ros.current.on('close', () => {
      console.log('Connection to ROS bridge closed');
    });

    cmdVel.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    // Subscribe to /serial_in topic
    serialSubscriber.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/serial_in',
      messageType: 'std_msgs/String',
    });

    serialSubscriber.current.subscribe((message) => {
      if (message.data.startsWith('RESP|')) {
        setSerialLogs(message.data);
      }
    });

    // Start continuous publishing to prevent timeouts
    publishInterval.current = setInterval(() => {
      publishCmdVel();
    }, 100); // Publish every 100ms (10Hz)

    // Clean up on unmount
    return () => {
      // Publish zero velocities to stop the robot
      if (cmdVel.current) {
        const twist = new ROSLIB.Message({
          linear: {
            x: 0,
            y: 0,
            z: 0,
          },
          angular: {
            x: 0,
            y: 0,
            z: 0,
          },
        });
        cmdVel.current.publish(twist);
      }

      // Stop publishing
      if (publishInterval.current) {
        clearInterval(publishInterval.current);
        publishInterval.current = null;
      }

      // Unsubscribe from /serial_in
      if (serialSubscriber.current) {
        serialSubscriber.current.unsubscribe();
      }

      // Close ROS connection
      if (ros.current) {
        ros.current.close();
      }
    };
  }, []);

  // Handle joystick movement
  const handleJoystickMove = (event) => {
    // Joystick inputs range from -100 to 100
    const xInput = event.y; // Up/down
    const zInput = event.x; // Left/right

    // Map joystick inputs to desired velocity ranges
    const mappedLinearX = xInput * 1; // User does not want scaling changed
    const mappedAngularZ = zInput * 5; // User does not want scaling changed

    // Update refs
    linearXRef.current = mappedLinearX;
    angularZRef.current = mappedAngularZ;

    // Publish immediately upon input change
    publishCmdVel();
  };

  // Handle joystick stop
  const handleJoystickStop = () => {
    // Stop movement
    linearXRef.current = 0;
    angularZRef.current = 0;

    // Publish immediately upon input change
    publishCmdVel();
  };

  // Handle brush motor slider change
  const handleBrushMotorChange = (event) => {
    const value = parseInt(event.target.value, 10);
    setBrushMotorPercent(value);
    brushMotorPercentRef.current = value;

    // Publish immediately upon input change
    publishCmdVel();
  };

  // Handle pump motor toggle
  const handlePumpMotorToggle = () => {
    setPumpMotorOn((prev) => {
      const newValue = !prev;
      pumpMotorOnRef.current = newValue;

      // Publish immediately upon input change
      publishCmdVel();

      return newValue;
    });
  };

  return (
    <div className="control-page">
      <h2>Manual Control</h2>

      <div className="joystick-container">
        <Joystick
          size={100}
          baseColor="#cceeff"
          stickColor="#0066cc"
          move={handleJoystickMove}
          stop={handleJoystickStop}
        />
      </div>

      <div className="control-elements">
        <div className="slider-container">
          <label htmlFor="brush-motor-slider">
            Brush Motor Percent ({brushMotorPercent}%)
          </label>
          <input
            type="range"
            id="brush-motor-slider"
            min="-50"
            max="50"
            value={brushMotorPercent}
            onChange={handleBrushMotorChange}
          />
        </div>

        <div className="toggle-container">
          <label htmlFor="pump-motor-toggle">Pump Motor</label>
          <button
            id="pump-motor-toggle"
            className={`toggle-button ${pumpMotorOn ? 'on' : 'off'}`}
            onClick={handlePumpMotorToggle}
          >
            {pumpMotorOn ? 'On' : 'Off'}
          </button>
        </div>
      </div>

      <div className="serial-logs-container">
        <label>Serial Logs:</label>
        <textarea
          value={serialLogs}
          readOnly
          rows={4}  // Increased number of rows to make the textbox larger
          cols={50}
          style={{ resize: 'none', fontSize: '1em' }}
        ></textarea>
      </div>
    </div>
  );
}

export default ControlPage;
