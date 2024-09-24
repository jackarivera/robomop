import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import '../styles/ControlPage.css';
import { Joystick } from 'react-joystick-component';

function ControlPage() {
  const ros = useRef(null);
  const cmdVel = useRef(null);
  const [mopMotorOn, setMopMotorOn] = useState(false);
  const [pumpMotorOn, setPumpMotorOn] = useState(false);

  useEffect(() => {
    ros.current = new ROSLIB.Ros({
      url: 'ws://localhost:9090',
    });

    cmdVel.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    // Implement publishers for mop and pump motors
    const mopMotorPublisher = new ROSLIB.Topic({
      ros: ros.current,
      name: '/mop_motor',
      messageType: 'std_msgs/Bool',
    });

    const pumpMotorPublisher = new ROSLIB.Topic({
      ros: ros.current,
      name: '/pump_motor',
      messageType: 'std_msgs/Bool',
    });

    // Store publishers in refs to access them in handlers
    mopMotorPublisherRef.current = mopMotorPublisher;
    pumpMotorPublisherRef.current = pumpMotorPublisher;

    return () => {
      ros.current.close();
    };
  }, []);

  const mopMotorPublisherRef = useRef(null);
  const pumpMotorPublisherRef = useRef(null);

  const handleMove = (event) => {
    const twist = new ROSLIB.Message({
      linear: {
        x: event.y / 100,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: -event.x / 100,
      },
    });
    cmdVel.current.publish(twist);
  };

  const handleStop = () => {
    const twist = new ROSLIB.Message({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
    cmdVel.current.publish(twist);
  };

  const toggleMopMotor = () => {
    setMopMotorOn(!mopMotorOn);
    const message = new ROSLIB.Message({ data: !mopMotorOn });
    mopMotorPublisherRef.current.publish(message);
  };

  const togglePumpMotor = () => {
    setPumpMotorOn(!pumpMotorOn);
    const message = new ROSLIB.Message({ data: !pumpMotorOn });
    pumpMotorPublisherRef.current.publish(message);
  };

  return (
    <div className="control-page">
      <div className="camera-feed">
        {/* Implement live camera feed */}
        <img src="camera_placeholder.jpg" alt="Live Camera Feed" />
      </div>
      <div className="joystick-container">
        <Joystick
          size={100}
          baseColor="#1565c0"
          stickColor="#1e88e5"
          move={handleMove}
          stop={handleStop}
        />
      </div>
      <div className="control-buttons">
        <button onClick={toggleMopMotor}>
          {mopMotorOn ? 'Turn Mop Motor Off' : 'Turn Mop Motor On'}
        </button>
        <button onClick={togglePumpMotor}>
          {pumpMotorOn ? 'Turn Pump Motor Off' : 'Turn Pump Motor On'}
        </button>
      </div>
    </div>
  );
}

export default ControlPage;
