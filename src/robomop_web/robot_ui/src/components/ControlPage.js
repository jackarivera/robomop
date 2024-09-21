import React, { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import '../styles/ControlPage.css';

function ControlPage() {
  const ros = useRef(null);
  const cmdVel = useRef(null);

  useEffect(() => {
    ros.current = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    cmdVel.current = new ROSLIB.Topic({
      ros: ros.current,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    return () => {
      ros.current.close();
    };
  }, []);

  const handleCommand = (linear, angular) => {
    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
    cmdVel.current.publish(twist);
  };

  return (
    <div className="control-page">
      {/* Live camera feed */}
      <div className="camera-feed">
        {/* Implement live feed */}
      </div>
      {/* Manual control buttons */}
      <div className="manual-controls">
        <button onMouseDown={() => handleCommand(0.5, 0)} onMouseUp={() => handleCommand(0, 0)}>Forward</button>
        <button onMouseDown={() => handleCommand(-0.5, 0)} onMouseUp={() => handleCommand(0, 0)}>Backward</button>
        <button onMouseDown={() => handleCommand(0, 0.5)} onMouseUp={() => handleCommand(0, 0)}>Left</button>
        <button onMouseDown={() => handleCommand(0, -0.5)} onMouseUp={() => handleCommand(0, 0)}>Right</button>
      </div>
    </div>
  );
}

export default ControlPage;
