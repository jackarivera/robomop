// HomePage.js

import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

function HomePage() {
  const [mapExists, setMapExists] = useState(false);
  const [mapsList, setMapsList] = useState([]);
  const [selectedMap, setSelectedMap] = useState('');
  const ros = new ROSLIB.Ros({
    url: 'ws://' + window.location.hostname + ':9090',
  });

  useEffect(() => {
    // Subscribe to map status
    const mapStatusListener = new ROSLIB.Topic({
      ros: ros,
      name: '/map_status',
      messageType: 'std_msgs/String',
    });

    mapStatusListener.subscribe((message) => {
      setMapExists(message.data === 'exists');
    });

    // Get list of maps (implement service or topic to provide this)
    // For now, assume a service called 'get_maps_list' that returns a list of map names

    const getMapsListClient = new ROSLIB.Service({
      ros: ros,
      name: '/get_maps_list',
      serviceType: 'your_custom_interface/GetMapsList',
    });

    const request = new ROSLIB.ServiceRequest({});
    getMapsListClient.callService(request, (result) => {
      setMapsList(result.maps);
    });

    return () => {
      mapStatusListener.unsubscribe();
    };
  }, [ros]);

  const createMap = () => {
    const createMapClient = new ROSLIB.Service({
      ros: ros,
      name: '/create_map',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new ROSLIB.ServiceRequest({});
    createMapClient.callService(request, (result) => {
      alert(result.message);
    });
  };

  const loadMap = () => {
    const loadMapClient = new ROSLIB.Service({
      ros: ros,
      name: '/load_map',
      serviceType: 'nav_msgs/LoadMap',
    });

    const request = new ROSLIB.ServiceRequest({
      map_url: selectedMap,
    });

    loadMapClient.callService(request, (result) => {
      alert('Map loaded successfully.');
    });
  };

  const startMopping = () => {
    const startMoppingClient = new ROSLIB.Service({
      ros: ros,
      name: '/start_mopping',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new ROSLIB.ServiceRequest({});
    startMoppingClient.callService(request, (result) => {
      alert(result.message);
    });
  };

  const homeRobot = () => {
    const homeRobotClient = new ROSLIB.Service({
      ros: ros,
      name: '/home_robot',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new ROSLIB.ServiceRequest({});
    homeRobotClient.callService(request, (result) => {
      alert(result.message);
    });
  };

  return (
    <div className="home-page">
      <h2>Robot Map Status</h2>
      {mapExists ? (
        <div>
          <p>A map exists.</p>
          <select
            value={selectedMap}
            onChange={(e) => setSelectedMap(e.target.value)}
          >
            <option value="">Select a map</option>
            {mapsList.map((map, index) => (
              <option key={index} value={map}>
                {map}
              </option>
            ))}
          </select>
          <button onClick={loadMap} disabled={!selectedMap}>
            Load Map
          </button>
          <button onClick={startMopping}>Start Mopping</button>
          <button onClick={homeRobot}>Home Robot</button>
        </div>
      ) : (
        <div>
          <p>No map found.</p>
          <button onClick={createMap}>Create Map</button>
        </div>
      )}
    </div>
  );
}

export default HomePage;
