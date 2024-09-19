import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import Navbar from './components/Navbar';
import MapView from './components/MapView';
import ControlPage from './components/ControlPage';
import SettingsPage from './components/SettingsPage';
import SchedulingPage from './components/SchedulingPage';

function App() {
  return (
    <Router>
      <Navbar />
      <Routes>
        <Route path="/" element={<MapView />} />
        <Route path="/control" element={<ControlPage />} />
        <Route path="/settings" element={<SettingsPage />} />
        <Route path="/scheduling" element={<SchedulingPage />} />
      </Routes>
    </Router>
  );
}

export default App;
