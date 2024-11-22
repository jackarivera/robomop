import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import Navbar from './components/Navbar';
import HomePage from './components/HomePage';
import MapView from './components/MapView';
import ControlPage from './components/ControlPage';
import SettingsPage from './components/SettingsPage';
import SchedulingPage from './components/SchedulingPage';

function App() {
  return (
    <Router>
      <Navbar />
      <Routes>
        <Route exact path="/" component={HomePage} />
        <Route path="/Map" element={<MapView />} />
        <Route path="/scheduling" element={<SchedulingPage />} />
        <Route path="/control" element={<ControlPage />} />
        <Route path="/settings" element={<SettingsPage />} />
      </Routes>
    </Router>
  );
}

export default App;
