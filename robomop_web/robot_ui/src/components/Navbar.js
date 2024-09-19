import React from 'react';
import { Link } from 'react-router-dom';
import '../styles/Navbar.css';

function Navbar() {
  return (
    <nav className="navbar">
      <div className="navbar-logo">RoboMop</div>
      <input type="checkbox" id="navbar-toggle" className="navbar-toggle" />
      <label htmlFor="navbar-toggle" className="navbar-hamburger">
        ☰
      </label>
      <ul className="navbar-links">
        <li>
          <Link to="/">Map</Link>
        </li>
        <li>
          <Link to="/control">Control</Link>
        </li>
        <li>
          <Link to="/settings">Settings</Link>
        </li>
        <li>
          <Link to="/scheduling">Scheduling</Link>
        </li>
      </ul>
    </nav>
  );
}

export default Navbar;
