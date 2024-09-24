import React from 'react';
import { Link, useLocation } from 'react-router-dom';
import '../styles/Navbar.css';
import { FaMap, FaGamepad, FaCogs, FaCalendarAlt } from 'react-icons/fa';

function Navbar() {
  const location = useLocation();

  return (
    <nav className="navbar">
      <div className="navbar-logo">Robot Mop</div>
      <input type="checkbox" id="navbar-toggle" className="navbar-toggle" />
      <label htmlFor="navbar-toggle" className="navbar-hamburger">
        ☰
      </label>
      <ul className="navbar-links">
        <li className={location.pathname === '/' ? 'active' : ''}>
          <Link to="/">
            <FaMap /> Map
          </Link>
        </li>
        <li className={location.pathname === '/control' ? 'active' : ''}>
          <Link to="/control">
            <FaGamepad /> Control
          </Link>
        </li>
        <li className={location.pathname === '/settings' ? 'active' : ''}>
          <Link to="/settings">
            <FaCogs /> Settings
          </Link>
        </li>
        <li className={location.pathname === '/scheduling' ? 'active' : ''}>
          <Link to="/scheduling">
            <FaCalendarAlt /> Scheduling
          </Link>
        </li>
      </ul>
    </nav>
  );
}

export default Navbar;
