import React, { useState, useEffect } from 'react';
import '../styles/SettingsPage.css';
import axios from 'axios';

function SettingsPage() {
  const [settings, setSettings] = useState({
    cleaning_mode: 'Standard',
    volume: 50,
    notifications: true,
  });

  useEffect(() => {
    // Fetch settings from backend
    axios
      .get('http://localhost:5000/api/settings')
      .then((response) => {
        setSettings(response.data);
      })
      .catch((error) => {
        console.error('Error fetching settings:', error);
      });
  }, []);

  const handleSave = () => {
    // Save settings to backend
    axios
      .post('http://localhost:5000/api/settings', settings)
      .then((response) => {
        alert('Settings saved!');
      })
      .catch((error) => {
        console.error('Error saving settings:', error);
      });
  };

  return (
    <div className="settings-page">
      <h2>Settings</h2>
      <div className="settings-section">
        <h3>Cleaning Modes</h3>
        <select
          value={settings.cleaning_mode}
          onChange={(e) =>
            setSettings({ ...settings, cleaning_mode: e.target.value })
          }
        >
          <option value="Standard">Standard</option>
          <option value="Eco">Eco</option>
          <option value="Turbo">Turbo</option>
        </select>
      </div>
      <div className="settings-section">
        <h3>System Preferences</h3>
        <label>
          Volume:
          <input
            type="range"
            min="0"
            max="100"
            value={settings.volume}
            onChange={(e) =>
              setSettings({ ...settings, volume: parseInt(e.target.value) })
            }
          />
        </label>
        <label>
          Notifications:
          <input
            type="checkbox"
            checked={settings.notifications}
            onChange={(e) =>
              setSettings({ ...settings, notifications: e.target.checked })
            }
          />
        </label>
      </div>
      <button className="save-button" onClick={handleSave}>
        Save Settings
      </button>
    </div>
  );
}

export default SettingsPage;
