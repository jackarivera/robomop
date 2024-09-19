import React, { useState } from 'react';
import '../styles/SettingsPage.css';

function SettingsPage() {
  const [cleaningMode, setCleaningMode] = useState('Standard');
  const [preferences, setPreferences] = useState({
    volume: 50,
    notifications: true,
  });

  const handleSave = () => {
    // Implement save logic
    alert('Settings saved!');
  };

  return (
    <div className="settings-page">
      <h2>Settings</h2>
      <div className="settings-section">
        <h3>Cleaning Modes</h3>
        <select
          value={cleaningMode}
          onChange={(e) => setCleaningMode(e.target.value)}
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
            value={preferences.volume}
            onChange={(e) =>
              setPreferences({ ...preferences, volume: e.target.value })
            }
          />
        </label>
        <label>
          Notifications:
          <input
            type="checkbox"
            checked={preferences.notifications}
            onChange={(e) =>
              setPreferences({ ...preferences, notifications: e.target.checked })
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
