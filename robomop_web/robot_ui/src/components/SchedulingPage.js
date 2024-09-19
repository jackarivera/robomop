import React, { useState } from 'react';
import '../styles/SchedulingPage.css';

function SchedulingPage() {
  const [schedules, setSchedules] = useState([
    { id: 1, time: '08:00 AM', days: ['Mon', 'Wed', 'Fri'] },
    { id: 2, time: '10:00 PM', days: ['Sat'] },
  ]);

  const handleAddSchedule = () => {
    // Implement add schedule logic
    alert('Add Schedule functionality not implemented.');
  };

  const handleDeleteSchedule = (id) => {
    setSchedules(schedules.filter((schedule) => schedule.id !== id));
  };

  return (
    <div className="scheduling-page">
      <h2>Scheduling</h2>
      <button className="add-button" onClick={handleAddSchedule}>
        Add Schedule
      </button>
      <div className="schedule-list">
        {schedules.map((schedule) => (
          <div key={schedule.id} className="schedule-item">
            <div>
              <strong>Time:</strong> {schedule.time}
            </div>
            <div>
              <strong>Days:</strong> {schedule.days.join(', ')}
            </div>
            <button
              className="delete-button"
              onClick={() => handleDeleteSchedule(schedule.id)}
            >
              Delete
            </button>
          </div>
        ))}
      </div>
    </div>
  );
}

export default SchedulingPage;
