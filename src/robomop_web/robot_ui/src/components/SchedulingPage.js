import React, { useState, useEffect } from 'react';
import '../styles/SchedulingPage.css';
import axios from 'axios';

function SchedulingPage() {
  const [schedules, setSchedules] = useState([]);

  useEffect(() => {
    // Fetch schedules from backend
    fetchSchedules();
  }, []);

  const fetchSchedules = () => {
    axios
      .get('http://localhost:5000/api/schedules')
      .then((response) => {
        setSchedules(response.data);
      })
      .catch((error) => {
        console.error('Error fetching schedules:', error);
      });
  };

  const handleAddSchedule = () => {
    // Implement add schedule logic
    const time = prompt('Enter time (e.g., 08:00 AM):');
    const days = prompt('Enter days (e.g., Mon,Wed,Fri):').split(',');

    if (time && days.length > 0) {
      axios
        .post('http://localhost:5000/api/schedules', { time, days })
        .then((response) => {
          fetchSchedules();
        })
        .catch((error) => {
          console.error('Error adding schedule:', error);
        });
    }
  };

  const handleDeleteSchedule = (id) => {
    axios
      .delete(`http://localhost:5000/api/schedules/${id}`)
      .then((response) => {
        fetchSchedules();
      })
      .catch((error) => {
        console.error('Error deleting schedule:', error);
      });
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
