import React from 'react';

import './WaypointPicker.css';

class WaypointPicker extends React.Component {

  sendSelectedWaypoint = () => {
    const selectedIndex = document.getElementById('waypoint-select').value;
    console.log(selectedIndex)
    const waypointPose = this.props.waypoints[selectedIndex].pose;
    console.log(waypointPose);
    this.props.onDeploy(waypointPose);
  };

  render() {
    const waypointOptions = this.props.waypoints ? (this.props.waypoints.map((waypoint, index) => (
      <option 
        value={index}
        key={index}
      >
        {waypoint.name}
      </option>
    ))) : (
      <option disabled={true}>
        Disconnected
      </option>
    )

    return (
      <div className='waypoint-picker'>
        <label htmlFor='waypoint-select'>Waypoints:</label>
        <select 
          id='waypoint-select'
        >
          {waypointOptions}
        </select>
        <br/>
        <button 
          onClick={this.sendSelectedWaypoint}
        >
          Deploy
        </button>
      </div>
    )
  }
}

export default WaypointPicker;
