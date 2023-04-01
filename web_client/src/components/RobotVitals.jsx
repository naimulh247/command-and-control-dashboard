import React from 'react';
import ProgressBar from 'react-progressbar';

import './RobotVitals.css'

class RobotVitals extends React.Component {
  render() {
    const laptopCharge = this.props.laptopCharge
    return(
      <div className='robot-vitals'>
        <p className='robot-vitals-title'>Robot Info</p>
        <p className='robot-vitals-state'><span className='text-bold'>STATUS:</span> Waiting for task</p>
        <p className='progressbar-title'><span className='text-bold'>LAPTOP POWER:</span> {laptopCharge}%</p>
        <ProgressBar
          completed={laptopCharge}
          color={'red'}
        />
      </div>
    )
  }
}

export default RobotVitals;
