import React, { Component } from 'react';
import ros_config from '../../configs/ros_config';

class BatteryIndicator extends Component {
    constructor(props) {
        super(props);

        this.state = {
            batteryPercentage: 0,
            batteryColor: '#00ff00',
        };

        this.updateBatteryState = this.updateBatteryState.bind(this);
    }

    componentDidMount() {
        this.updateBatteryState();
    }

    componentDidUpdate(prevProps) {
        if (this.props.ros && prevProps.ros !== this.props.ros) {
            this.updateBatteryState()
        }
    }

    updateBatteryState = () => {
        const { ros } = this.props;

        if (!ros) {
            console.warn('ROS/ RosBridge not initialized: Battery State');
            return;
        }
        else{
            console.warn('Battery State initialized')
        }

        // Subscribe to the /battery_state topic
        const batteryStateListener = new window.ROSLIB.Topic({
            ros: ros,
            name: `${localStorage.getItem('batteryTopic') || ros_config.ROSBRIDGE_BATTERY_TOPIC}`,
            messageType: 'sensor_msgs/BatteryState',
            throttle_rate: 5000
        });
        
        batteryStateListener.subscribe((message) => {
            // Extract the battery percentage from the message
            const batteryPercentage = Math.min(Math.round(message.percentage * 100), 100);

            // Update the component state with the new battery percentage
            this.setState({ batteryPercentage });

            // Update the battery color based on the battery percentage
            let batteryColor = '#00ff00'; // Default color (green)

            if (batteryPercentage <= 20) {
                batteryColor = 'red'; // Change to red if battery percentage is below 20
            } else if (batteryPercentage > 20 && batteryPercentage <= 30) {
                batteryColor = 'yellow'; // Change to yellow if battery percentage is between 20 and 30
            }

            this.setState({ batteryColor });
        }); 
    };

    render() {
        const { batteryPercentage, batteryColor } = this.state;

        return (
            <div style={{ display: 'flex', justifyContent: 'row', paddingBottom: '4%' }}>
                <div style={{ paddingRight: '2%' }}>Robot Battery: {batteryPercentage}%</div>
                <div>
                    <div
                        style={{
                            border: '1px solid #ccc',
                            borderRadius: '5px',
                            width: '80px',
                            height: '30px',
                        }}
                    >
                        <div
                            style={{
                                backgroundColor: batteryColor,
                                height: '100%',
                                width: `${batteryPercentage}%`,
                            }}
                        />
                    </div>
                </div>
            </div>
        );
    }
}

export default BatteryIndicator;
