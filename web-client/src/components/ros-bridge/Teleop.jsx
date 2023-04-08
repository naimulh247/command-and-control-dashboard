import React, { Component } from 'react';
import { Container } from 'react-bootstrap';
import { Joystick } from 'react-joystick-component'
import ros_config from '../../configs/ros_config';
class Teleop extends Component {
    
    constructor(props) {
        super(props)
        // const { ros } = this.ros
        // bind the this key word to make sure we have access to ros connection from ROSConnect
        this.handleMove = this.handleMove.bind(this);
        this.handleStop = this.handleStop.bind(this);


        // console.log(this.props);
    }

    componentDidMount() {
        // check if the ros mounted from ROSConnect
        // const { ros } = this.props
        // console.log(this.props);
    }

    handleMove(event) {
        const { ros } = this.props
        // console.log(ros);
        // console.log(event);
        // is ros is not intialized return
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized")
            return
        }
        // console.log(ros_config.ROSBRIDGE_CMD_VEL, event);
        const cmd_vel = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_CMD_VEL}`,
            messageType: 'geometry_msgs/Twist',
        });

        const twist = new window.ROSLIB.Message({
            linear: {
                x: event.y ,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0,
                y: 0,
                z: -event.x,
            },
        });

        cmd_vel.publish(twist)

    }

    handleStop(event) {
        console.log(event);

        // handleStop takes care of drifting issue, where the robot moves eventhough the joystick stopped
        const { ros } = this.props
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized")
            return
        }

        const cmd_vel = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_CMD_VEL}`,
            messageType: 'geometry_msgs/Twist',
        });

        const twist = new window.ROSLIB.Message({
            linear: {
                x: 0,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0,
                y: 0,
                z: 0,
            },
        });
        console.log(twist);
        cmd_vel.publish(twist)

    }
    render() {
        return (
            <Container>
                <Joystick 
                    size={100} 
                    baseColor="#EEEEEE" 
                    stickColor="#BBBBBB" 
                    move={this.handleMove} 
                    stop={this.handleStop}
                ></Joystick>
            </Container>
        );
    }
}

export default Teleop;