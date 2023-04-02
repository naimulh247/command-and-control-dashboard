import React, { Component } from 'react';
import { Container } from 'react-bootstrap';
import { Joystick } from 'react-joystick-component'
class Teleop extends Component {
    
    constructor(props) {
        super(props)
        // const { ros } = this.ros
        // bind the this key word to make sure we have access to ros connection from ROSConnect
        this.handleMove = this.handleMove.bind(this);

        // console.log(this.props);
    }

    componentDidMount() {
        // check if the ros mounted from ROSConnect
        // const { ros } = this.props
        // console.log(this.props);
    }

    handleMove() {
        const { ros } = this.props
        console.log(ros);
        // is ros is not intialized return
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized")
            return
        }

        const cmd_vel = new window.ROSLIB.Topic({
            ros: ros,
            name: '/turtle1/cmd_vel',
            messageType: 'geometry_msgs/Twist',
        });

        const twist = new window.ROSLIB.Message({
            linear: {
                x: 0.5,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0.2,
                y: 0,
                z: 0,
            },
        });

        cmd_vel.publish(twist)

    }

    handleStop() {

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