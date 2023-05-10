import React, { Component } from 'react';
import { Col, Row } from 'react-bootstrap';
import * as Three from 'three'
import ros_config from '../../configs/ros_config';

class RobotState extends Component {

    constructor(props) {
        super(props)
        // bind `this` to the function 
        this.getRobotState = this.getRobotState.bind(this)
    }
    
    state = { 
        x: 0,
        y: 0,
        z: 0,
        orientation: 0,
        linear_velocity: 0,
        angular_velocity: 0,
    }

    componentDidMount ()  {
        // known issue: getRobotState() is being called asynchronously after the component mounts, 
        // and the ros variable might not have been passed down to the component at the time when the function is called.
        this.getRobotState()
    }

    componentDidUpdate(prevProps) {
        // fix for the known issue: componentDidUpdate() checks if the ros variable has been updated, 
        // and calls getRobotState() again if it has
        if (this.props.ros && prevProps.ros !== this.props.ros) {
            this.getRobotState()
        }
    }

    // a helper function to get the orientation of the robot
    getOrientation(quaternion) {
        const q = new Three.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        // get the roll, pitch, and yaw
        const rpy = new Three.Euler().setFromQuaternion(q)
        // console.log(rpy);
        // return in yaw in radians (z)
        return rpy["_z"]
    }

    getRobotState () {
        const { ros } = this.props 
        // console.log(this.props);
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized")
            return
        }
        // this will be used later for map using amcl topic
        // const pose_subscriber = new window.ROSLIB.Topic({
        //     ros: ros,
        //     name: "/amcl_pose",
        //     messageType: "geometry_msgs/PoseWithCovarianceStamped",
        // })

        // pose_subscriber.subscribe((message) => {
        //     this.setState({x: message.pose.pose.position.x, y: message.pose.pose.position.y})
        // })

        // get position and velocity using odom
        const odom_subscriber = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_ODOM}`,
            messageType: "nav_msgs/Odometry",
        });
    
        odom_subscriber.subscribe((message) => {
            this.setState({
                x: message.pose.pose.position.x.toFixed(2),
                y: message.pose.pose.position.y.toFixed(2),
                z: message.pose.pose.position.z.toFixed(2),
                orientation: this.getOrientation(message.pose.pose.orientation).toFixed(2),
                linear_velocity: message.twist.twist.linear.x.toFixed(2),
                angular_velocity: message.twist.twist.angular.z.toFixed(2),
            });
        });
         // subscribe to the gps data being published from the app through ros
        //  const gpsSubscriber = new window.ROSLIB.Topic({
        //     ros: ros,
        //     name: '/gps',
        //     messageType: 'std_msgs/String',
        //     queue_size: 1,
        //   });
      
        //   gpsSubscriber.subscribe(message => {
        //       // console.log(message)
        //       const data = JSON.parse(message.data);
        //         this.setState({
        //             x: data.longitude,
        //             y: data.latitude
        //         })
        //   })
    }
    
    render() { 
        return (
            <Row>
                <Col>
                    <h4 className='mt-4'>Positions</h4>
                    <p className='mt-0'>x: {this.state.x} </p>
                    <p className='mt-0'>y: {this.state.y} </p>
                    {/* <p className='mt-0'>z: {this.state.z} </p> */}
                    {/* <p className='mt-0'>Orientation: {this.state.orientation} </p> */}
                </Col>
                <Col>
                    <h4 className='mt-4'>Velocities</h4>
                    <p className='mt-0'>Linear velocity: {this.state.linear_velocity} </p>
                    <p className='mt-0'>Angular velocity: {this.state.angular_velocity} </p>
                </Col>
            </Row>
        );
    }
}
 
export default RobotState;