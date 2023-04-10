import React, { Component } from 'react';
import ROSConnect from '../ros-bridge/ROSConnect';
import Teleop from '../ros-bridge/Teleop';
import { Col, Container, Row } from 'react-bootstrap';
import RobotState from '../ros-bridge/RobotState';
import Map from '../ros-bridge/Map';
import VideoFeed from '../ros-bridge/VideoFeed';
import ros_config from '../../configs/ros_config';
import ManualTeleop from '../ros-bridge/ManualTeleop';
import RosTopicList from '../ros-bridge/RosTopicList';

class Home extends Component {
    constructor(props) {
        super(props);
        this.state = {
            ros: null
        }
        this.setRos = this.setRos.bind(this);
    }

    setRos(ros) {
        this.setState({ ros });
        // console.log(ros);
    }

    render() { 
        // console.log(this.state.ros)
        const {ros} = this.state
        return (
            <main>
                <Container>
                    <Row style={{marginTop: "2%"}}>
                        <Col md={8}>
                            <VideoFeed ip={ros_config.ROSBRIDGE_SERVER_IP}/>
                        </Col>
                        <Col md={4}>
                            <ROSConnect setRos={this.setRos}/>
                        </Col>
                        <Col style={{marginLeft: "79%"}}>
                            <Teleop ros={ros}/>
                        </Col>
                    </Row>
                    <Row style={{marginTop: "14%"}}>
                        <Col>
                            <RobotState ros={ros}/>
                            <RosTopicList ros={ros}/>
                        </Col>
                        <Col>
                            <ManualTeleop ros={ros}/>
                        </Col>
                    </Row>
                </Container>
            </main>
        );
    }
}
 
export default Home;
