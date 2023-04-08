import React, { Component } from 'react';
import ROSConnect from '../ros-bridge/ROSConnect';
import Teleop from '../ros-bridge/Teleop';
import { Col, Container, Row } from 'react-bootstrap';
import RobotState from '../ros-bridge/RobotState';
import Map from '../ros-bridge/Map';
import VideoFeed from '../ros-bridge/VideoFeed';
import ros_config from '../../configs/ros_config';

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
                    <Row>
                        <Col md={8}>
                            <VideoFeed ip={ros_config.ROSBRIDGE_SERVER_IP}/>
                        </Col>
                        <Col md={4}>
                            <Teleop ros={ros}/>
                        </Col>
                    </Row>
                    <Row style={{marginTop: "250px"}}>
                        <Col>
                            <RobotState ros={ros}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col>
                            <ROSConnect setRos={this.setRos}/>
                        </Col>
                    </Row>
                </Container>
            </main>
        );
    }
}
 
export default Home;
