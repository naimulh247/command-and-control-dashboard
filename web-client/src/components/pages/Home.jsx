import React, { Component } from 'react';
import ROSConnect from '../ros-bridge/ROSConnect';
import Teleop from '../ros-bridge/Teleop';
import { Col, Container, Row } from 'react-bootstrap';
import RobotState from '../ros-bridge/RobotState';
import Map from '../ros-bridge/Map';
import VideoFeed from '../ros-bridge/VideoFeed';

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
                        <Col>
                            <ROSConnect setRos={this.setRos}/>
                            <VideoFeed ros={ros} />

                        </Col>
                    </Row>
                    <Row>
                        <Col>
                            <Teleop ros={ros}/>
                        </Col>
                        <Col>
                            <Map ros={ros}/>
                            {/* <VideoFeed ros={ros} /> */}
                        </Col>
                    </Row>
                    <Row>
                        <Col>
                            <RobotState ros={ros}/>
                        </Col>
                    </Row>
                </Container>
            </main>
        );
    }
}
 
export default Home;
