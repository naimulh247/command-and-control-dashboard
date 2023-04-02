import React, { Component } from 'react';
import ROSConnect from '../ros-bridge/ROSConnect';
import Teleop from '../ros-bridge/Teleop';
import { Col, Container, Row } from 'react-bootstrap';

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
        console.log(ros);
    }

    render() { 
        return (
            <main>
                <Container>
                    <Row>
                        <Col>
                            <ROSConnect setRos={this.setRos}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col>
                            <Teleop ros={this.state.ros}/>
                        </Col>
                    </Row>
                </Container>
            </main>
        );
    }
}
 
export default Home;
