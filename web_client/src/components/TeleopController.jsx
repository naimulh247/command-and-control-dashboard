import React from 'react';
import {Row, Col} from 'reactstrap';
import TeleopArrows from './TeleopArrows.jsx';
import TeleopData from './TeleopData.jsx';
import VideoFeed from './VideoFeed.jsx';
import WaypointPicker from './WaypointPicker.jsx';

class TeleopController extends React.Component {
  render() {
    return (
      <Row className='md-margin-top'>
        <Col xs='8'>
          <VideoFeed 
            ip={this.props.ip}
          />
        </Col>
        <Col xs='4'>
          <TeleopArrows
            onAngularArrowClick={this.props.onAngularVelocityChange}
            onLinearArrowClick={this.props.onLinearVelocityChange}
            onStopButtonPress={this.props.onStopButtonPress}
            angularVelocity={this.props.angularVelocity}
            linearVelocity={this.props.linearVelocity}
          />
        </Col>
        <Col xs='8'>
          <WaypointPicker
            waypoints={this.props.waypoints}
            onDeploy={this.props.onDeploy}
          />
        </Col>
        <Col xs='4'>
          <TeleopData
            angularVelocity={this.props.angularVelocity}
            linearVelocity={this.props.linearVelocity}
          />
        </Col>
      </Row>
    );
  }
}

export default TeleopController;