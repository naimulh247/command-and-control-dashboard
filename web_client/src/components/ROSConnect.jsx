import React from 'react';
import { validateNum } from '../lib/helper';
import { Row, Col } from 'reactstrap';

class ROSConnect extends React.Component {

  state = {
    ros: this.props.ros,
    ipAddress: 0,
    port: 0,
  }

  handleInputChange = (event) => {
    const target = event.target;
    const value = target.value;
    const name = target.name;
    this.setState({
      [name]: value
    });
  }

  handleSubmit = (event) => {
    event.preventDefault();
    if (this.props.connected) {
      this.props.onDisconnect();
      return;
    }
    var ipAddress = this.state.ipAddress;
    var port = this.state.port;
    if (this.validateIpAndPort(ipAddress, port)) {
      this.props.onConnect(ipAddress, port);
    } else {
      alert('Invalid IP Address / Port Number!');
    }
  }

  validateIpAndPort = (ip, port) => {
    var ipAddress = ip.split(".");
    return validateNum(port, 1, 65535) &&
        ((ipAddress.length === 1 && ipAddress[0] === 'localhost') ||
        (ipAddress.length === 4 && ipAddress.every((segment) => {
            return validateNum(segment, 0, 255);
        })));
  }

  render() {
    return(
      <div className='ros-connect'>
        <Row>
          <Col xs='12'>
            <h1 className='text-center'>
              ROS Status:
              <span className={this.props.connected ? 'green-text' : 'red-text'}>
                {this.props.connected ? " ● Connected" : " ● Disconnected"}
              </span>
            </h1>
          </Col>
          <Col xs='12'>
            <form className='ros-connection-form' onSubmit={this.handleSubmit}>
              <label>
                IP Address:
                <input
                  disabled={this.props.connected}
                  name='ipAddress'
                  onChange={this.handleInputChange}
                  type='text'
                />
              </label>
              <label>
                Port:
                <input
                  disabled={this.props.connected}
                  name='port'
                  onChange={this.handleInputChange}
                  type='text'
                />
              </label>
              <input type='submit' value={this.props.connected ? 'Disconnect' : 'Connect'} />
            </form>
          </Col>
        </Row>
      </div>
    )
  }
}

export default ROSConnect;
