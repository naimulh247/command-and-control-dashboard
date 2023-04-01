import React from 'react';
import { Row, Col } from 'reactstrap';

class TeleopData extends React.Component {
  render() {
    return (
      <div className='teleop-data'>
        <Row>
          <Col>
            <table align='center' cellPadding='5'>
              <tbody>
                <tr>
                  <td className='data-title'>Linear Velocity:</td>
                  <td>{this.props.linearVelocity}</td>
                </tr>
                <tr>
                  <td className='data-title'>Angular Velocity:</td>
                  <td>{this.props.angularVelocity}</td>
                </tr>
              </tbody>
            </table>
          </Col>
        </Row>
      </div>
    )
  }
}

export default TeleopData;
