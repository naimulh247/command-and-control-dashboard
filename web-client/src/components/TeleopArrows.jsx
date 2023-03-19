import React from 'react';
import { Row, Col } from 'reactstrap';
import KeyListener from './KeyListener.jsx';

class TeleopArrows extends React.Component {

  state = {
    activeArrow: null
  }

  handleMouseUp = () => {
    this.setState({ activeArrow: null })
  }

  handleStopButtonDownPress = () => {
    this.props.onStopButtonPress();
    this.setState({activeArrow: 'stop'})
  }

  handleUpArrowDownPress = () => {
    this.props.onLinearArrowClick(0.1);
    this.setState({activeArrow: 'up'})
  }

  handleDownArrowDownPress = () => {
    this.props.onLinearArrowClick(-0.1);
    this.setState({activeArrow: 'down'})
  }

  handleRightArrowDownPress = () => {
    this.props.onAngularArrowClick(-0.1);
    this.setState({activeArrow: 'right'})
  }

  handleLeftArrowDownPress = () => {
    this.props.onAngularArrowClick(0.1);
    this.setState({activeArrow: 'left'})
  }

  render() {
    return (
        <div className='teleop-arrows'>
          <Row className='justify-content-center'>
            <Col xs='4' className='center-text'>
              <a
                onMouseDown={this.handleUpArrowDownPress}
                onMouseUp={this.handleMouseUp}
              >
                <img
                  className={this.state.activeArrow === 'up' ? 'active-arrow' : 'inactive-arrow'}
                  src='images/UpArrow.png'
                  alt='Up Arrow'
                />
              </a>
            </Col>
          </Row>
          <Row className='justify-content-center'>
            <Col xs='4' className='center-text'>
              <a
                onMouseDown={this.handleLeftArrowDownPress}
                onMouseUp={this.handleMouseUp}
              >
                <img
                  className={this.state.activeArrow === 'left' ? 'active-arrow' : 'inactive-arrow'}
                  src='images/LeftArrow.png'
                  alt='Left Arrow'
                />
              </a>
            </Col>
            <Col xs='4' className='center-text'>
              <a
                onMouseDown={this.handleStopButtonDownPress}
                onMouseUp={this.handleMouseUp}
              >
                <img
                  className={this.state.activeArrow === 'stop' ? 'active-arrow' : 'inactive-arrow'}
                  src='images/StopButton.png'
                  alt='Stop Button'
                />
              </a>
            </Col>
            <Col xs='4' className='center-text'>
              <a
                onMouseDown={this.handleRightArrowDownPress}
                onMouseUp={this.handleMouseUp}
              >
                <img
                  className={this.state.activeArrow === 'right' ? 'active-arrow' : 'inactive-arrow'}
                  src='images/RightArrow.png'
                  alt='Right Arrow'
                />
              </a>
            </Col>
          </Row>
          <Row className='justify-content-center'>
            <Col xs='4' className='center-text'>
              <a
                onMouseDown={this.handleDownArrowDownPress}
                onMouseUp={this.handleMouseUp}
              >
                <img
                  className={this.state.activeArrow === 'down' ? 'active-arrow' : 'inactive-arrow'}
                  src='images/DownArrow.png'
                  alt='Down Arrow'></img>
              </a>
            </Col>
          </Row>
          <KeyListener
            onUpKeyDownPress = {this.handleUpArrowDownPress}
            onDownKeyDownPress = {this.handleDownArrowDownPress}
            onLeftKeyDownPress = {this.handleLeftArrowDownPress}
            onRightKeyDownPress = {this.handleRightArrowDownPress}
            onStopKeyDownPress = {this.handleStopButtonDownPress}
            onKeyUp = {this.handleMouseUp}
          />
        </div>
    )
  }
}

export default TeleopArrows;
