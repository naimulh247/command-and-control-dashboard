import React from 'react';

class KeyListener extends React.Component {

  render() {
    return null;
  }

  handleKeyPress = (event) => {
    // Return if a form is currently selected
    if (event.target.type === 'text') {
      return
    }

    switch(event.key) {
      case 'w':
        this.props.onUpKeyDownPress();
        break;
      case 'a':
        this.props.onLeftKeyDownPress();
        break;
      case 'd':
        this.props.onRightKeyDownPress();
        break;
      case 'x':
        this.props.onDownKeyDownPress();
        break;
      case 's':
        this.props.onStopKeyDownPress();
        break;
      default: break;
    }
  }

  componentDidMount() {
    window.addEventListener('keydown', this.handleKeyPress, false);
    window.addEventListener('keyup', this.props.onKeyUp, false);
  }

  componentWillUnmount() {
    window.removeEventListener('keydown', this.handleKeyPress, false);
    window.removeEventListener('keyup', this.props.onKeyUp, false);
  }

}

export default KeyListener;
