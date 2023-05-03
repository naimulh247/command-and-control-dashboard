import React, { Component } from 'react';
import Footer from './components/main/Footer';
import Header from './components/main/Header';
import Body from './components/main/Body';
import ros_config from './configs/ros_config';

class App extends Component {
  constructor(props) {
    super(props);
    this.state = {  
      isDarkMode: localStorage.getItem('darkMode') !== null ? localStorage.getItem('darkMode') === "true" : ros_config.DARK_MODE
    } 
  }

  render() {
    return (
      <div className={this.state.isDarkMode ? 'dark-mode' : ''}>
        <Header />
          <Body />
        <Footer />
      </div>
    );
  }
}

export default App;
