import React, { Component } from 'react'
import { Container } from 'react-bootstrap';
import ros_config from '../../configs/ros_config';

class Footer extends Component {
    state = {
        isDarkMode: localStorage.getItem('darkMode') !== null ? localStorage.getItem('darkMode') === "true" : ros_config.DARK_MODE
    }

    render() { 
        const { isDarkMode } = this.state;
        const navbarClassName = isDarkMode ? 'footer-dark-style' : 'footer-style';

        return (
            <Container className={`text-center ${navbarClassName}`}>
                <p className='footer-p-style'>Designed by Naimul Hasan, James Kong, Brandon Lacy &copy; 2023</p>
            </Container>
        );
    }
}
 
export default Footer;