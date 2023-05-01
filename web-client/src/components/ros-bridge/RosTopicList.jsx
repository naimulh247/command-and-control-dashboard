import React from 'react';
import { Dropdown } from 'react-bootstrap';
import ros_config from '../../configs/ros_config';

class RosTopicList extends React.Component {
  constructor(props) {
    super(props);
    this.state = { topics: [], showMenu: false };
    this.toggleMenu = this.toggleMenu.bind(this);
  }

  componentDidMount() {
    this.getTopicList();
  }

  //subscribe to rostopiclist topic to get a list of the current rostopics being published
  getTopicList() {
    const { ros } = this.props;
    // if ros is not intialized return
    if (!ros) {
      console.warn('ROS/ RosBridge not intialized');
      return;
    }

    const topicList = new window.ROSLIB.Topic({
      ros: ros,
      name: `${ros_config.ROSBRIDGE_ROSTOPICS_LIST}`,
      messageType: 'std_msgs/String',
    });

    topicList.subscribe((message) => {
      const topics = message.data.split(',');
      this.setState({ topics });
    });
  }

  componentDidUpdate(prevProps) {
    if (this.props.ros && prevProps.ros !== this.props.ros) {
      this.getTopicList()
    }
  }

  //change status of menu state to open up the menu to display the ros topics
  toggleMenu(isOpen) {
    this.setState({ showMenu: isOpen });
  }

  render() {
    const { topics, showMenu } = this.state;

    return (
      <Dropdown onToggle={this.toggleMenu} show={showMenu}>
        <Dropdown.Toggle id="dropdown-basic-button" variant="info">
          {showMenu ? 'Hide Available Topics' : 'Show Available Topics'}
        </Dropdown.Toggle>
        <Dropdown.Menu className="scrollable-menu">
          {topics.length === 0 ? (
            <Dropdown.Item>No current ROS topics being published!</Dropdown.Item>
          ) : (
            topics.map((topic, i) => (
              <Dropdown.Item key={i}>{topic}</Dropdown.Item>
            ))
          )}
        </Dropdown.Menu>
      </Dropdown>
    );
  }
}

export default RosTopicList;
