import React from 'react';

class RosTopicList extends React.Component {
    constructor(props) {
        super(props);
        this.state = { topics: [], showMenu: false };
    }

    componentDidMount() {
        this.getTopicList();
    }

    getTopicList() {
        const { ros } = this.props;
        // if ros is not intialized return
        if (!ros) {
        console.warn('ROS/ RosBridge not intialized');
        return;
        }

        const topicList = new window.ROSLIB.Topic({
        ros: ros,
        name: '/rostopic_list',
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

    toggleMenu = () => {
        this.setState((prevState) => ({ showMenu: !prevState.showMenu }));
    };

    render() {
        const { topics, showMenu } = this.state;

        return (
            <div className="showTopics">
                <button onClick={this.toggleMenu}>
                    {showMenu ? 'Hide Available Topics' : 'Show Available Topics'}
                </button>
                {showMenu && (
                <ul>
                    {topics.map((topic, i) => (
                    <li key={i}>{topic}</li>
                    ))}
                </ul>
                )}
            </div>
        );
    }
}

export default RosTopicList;
