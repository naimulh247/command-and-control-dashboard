import React, { Component } from 'react';
import ros_config from '../../configs/ros_config';

class VideoFeed extends Component {
  render() {
    return (
      <iframe className='video-feed' title='robot-video-feed' src={`http://${this.props.ip}:${ros_config.VIDEO_WEB_SERVER_PORT}`}></iframe>
    );
  }
}

export default VideoFeed;