import React from 'react';
import ros_config from '../../configs/ros_config';

const VideoFeed = (props) => (
    <iframe className='video-feed' title='robot-video-feed' src={`http://${props.ip}:${ros_config.VIDEO_WEB_SERVER_PORT}`}></iframe>
)
export default VideoFeed