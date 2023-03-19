import React from 'react';

const VideoFeed = (props) => (
    <iframe className='video-feed' title='robot-video-feed' src={`http://${props.ip}:8080/`}></iframe>
)
export default VideoFeed
