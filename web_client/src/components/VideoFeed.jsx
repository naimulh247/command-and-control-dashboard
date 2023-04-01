import React from 'react';

const VideoFeed = (props) => (
    <iframe className='video-feed' title='robot-video-feed' src={`http://${props.ip}:8090/stream_viewer?topic=/camera/rgb/image_raw`}></iframe>
)
export default VideoFeed
