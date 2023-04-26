import React, { Component } from 'react'

class VideoFeed extends Component {
    render() {
      return (
        <iframe className='video-feed' title='robot-video-feed' src={"http://localhost:8090/stream?topic=/camera/rgb/image_raw&width=320&height=240"}></iframe>
      );
    }
  }
  
export default VideoFeed;