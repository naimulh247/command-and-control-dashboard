import React, { Component } from 'react';
import { Container } from 'react-bootstrap';
import ros_config from '../../configs/ros_config';

class VideoFeed extends Component {

    constructor(props) {
        super(props);
        this.canvasRef = React.createRef();
        this.getVideoFeed = this.getVideoFeed.bind(this);

        this.state = {
          frameWidth: localStorage.getItem('frameWidth') || ros_config.ROSBRIDGE_FRAME_WIDTH,
          frameHeight: localStorage.getItem('frameHeight') || ros_config.ROSBRIDGE_FRAME_HEIGHT,
          videoFeedDetected: false,
          isDarkMode: localStorage.getItem('darkMode') !== null ? localStorage.getItem('darkMode') === "true" : ros_config.DARK_MODE
        };
    }

    componentDidMount() {
        this.getVideoFeed();
        this.imageConfigInterval = setInterval(() => {
          this.pubImageConfigs();
        }, ros_config.CHECK_IMAGE_CONFIG);
    }

    componentDidUpdate( ) {
      this.getVideoFeed()
    }

    componentWillUnmount() {
      clearInterval(this.imageConfigInterval);
    }
    
    //This function publishes image width and height configurations as a ROS topic. It checks if the ROS connection is initialized and 
    //creates a new ROS topic to publish the configurations for the image compress Python node to subscribe to
    pubImageConfigs() {
      const { ros } = this.props;

      if (!ros) {
        console.warn("ROS/ RosBridge not intitialized: VideoFeed");
        return;
      }

      // publish imageWidth and imageHeight as a rostopic
      const imageConfig_publisher = new window.ROSLIB.Topic({
          ros: ros,
          name: `${ros_config.ROSBRIDGE_IMAGE_CONFIGS}`,
          messageType: 'std_msgs/String'
      });

      const imageWidth = localStorage.getItem('imageWidth') || ros_config.ROSBRIDGE_IMAGE_WIDTH;
      const imageHeight = localStorage.getItem('imageHeight') || ros_config.ROSBRIDGE_IMAGE_HEIGHT;
      const message = new window.ROSLIB.Message({data: `${imageWidth},${imageHeight}`});

      imageConfig_publisher.publish(message);
    }

    //This function subscribes to a ROS topic that provides a compressed video feed. It checks if the ROS connection is 
    //initialized and creates a new ROS topic subscriber. When a new message is received, it updates the canvas element with the video feed. If no video feed is detected
    //it changes the videoFeedDetected status to show a placeholder
    getVideoFeed() {
      const { ros } = this.props;
      const canvas = this.canvasRef.current;

      if (!ros) {
        console.warn("ROS/ RosBridge not intitialized: VideoFeed");
        return;
      }
      console.log(ros);

      // create a new video_subscriber with the topics
      const video_subscriber = new window.ROSLIB.Topic({
          ros: ros,
          name: `${ros_config.ROSBRIDGE_RASPICAM_TOPIC}`,
          messageType: 'sensor_msgs/CompressedImage',
      })

      // if there is a message update the canvas with the video feed
      video_subscriber.subscribe((message) => {
        const data = message.data;
        const frame = new Image();
      
        frame.onload = () => {
          // draw the image on the canvas
          const ctx = canvas.getContext('2d');
          ctx.drawImage(frame, 0, 0, canvas.width, canvas.height);
        };
      
        if (!data || data.length === 0) {
          if (this.state.videoFeedDetected) {
            this.setState({ videoFeedDetected: false });
          }
        } 
        else {
          if (!this.state.videoFeedDetected) {
            this.setState({ videoFeedDetected: true });
          }
          frame.src = 'data:image/jpeg;base64,' + data;
        }
      });
      
    }

    render() {
      const { videoFeedDetected, frameWidth, frameHeight, isDarkMode } = this.state;
      const placeHolderBG = isDarkMode ? 'BG-dark' : 'placeHolder-light';
      console.log(placeHolderBG)
      return (
        <Container className="d-flex justify-content-center align-items-center" style={{ paddingBottom: '1.5%' }}>
          <canvas
            className="center"
            ref={this.canvasRef}
            width={frameWidth}
            height={frameHeight}
            style={{ display: videoFeedDetected ? 'block' : 'none' }} // Added style to hide/show canvas
          ></canvas>
          {!videoFeedDetected && (
            <div
              className={placeHolderBG}
              style={{
                width: 640,
                height: 242,
                margin: 'auto',
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center',
              }}
            >
              <div style={{ textAlign: 'center' }}>
                <p>No video feed detected! Please ensure:</p>
                <ul style={{ textAlign: 'left' }}>
                  <li>Rosbridge is properly running and connected with the web client.</li>
                  <li>A camera node (such as /camera or /raspicam) is running in ROS (you should be able to check these within settings for current rostopics).</li>
                  <li>The correct topic is listed within the subscriber in the VideoFeed component (you can just swap the config values or create your own if it's not already listed within configs).</li>
                </ul>
              </div>
            </div>
          )}
        </Container>
      );
    }
    
}

export default VideoFeed;
