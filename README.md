# FARMBOT NAVIGATION
## sample Action call 
```
ros2 action send_goal /navigation farmbot_interfaces/action/Nav "{
  initial_path: {
    header: {
      frame_id: 'some_frame'
    },
    poses: [
      {
        header: {
          stamp: {sec: 0, nanosec: 0},
          frame_id: 'some_frame'
        },
        pose: {
          position: { x: 10.0, y: 0.0, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
      }
    ]
  }
}"
```