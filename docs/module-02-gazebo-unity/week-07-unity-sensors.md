---
title: "Week 7: Unity & Sensor Simulation"
description: "Integrate Unity for photorealistic rendering and simulate LiDAR, depth cameras, and IMUs for humanoid robot perception."
sidebar_position: 3
week_id: "week-07-unity-sensors"
week_number: 7
module: "module-02-gazebo-unity"
learning_objectives:
  - "Set up Unity for robotics simulation"
  - "Simulate depth cameras and RGB-D sensors"
  - "Implement LiDAR point cloud generation"
  - "Model IMU sensor data with realistic noise"
prerequisites:
  - "Week 6: Gazebo Fundamentals"
estimated_time: "4 hours"
difficulty: "intermediate"
topics_covered:
  - "Unity Robotics Hub"
  - "Depth camera simulation"
  - "LiDAR sensors"
  - "IMU noise models"
  - "Sensor data publishing to ROS 2"
hands_on_exercises:
  - title: "Simulate Depth Camera in Unity"
    description: "Create a Unity scene with depth camera and publish to ROS 2"
    estimated_time: "60 minutes"
    tools_required: ["Unity 2022.3 LTS", "Unity Robotics Hub", "ROS 2 Humble"]
keywords:
  - "Unity"
  - "depth camera"
  - "LiDAR"
  - "IMU"
  - "sensor simulation"
references:
  - title: "Unity Robotics Hub"
    url: "https://github.com/Unity-Technologies/Unity-Robotics-Hub"
---

# Week 7: Unity & Sensor Simulation

Unity provides photorealistic rendering and advanced sensor simulation for humanoid robots. This week focuses on simulating vision, LiDAR, and inertial sensors with accurate noise models.

## Unity for Robotics

**Unity Robotics Hub** integrates Unity with ROS 2:
- **URDF Importer**: Load robot models from ROS
- **ROS TCP Connector**: Bidirectional ROS 2 ↔ Unity communication
- **Sensor plugins**: Cameras, LiDAR, IMU

### Setup Unity Robotics

1. Install Unity 2022.3 LTS
2. Create new project (URP template)
3. Add Unity Robotics Hub package:
```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

## Depth Camera Simulation

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class DepthCameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    Camera depthCamera;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("/camera/depth/image_raw");

        depthCamera = GetComponent<Camera>();
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
    }

    void PublishDepthImage()
    {
        // Render depth buffer to texture
        RenderTexture depthTexture = RenderToTexture(depthCamera);

        // Convert to ROS message
        ImageMsg msg = new ImageMsg
        {
            header = new HeaderMsg { stamp = GetROSTime(), frame_id = "camera_depth" },
            height = (uint)depthTexture.height,
            width = (uint)depthTexture.width,
            encoding = "32FC1",
            data = ConvertTextureToBytes(depthTexture)
        };

        ros.Publish("/camera/depth/image_raw", msg);
    }
}
```

## LiDAR Simulation

**Raycasting Approach**:
```csharp
public class LiDARSensor : MonoBehaviour
{
    public int numRays = 360;
    public float range = 10f;
    public float angleMin = 0f;
    public float angleMax = 360f;

    void PublishLaserScan()
    {
        float[] ranges = new float[numRays];
        float angleIncrement = (angleMax - angleMin) / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = angleMin + i * angleIncrement;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, range))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = range;  // Max range if no hit
            }
        }

        // Publish LaserScan message
        LaserScanMsg msg = new LaserScanMsg
        {
            header = new HeaderMsg { stamp = GetROSTime(), frame_id = "lidar" },
            angle_min = angleMin * Mathf.Deg2Rad,
            angle_max = angleMax * Mathf.Deg2Rad,
            angle_increment = angleIncrement * Mathf.Deg2Rad,
            range_min = 0.1f,
            range_max = range,
            ranges = ranges
        };

        ros.Publish("/scan", msg);
    }
}
```

## IMU Simulation with Noise

```csharp
public class IMUSensor : MonoBehaviour
{
    public float accelerometerNoise = 0.01f;
    public float gyroscopeNoise = 0.001f;

    private Vector3 lastVelocity;

    void PublishIMU()
    {
        // Calculate linear acceleration
        Vector3 acceleration = (rigidbody.velocity - lastVelocity) / Time.deltaTime;
        acceleration += AddNoise(accelerometerNoise);
        lastVelocity = rigidbody.velocity;

        // Get angular velocity
        Vector3 angularVelocity = rigidbody.angularVelocity + AddNoise(gyroscopeNoise);

        ImuMsg msg = new ImuMsg
        {
            header = new HeaderMsg { stamp = GetROSTime(), frame_id = "imu" },
            linear_acceleration = new Vector3Msg {
                x = acceleration.x, y = acceleration.y, z = acceleration.z
            },
            angular_velocity = new Vector3Msg {
                x = angularVelocity.x, y = angularVelocity.y, z = angularVelocity.z
            }
        };

        ros.Publish("/imu/data", msg);
    }

    Vector3 AddNoise(float stdDev)
    {
        return new Vector3(
            GaussianNoise(0, stdDev),
            GaussianNoise(0, stdDev),
            GaussianNoise(0, stdDev)
        );
    }
}
```

## Hands-On Exercise

### Exercise: Simulate Depth Camera in Unity

**Steps**:

1. **Create Unity Scene**:
   - Add plane (ground)
   - Add cubes/obstacles
   - Create camera object

2. **Attach Depth Camera Script**:
   - Add `DepthCameraPublisher.cs` to camera
   - Configure ROS connection

3. **Run ROS 2 Bridge**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

4. **Verify Data**:
```bash
ros2 topic echo /camera/depth/image_raw
ros2 run rviz2 rviz2
```

**Expected Outcome**: Depth images published to ROS 2, visualized in RViz.

**Complete Code**: [GitHub: Unity Robotics Hub Examples](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

## Summary

- **Unity** provides photorealistic rendering for HRI scenarios
- **Depth cameras** simulate RGB-D sensors (RealSense, Kinect)
- **LiDAR** uses raycasting for point cloud generation
- **IMUs** model acceleration and gyroscope with realistic noise
- **ROS TCP Connector** bridges Unity ↔ ROS 2 communication

## Next Steps

Continue to [Module 3: NVIDIA Isaac Platform](../module-03-nvidia-isaac/) to leverage GPU-accelerated simulation and AI-powered perception.

---

**Estimated Time**: 4 hours
**Difficulty**: Intermediate
