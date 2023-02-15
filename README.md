# image_collector
<!-- Required -->
<!-- Package description -->
A node for saving RGB and depth images from camera input via service request.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to image_collector
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
1. Validate topic in launch file and parameters in param file.
2. Run your camera node.
3. Run image_collector node.
```bash
ros2 launch image_collector image_collector.launch.py
```
4. Call the service.
```bash
ros2 service call /image_collector_node/save_images std_srvs/srv/Trigger
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name          | Type                    | Description         |
| ------------- | ----------------------- | ------------------- |
| `image_rgb`   | sensor_msgs::msg::Image | RGB camera input.   |
| `image_depth` | sensor_msgs::msg::Image | Depth camera input. |


### Services

| Name                              | Type                   | Description          |
| --------------------------------- | ---------------------- | -------------------- |
| `/image_collect_node/save_images` | std_srvs::srv::Trigger | Save images request. |

### Parameters

| Name          | Type   | Description                              |
| ------------- | ------ | ---------------------------------------- |
| `save_format` | string | Saving file format for RGB images.       |
| `save_dir`    | string | Absolute path for storing output images. |



## References / External links
<!-- Optional -->
