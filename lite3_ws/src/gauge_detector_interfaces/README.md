# gauge_detector_interfaces

ROS2 接口定义包，为仪表盘识别服务提供自定义服务类型。

## 服务类型

### `gauge_detector_interfaces/srv/GaugeDetect`

无请求字段，响应字段如下：

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 识别是否成功 |
| `letter` | `string` | 识别到的仪表盘字母：`A` / `B` / `C` / `D` |
| `zone` | `string` | 指针所在区域：`RED` / `GREEN` / `YELLOW` |
| `state` | `string` | 整体状态：`GREEN` 区为 `normal`，其他为 `abnormal` |
| `message` | `string` | 提示信息 |

## 依赖

- `rosidl_default_generators`
- `rosidl_default_runtime`

## 编译

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select gauge_detector_interfaces --symlink-install
```

## 使用

在 Python 节点中导入：

```python
from gauge_detector_interfaces.srv import GaugeDetect
```

## 注意

本包仅包含接口定义，不含业务节点。实际识别服务见 `gauge_detector` 包。
