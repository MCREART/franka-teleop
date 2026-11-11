# Franka iPhone 遥操

使用 Xcode 打开 `FrankaTeleop.xcodeproj`，选择 `FrankaTeleop` scheme。应用需要 iOS 17 或更新版本、支持 ARKit 世界追踪的 iPhone，以及相机和本地网络权限。

在 Signing & Capabilities 中将 Bundle Identifier 从示例值 `com.example.FrankaTeleop` 改为自己的唯一标识，选择自己的开发团队，然后选中已连接的 iPhone 运行。工程没有外部包依赖，使用 SwiftUI、ARKit、URLSession 和 UIKit。

## 操作

1. 在 Mac 上启动项目根目录说明中的 MuJoCo 服务。
2. iPhone 与 Mac 加入同一局域网，在地址框填写 Mac 的 `ws://局域网IP:8765`，点“连接”。首次运行时地址为空，填写后的地址会保存在手机上，后续运行自动恢复。
3. 允许相机和本地网络访问。右手侧握手机，顶部朝上、屏幕朝身体左侧，保持这个握姿点一次“重置追踪”。缓慢移动手机，等待“位姿追踪正常”。服务端默认使用适应该握姿的水平 90° 坐标补偿。
4. 按住底部“按住遥操”，移动或旋转手机控制机械臂末端。每次按下以当前位置为相对起点，松开或手指移出按钮会暂停。追踪恢复后需要再次按下。
5. 使用固定在屏幕左侧的竖向开度滑条控制夹爪，方便右手侧握时用拇指滑动，上滑张开至 100%，下滑闭合至 0%。服务端判定夹到物体后，应用触发一次成功触觉反馈。

左侧夹爪滑条与底部按住按钮在 iPhone SE 等小屏设备上保持可见。应用切到后台、相机被打断或连接异常时会结束本次遥操，回到前台后重新连接。

## 通信

客户端每秒最多发送 30 帧 JSON，最多保留一个正在发送的请求，每次取最新位姿。位置单位为米，四元数顺序为 `[x, y, z, w]`。服务端负责坐标映射和相对起点校准。

```json
{"type":"control","seq":1,"position":[0,0,0],"quaternion":[0,0,0,1],"gripper":1.0,"engaged":false}
```

应用接收 `state` 中的 `ee_position`、`tracking_active`、`gripper`、`gripper_command`、`grasped` 和 `message`，也接收 `grasp` 事件。首次 `state` 到达后先同步夹爪指令，再开始发送，防止重连时覆盖原来的夹持开度。抓取事件与状态包通过 `grasped` 边沿合并，单次抓取只触发一次震动。

## 构建验证

在 Mac 上可以运行无签名编译检查。

```sh
DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer \
xcodebuild -project FrankaTeleop.xcodeproj -scheme FrankaTeleop \
  -configuration Debug -destination 'generic/platform=iOS' \
  CODE_SIGNING_ALLOWED=NO build
```

真机相机位姿、局域网权限和触觉反馈需要在 iPhone 上验证。模拟器不提供完整的 ARKit 世界追踪与手机震动。

实现参考 Apple 官方的 [ARSessionDelegate](https://developer.apple.com/documentation/arkit/arsessiondelegate)、[URLSessionWebSocketTask](https://developer.apple.com/documentation/foundation/urlsessionwebsockettask) 和 [ARKit 追踪状态处理](https://developer.apple.com/documentation/arkit/placing-objects-and-handling-3d-interaction)。
