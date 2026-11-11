import ARKit
import AVFoundation
import Combine
import Foundation
import UIKit
import simd

/// All published state and AR callbacks are handled on the main queue.
final class TeleopController: NSObject, ObservableObject, ARSessionDelegate, URLSessionWebSocketDelegate {
    @Published var serverAddress: String = UserDefaults.standard.string(forKey: "serverAddress") ?? ""
    @Published var gripper: Double = 1
    @Published private(set) var connected = false
    @Published private(set) var connecting = false
    @Published private(set) var trackingReady = false
    @Published private(set) var trackingDescription = "正在启动相机"
    @Published private(set) var connectionDescription = "未连接"
    @Published private(set) var isClutched = false
    @Published private(set) var grasped = false
    @Published private(set) var position = SIMD3<Float>.zero
    @Published private(set) var robotPosition = SIMD3<Double>.zero
    @Published private(set) var actualGripper: Double = 1
    @Published private(set) var serverTracking = false
    @Published private(set) var releaseRequired = true

    let arSession = ARSession()
    private var latestTransform = matrix_identity_float4x4
    private var socket: URLSessionWebSocketTask?
    private var networkSession: URLSession?
    private var timer: Timer?
    private var sequence = 0
    private var sending = false
    private var active = false
    private var firstStateReceived = false
    private var lastServerMessage = Date.distantPast
    private var lastFrameTime = Date.distantPast
    private var lastPublishedFrame = Date.distantPast
    private let gripFeedback = UINotificationFeedbackGenerator()

    var canEngage: Bool {
        connected && firstStateReceived && trackingReady && !releaseRequired && active
            && Date().timeIntervalSince(lastServerMessage) <= 0.5
    }

    override init() {
        super.init()
        arSession.delegate = self
        arSession.delegateQueue = .main
    }

    func activate() {
        guard !active else { return }
        active = true
        UIApplication.shared.isIdleTimerDisabled = true
        timer = Timer.scheduledTimer(withTimeInterval: 1.0 / 30.0, repeats: true) { [weak self] _ in
            self?.tick()
        }
        gripFeedback.prepare()
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            startTracking()
        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { [weak self] allowed in
                DispatchQueue.main.async {
                    guard let self, self.active else { return }
                    if allowed { self.startTracking() }
                    else { self.trackingDescription = "请在系统设置中允许相机权限" }
                }
            }
        default:
            trackingDescription = "请在系统设置中允许相机权限"
        }
    }

    func suspend() {
        active = false
        releaseClutch()
        disconnect()
        arSession.pause()
        trackingReady = false
        trackingDescription = "相机已暂停"
        timer?.invalidate()
        timer = nil
        UIApplication.shared.isIdleTimerDisabled = false
    }

    private func startTracking() {
        guard ARWorldTrackingConfiguration.isSupported else {
            trackingDescription = "此设备不支持 ARKit 位姿追踪，请使用真机"
            return
        }
        trackingReady = false
        trackingDescription = "缓慢移动手机，等待追踪正常"
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        configuration.isAutoFocusEnabled = true
        arSession.run(configuration, options: [.resetTracking, .removeExistingAnchors])
    }

    func resetTracking() {
        releaseClutch()
        startTracking()
    }

    func connect() {
        disconnect()
        var address = serverAddress.trimmingCharacters(in: .whitespacesAndNewlines)
        if !address.contains("://") { address = "ws://" + address }
        guard var components = URLComponents(string: address),
              let scheme = components.scheme, ["ws", "wss"].contains(scheme),
              let host = components.host, !host.isEmpty,
              components.user == nil, components.password == nil else {
            connectionDescription = "请输入 ws://Mac局域网IP:8765"
            return
        }
        if components.port == nil { components.port = 8765 }
        guard let url = components.url else { return }
        serverAddress = url.absoluteString
        UserDefaults.standard.set(serverAddress, forKey: "serverAddress")
        let configuration = URLSessionConfiguration.ephemeral
        configuration.timeoutIntervalForRequest = 5
        configuration.waitsForConnectivity = false
        let session = URLSession(configuration: configuration, delegate: self, delegateQueue: .main)
        let task = session.webSocketTask(with: url)
        networkSession = session
        socket = task
        sequence = 0
        lastServerMessage = Date()
        connecting = true
        connectionDescription = "连接中…"
        task.resume()
        receive(task)
    }

    func disconnect() {
        isClutched = false
        releaseRequired = true
        connected = false
        firstStateReceived = false
        connecting = false
        serverTracking = false
        grasped = false
        sending = false
        let oldSocket = socket
        socket = nil
        oldSocket?.cancel(with: .goingAway, reason: nil)
        networkSession?.invalidateAndCancel()
        networkSession = nil
        connectionDescription = "未连接"
    }

    func beginClutch() {
        guard canEngage else { return }
        isClutched = true
    }

    func releaseClutch() {
        if isClutched { releaseRequired = true }
        isClutched = false
        sendLatestControl()
    }

    private func tick() {
        guard active else { return }
        if Date().timeIntervalSince(lastFrameTime) > 0.25 && trackingReady {
            trackingReady = false
            trackingDescription = "相机帧中断，已暂停遥操"
            releaseClutch()
        }
        let silence = Date().timeIntervalSince(lastServerMessage)
        if connected && silence > 0.5 {
            releaseClutch()
            connectionDescription = "反馈延迟，已暂停遥操"
        }
        if (connected || connecting) && silence > 3 {
            failConnection("连接超时，请检查 Mac 服务和同一局域网")
            return
        }
        sendLatestControl()
    }

    private struct ControlFrame: Encodable {
        let type = "control"
        let seq: Int
        let position: [Double]
        let quaternion: [Double]
        let gripper: Double
        let engaged: Bool
    }

    private func sendLatestControl() {
        // Keep a single send in flight. The next tick takes the newest pose.
        guard connected, firstStateReceived, let task = socket, !sending else { return }
        let quaternion = simd_quatf(latestTransform)
        let translation = latestTransform.columns.3
        let engaged = isClutched && trackingReady && active && !releaseRequired
            && Date().timeIntervalSince(lastServerMessage) <= 0.5
        let frame = ControlFrame(
            seq: sequence,
            position: [Double(translation.x), Double(translation.y), Double(translation.z)],
            quaternion: [Double(quaternion.imag.x), Double(quaternion.imag.y), Double(quaternion.imag.z), Double(quaternion.real)],
            gripper: gripper,
            engaged: engaged
        )
        guard let data = try? JSONEncoder().encode(frame), let text = String(data: data, encoding: .utf8) else { return }
        sequence += 1
        sending = true
        task.send(.string(text)) { [weak self, weak task] error in
            DispatchQueue.main.async {
                guard let self, let task, self.socket === task else { return }
                self.sending = false
                if let error { self.failConnection(error.localizedDescription); return }
                if !engaged { self.releaseRequired = false }
            }
        }
    }

    private func receive(_ task: URLSessionWebSocketTask) {
        task.receive { [weak self, weak task] result in
            DispatchQueue.main.async {
                guard let self, let task, self.socket === task else { return }
                switch result {
                case .success(let message):
                    let data: Data
                    switch message {
                    case .string(let string): data = Data(string.utf8)
                    case .data(let bytes): data = bytes
                    @unknown default: self.receive(task); return
                    }
                    self.handleServerMessage(data)
                    self.receive(task)
                case .failure(let error):
                    self.failConnection(error.localizedDescription)
                }
            }
        }
    }

    private func handleServerMessage(_ data: Data) {
        guard let object = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let type = object["type"] as? String else { return }
        lastServerMessage = Date()
        if type == "state" {
            if !firstStateReceived {
                if let value = (object["gripper_command"] ?? object["gripper"]) as? Double {
                    gripper = min(1, max(0, value))
                }
                firstStateReceived = true
            }
            serverTracking = object["tracking_active"] as? Bool ?? false
            if let values = object["ee_position"] as? [Double], values.count == 3 {
                robotPosition = SIMD3(values[0], values[1], values[2])
            }
            if let value = object["gripper"] as? Double { actualGripper = min(1, max(0, value)) }
            if let value = object["grasped"] as? Bool { updateGrasp(value) }
            connectionDescription = object["message"] as? String ?? "已连接"
        } else if type == "grasp" {
            updateGrasp(object["grasped"] as? Bool ?? true)
        } else if type == "error" {
            releaseClutch()
            connectionDescription = object["message"] as? String ?? "服务端拒绝控制"
        }
    }

    private func updateGrasp(_ value: Bool) {
        if value && !grasped && active {
            gripFeedback.notificationOccurred(.success)
            gripFeedback.prepare()
        }
        grasped = value
    }

    private func failConnection(_ message: String) {
        disconnect()
        connectionDescription = message
    }

    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didOpenWithProtocol protocol: String?) {
        guard socket === webSocketTask else { return }
        connecting = false
        connected = true
        releaseRequired = true
        lastServerMessage = Date()
        connectionDescription = "已连接，松开后按住遥操"
        sendLatestControl()
    }

    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didCloseWith closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?) {
        guard socket === webSocketTask else { return }
        failConnection("连接已关闭")
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        latestTransform = frame.camera.transform
        lastFrameTime = Date()
        switch frame.camera.trackingState {
        case .normal:
            trackingReady = true
            trackingDescription = "位姿追踪正常"
        case .notAvailable:
            loseTracking("位姿不可用")
        case .limited(let reason):
            switch reason {
            case .initializing: loseTracking("初始化中，请缓慢移动手机")
            case .excessiveMotion: loseTracking("移动过快，已暂停遥操")
            case .insufficientFeatures: loseTracking("画面特征不足，请对准有纹理区域")
            case .relocalizing: loseTracking("正在重新定位，请稍候")
            @unknown default: loseTracking("追踪受限，已暂停遥操")
            }
        }
        if Date().timeIntervalSince(lastPublishedFrame) > 0.1 {
            let p = frame.camera.transform.columns.3
            position = SIMD3(p.x, p.y, p.z)
            lastPublishedFrame = Date()
        }
    }

    private func loseTracking(_ message: String) {
        trackingReady = false
        trackingDescription = message
        if isClutched { releaseClutch() }
    }

    func sessionWasInterrupted(_ session: ARSession) {
        loseTracking("相机被打断，已暂停遥操")
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        releaseClutch()
        startTracking()
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        loseTracking(error.localizedDescription)
    }
}
