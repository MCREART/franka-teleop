import ARKit
import SceneKit
import SwiftUI

struct ContentView: View {
    @StateObject private var controller = TeleopController()
    @Environment(\.scenePhase) private var scenePhase
    private let accent = Color(red: 0.35, green: 0.89, blue: 0.75)

    var body: some View {
        VStack(spacing: 0) {
            VStack(alignment: .leading, spacing: 12) {
                header
                connectionCard
            }
            .padding(.horizontal, 20)
            .padding(.top, 12)
            .padding(.bottom, 12)
            HStack(alignment: .top, spacing: 12) {
                gripperCard
                    .frame(width: 84)
                ScrollView {
                    cameraCard
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            }
            .padding(.horizontal, 20)
            clutchPanel
        }
        .background(Color(red: 0.055, green: 0.07, blue: 0.09))
        .tint(accent)
        .onAppear { controller.activate() }
        .onDisappear { controller.suspend() }
        .onChange(of: scenePhase) { _, phase in
            if phase == .active { controller.activate() }
            else { controller.suspend() }
        }
    }

    private var header: some View {
        HStack(alignment: .top) {
            VStack(alignment: .leading, spacing: 5) {
                Text("FRANKA / TELEOP")
                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                    .tracking(3)
                    .foregroundStyle(accent)
                Text("掌上遥操")
                    .font(.system(size: 30, weight: .bold))
                Text("移动手机，控制机械臂末端")
                    .font(.subheadline)
                    .foregroundStyle(.secondary)
            }
            Spacer()
            Image(systemName: "move.3d")
                .font(.system(size: 32, weight: .light))
                .foregroundStyle(accent)
                .padding(.top, 8)
        }
    }

    private var connectionCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                statusDot(controller.connected)
                Text(controller.connected ? "Mac 已连接" : "连接 MuJoCo")
                    .font(.subheadline.weight(.semibold))
                Spacer()
                Text("WEBSOCKET · 30 Hz")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundStyle(.secondary)
            }
            HStack(spacing: 10) {
                TextField("ws://Mac局域网IP:8765", text: $controller.serverAddress)
                    .font(.system(size: 13, design: .monospaced))
                    .keyboardType(.URL)
                    .textInputAutocapitalization(.never)
                    .autocorrectionDisabled()
                    .submitLabel(.go)
                    .onSubmit { controller.connect() }
                    .disabled(controller.connected || controller.connecting)
                Button {
                    if controller.connected || controller.connecting { controller.disconnect() }
                    else { controller.connect() }
                } label: {
                    Text(controller.connected ? "断开" : controller.connecting ? "取消" : "连接")
                        .font(.subheadline.weight(.semibold))
                        .padding(.horizontal, 14)
                        .padding(.vertical, 9)
                        .background(accent.opacity(0.14), in: RoundedRectangle(cornerRadius: 10))
                }
            }
            Text(controller.connectionDescription)
                .font(.caption)
                .foregroundStyle(.secondary)
                .lineLimit(3)
        }
        .padding(16)
        .background(.white.opacity(0.05), in: RoundedRectangle(cornerRadius: 18))
    }

    private var cameraCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            ZStack(alignment: .bottomLeading) {
                CameraPreview(session: controller.arSession)
                LinearGradient(colors: [.clear, .black.opacity(0.8)], startPoint: .center, endPoint: .bottom)
                VStack(alignment: .leading, spacing: 7) {
                    HStack(spacing: 7) {
                        statusDot(controller.trackingReady)
                        Text(controller.trackingDescription)
                            .font(.caption.weight(.semibold))
                    }
                    Text(String(format: "手机  X %+.3f   Y %+.3f   Z %+.3f m", controller.position.x, controller.position.y, controller.position.z))
                        .font(.system(size: 10, design: .monospaced))
                        .foregroundStyle(.white.opacity(0.7))
                        .lineLimit(1)
                        .minimumScaleFactor(0.7)
                }
                .padding(14)
            }
            .frame(height: 210)
            .clipShape(RoundedRectangle(cornerRadius: 16))
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("机械臂末端 · m")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Spacer()
                    Button("重置追踪") { controller.resetTracking() }
                        .font(.caption)
                        .disabled(controller.isClutched)
                }
                Text(String(format: "X %.3f  Y %.3f  Z %.3f", controller.robotPosition.x, controller.robotPosition.y, controller.robotPosition.z))
                    .font(.system(size: 12, weight: .medium, design: .monospaced))
                    .lineLimit(1)
                    .minimumScaleFactor(0.8)
            }
        }
    }

    private var gripperCard: some View {
        VStack(spacing: 6) {
            Text("夹爪开度")
                .font(.caption.weight(.semibold))
            Text("\(Int(controller.gripper * 100))%")
                .font(.system(size: 23, weight: .medium, design: .monospaced))
                .monospacedDigit()
            Text("实际 \(Int(controller.actualGripper * 100))%")
                .font(.system(size: 10))
                .foregroundStyle(.secondary)
            Text("张开 100%")
                .font(.system(size: 10))
                .foregroundStyle(.secondary)
            GeometryReader { geometry in
                // Reserve the rotated slider's full length and a 44-point touch width.
                Slider(value: $controller.gripper, in: 0...1)
                    .frame(width: geometry.size.height, height: 44)
                    .rotationEffect(.degrees(-90))
                    .frame(width: 44, height: geometry.size.height)
                    .position(x: geometry.size.width / 2, y: geometry.size.height / 2)
                    .disabled(!controller.connected)
                    .accessibilityLabel("夹爪开度，上方张开，下方闭合")
                    .accessibilityValue("百分之\(Int(controller.gripper * 100))")
            }
            .frame(minHeight: 80)
            Text("闭合 0%")
                .font(.system(size: 10))
                .foregroundStyle(.secondary)
            VStack(spacing: 3) {
                Image(systemName: controller.grasped ? "checkmark.circle.fill" : "waveform")
                Text(controller.grasped ? "已夹取 · 震动" : "夹取时震动")
                    .font(.system(size: 10))
            }
            .foregroundStyle(controller.grasped ? accent : .secondary)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 12)
        .background(.white.opacity(0.05), in: RoundedRectangle(cornerRadius: 18))
    }

    private var clutchPanel: some View {
        VStack(spacing: 9) {
            HoldControl(
                enabled: controller.canEngage || controller.isClutched,
                engaged: controller.isClutched,
                onPress: controller.beginClutch,
                onRelease: controller.releaseClutch
            )
            .frame(height: 60)
            Text(controller.isClutched ? "移动或旋转手机 · 松开立即暂停" : "按住遥操 · 每次按下以当前位置为起点")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding(.horizontal, 20)
        .padding(.top, 14)
        .padding(.bottom, 12)
        .background(.ultraThinMaterial)
    }

    private func statusDot(_ ready: Bool) -> some View {
        Circle().fill(ready ? accent : Color.orange).frame(width: 7, height: 7)
    }
}

private struct CameraPreview: UIViewRepresentable {
    let session: ARSession

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.session = session
        view.scene = SCNScene()
        view.backgroundColor = .black
        view.isUserInteractionEnabled = false
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}

/// UIKit touch events ensure a cancelled gesture releases the clutch.
private struct HoldControl: UIViewRepresentable {
    let enabled: Bool
    let engaged: Bool
    let onPress: () -> Void
    let onRelease: () -> Void

    func makeCoordinator() -> Coordinator { Coordinator(onPress: onPress, onRelease: onRelease) }

    func makeUIView(context: Context) -> UIButton {
        let button = UIButton(type: .custom)
        button.layer.cornerRadius = 16
        button.titleLabel?.font = .systemFont(ofSize: 19, weight: .bold)
        button.addTarget(context.coordinator, action: #selector(Coordinator.press), for: .touchDown)
        button.addTarget(context.coordinator, action: #selector(Coordinator.endHold), for: [.touchUpInside, .touchUpOutside, .touchCancel, .touchDragExit])
        button.accessibilityLabel = "按住遥操"
        button.accessibilityHint = "按住时移动手机控制末端，松开暂停"
        return button
    }

    func updateUIView(_ button: UIButton, context: Context) {
        button.isEnabled = enabled
        button.setTitle(engaged ? "遥操中 · 松开暂停" : "按住遥操", for: .normal)
        button.backgroundColor = engaged ? UIColor.systemOrange : UIColor(red: 0.35, green: 0.89, blue: 0.75, alpha: enabled ? 1 : 0.18)
        button.setTitleColor(enabled ? .black : .gray, for: .normal)
    }

    final class Coordinator: NSObject {
        let onPress: () -> Void
        let onRelease: () -> Void

        init(onPress: @escaping () -> Void, onRelease: @escaping () -> Void) {
            self.onPress = onPress
            self.onRelease = onRelease
        }

        @objc func press() { onPress() }
        @objc func endHold() { onRelease() }
    }
}
