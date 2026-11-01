# Kinect2VMC for Warudo

Kinect 2.0 全身 / 半身 3D 动作捕捉，用于 Warudo 虚拟直播。

> 本项目代码由 **dsh**（DeepSeek Harness AI 助手）协助制作，
> 本文档的项目介绍由 **AI 填写**。
> **项目尚未整体完成**，仍处于开发 / 验证阶段。

---

## 简介

用一台 Kinect 2.0 传感器捕捉人体骨架和手部动作，实时驱动 Warudo 里的 3D 角色模型，用于虚拟直播。

主要分三个部分：

| 部分 | 语言 | 作用 |
|---|---|---|
| `src/`（Kinect2VMC.exe） | C# / .NET | 读取 Kinect 骨架，计算上身骨骼旋转，把彩色画面转发给手部识别 |
| `tools/hand_tracker.py` | Python / MediaPipe | 从彩色画面追踪双手，算出掌心朝向（世界旋转）和每根手指的弯曲量 |
| `warudo-mod/BonePortProbeAsset.cs` | C#（Warudo Playground 插件） | 直接写入角色骨骼，完成骨架、掌心朝向、手指的应用 |

数据通过本地 UDP（端口 39542）在程序、手部识别、Warudo 插件之间传递。

## 手部追踪的三种形态

手部数据**只来自 MediaPipe**（Kinect 骨架没有足够的手指信息）：

| 形态 | 数据来源 | 说明 |
|---|---|---|
| 张开 / 握拳（死数据） | MediaPipe 每指弯曲量 → 整体开合混合 | 稳定，但只有两个状态 |
| 逐指原生 | MediaPipe 每指弯曲量 → 1:1 投射到骨骼 | 最还原，但精度不稳定 |

## 当前状态

- 上身骨架跟踪、转身、手部朝向（掌心翻转）：已完成并通过验证
- 手指逐指原生追踪：仍在开发 / 验证
- 手腕 / 手肘扭转不扭曲（FinalIK 扭转分摊）：仍在验证
- 左右镜像、更多手势姿态：计划中，未完成

本项目尚未整体完成，代码中部分实验性功能仍在使用默认配置运行。

## 构建

需要：

- .NET SDK（含 MSBuild）
- Kinect for Windows SDK 2.0

在项目根目录运行：

```
build.cmd
```

构建产物输出到 `dist\`，自测（360+ 项，无需 Kinect）也会一起跑。

## 运行（简要）

1. 连接 Kinect 2.0，运行 `dist\Kinect2VMC.exe`。
2. 在 Warudo 中把插件 `BonePortProbeAsset.cs` 放入 `StreamingAssets\Playground\`。
3. 启动手部识别（程序会自动拉起 `tools\hand_tracker.py`）。

## 许可证

[MIT](LICENSE)

## 免责声明

本项目是一个正在进行中的个人 / 学习项目，用于说明如何把 Kinect 2.0 的手部与骨架数据接入 Warudo。
代码按原样提供，不附带任何明示或默示的担保。