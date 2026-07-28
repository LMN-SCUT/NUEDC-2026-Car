# YOLO 训练工作区

本目录用于蓝色盒子与钢球目标检测的训练、导出和 K230 模型转换。

## 入口

- `luanch.cmd`：启动图形界面。
- `yolo_k230_gui.py`：唯一保留的训练与转换脚本，包含训练、导出 ONNX、转换 KModel 等功能。

命令行转换示例：

```powershell
python yolo_k230_gui.py --compile-kmodel model.onnx calibration_images
```

## 目录

- `datasets/steel_ball/derived/selfmake_k230_v2_supplement_train`：V6 使用的基础数据加困难补充数据集。
- `datasets/steel_ball/source_download`：购入或下载的原始钢球数据集。
- `datasets/steel_ball/legacy_source`：早期钢球数据与旧版来源，保留用于追溯。
- `datasets/bluebox_archive`：已经跑通的蓝色盒子数据归档。
- `models/pretrained`：通用预训练权重。
- `models/steel_ball/candidates`：从历史训练中提取出的三组钢球候选模型及其指标、曲线。
- `runs/detect`：本地训练输出目录；仓库仅封存明确选定的基线文件。
- `archives`：原始压缩包。
- `docs`：已有说明文档。

## 钢球候选模型

| 模型目录 | mAP50 | mAP50-95 | 备注 |
|---|---:|---:|---|
| `test_new_dataset2` | 0.92582 | 0.72994 | 当前较均衡，适合作为实机初测候选 |
| `steel_ball_combined_clean2` | 0.89610 | 0.66764 | 基于整理后组合数据训练 |
| `steel_ball_base` | 0.99405 | 0.91189 | 指标异常高，需排查训练/验证集重复或场景泄漏 |

### 当前版本命名

| 版本 | 数据与配置 | 状态 |
|---|---|---|
| V1 | 最开始购入/收集的真实钢球数据 | 历史基线 |
| V2、V3 | 混入网上下载的合成钢球数据 | 已废弃，不再用于训练或部署 |
| V4 | `selfmake_k230_v1_80_20`，YOLOv8n，224×224 | 实机可用基线 |
| V5 | 与 V4 相同真实数据，YOLOv8n，320×320 | 历史 320 基线 |
| V6 | V5 数据加 158 张困难补充图，YOLOv8n，320×320 | 当前封存部署基线；高光、远距离和贴线识别优于 V5 |

不要只按训练指标选择最终模型。应使用未参与训练的独立实拍钢球图片，在 K230 上比较漏检、误检、距离变化和复杂背景表现。

## 当前默认配置

图形界面默认使用：

- 预训练权重：`models/pretrained/yolov8n.pt`
- 数据配置：`datasets/steel_ball/derived/selfmake_k230_v2_supplement_train/data.yaml`
- 训练输出：`runs/detect`

数据集移动后，各 `data.yaml` 已更新为当前绝对路径。若整个工程迁移到其他电脑或目录，需要同步修改其中的 `path`。
