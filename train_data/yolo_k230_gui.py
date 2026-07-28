"""YOLO training and K230 deployment helper.

GUI mode:
    python yolo_k230_gui.py

Command-line KModel conversion:
    python yolo_k230_gui.py --compile-kmodel model.onnx calibration_images
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import threading
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_PYTHON = Path(r"D:\anacodaprogram\envs\py38_202530350942\vision\python.exe")
DEFAULT_YOLO = DEFAULT_PYTHON.parent / "Scripts" / "yolo.exe"
DEFAULT_MODEL = BASE_DIR / "models" / "pretrained" / "yolov8n.pt"
DEFAULT_DATA = (
    BASE_DIR
    / "datasets"
    / "steel_ball"
    / "derived"
    / "selfmake_k230_v2_supplement_train"
    / "data.yaml"
)
RUNS_DIR = BASE_DIR / "runs" / "detect"


def _onnx_input_shape(model_path: Path) -> list[int]:
    import onnx

    model = onnx.load(str(model_path))
    dims = model.graph.input[0].type.tensor_type.shape.dim
    shape: list[int] = []
    for dim in dims:
        value = int(dim.dim_value or 1)
        shape.append(value)
    if len(shape) != 4:
        raise ValueError(f"仅支持 NCHW 四维输入，实际输入形状为 {shape}")
    shape[2] = int((shape[2] + 31) // 32 * 32)
    shape[3] = int((shape[3] + 31) // 32 * 32)
    return shape


def compile_kmodel(
    onnx_path: Path,
    calibration_dir: Path,
    output_path: Path | None = None,
    sample_count: int = 20,
) -> Path:
    """Compile a YOLO ONNX model into a K230 KModel using nncase."""
    import numpy as np
    import onnx
    import onnxsim
    from PIL import Image
    import nncase

    onnx_path = onnx_path.resolve()
    calibration_dir = calibration_dir.resolve()
    output_path = (output_path or onnx_path.with_suffix(".kmodel")).resolve()

    if not onnx_path.is_file():
        raise FileNotFoundError(f"找不到 ONNX：{onnx_path}")
    if not calibration_dir.is_dir():
        raise FileNotFoundError(f"找不到校准图片目录：{calibration_dir}")

    image_files = sorted(
        p
        for p in calibration_dir.rglob("*")
        if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
    )
    if not image_files:
        raise ValueError(f"校准目录中没有可用图片：{calibration_dir}")
    image_files = image_files[: max(1, sample_count)]

    input_shape = _onnx_input_shape(onnx_path)
    _, channels, height, width = input_shape
    if channels != 3:
        raise ValueError(f"当前转换器要求 3 通道输入，实际为 {channels}")

    with tempfile.TemporaryDirectory(prefix="k230_compile_") as temp_dir:
        simplified_path = Path(temp_dir) / "simplified.onnx"
        model = onnx.load(str(onnx_path))
        simplified, ok = onnxsim.simplify(model, input_shapes={model.graph.input[0].name: input_shape})
        if not ok:
            raise RuntimeError("ONNX simplify 校验失败")
        onnx.save(simplified, str(simplified_path))

        compile_options = nncase.CompileOptions()
        compile_options.target = "k230"
        compile_options.preprocess = True
        compile_options.swapRB = False
        compile_options.input_shape = input_shape
        compile_options.input_type = "uint8"
        compile_options.input_range = [0, 1]
        compile_options.mean = [0, 0, 0]
        compile_options.std = [1, 1, 1]
        compile_options.input_layout = "NCHW"
        compile_options.dump_ir = False
        compile_options.dump_asm = False

        compiler = nncase.Compiler(compile_options)
        compiler.import_onnx(simplified_path.read_bytes(), nncase.ImportOptions())

        calibration_data = []
        for image_path in image_files:
            image = Image.open(image_path).convert("RGB").resize((width, height))
            array = np.asarray(image, dtype=np.uint8)
            array = np.transpose(array, (2, 0, 1))[None, ...]
            # nncase expects one list of input tensors per calibration sample.
            calibration_data.append([array])

        ptq_options = nncase.PTQTensorOptions()
        ptq_options.samples_count = len(calibration_data)
        ptq_options.set_tensor_data(calibration_data)
        ptq_options.quant_type = "uint8"
        ptq_options.w_quant_type = "uint8"
        compiler.use_ptq(ptq_options)
        compiler.compile()

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(compiler.gencode_tobytes())

    return output_path


def run_gui() -> None:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    class App:
        def __init__(self, root: tk.Tk) -> None:
            self.root = root
            root.title("YOLO 训练与 K230 模型转换")
            root.geometry("900x650")

            self.python_var = tk.StringVar(value=str(DEFAULT_PYTHON))
            self.yolo_var = tk.StringVar(value=str(DEFAULT_YOLO))
            self.model_var = tk.StringVar(value=str(DEFAULT_MODEL))
            self.data_var = tk.StringVar(value=str(DEFAULT_DATA))
            self.run_name_var = tk.StringVar(value="steel_ball_v6_yolov8n_320_supplement")
            self.epochs_var = tk.StringVar(value="100")
            self.imgsz_var = tk.StringVar(value="320")
            self.batch_var = tk.StringVar(value="32")
            self.device_var = tk.StringVar(value="0")
            self.calibration_var = tk.StringVar(
                value=str(
                    BASE_DIR
                    / "datasets"
                    / "steel_ball"
                    / "derived"
                    / "selfmake_k230_v2_supplement_train"
                    / "images"
                    / "train"
                )
            )
            self.process: subprocess.Popen[str] | None = None

            form = ttk.Frame(root, padding=12)
            form.pack(fill=tk.X)

            self._path_row(form, 0, "Python", self.python_var, False)
            self._path_row(form, 1, "YOLO 命令", self.yolo_var, False)
            self._path_row(form, 2, "预训练模型", self.model_var, False)
            self._path_row(form, 3, "数据集 YAML", self.data_var, False)
            self._path_row(form, 4, "校准图片目录", self.calibration_var, True)

            options = ttk.Frame(root, padding=(12, 0, 12, 8))
            options.pack(fill=tk.X)
            fields = [
                ("运行名", self.run_name_var),
                ("Epochs", self.epochs_var),
                ("ImgSz", self.imgsz_var),
                ("Batch", self.batch_var),
                ("Device", self.device_var),
            ]
            for index, (label, variable) in enumerate(fields):
                ttk.Label(options, text=label).grid(row=0, column=index * 2, padx=(0, 4))
                ttk.Entry(options, textvariable=variable, width=14).grid(
                    row=0, column=index * 2 + 1, padx=(0, 12)
                )

            buttons = ttk.Frame(root, padding=(12, 0, 12, 8))
            buttons.pack(fill=tk.X)
            ttk.Button(buttons, text="开始训练", command=self.start_training).pack(side=tk.LEFT, padx=4)
            ttk.Button(buttons, text="导出 best.pt → ONNX → KModel", command=self.export_best).pack(
                side=tk.LEFT, padx=4
            )
            ttk.Button(buttons, text="停止任务", command=self.stop_process).pack(side=tk.LEFT, padx=4)
            ttk.Button(buttons, text="打开运行目录", command=self.open_runs).pack(side=tk.LEFT, padx=4)

            self.log = tk.Text(root, wrap=tk.WORD, font=("Consolas", 10))
            self.log.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))
            self._write_log(f"工作目录：{BASE_DIR}\n")
            self._write_log("GUI 已内置 ONNX→KModel 转换，不再依赖外部 to_kmodel.py。\n")

        def _path_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar, directory: bool) -> None:
            ttk.Label(parent, text=label, width=14).grid(row=row, column=0, sticky=tk.W, pady=3)
            ttk.Entry(parent, textvariable=variable).grid(row=row, column=1, sticky=tk.EW, pady=3)
            ttk.Button(
                parent,
                text="浏览",
                command=lambda: self._browse(variable, directory),
            ).grid(row=row, column=2, padx=(8, 0), pady=3)
            parent.columnconfigure(1, weight=1)

        def _browse(self, variable: tk.StringVar, directory: bool) -> None:
            selected = (
                filedialog.askdirectory(initialdir=variable.get() or str(BASE_DIR))
                if directory
                else filedialog.askopenfilename(initialdir=str(Path(variable.get()).parent))
            )
            if selected:
                variable.set(selected)

        def _write_log(self, text: str) -> None:
            self.log.insert(tk.END, text)
            self.log.see(tk.END)

        def _run(self, command: list[str]) -> None:
            if self.process and self.process.poll() is None:
                messagebox.showwarning("任务进行中", "请先停止当前任务。")
                return

            def worker() -> None:
                try:
                    self.root.after(0, self._write_log, "\n> " + subprocess.list2cmdline(command) + "\n")
                    self.process = subprocess.Popen(
                        command,
                        cwd=str(BASE_DIR),
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        encoding="utf-8",
                        errors="replace",
                    )
                    assert self.process.stdout is not None
                    for line in self.process.stdout:
                        self.root.after(0, self._write_log, line)
                    code = self.process.wait()
                    self.root.after(0, self._write_log, f"\n任务结束，退出码：{code}\n")
                except Exception as exc:
                    self.root.after(0, messagebox.showerror, "执行失败", str(exc))
                finally:
                    self.process = None

            threading.Thread(target=worker, daemon=True).start()

        def start_training(self) -> None:
            RUNS_DIR.mkdir(parents=True, exist_ok=True)
            run_name = self.run_name_var.get().strip()
            if not run_name:
                messagebox.showerror("运行名无效", "运行名不能为空。")
                return

            try:
                image_size = int(self.imgsz_var.get())
            except ValueError:
                messagebox.showerror("ImgSz 无效", "ImgSz 必须是整数。")
                return

            size_in_name = re.search(r"(?:^|_)(224|320|640)(?:_|$)", run_name)
            if size_in_name and int(size_in_name.group(1)) != image_size:
                messagebox.showerror(
                    "分辨率不一致",
                    f"运行名写的是 {size_in_name.group(1)}，但 ImgSz 是 {image_size}。\n"
                    "请修改其中一项后再开始训练。",
                )
                return

            run_dir = RUNS_DIR / run_name
            if run_dir.exists():
                messagebox.showerror(
                    "运行目录已存在",
                    f"不会覆盖已有模型：\n{run_dir}\n\n请使用一个新的运行名。",
                )
                return

            command = [
                self.yolo_var.get(),
                "detect",
                "train",
                f"model={Path(self.model_var.get()).resolve()}",
                f"data={Path(self.data_var.get()).resolve()}",
                f"epochs={self.epochs_var.get()}",
                f"imgsz={image_size}",
                f"batch={self.batch_var.get()}",
                f"device={self.device_var.get()}",
                f"name={run_name}",
                f"project={RUNS_DIR}",
            ]
            self._run(command)

        def export_best(self) -> None:
            run_dir = RUNS_DIR / self.run_name_var.get()
            best_pt = run_dir / "weights" / "best.pt"
            if not best_pt.is_file():
                messagebox.showerror("找不到模型", f"不存在：{best_pt}")
                return
            command = [
                self.yolo_var.get(),
                "export",
                f"model={best_pt}",
                "format=onnx",
                "opset=11",
                "simplify=True",
            ]
            self._run_export_pipeline(command, best_pt.with_suffix(".onnx"))

        def _run_export_pipeline(self, export_command: list[str], onnx_path: Path) -> None:
            if self.process and self.process.poll() is None:
                messagebox.showwarning("任务进行中", "请先停止当前任务。")
                return

            def worker() -> None:
                try:
                    commands = [
                        export_command,
                        [
                            self.python_var.get(),
                            str(Path(__file__).resolve()),
                            "--compile-kmodel",
                            str(onnx_path),
                            self.calibration_var.get(),
                        ],
                    ]
                    for command in commands:
                        self.root.after(0, self._write_log, "\n> " + subprocess.list2cmdline(command) + "\n")
                        self.process = subprocess.Popen(
                            command,
                            cwd=str(BASE_DIR),
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            encoding="utf-8",
                            errors="replace",
                        )
                        assert self.process.stdout is not None
                        for line in self.process.stdout:
                            self.root.after(0, self._write_log, line)
                        if self.process.wait() != 0:
                            raise RuntimeError("流水线中止，请检查上方日志。")
                    self.root.after(0, self._write_log, "\n导出与 KModel 转换完成。\n")
                except Exception as exc:
                    self.root.after(0, messagebox.showerror, "导出失败", str(exc))
                finally:
                    self.process = None

            threading.Thread(target=worker, daemon=True).start()

        def stop_process(self) -> None:
            if self.process and self.process.poll() is None:
                self.process.terminate()
                self._write_log("\n已请求停止任务。\n")

        def open_runs(self) -> None:
            RUNS_DIR.mkdir(parents=True, exist_ok=True)
            os.startfile(RUNS_DIR)

    root = tk.Tk()
    App(root)
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--compile-kmodel", nargs=2, metavar=("ONNX", "CALIBRATION_DIR"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=20)
    args = parser.parse_args()

    if args.compile_kmodel:
        result = compile_kmodel(
            Path(args.compile_kmodel[0]),
            Path(args.compile_kmodel[1]),
            args.output,
            args.samples,
        )
        print(f"KModel written: {result}")
    else:
        run_gui()


if __name__ == "__main__":
    main()
