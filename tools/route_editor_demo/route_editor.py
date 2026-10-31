"""Offline route storyboard. No ROS, network, coordinates or hardware adapters."""
import json
import math
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

FORMAT = "offline_route_storyboard/v1"


def validate_steps(raw):
    if not isinstance(raw, list) or len(raw) > 1000:
        raise ValueError("步骤必须是数组，最多 1000 项。")
    result = []
    for index, item in enumerate(raw, 1):
        if not isinstance(item, dict):
            raise ValueError(f"第 {index} 项格式错误。")
        kind = item.get("type")
        if kind == "waypoint":
            label = item.get("label")
            if not isinstance(label, str) or not label.strip() or len(label) > 80:
                raise ValueError(f"第 {index} 项航点名称须为 1～80 个字符。")
            result.append({"type": kind, "label": label.strip()})
        elif kind == "wait":
            value = item.get("seconds")
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"第 {index} 项等待秒数必须是数字。")
            if not math.isfinite(value) or not 0 < value <= 86400:
                raise ValueError(f"第 {index} 项等待时间须大于 0 且不超过 86400 秒。")
            result.append({"type": kind, "seconds": float(value)})
        else:
            raise ValueError(f"第 {index} 项类型不受支持。")
    return result


def load_storyboard(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ValueError("请选择本演示保存的路线文件；不支持实车路线格式。")
    return validate_steps(data.get("steps"))


def save_storyboard(path, steps):
    content = json.dumps({"format": FORMAT, "steps": validate_steps(steps)},
                         ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    target = Path(path)
    temporary = target.with_name(target.name + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(target)


class PreviewClock:
    """Single-threaded monotonic timer; pause freezes remaining time."""
    def __init__(self):
        self.deadline = None
        self.remaining = 0.0
        self.paused = False

    def start(self, seconds, now):
        self.remaining = float(seconds)
        self.deadline = now + self.remaining
        self.paused = False

    def left(self, now):
        if self.deadline is None or self.paused:
            return self.remaining
        return max(0.0, self.deadline - now)

    def pause(self, now):
        self.remaining = self.left(now)
        self.paused = True

    def resume(self, now):
        self.deadline = now + self.remaining
        self.paused = False


class RouteEditor:
    def __init__(self, root):
        self.root = root
        self.steps = []
        self.dirty = False
        self.path = None
        self.running = False
        self.index = 0
        self.after_id = None
        self.clock = PreviewClock()
        root.title("独立路线编排演示 · 离线模拟")
        root.geometry("960x650")
        root.minsize(780, 540)
        root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 16, "bold"))
        style.configure("Treeview", rowheight=34, font=("Microsoft YaHei UI", 10))
        style.configure("Treeview.Heading", font=("Microsoft YaHei UI", 10, "bold"))
        frame = ttk.Frame(root, padding=20)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="路线编排", style="Title.TLabel").pack(anchor="w")
        ttk.Label(frame, text="航点与等待块同级 · 仅模拟预览 · 不连接机器人、ROS 或网络").pack(anchor="w", pady=(5, 15))
        tools = ttk.Frame(frame)
        tools.pack(fill="x", pady=(0, 10))
        self.edit_buttons = []
        for label, command in [("添加航点", self.add_waypoint), ("添加等待", self.add_wait),
                               ("编辑", self.edit), ("上移 ↑", lambda: self.move(-1)),
                               ("下移 ↓", lambda: self.move(1)), ("删除", self.delete)]:
            button = ttk.Button(tools, text=label, command=command)
            button.pack(side="left", padx=(0, 6))
            self.edit_buttons.append(button)
        table_frame = ttk.Frame(frame)
        table_frame.pack(fill="both", expand=True)
        self.table = ttk.Treeview(table_frame, columns=("order", "kind", "value", "state"),
                                  show="headings", selectmode="browse")
        for column, title, width in [("order", "顺序", 60), ("kind", "块类型", 110),
                                     ("value", "名称 / 时长", 330), ("state", "预览状态", 150)]:
            self.table.heading(column, text=title)
            self.table.column(column, width=width, minwidth=50, stretch=column == "value")
        scroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        self.table.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.table.tag_configure("wait", background="#fff4d6")
        self.table.tag_configure("active", background="#d8ebff")
        self.table.bind("<Double-1>", lambda event: self.edit())
        self.summary = tk.StringVar()
        ttk.Label(frame, textvariable=self.summary).pack(anchor="w", pady=(10, 5))
        self.status = tk.StringVar(value="添加步骤，或点击“示例路线”开始。")
        ttk.Label(frame, textvariable=self.status, wraplength=880).pack(anchor="w", pady=(0, 12))
        files = ttk.Frame(frame)
        files.pack(fill="x")
        for label, command in [("新建", self.new), ("打开 JSON", self.open),
                               ("保存 JSON", self.save), ("示例路线", self.example)]:
            button = ttk.Button(files, text=label, command=command)
            button.pack(side="left", padx=(0, 6))
            self.edit_buttons.append(button)
        self.stop_button = ttk.Button(files, text="取消预览", command=self.cancel, state="disabled")
        self.stop_button.pack(side="right")
        self.pause_button = ttk.Button(files, text="暂停", command=self.toggle_pause, state="disabled")
        self.pause_button.pack(side="right", padx=6)
        self.play_button = ttk.Button(files, text="模拟预览", command=self.play)
        self.play_button.pack(side="right")
        ttk.Label(frame, text="说明：航点只显示名称，不含实际坐标；每个航点固定模拟 0.8 秒。等待块按设定秒数计时。",
                  wraplength=880).pack(anchor="w", pady=(12, 0))
        self.refresh()

    def selected(self):
        selection = self.table.selection()
        return int(selection[0]) if selection else None

    def refresh(self, selection=None):
        self.table.delete(*self.table.get_children())
        for i, item in enumerate(self.steps):
            wait = item["type"] == "wait"
            value = f"{item['seconds']:g} 秒" if wait else item["label"]
            self.table.insert("", "end", iid=str(i), values=(i+1, "⏱ 等待" if wait else "航点", value, ""),
                              tags=("wait",) if wait else ())
        if selection is not None and self.steps:
            selected = str(max(0, min(selection, len(self.steps)-1)))
            self.table.selection_set(selected)
            self.table.see(selected)
        wait_total = sum(s["seconds"] for s in self.steps if s["type"] == "wait")
        count = sum(s["type"] == "waypoint" for s in self.steps)
        self.summary.set(f"共 {len(self.steps)} 块 · {count} 个航点 · 等待合计 {wait_total:g} 秒")
        self.root.title(f"{'* ' if self.dirty else ''}{self.path.name if self.path else '未命名'} · 独立路线编排演示")

    def insert(self, item):
        selected = self.selected()
        pos = len(self.steps) if selected is None else selected + 1
        self.steps.insert(pos, item)
        self.dirty = True
        self.refresh(pos)

    def add_waypoint(self):
        if self.running: return
        label = simpledialog.askstring("添加航点", "输入名称，例如 a1（仅作模拟标签）：", parent=self.root)
        if label is None: return
        try: self.insert(validate_steps([{"type": "waypoint", "label": label}])[0])
        except ValueError as exc: messagebox.showerror("名称错误", str(exc), parent=self.root)

    def ask_seconds(self, initial=15):
        return simpledialog.askfloat("等待块", "等待秒数（大于 0，最多 86400）：", initialvalue=initial,
                                     minvalue=0.001, maxvalue=86400, parent=self.root)

    def add_wait(self):
        if self.running: return
        seconds = self.ask_seconds()
        if seconds is not None: self.insert({"type": "wait", "seconds": seconds})

    def edit(self):
        if self.running: return
        i = self.selected()
        if i is None: return
        item = self.steps[i]
        if item["type"] == "wait":
            value = self.ask_seconds(item["seconds"])
            if value is None: return
            updated = {"type": "wait", "seconds": value}
        else:
            value = simpledialog.askstring("编辑航点", "航点名称：", initialvalue=item["label"], parent=self.root)
            if value is None: return
            updated = {"type": "waypoint", "label": value}
        try: self.steps[i] = validate_steps([updated])[0]
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc), parent=self.root)
            return
        self.dirty = True
        self.refresh(i)

    def move(self, offset):
        if self.running: return
        i = self.selected()
        if i is None or not 0 <= i+offset < len(self.steps): return
        self.steps[i], self.steps[i+offset] = self.steps[i+offset], self.steps[i]
        self.dirty = True
        self.refresh(i+offset)

    def delete(self):
        if self.running: return
        i = self.selected()
        if i is None: return
        self.steps.pop(i)
        self.dirty = True
        self.refresh(i)

    def may_discard(self):
        return not self.dirty or messagebox.askyesno("尚未保存", "放弃未保存的更改？", parent=self.root)

    def new(self):
        if self.running or not self.may_discard(): return
        self.steps, self.path, self.dirty = [], None, False
        self.refresh()
        self.status.set("已新建空路线。")

    def example(self):
        if self.running or not self.may_discard(): return
        self.steps = [{"type": "waypoint", "label": label} for label in ["a1", "a2", "a3", "a4"]]
        self.steps += [{"type": "wait", "seconds": 15.0}]
        self.steps += [{"type": "waypoint", "label": label} for label in ["a3", "a2", "a1"]]
        self.path, self.dirty = None, True
        self.refresh()
        self.status.set("示例：a1 → a2 → a3 → a4 → 等待 15 秒 → a3 → a2 → a1。")

    def open(self):
        if self.running or not self.may_discard(): return
        path = filedialog.askopenfilename(parent=self.root, filetypes=[("路线演示 JSON", "*.json")])
        if not path: return
        try: steps = load_storyboard(path)
        except (OSError, ValueError, TypeError, RecursionError) as exc:
            messagebox.showerror("打开失败", str(exc), parent=self.root)
            return
        self.steps, self.path, self.dirty = steps, Path(path), False
        self.refresh()
        self.status.set("已读取离线路线。")

    def save(self):
        if self.running: return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json",
                    initialfile=self.path.name if self.path else "offline_route.json", filetypes=[("JSON", "*.json")])
        if not path: return
        try: save_storyboard(path, self.steps)
        except (OSError, ValueError) as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)
            return
        self.path, self.dirty = Path(path), False
        self.refresh()
        self.status.set(f"已保存：{self.path}")

    def lock(self, running):
        self.running = running
        for button in self.edit_buttons: button.configure(state="disabled" if running else "normal")
        self.play_button.configure(state="disabled" if running else "normal")
        self.pause_button.configure(state="normal" if running else "disabled", text="暂停")
        self.stop_button.configure(state="normal" if running else "disabled")

    def play(self):
        if self.running: return
        if not self.steps:
            messagebox.showinfo("空路线", "请先添加航点或等待块。", parent=self.root)
            return
        self.refresh()
        self.index = 0
        self.lock(True)
        self.start_step()
        self.tick()

    def start_step(self):
        item = self.steps[self.index]
        self.clock.start(item["seconds"] if item["type"] == "wait" else 0.8, time.monotonic())
        self.table.item(str(self.index), tags=("active",))
        self.table.see(str(self.index))

    def tick(self):
        self.after_id = None
        if not self.running: return
        left = self.clock.left(time.monotonic())
        item = self.steps[self.index]
        state = "已暂停" if self.clock.paused else f"剩余 {left:.1f} 秒"
        self.table.set(str(self.index), "state", state)
        label = f"等待 {item['seconds']:g} 秒" if item["type"] == "wait" else f"模拟航点 {item['label']}"
        self.status.set(f"离线预览 {self.index+1}/{len(self.steps)}：{label} · {state}")
        if not self.clock.paused and left <= 0:
            self.table.set(str(self.index), "state", "完成（模拟）")
            self.table.item(str(self.index), tags=("wait",) if item["type"] == "wait" else ())
            self.index += 1
            if self.index == len(self.steps):
                self.lock(False)
                self.status.set("离线预览完成。未向任何机器人发送命令。")
                return
            self.start_step()
        self.after_id = self.root.after(50, self.tick)

    def toggle_pause(self):
        if not self.running: return
        if self.clock.paused:
            self.clock.resume(time.monotonic())
            self.pause_button.configure(text="暂停")
        else:
            self.clock.pause(time.monotonic())
            self.pause_button.configure(text="继续")

    def cancel(self):
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None
        if self.running:
            self.table.set(str(self.index), "state", "已取消")
        self.lock(False)
        self.status.set("预览已取消，不再推进后续步骤。")

    def close(self):
        if not self.may_discard(): return
        self.cancel()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    RouteEditor(root)
    root.mainloop()