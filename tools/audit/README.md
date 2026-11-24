# workspace_audit.py — 离线只读工作区审计

一个只用 Python 标准库的单文件 CLI，在**不导入、不执行、不修改**被审计工程的前提下，
收集 ROS/导航工作区的**结构证据**：包结构与依赖、Python 语法、Shell 语法、已知导航入口资产的存留。

它回答的是"仓库结构与语法是否自洽"，**不回答**"车能不能跑"。

## 安全边界（硬约束）

| 项目 | 本工具的做法 |
| --- | --- |
| 导入工程代码 | 从不。`.py` 只用 `ast.parse()` 解析语法，绝不 import/exec |
| 执行工程脚本 | 从不。`.sh` 只跑 `bash -n`（仅语法解析，不执行任何语句） |
| ROS | 不启动、不 source、不调用 `ros2`/`colcon` |
| 网络 | 不联网 |
| 安装/修复 | 不安装依赖，不修改任何 C++ / 配置 / launch / package.xml |
| 写入 | 默认不写任何文件；仅当显式传 `--output` 时写一个 JSON（同目录临时文件 + `os.replace` 原子替换，失败时清理临时文件且不触碰原文件） |
| 扫描范围 | 跳过 `.git` / `build` / `install` / `log` / `__pycache__`；跳过符号链接；不越出 `--root` |

`bash -n` 只做词法/语法解析，不展开命令替换、不执行任何命令，因此对含 `touch`/`rm` 等的脚本是安全的；
这一点有回归测试守护（见 `test_project_scripts_are_never_executed`）。

### 静态工作树假设（重要）

本工具按**静态工作树**假设工作：审计期间假定工作区内容不被并发修改，输入是可信的本地文件。
它**不做** TOCTOU（检查—使用竞态）防护，**不是**对抗性输入的安全沙箱，也不试图防御恶意构造的
`package.xml` / 巨大目录树 / 递归挂载点。若需要审计来源不可信的工作树，必须另行加固（容器、只读挂载、
资源上限），不在本工具职责内。

### 符号链接与越界策略

- 遍历：文件与目录的符号链接**都不跟随**，只计入 `summary.symlinks_skipped`。
- 资产/布局路径：`src/pb2025_sentry_nav`、`install/setup.bash`、`game.yaml`、`game.pgm`、`game.pcd`
  的**每个路径组成部分**都用 `lstat`（`follow_symlinks=False`）逐级检查；
  只要任一级是符号链接——**即使它指向 root 内部**——该资产判为"不可验证"（`ASSET_UNVERIFIED`，`skipped`），
  绝不会因为"最终目标是普通文件"就记成 `passed`。
- 因为逐级用 `lstat` 而不是对最终路径 `stat`，**不存在经由父目录符号链接逃出 root 的路径**；
  另外还有一次 `commonpath` 兜底检查。
- `stat` 本身失败（EACCES 等）同样记为"不可验证"（`skipped`），不是 `passed`，也不是 `warning` 级别之外的成功。
- 语义区分：布局根**不存在** → `required_assets: not_applicable`（不适用）；
  布局根存在但被链接/不可 stat → `required_assets: skipped`（未验证）。两者都不等于 `passed`。

## 运行

推荐的"干净运行"方式（不生成字节码、临时与输出都落在本次会话的 scratch 目录）：

```bash
PYTHONDONTWRITEBYTECODE=1 TMPDIR=$PI_SCRATCH_DIR python3 -B tools/audit/workspace_audit.py \
  --root /home/tompig/src/sight/pfa-nav-own \
  --output $PI_SCRATCH_DIR/workspace_audit.json
```

默认只把 JSON 打到 stdout（不写文件）：

```bash
PYTHONDONTWRITEBYTECODE=1 TMPDIR=$PI_SCRATCH_DIR python3 -B tools/audit/workspace_audit.py \
  --root /home/tompig/src/sight/pfa-nav-own
```

参数：

- `--root`（必填）：已存在的工作区根目录。
- `--output`（可选）：JSON 输出路径。**其父目录必须已存在**（工具不会替你建目录）；
  指定后 stdout 为空，只在 stderr 打印一行写入提示。

## 退出码

| 码 | 含义 |
| --- | --- |
| 0 | 未发现 error，且遍历完整（可能仍有 warning / skipped；**不代表可上车**） |
| 1 | 发现至少一个 error（XML 结构/根标签、包名、Python 语法、Shell 语法） |
| 2 | CLI / IO 失败：`--root` 不存在或不可读、扫描抛 OSError、`--output` 写入失败、**或目录遍历失败** |

遍历失败时本工具**仍会把 JSON 报告输出**（作为证据）后再返回 2，并在 stderr 说明"扫描不完整"。
退出码由 `summary.traversal_failures` / `summary.errors` 决定，warning 与 skipped 不会把退出码变成 1。

## 检查项与状态

固定 5 个检查项，`checks[].status` 取值与优先级为
`failed > not_applicable > skipped > warning > passed`：

| 检查项 | 内容 | 何时不是 passed |
| --- | --- | --- |
| `workspace_layout` | 布局根 `src/pb2025_sentry_nav` 的逐级状态、扫描文件数、跳过链接数 | 布局缺失→`not_applicable`；被链接/不可 stat/遍历失败→`skipped` |
| `ros_packages` | `ElementTree` 解析所有 `package.xml`：根标签必须是 `<package>`，包名、相对路径、声明依赖、本地依赖、重名 | 坏 XML / 根标签不是 `package` / 缺 `<name>` / 包名重复 → `failed` |
| `python_syntax` | `ast.parse` 每个 `.py` | 语法错误 → `failed`；文件不可读 → `skipped` |
| `shell_syntax` | 每个 `.sh` 跑 `bash -n`（隔离环境，超时 10s） | 语法错误(rc=2) → `failed`；bash 缺失/超时/无法启动/rc∉{0,2}/脚本不可读 → `skipped` |
| `required_assets` | 4 个已知导航入口资产的存在与非空（逐级 `lstat`） | 缺/空 → `warning`；被链接/不可 stat → `skipped`；布局缺失 → `not_applicable` |

**`skipped` 与 `not_applicable` 都不等于 `passed`。** `skipped` 明确表示"没验证"，`not_applicable` 表示"该布局下不适用"。

### 遍历失败（`TRAVERSAL_FAILED`）

`os.walk` 通过 `onerror` 收集目录读取失败（典型是 EACCES 的子目录），并：

- 每个失败目录产生一条 `TRAVERSAL_FAILED`（warning）诊断，`path` 为相对 root 的路径；
- `summary.traversal_failures` 计数，`summary.scan_complete` 置为 `false`；
- `workspace_layout` 降级为 `skipped`（即便布局根本来不存在，也不允许声称"不适用"）；
- 进程退出码为 2。

因此不存在"目录被静默跳过、`errors=0` 且没有任何警告"的情形。

### Shell 检查的环境隔离与返回码语义

`bash -n` 使用**最小环境**（`PATH`/`LC_ALL=C`/`LANG=C`/`TERM=dumb`），
不继承 `BASH_ENV`/`ENV`/`BASHOPTS`/`SHELLOPTS`/`CDPATH`/`PS4`/`PROMPT_COMMAND` 等 shell 启动配置。
这不是洁癖：本机实测 `BASH_ENV` 指向的文件确实会被 `bash -n` 读取并参与解析，父进程环境会污染语法判定。

返回码按 bash 语义分类（本机实测）：

| rc | 含义 | 本工具判定 |
| --- | --- | --- |
| 0 | 解析通过 | 计入通过 |
| 2 | 语法错误 | `SHELL_SYNTAX`（error） |
| 126 | 脚本不可读/不可执行 | `SHELL_CHECK_FAILED`（warning）+ `skipped`，**不叫语法错误** |
| 127 | 文件不存在 | 同上 |
| 其它 | 未知 | 同上 |

另外，调用 bash 前会先自己 `open(...,"rb")` 预读 1 字节：脚本不可读直接记 `FILE_UNREADABLE` 并 `skipped`，
不把 IO 问题误报成语法错误。bash 无法启动（`OSError`）记 `SHELL_CHECKER_UNAVAILABLE` 并 `skipped`。

### 资产检查范围（刻意限定）

只查这 4 个已知入口依赖，且只验证存在与非空：

1. `<root>/install/setup.bash`（bringup 构建产物）
2. `<root>/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.yaml`
3. `<root>/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.pgm`（按 `game.yaml` → 同名 `.pgm` 的约定推导）
4. `<root>/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd`

`install/` 在遍历时被排除（构建产物不做语法检查），但资产检查按显式路径逐级 `lstat` 它，两件事互不影响。

**不解析 YAML**，因此没有校验 `image:` 字段指向的文件名；"`game.pgm` 就是 `game.yaml` 实际引用的地图"这一点是约定假设，不是本工具证明的结论。

## 输出结构

```json
{
  "schema_version": 1,
  "tool": "workspace_audit",
  "root": "/abs/path",
  "summary": {"errors": 0, "warnings": 2,
              "checks": {"passed": 4, "failed": 0, "skipped": 0, "warning": 1, "not_applicable": 0},
              "files_scanned": 1164, "symlinks_skipped": 0, "packages_found": 37,
              "traversal_failures": 0, "scan_complete": true},
  "checks":    [{"id": "...", "status": "...", "detail": "..."}],
  "packages":  [{"name": "...", "version": "...", "path": "src/xxx",
                 "declared_dependencies": [{"name": "...", "type": "depend", "condition": "可选：条件原文"}],
                 "local_dependencies": ["..."]}],
  "findings":  [{"severity": "error|warning", "code": "...", "path": "rel/path", "message": "..."}],
  "notes":     ["..."]
}
```

- `path` 一律是相对 `--root` 的 POSIX 路径（`findings`、`packages[].path` 均如此）。
- `packages` 按 `(name, path)` 排序，`findings` 按 `(code, path, message)` 排序；输出不含时间戳，
  同一工作区两次运行的 JSON 逐字节相同（有回归测试）。
- 诊断码：`PACKAGE_XML_INVALID`、`PACKAGE_ROOT_TAG_INVALID`、`PACKAGE_NAME_MISSING`、
  `PACKAGE_NAME_DUPLICATE`、`PACKAGE_DEPENDENCY_EMPTY`（warning）、`PYTHON_SYNTAX`、
  `PYTHON_PARSE_FAILED`、`SHELL_SYNTAX`、`SHELL_CHECK_FAILED`（warning）、
  `SHELL_CHECKER_UNAVAILABLE`（warning）、`ASSET_MISSING`（warning）、`ASSET_EMPTY`（warning）、
  `ASSET_UNVERIFIED`（warning）、`LAYOUT_UNVERIFIED`（warning）、`TRAVERSAL_FAILED`（warning）、
  `FILE_UNREADABLE`（warning + 计入 skipped）。

### 依赖语义：声明图，不是运行图

- `declared_dependencies` 汇总 `depend`、`build_depend`、`build_export_depend`、`exec_depend`、
  `buildtool_depend`、`buildtool_export_depend`、`test_depend`、`doc_depend` 的直接子元素。
- 带 `condition="..."` 的属性**只保留原文**（`condition` 字段），**不求值**、不判断 ROS 版本/环境。
- `local_dependencies` = 本包声明依赖 与 工作区内其它包名 的**交集**，同样是**声明图**，
  不代表运行期真实加载的包，也不代表某个 `condition` 成立后被选中的那部分。

单个文件读取失败（OSError）会生成 `FILE_UNREADABLE` 诊断并把对应检查项降级为 `skipped`，
不会静默当作通过。

## 验证边界（不要过度解读）

本工具**不能**证明以下任何一项，也没有尝试：

- LIO / 点云配准、TF 树正确性与连续性、GICP/NDT 匹配质量；
- NAV2 行为、代价地图、控制器/规划器参数合理性；
- 地图与点云是否匹配、`game.yaml` 的 origin/resolution 是否与 `.pgm` 一致；
- 运行时频率、延迟、定位误差；
- C++ 能否编译（不调用 catkin/colcon/编译器）；
- 底盘、串口、CAN、硬件连接。

也**没有**实现"浅字符串算法探测"（例如 grep 某个阈值常量就推断算法有 bug）——那类结论不可验证，容易制造伪发现。

### 验收分层：离线 / 隔离硬件 / 实车，各自独立

`errors == 0` 只能说明"结构上没有发现明显错误"。可上车性不是一个离线工具能给出的结论，
必须分层验收，且**上一层不能替代下一层**：

1. **离线层（本工具所在层）**：结构、语法、依赖声明、资产存在性；以及构建与单元测试。
   rosbag 回放得到的定位误差、频率、延迟、TF 连续性、配准质量指标也属于这一层（离线数据分析）。
2. **隔离硬件层**：台架/吊起/限速/物理急停回路下的启动、通信、急停响应验证。
3. **实车层**：**安全全停是任何实车测试的前提**（物理急停可达、限速、场地隔离、人员就位）。
   还需要独立的停障/断联实验（遮挡激光、断网、断串口、掉电重启）。

rosbag 指标不能替代实车安全实验；离线层全部通过也不能跳过隔离硬件层。本工具只覆盖第 1 层中很小的一部分。

## 测试

```bash
cd tools/audit
PYTHONDONTWRITEBYTECODE=1 TMPDIR=$PI_SCRATCH_DIR python3 -B -m unittest -v test_workspace_audit
```

44 个用例覆盖：正常包/依赖、坏 XML、根标签不是 `<package>`、缺 `<name>`、重复包名、
适用性判定（全部不可读→`skipped`、无 manifest→`not_applicable`、坏 XML→`failed`）、
`condition` 原文保留且不求值、`buildtool_export_depend` 识别、Python/Shell 语法错误、缺资产与非空资产、
资产为符号链接（指向 root 内/root 外）、父目录符号链接不可逃逸、布局根为符号链接时"未验证≠不适用"、
未知布局 `not_applicable`、排除目录与 symlink 跳过、`os.walk.onerror` 被报告（模拟 + 真实权限）、
遍历失败退出 2 且仍输出报告、工程脚本从未被执行（含写文件语句的 `.sh`/`.py`）、
bash 缺失/超时/启动失败/rc∈{1,126,127} 均不误报语法错误、不可读 `.sh` 不误报语法、
`bash -n` 子进程环境不含 `BASH_ENV`/`ENV`/`BASHOPTS`/`SHELLOPTS` 等、恶意父进程环境不影响结果、
`--output` 写入/静默/父目录缺失退 2、`os.replace` 失败后原文件不被破坏且无残留临时文件、
`--root` 缺失/是文件/缺参数退 2、两次运行逐字节一致、不可读 `.py` 被报告且不通过，
以及文档边界回归（"静态工作树""安全全停""隔离硬件"等措辞不得丢失）。

测试临时目录全部由 `tempfile.mkdtemp()` 创建（落在 `TMPDIR` 下），并在 `setUp` 中断言确实位于 `tempfile.gettempdir()` 之内。
