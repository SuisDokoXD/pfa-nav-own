# TASKS — 审计工具与本轮工作边界

本目录只做**离线只读审计**。下面记录"已经做掉的重复性工作"和"下一步需要证据支撑的单模块任务"。
所有任务都遵守：一次只动一个核心模块、必须有对照证据、不以 RViz 视觉效果代替定量指标。

## 明确不改（本目录任何任务都不涉及）

LIO 核心算法与参数、TF 树与坐标变换逻辑、GICP/NDT 配准参数、NAV2 规划器/控制器/代价地图参数、底盘驱动与串口/CAN 层。
这些属于核心导航行为，测试与审计工具都不去触碰，也不对它们下结论。

## 已完成（本轮，纯工具侧重复性工作）

- [x] 新建 `tools/audit/`：`workspace_audit.py`、`test_workspace_audit.py`、`README.md`、`TASKS.md`。
- [x] 工作区遍历：排除 `.git`/`build`/`install`/`log`/`__pycache__`，跳过符号链接，保证不越出 `--root`，顺序稳定；
      `os.walk(onerror=...)` 收集目录读取失败 → `TRAVERSAL_FAILED` + `scan_complete=false` + 退出码 2。
- [x] `package.xml` 解析（`ElementTree`）：根标签必须是 `<package>`（否则 `PACKAGE_ROOT_TAG_INVALID`）、包名、版本、
      相对路径、声明依赖（含 `buildtool_export_depend`）、`condition` 原文保留不求值、本地依赖（与工作区内其它包名求交）、
      重复包名、坏 XML、缺 `<name>`。
- [x] `.py` 语法检查：`ast.parse`，绝不 import；逐文件 OSError 报告为 `FILE_UNREADABLE` 并降级为 `skipped`。
- [x] `.sh` 语法检查：`bash -n`（绝不执行，最小环境隔离 `BASH_ENV`/`ENV`/`BASHOPTS`/`SHELLOPTS`）；
      只有 rc=2 才算语法错误，126/127/其它记 `SHELL_CHECK_FAILED` + `skipped`；bash 缺失、超时、无法启动、
      脚本本身不可读一律记 `skipped`，不算通过。
- [x] 已知导航入口资产的存在/非空检查（4 个路径），逐级 `lstat`：任一级为符号链接（含指向 root 内部）或 stat 失败
      → `ASSET_UNVERIFIED` + `skipped`；布局根缺失 → `not_applicable`；布局根被链接 → `skipped`（未验证≠不适用）。
- [x] JSON 输出契约：`--root` 必填、默认 stdout、`--output` 才写、原子写（同目录临时文件 + `fsync` + `os.replace` + 失败清理）。
- [x] 退出码 0/1/2 与 `passed/failed/skipped/warning/not_applicable` 统计分离；遍历失败同样退 2（报告仍然输出）。
- [x] 44 个 unittest 用例（含符号链接逃逸、`onerror`、适用性判定、bash 环境隔离、rc 语义、文档边界回归）。
- [x] 实测：`pfa-nav-own` 下 37 个包、0 error、2 warning（`install/setup.bash` 与 `pcd/reality/game.pcd` 缺失）。

## 下一步：单核心模块任务（每项都需要先拿到证据）

下面的顺序遵循"先证据、后改动，一次只碰一个模块"。每一项在动手前都应先跑一次
`workspace_audit.py` 存档基线，并把 JSON 放进 scratch，作为改动前证据。

**本轮不执行任何需要实车/实机动作的验收。** 涉及启动、实车、硬件接线的动作一律只登记为
"未来需授权操作"，必须先取得用户明确授权，并满足第 3 条的安全前提。

### T1. 资产基线核对（只读，不改代码）

- 目标：判断 `install/setup.bash` 与 `pcd/reality/game.pcd` 的缺失属于哪种情况：
  "本地未构建"、"构建产物从未提交（被 .gitignore 忽略）"、还是"曾经提交后被删除"。
- 需要的证据：构建日志/CI 产物、`.gitignore` 条目、`git log --diff-filter=D -- <path>`、历史 tag 内的目录清单。
- 结论规则：`git log` 只能看到**提交历史**，看不到构建产物——构建产物本来就可能被 `.gitignore` 忽略，
  所以**状态可能为未知**。三种证据不足以区分时，必须如实标注 `unknown`，不得猜成"未构建"或"从未提交"。
- 验收：给出证据清单 + 结论（含 `unknown` 情形）；**不得据此修改**代码或资产。
- 风险：把构建产物当源码资产提交进仓库；或凭单一线索（仅 git log）下结论。

### T2. `package.xml` 依赖一致性（单模块：包元数据）

- 目标：找出 `CMakeLists.txt` 已用但 `package.xml` 未声明的依赖（只针对一个包，先做 `pb2025_nav_bringup`）。
- 需要的证据：该包的 `CMakeLists.txt`/`setup.py` 中的依赖列表，与审计输出的 `declared_dependencies` 逐项对照。
- 验收：差异清单 + 一次只改该包 `package.xml` 的最小补丁；改后 `colcon build --packages-select <包名>` 通过。
- 注意：审计输出的 `local_dependencies` 是**声明图**，`condition` 未求值，不能当成运行期依赖图；
  不能据此断言某个包"运行时必然被加载"。
- 风险：往 `package.xml` 里塞 `depend` 掩盖真实缺包，导致运行期才暴露。

### T3. Shell 入口脚本的失败传播（单模块：launch/自启动脚本）

- 目标：对自启动脚本做失败传播审查（`bash -n` 已覆盖语法，人工看 `set -e`/`set -o pipefail`/错误吞掉）。
- 需要的证据：`autostart_navigation/`、`autostart_mid360_record/` 下的脚本清单与审计 JSON。
- 验收：仅修改被点名的脚本，改后用 `bash -n` 复核语法。
- **本轮不做**真实开机/自启动验证（会改变开机行为）。
- 未来需授权操作：台架上的一次真实启动与日志采集（含急停可用性确认），授权后单独执行。
- 风险：改动自启动脚本会改变开机行为，必须先在台架上验证。

### T4. 地图/点云资产一致性（单模块：地图资产，不动配准）

- 目标：确认 `game.yaml` 的 `image`/`resolution`/`origin` 与 `game.pgm` 的**像素/米制关系**是否自洽
  （宽高与 resolution）、以及 `game.pcd` 的坐标系与地图 origin 的关系是否**有文档记录**。
- 需要的证据：`game.yaml` 四个字段、`game.pgm` 的 PGM 头（宽/高/最大灰度）、`map_server` 加载日志、
  以及（如需涉及点云）一次 rosbag 回放的配准/定位指标作为对照。
- 关键限制：**PGM 头不能证明 origin 与坐标系一致**。头信息只给出像素尺寸与灰度范围，
  origin/坐标系属于 YAML 与外参语义，必须靠单独的量测或日志证明，不能由 PGM 头"推出"。
- 结论规则：不一致时只写清实测数字与差异来源，**不自动修改 YAML、不自动补 PCD**；
  任何修改都需要单独提案 + 证据 + 授权。
- 风险：没有 rosbag 对照就改 `pcd` 或 origin，会直接改变定位先验；审计工具只查文件存在，不构成匹配证据。

### T5. 未知布局判定规则（单模块：本审计工具自身）

- 目标：`pb2025_sentry_nav`（另一个 root）下 `src/` 没有嵌套同名目录时，`required_assets` 走 `not_applicable`；
  若要支持第二种布局，需要明确入口路径表而不是模糊匹配。
- 需要的证据：两个 root 的扫描 JSON（`pfa-nav-own` 与 `pb2025_sentry_nav`）。
- 验收：为第二种布局增加一张**显式**路径表并存档两份 JSON；不引入"猜路径"的启发式。
- 风险：把 `not_applicable` 或 `skipped` 误读成"通过"。

## 验收分层与安全前提（与 README 一致）

1. **离线层**（本工具、构建、单元测试、rosbag 回放的定位误差/频率/延迟/TF 连续性/配准质量）：
   可以在无硬件条件下执行。
2. **隔离硬件层**（台架/吊起/限速/物理急停）：需要授权与场地。
3. **实车层**：**安全全停是任何实车测试的前提**；`rosbag` 指标不能替代实车安全实验，
   停障（遮挡激光）、断联（断网/断串口）、急停响应必须单独实测。

## 使用约定

1. 任何任务开始前：`PYTHONDONTWRITEBYTECODE=1 TMPDIR=$PI_SCRATCH_DIR python3 -B tools/audit/workspace_audit.py --root <ws> --output $PI_SCRATCH_DIR/before.json`。
2. 改动后：同样命令输出 `after.json`，对比 `summary`（含 `scan_complete`/`traversal_failures`）、`checks`、`packages`、`findings` 的差异。
   注意退出码 2 代表扫描不完整，此时 `errors=0` 不能当作"没问题"。
3. 审计 exit 0 只是"没有结构 error 且遍历完整"，禁止据此声称可上车；可上车结论需要离线层 + 隔离硬件层 + 实车层分别验收，
   实车层以安全全停为前提。
4. 一次提交只动一个核心模块；不修改 LIO/TF/GICP/NAV2/底盘核心。
5. 本工具按静态工作树假设工作，不做 TOCTOU 防护，也不是安全沙箱；不可信来源的工作区需另行加固。
