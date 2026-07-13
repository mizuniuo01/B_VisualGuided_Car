# 嵌入式项目开发规范（个人速查版）

> 给自己看的速查手册。开发时扫一眼，不用背。

---

## 一、项目目录结构（通用部分）

### 1.1 根目录文件（Root Files）

| 文件 | 用途 | 优先级 |
|------|------|--------|
| `README.md` | 项目概述、快速开始、编译烧录说明 | **必选** |
| `LICENSE` | 开源许可证（个人推荐 MIT） | **必选** |
| `.gitignore` | Git 忽略规则（编译产物、IDE 临时文件） | **必选** |
| `CHANGELOG.md` | 版本更新记录 | **必选** |
| `Makefile` / `CMakeLists.txt` | 构建系统入口 | **必选** |
| `.editorconfig` | 统一编辑器缩进、换行符 | **推荐** |
| `.clang-format` | 代码格式化规则 | **推荐** |
| `.gitattributes` | Git 属性（行尾符、合并策略） | 推荐 |
| `CONTRIBUTING.md` | 贡献指南（个人用可略） | 可选 |
| `CODE_OF_CONDUCT.md` | 行为准则（个人用可略） | 可选 |
| `.env` / `.env.example` | 环境变量模板 | 可选 |
| `pyproject.toml` / `requirements.txt` | Python 依赖（若 scripts/ 用 Python） | 可选 |
| `Doxyfile` | 自动文档生成配置 | 可选 |
| `.dockerignore` | Docker 忽略文件 | 可选 |
| `docker-compose.yml` | 容器编排（非嵌入式常用） | 可选 |

### 1.2 根目录文件夹（Root Directories）

| 目录 | 用途 | 优先级 |
|------|------|--------|
| `docs/` | 详细文档（架构设计、调试指南、硬件手册） | **必选** |
| `scripts/` | 自动化脚本（格式化、下载、打包、版本生成） | **必选** |
| `.github/` | GitHub 专属配置（Issue 模板、PR 模板、Actions） | **推荐**（用 GitHub 则必选） |
| `src/` | 源码目录（嵌入式细分见下节） | **必选** |
| `lib/` / `third_party/` | 第三方库 / SDK（如 STM32 HAL、FreeRTOS） | **推荐** |
| `build/` | 编译输出目录（.o、.elf、.map），加入 `.gitignore` | **推荐** |
| `tests/` | 单元测试 / 集成测试代码 | 可选 |
| `examples/` | 示例代码（供自己日后复用） | 可选 |
| `tools/` | 工具软件（烧录器、调试器可执行文件）或工具源码 | 可选 |
| `dist/` / `output/` | 发布产物（生成的 .bin / .hex 固件） | 可选 |
| `config/` | 配置文件（设备树、JSON / YAML 参数配置） | 可选 |
| `.vscode/` | VS Code 工作区配置（launch.json、tasks.json、推荐插件） | 可选（个人习惯） |
| `.idea/` | CLion / IDEA 配置 | 可选（个人习惯） |
| `data/` | 数据文件（测试向量、预置资源） | 可选 |

---

## 二、重点目录/文件详解

### 2.1 README.md（项目门面）

必须包含六项，确保半年后自己能快速上手：

```
1. 项目一句话简介
2. 支持芯片 / 开发板
3. 环境依赖（工具链版本、烧录器驱动）
4. 编译命令（make 或 cmake 指令）
5. 烧录命令
6. 最小使用示例
```

### 2.2 CHANGELOG.md（版本时间线）

按版本倒序记录，比翻 `git log` 快得多：

```markdown
## [v1.1.0] - 2026-07-13

### Added
- 新增 UART DMA 发送接口

### Fixed
- 修复 GPIO 初始化引脚模式错误

### Changed
- 优化 SPI 时钟分频配置

### Removed
- 移除废弃的旧版校验函数
```

规则：**每次打 Tag 发版前更新此文件**。

### 2.3 .gitignore（核心忽略清单）

嵌入式通用忽略模板：

```
# 编译产物
*.o
*.elf
*.hex
*.bin
*.map
*.lst
build/
dist/

# IDE 临时文件
.vscode/
.idea/
*.swp
*.swo
*~

# 操作系统垃圾
.DS_Store
Thumbs.db

# 环境变量（若含敏感路径）
.env
```

### 2.4 docs/（知识库）

存放 README 放不下的深度内容：

```
docs/
├── architecture.md   # 软件架构说明
├── build.md          # 详细编译指南（多环境）
├── debug.md          # 调试方法（JTAG/SWD、日志）
├── hardware.md       # 硬件原理图要点 / 引脚分配表
└── migration.md      # 芯片/版本迁移记录
```

### 2.5 scripts/（自动化军火库）

把重复操作写成脚本，一次编写永久复用：

```
scripts/
├── format.sh            # 调用 clang-format 格式化全部源码
├── build_all.py         # 批量编译多个配置（Debug/Release）
├── download.sh          # 调用 openocd / pyocd 烧录
├── gen_version.sh       # 从 Git tag 自动生成 version.h
└── clean.sh             # 清理编译产物
```

### 2.6 .github/（GitHub 全能力）

```
.github/
├── ISSUE_TEMPLATE/
│   └── bug_report.md    # Bug 报告模板
├── PULL_REQUEST_TEMPLATE.md  # PR 模板（个人用可略）
└── workflows/
    └── build.yml        # CI：每次 push 自动编译检查
```

---

## 三、源码目录分层（嵌入式次要部分）

```
src/
├── app/          # 应用层（业务逻辑、状态机）—— 不准直接操作寄存器
├── bsp/          # 板级支持包（LED1→PA5 引脚映射、板级初始化）
├── drivers/      # 外设驱动（uart.c / spi.c / gpio.c 功能实现）
├── hal/          # 硬件抽象层（屏蔽芯片差异，提供统一接口）
├── device/       # 芯片底层（startup.s、中断向量表、链接脚本 .ld）
└── include/      # 全局公共头文件（若各层头文件已内聚可不要）
```

### 层间调用铁律

```
app → bsp → drivers → hal → device
```

- `app` 不允许直接调用 `hal`/`device`，必须通过 `bsp` 或 `drivers` 中转。
- 换芯片时只动 `hal/` 和 `device/`，上层代码（app/bsp）理论上不需要改。

---

## 四、Git Commit 规范

### 格式

```
<type>(<scope>): <英文描述>
```

标题不超过 50 字符。描述“做了什么”，不描述“开发过程”。

### Type（必选）

| Type | 含义 |
|------|------|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `docs` | 文档修改 |
| `refactor` | 重构（不改变功能） |
| `chore` | 工程配置 / 脚本 / 工具链 |
| `test` | 测试相关 |
| `style` | 格式化（不影响逻辑） |

### Scope（建议）

```
app / bsp / driver / hal / device / build / docs / script / github / lib
```

### 正反例

❌ 差：

```
update
fix bug
改代码
```

✅ 好：

```
feat(uart): add dma transmit support
fix(gpio): correct pin mode for output
chore(build): update arm-none-eabi-gcc to 13.2
docs(readme): add flash instruction
refactor(hal): simplify spi callback
```

### 粒度原则

一个 commit 只做一件事，不要把“加 UART + 改 GPIO + 更新文档”混在一起。

---

## 五、GitHub 使用（个人精简版）

### 分支策略

```
main（稳定版，永远可编译可运行）
 └── feature/xxx（功能开发 / Bug 修复）
```

- 个人项目不需要 `develop` / `release` / `hotfix`，太重了。
- 功能完成后合并回 `main`，删除远端 feature 分支（可选）。

### Tag / Release（版本号）

格式：`v主版本.次版本.修订版`

| 版本 | 场景 |
|------|------|
| `v1.0.0` | 首个可用版本 |
| `v1.1.0` | 向下兼容的新功能 |
| `v1.1.1` | Bug 修复 |
| `v2.0.0` | 不兼容的大改 |

> 每次稳定版本打 Tag，并在 Release 页面附上编译好的 `.bin` / `.hex` 固件。

### Issue（记两件事）

1. **Bug**：现象、复现步骤、期望结果
2. **TODO**：下一步要开发的功能规划

### GitHub Actions（自动编译）

至少做这一个 CI，防止 push 坏代码：

```yaml
name: Build
on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Build
        run: make
```

---

## 六、推荐开发流程（完整闭环）

```
1. 从 main 切出 feature/xxx
          ↓
2. 开发功能，按规范逐条 commit
          ↓
3. 本地编译 + 烧录 + 功能测试
          ↓
4. 合并回 main
          ↓
5. 更新 CHANGELOG.md
          ↓
6. 打 Tag（如 v1.1.0）
          ↓
7. 创建 Release，上传固件
```

---

## 七、最小工程检查清单

新建项目时，逐项打勾：

- [ ] `README.md`（项目说明 + 快速开始）
- [ ] `LICENSE`（MIT）
- [ ] `.gitignore`（忽略编译产物和 IDE 文件）
- [ ] `CHANGELOG.md`（版本记录）
- [ ] `docs/` 目录存在
- [ ] `scripts/` 目录存在
- [ ] `src/` 目录存在（内含 app/bsp/drivers/hal/device）
- [ ] `lib/` 或 `third_party/` 存放 SDK
- [ ] `build/` 目录已加入 `.gitignore`
- [ ] `.github/workflows/build.yml` 自动编译
- [ ] 首个稳定版本打了 `v1.0.0` Tag
- [ ] 每次 commit 符合 `type(scope): description` 格式

---

## 八、快速参考卡片（放在桌面）

```
Commit: feat(scope): desc
Scope: app | bsp | driver | hal | device | build | docs | script | github | lib
Tag:    v1.0.0 → v1.1.0（新功能）→ v1.1.1（修 Bug）

目录:
  docs/      详细文档
  scripts/   自动化脚本
  .github/   Issue + Actions
  src/       app/bsp/drivers/hal/device
  lib/       第三方 SDK
  build/     (.gitignore)

规则: app 不准直接调寄存器，必须走下层接口
```