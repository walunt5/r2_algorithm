# R2 Touch UI KFS Route Editor

新增功能：二区路线模式/KFS模式，保存到 `~/techx_R2_algorithm/r2_algorithm/src/r2_bt_executor/config/meilin_map.yaml`。

运行：

```bash
pip3 install -r requirements.txt
python3 main.py
```

## 本版修改说明：路线合法性限制 + 颜色互换

### 颜色修改

保留“颜色随 YAML 高度变化”的版本，但是把 200mm 和 400mm 的颜色互换：

- `Block_200` 使用原来 400mm 的颜色
- `Block_400` 使用原来 200mm 的颜色

### 路线选择限制

在“路线模式”下：

1. 第一个方块只能选择 `B1 / B2 / B3`
2. 后续只能选择和上一个方块上下左右相邻的方块
3. 已选路线中只能撤销最后一个方块
4. 如果想从中间改路线，先点击 `CLEAR 清`

### KFS 模式不受路线限制

在“KFS模式”下，点击任意方块只会切换：

```yaml
blocks.Bx.has_kfs
```

不会影响路线顺序，也不会修改：

```yaml
height
kfs_height
```

## 本版 UI 修改说明

本工程可以直接覆盖到：

```bash
/home/xie/techx_R2_algorithm/r2_algorithm/ui
```

修改内容：

1. 分辨率保持 `900x600`。
2. 保留“方块颜色随 `meilin_map.yaml` 中 `height` 变化”的逻辑。
3. 交换 200mm 和 400mm 的颜色：
   - 200mm 使用原来 400mm 的颜色
   - 400mm 使用原来 200mm 的颜色
4. KFS 状态更醒目：
   - 有 KFS 的方块显示 `★有KFS★`
   - 有 KFS 的方块使用亮紫/亮粉色粗边框
   - 有 KFS 的方块字体略微缩小，避免三行文字太挤
5. 保留路线限制：
   - 第一个方块只能选 B1/B2/B3
   - 后续只能选上下左右相邻方块
   - 只能撤销最后一个方块
6. 保存配置时只修改：
   - `routes.zone2_main.blocks`
   - `blocks.Bx.has_kfs`
7. 保存配置时不会修改：
   - `blocks.Bx.height`
   - `blocks.Bx.kfs_height`

覆盖方法：

```bash
cd /home/xie/techx_R2_algorithm/r2_algorithm
cp -r ui ui_backup_$(date +%Y%m%d_%H%M%S)
unzip -o /你的下载路径/ui_kfs_bright_900x600_overwrite.zip -d ui
cd ui
python3 main.py
```


## 本版修改说明：KFS 改为字体颜色提示

本版用于直接覆盖：

```bash
/home/xie/techx_R2_algorithm/r2_algorithm/ui
```

修改内容：

1. 保留“方块颜色随 `meilin_map.yaml` 中 `height` 变化”的逻辑。
2. 交换 200mm 和 400mm 的颜色：
   - 200mm 使用原来 400mm 的颜色
   - 400mm 使用原来 200mm 的颜色
3. 不再给 KFS 方块添加紫色/粉色边框。
4. KFS 状态通过方块内部第三行字体颜色提示：
   - 有 KFS：显示 `★ 有KFS ★`，字体为金色
   - 无 KFS：显示 `无KFS`，字体为白色
5. KFS 第三行字体略微缩小，避免三行文字太挤。
6. 保留路线限制：
   - 第一个方块只能选 B1/B2/B3
   - 后续只能选上下左右相邻方块
   - 只能撤销最后一个方块

覆盖方法：

```bash
cd /home/xie/techx_R2_algorithm/r2_algorithm
cp -r ui ui_backup_$(date +%Y%m%d_%H%M%S)
unzip -o /你的下载路径/ui_kfs_font_color_900x600_overwrite.zip -d ui
cd ui
python3 main.py
```


## 本版修改说明：去除路线限制 + 修复保存配置

本版可以直接覆盖到：

```bash
/home/xie/techx_R2_algorithm/r2_algorithm/ui
```

修改内容：

1. 去除路线选择限制：
   - 第一个方块不再限制为 B1/B2/B3。
   - 后续方块不再要求和上一个相邻。
   - 已选择的任意方块再次点击即可移除。
2. 修复保存配置：
   - 保存路径会从当前 UI 文件位置反推工程根目录。
   - 目标文件为：`r2_algorithm/src/r2_bt_executor/config/meilin_map.yaml`。
   - 如果 UI 工程确实在 `r2_algorithm/ui` 下，点击“保存配置”会直接修改源码目录里的 `meilin_map.yaml`。
3. 保存时只修改：
   - `routes.zone2_main.blocks`
   - `blocks.Bx.has_kfs`
4. 保存时不会修改：
   - `blocks.Bx.height`
   - `blocks.Bx.kfs_height`
5. 如果路线为空，会保留 YAML 里的原路线，只保存 KFS 状态。
6. 保留 KFS 字体颜色提示：
   - 有 KFS：金色 `★ 有KFS ★`
   - 无 KFS：白色 `无KFS`
7. 保留 200mm 和 400mm 方块颜色互换。

如果你的工程根目录不是默认结构，可以启动前设置：

```bash
export R2_ALGORITHM_ROOT=/home/xie/techx_R2_algorithm/r2_algorithm
python3 main.py
```
