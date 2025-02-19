# 多船路徑規劃與 FCC-A*

本存儲庫包含一個基於 FCC-A* 演算法的多船路徑規劃系統的實現。該系統旨在計算多個船隻在共用環境中航行的無碰撞路徑。實現包括路徑平滑和使用 `pygame` 進行視覺化的選項。

## 文件概覽

- **`multi_ship_planner_v1.py`**：多船路徑規劃演算法的核心實現，包括路徑衝突檢測和平滑技術。
- **`ship_navigation_v1.py`**：提供基於 FCC-A* 計算和管理船舶路徑的介面。
- **`plain_animation.py`**：使用 `pygame` 進行視覺化，可進行步進式路徑視覺化和平滑動畫船舶移動。



## 使用方法

### 運行視覺化

要啟動視覺化（支援步進模式和動畫模式），運行：

```sh
python plain_animation.py
```

按 `Shift` 可在 **步進模式** 和 **動畫模式** 之間切換。
按 `Space` 可暫停/恢復動畫。

### 導入多船路徑規劃

可以在自己的 Python 腳本中使用 `multi_ship_planning` 函數，如下所示：

```python
from ship_navigation_v1 import multi_ship_planning

# 定義多個船隻的起始位置和目標
ships_data = [
    {"id": "ShipA", "pos": (2.2, 1.5), "goal": (17.4, 9.7)},
    {"id": "ShipB", "pos": (5.1, 1.5), "goal": (15.3, 8.7)},
    {"id": "ShipC", "pos": (7.9, 1.5), "goal": (14.7, 12.4)},
]

# 運行多船路徑規劃
results = multi_ship_planning(
    ships=ships_data,
    safe_distance=1.0,  # 船隻之間的最小安全距離
    grid_scale=0.2,  # 網格解析度
    smoothing_method="bezier",  # 可選："none", "moving_average", "bezier"
)

# 輸出每艘船的路徑
for ship_id, data in results.items():
    print(f"{ship_id} 路徑: {data['path']}")
    print(f"{ship_id} 航向: {data['headings']}")
```

## 功能特點

- **多船路徑規劃**：計算多個船隻的安全無碰撞航線。
- **路徑衝突解決**：動態檢測和解決潛在衝突。
- **路徑平滑**：支援不同的平滑方法（`moving_average`, `bezier`）。
- **即時視覺化**：使用 `pygame` 顯示路徑和動畫。
- **用戶交互**：可交互地切換視覺化模式和調整路徑平滑方式。


