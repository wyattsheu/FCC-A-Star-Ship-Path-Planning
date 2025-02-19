import math
import numpy as np
from scipy.interpolate import splprep, splev
from multi_ship_planner_v1 import n_fcc_a


def distance(p1, p2):
    """計算兩點 p1 和 p2 之間的歐幾里得距離."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def get_position_at_step(path, step_index):
    """
     取得路徑中指定步數的座標。

    參數:
      path (list): 座標列表。
      step_index (int): 步數索引。

    傳回:
      若 step_index < len(path)，則回傳 path[step_index]；
      否則回傳 path[-1]（最後一個座標）。
    """
    if step_index < len(path):
        return path[step_index]
    else:
        return path[-1]


def paths_conflict(pathA, pathB, safe_distance):
    """
    判斷兩條路徑在同一時間步是否發生衝突。

    如果在任何一個時間步，兩船之間的距離小於 safe_distance，
    則視為有衝突。

    比較時:
      - 取兩條路徑中較長的步數作為最大步數，
      - 超出部分以該船最後一個位置作為參考。

    參數:
      pathA (list): 第一條路徑（公尺座標列表）。
      pathB (list): 第二條路徑（公尺座標列表）。
      safe_distance (float): 安全距離閾值。

    傳回:
      bool: 若衝突則回傳 True，否則回傳 False。
    """
    max_steps = max(len(pathA), len(pathB))
    for step in range(max_steps):
        posA = get_position_at_step(pathA, step)
        posB = get_position_at_step(pathB, step)
        if distance(posA, posB) < safe_distance:
            return True
    return False


# --------------------
# 平滑化相關函式
# --------------------
def moving_average_smooth(path, window_size=4):
    """
    使用移動平均法對路徑進行平滑化，並保持輸出長度與輸入一致.

    參數:
      path (list): 原始路徑（座標列表）。
      window_size (int): 平滑視窗大小。

    傳回:
      list: 平滑化後的路徑。
    """
    if len(path) < window_size:
        return path[:]
    smoothed = []
    half_window = window_size // 2
    for i in range(len(path)):
        indices = range(max(0, i - half_window), min(len(path), i + half_window + 1))
        avg_x = np.mean([path[j][0] for j in indices])
        avg_y = np.mean([path[j][1] for j in indices])
        smoothed.append((avg_x, avg_y))
    return smoothed


def bezier_smooth(path, smooth_factor=0.1):
    """
    使用樣條曲線（Bézier/Spline）對路徑進行平滑化，保持輸出長度與輸入一致.

    參數:
      path (list): 原始路徑（座標列表）。
      smooth_factor (float): 平滑參數，對應於 splprep 的 s 參數。

    傳回:
      list: 平滑化後的路徑。
    """
    if len(path) < 3:
        return path[:]
    x, y = zip(*path)
    tck, u = splprep([x, y], s=smooth_factor)
    u_new = np.linspace(0, 1, len(path))  # 產生與原始點數相同的參數
    x_new, y_new = splev(u_new, tck)
    return list(zip(x_new, y_new))


def recalc_headings(path):
    """
    根據路徑計算每個步驟的航向（以度表示）。

    計算方式：
      heading[i] 為從 path[i] 指向 path[i+1] 的角度，
      最後一點延用前一點的角度。

    參數:
      path (list): 座標列表。

    傳回:
      list: 每步的航向角度（度）。
    """
    headings = []
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        heading_deg = math.degrees(math.atan2(dy, dx))
        headings.append(heading_deg)
    if len(path) > 1:
        headings.append(headings[-1])
    else:
        headings.append(0.0)
    return headings


def smooth_path(path, method="none"):
    """
    根據指定的方法對路徑進行平滑化，並重新計算航向。

    參數:
      path (list): 原始路徑（座標列表）。
      method (str): 平滑方法，可選擇 "none"、"moving_average"、"bezier"。

    傳回:
      tuple: (平滑化後的路徑, 重新計算的航向列表)
    """
    if len(path) <= 1:
        return path[:], [0.0] * len(path)

    if method == "none":
        new_path = path[:]
    elif method == "moving_average":
        new_path = moving_average_smooth(path)
    elif method == "bezier":
        new_path = bezier_smooth(path)
    else:
        new_path = path[:]

    new_headings = recalc_headings(new_path)
    return new_path, new_headings


# --------------------
# 多船規劃主函式
# --------------------
def multi_ship_planning(
    ships, safe_distance=1, grid_scale=0.2, smoothing_method="none"
):
    """
    根據多艘船的起點與目標規劃路徑，並進行衝突檢查與平滑化處理。

    規則:
      1. 根據船與目標的直線距離排序（距離最遠者優先）。
      2. 每艘船規劃時，先嘗試不加入干擾路徑。如果與已規劃船的路徑在同一時間步
         衝突（距離 < safe_distance），則將該干擾船的路徑加入干擾路徑中，並重算。
         重複此過程直到無衝突。
      3. 完成後，將每艘船的結果存入 planning_results，同時記錄新船的路徑於 previous_planned_paths，
         以供後續衝突檢查。
      4. 最後，根據 smoothing_method 參數對所有結果進行平滑化處理。

    參數:
      ships (list): 每個元素為一艘船的資訊，格式例如：
          {"id": "A", "pos": (x1, y1), "goal": (gx1, gy1)}
      safe_distance (float): 衝突判斷的安全距離閾值。
      grid_scale (float): 每格代表的公尺數。
      smoothing_method (str): 平滑化方法，可選 "none"、"moving_average" 或 "bezier"。

    傳回:
      dict: 規劃結果，格式如下：
          {
            "船id或index": {
              "path": [(x,y), (x,y), ...],   # 船的路徑（公尺座標列表）
              "headings": [h1, h2, ...],       # 對應的航向（度）
              "analysis": {...}                # (可選) n_fcc_a.analysis 的資訊
            },
            ...
          }
    """
    # 依直線距離排序（距離遠者先規劃）
    sorted_ships = sorted(
        ships, key=lambda s: distance(s["pos"], s["goal"]), reverse=True
    )

    planning_results = {}  # 存放最終規劃結果
    previous_planned_paths = []  # 存放已確定的路徑，以便後續比對

    # 逐艘船進行規劃
    for ship_data in sorted_ships:
        ship_id = ship_data.get("id", f"{ship_data['pos']}->{ship_data['goal']}")
        print(f"開始規劃船 {ship_id} ...")
        ship_info = {"pos": ship_data["pos"], "goal": ship_data["goal"]}
        interfering_paths = []
        while True:
            planner = n_fcc_a(
                ship_info=ship_info,
                interfering_paths=interfering_paths,
                grid_scale=grid_scale,
            )
            path_m, headings = planner.calculate_path()
            new_conflict_found = False
            for prev_ship in previous_planned_paths:
                if prev_ship not in interfering_paths:
                    if paths_conflict(path_m, prev_ship["path"], safe_distance):
                        print(f"  - 發現與船 {prev_ship['id']} 衝突，加入干擾重算。")
                        interfering_paths.append(prev_ship)
                        new_conflict_found = True
            if not new_conflict_found:
                break
        print(
            f"船 {ship_id} 規劃完成！路徑長度={len(path_m)} 步\n"
            f"參照路徑: {', '.join([str(p['id']) for p in interfering_paths])}\n"
        )
        planning_results[ship_id] = {
            "path": path_m,
            "headings": headings,
            "analysis": planner.analysis,
        }
        previous_planned_paths.append(
            {"id": ship_id, "path": path_m, "headings": headings}
        )

    # 平滑化所有規劃結果
    for ship_id in planning_results:
        raw_path, raw_headings = (
            planning_results[ship_id]["path"],
            planning_results[ship_id]["headings"],
        )
        if smoothing_method != "none":
            new_path, new_headings = smooth_path(raw_path, method=smoothing_method)
        else:
            new_path, new_headings = raw_path, raw_headings
        planning_results[ship_id]["path"] = new_path
        planning_results[ship_id]["headings"] = new_headings

    return planning_results


# --------------------
# Pygame 視覺化
# --------------------
import sys
import pygame
import random


def draw_arrow(surface, color, center, angle, size=10):
    """
    使用三角形繪製箭頭表示航向。

    參數:
      surface (pygame.Surface): 畫布對象。
      color (tuple): 顏色 (R, G, B)。
      center (tuple): 箭頭中心座標。
      angle (float): 航向角度（度）。
      size (int): 箭頭大小。
    """
    rad = math.radians(angle)
    tip = (center[0] + size * math.sin(rad), center[1] - size * math.cos(rad))
    side_angle = math.radians(150)
    left = (
        center[0] + (size * 0.5) * math.sin(rad + side_angle),
        center[1] - (size * 0.5) * math.cos(rad + side_angle),
    )
    right = (
        center[0] + (size * 0.5) * math.sin(rad - side_angle),
        center[1] - (size * 0.5) * math.cos(rad - side_angle),
    )
    pygame.draw.polygon(surface, color, [tip, left, right])


def main():
    # ---------------------------
    # 定義多艘船 (示範資料)
    # ---------------------------
    ships_data = [
        {"id": "ShipA", "pos": (2.2, 1.5), "goal": (17.4, 9.7)},
        {"id": "ShipB", "pos": (5.1, 1.5), "goal": (15.3, 8.7)},
        {"id": "ShipC", "pos": (7.9, 1.5), "goal": (14.7, 12.4)},
        {"id": "ShipD", "pos": (7.9, -1.4), "goal": (13.7, 10.3)},
        {"id": "ShipE", "pos": (7.9, -4.2), "goal": (17.1, 12.1)},
    ]

    # ---------------------------
    # 呼叫多船規劃，並選擇平滑化參數
    # ---------------------------
    grid_scale = 0.2
    smoothing_method = "bezier"  # 可選 "none", "moving_average", "bezier"
    results = multi_ship_planning(
        ships_data,
        safe_distance=1,
        grid_scale=grid_scale,
        smoothing_method=smoothing_method,
    )

    # ---------------------------
    # Pygame 視覺化設定
    # ---------------------------
    pygame.init()
    desired_draw_area_width = 1200
    desired_draw_area_height = 800

    # 整合所有船的路徑點以決定顯示範圍
    all_x = []
    all_y = []
    for sid, data in results.items():
        for px, py in data["path"]:
            all_x.append(px)
            all_y.append(py)
    margin_m = 3
    min_x = int(min(all_x)) - margin_m
    max_x = int(max(all_x)) + margin_m
    min_y = int(min(all_y)) - margin_m
    max_y = int(max(all_y)) + margin_m

    m_range_x = max_x - min_x
    m_range_y = max_y - min_y
    if m_range_x < 1e-6:
        m_range_x = 1
    if m_range_y < 1e-6:
        m_range_y = 1

    vis_scale = min(
        desired_draw_area_width / m_range_x, desired_draw_area_height / m_range_y
    )
    ship_path_width = m_range_x * vis_scale
    ship_path_height = m_range_y * vis_scale

    screen = pygame.display.set_mode((int(ship_path_width), int(ship_path_height)))
    pygame.display.set_caption("多船 FCC 路徑規劃")
    font = pygame.font.SysFont(None, 16)
    clock = pygame.time.Clock()

    # 為每艘船指定隨機顏色
    color_map = {}
    for sid in results.keys():
        color_map[sid] = (
            random.randint(50, 255),
            random.randint(50, 255),
            random.randint(50, 255),
        )

    def trans_to_screen(px, py):
        """
        將公尺座標轉換為螢幕座標。

        參數:
          px (float): X 座標（公尺）。
          py (float): Y 座標（公尺）。

        傳回:
          tuple: 螢幕座標 (sx, sy)。
        """
        sx = (px - min_x) * vis_scale
        sy = ship_path_height - ((py - min_y) * vis_scale)
        return sx, sy

    running = True
    while running:
        clock.tick(10)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))

        # 繪製網格
        for gx in range(min_x, max_x + 1):
            xx = (gx - min_x) * vis_scale
            pygame.draw.line(screen, (220, 220, 220), (xx, 0), (xx, ship_path_height))
        for gy in range(min_y, max_y + 1):
            yy = ship_path_height - ((gy - min_y) * vis_scale)
            pygame.draw.line(screen, (220, 220, 220), (0, yy), (ship_path_width, yy))

        # 繪製每艘船的路徑與步數
        for sid, data in results.items():
            path = data["path"]
            headings = data["headings"]
            col = color_map[sid]
            for i, (px, py) in enumerate(path):
                sx, sy = trans_to_screen(px, py)
                draw_arrow(screen, col, (sx, sy), headings[i], 10)
                step_txt = font.render(str(i), True, col)
                screen.blit(step_txt, (sx + 2, sy + 2))

        # 在右上角繪製圖例 (Legend)，顯示船 id 與對應顏色
        legend_x = 10
        legend_y = int(ship_path_width) - 150
        legend_width = 140
        legend_height = 20 * len(results) + 10
        pygame.draw.rect(
            screen, (240, 240, 240), (legend_x, legend_y, legend_width, legend_height)
        )
        pygame.draw.rect(
            screen, (0, 0, 0), (legend_x, legend_y, legend_width, legend_height), 1
        )
        for i, sid in enumerate(results.keys()):
            text = font.render(sid, True, color_map[sid])
            screen.blit(text, (legend_x + 5, legend_y + 5 + i * 20))

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
