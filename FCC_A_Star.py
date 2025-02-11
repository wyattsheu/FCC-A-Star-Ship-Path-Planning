import math
import heapq

# =============================
# 原始 GOODWIN 模型參數 (單位：公尺)
# =============================
SECTER_RADIUS_LEFT = 0.7 * 5  # 例如：3.5 m
SECTER_RADIUS_RIGHT = 0.85 * 5  # 例如：4.25 m
SECTER_RADIUS_BACK = 0.45 * 5  # 例如：2.25 m


# =============================================================================
# FCC-A* 規劃類別
# =============================================================================
class fcc_a:
    """
    FCC-A* 路徑規劃類別
    (詳細註解略……)
    """

    def __init__(
        self,
        ship1_speed,
        ship1_pos,
        ship2_speed,
        ship2_pos,
        ship1_goal,
        ship2_goal,
        yield_ship,
        grid_scale=0.1,
    ):
        self.grid_scale = grid_scale

        # 計算各船初始航向：從起始點指向目標
        def compute_heading(pos, goal):
            dx = goal[0] - pos[0]
            dy = goal[1] - pos[1]
            # 定義：正北為 0，順時針增加（利用 atan2(dx, dy)）
            return math.degrees(math.atan2(dx, dy)) % 360

        self.ship1_speed = ship1_speed
        self.ship2_speed = ship2_speed

        # 將位置轉換為格單位
        self.ship1_pos = (
            int(round(ship1_pos[0] / grid_scale)),
            int(round(ship1_pos[1] / grid_scale)),
        )
        self.ship2_pos = (
            int(round(ship2_pos[0] / grid_scale)),
            int(round(ship2_pos[1] / grid_scale)),
        )
        self.ship1_goal = (
            int(round(ship1_goal[0] / grid_scale)),
            int(round(ship1_goal[1] / grid_scale)),
        )
        self.ship2_goal = (
            int(round(ship2_goal[0] / grid_scale)),
            int(round(ship2_goal[1] / grid_scale)),
        )
        self.ship1_heading = compute_heading(ship1_pos, ship1_goal)
        self.ship2_heading = compute_heading(ship2_pos, ship2_goal)

        # 根據 yield_ship 決定角色
        if yield_ship == 1:
            self.yield_speed = ship1_speed
            self.yield_pos = self.ship1_pos
            self.yield_heading = self.ship1_heading
            self.direct_speed = ship2_speed
            self.direct_pos = (ship2_pos[0] / grid_scale, ship2_pos[1] / grid_scale)
            self.direct_heading = self.ship2_heading
            self.yield_goal = self.ship1_goal
            self.direct_goal = self.ship2_goal
        else:
            self.yield_speed = ship2_speed
            self.yield_pos = self.ship2_pos
            self.yield_heading = self.ship2_heading
            self.direct_speed = ship1_speed
            self.direct_pos = (ship1_pos[0] / grid_scale, ship1_pos[1] / grid_scale)
            self.direct_heading = self.ship1_heading
            self.yield_goal = self.ship2_goal
            self.direct_goal = self.ship1_goal

        # GOODWIN 模型參數換算為格單位
        self.g_secter_radius_left = SECTER_RADIUS_LEFT / grid_scale
        self.g_secter_radius_right = SECTER_RADIUS_RIGHT / grid_scale
        self.g_secter_radius_back = SECTER_RADIUS_BACK / grid_scale

        self.TWICE_SECTER_RADIUS_MAX = 2 * self.g_secter_radius_right
        self.TWICE_SECTER_RADIUS_MIN = 2 * self.g_secter_radius_back
        self.INTERVAL_PARAMETER_67_180 = (
            self.g_secter_radius_left + self.g_secter_radius_right
        )
        self.INTERVAL_PARAMETER_245_360 = (
            self.g_secter_radius_left + self.g_secter_radius_back
        )
        self.INTERVAL_PARAMETER_0_67 = (
            self.g_secter_radius_right + self.g_secter_radius_back
        )

        # 設定 FCC_SCALE 據 grid_scale 調整：當 grid_scale=0.1 則 FCC_SCALE 為 10
        self.fcc_scale = 10 * (grid_scale / 0.1)

        # 定義 A* 搜索區域（以讓路船起點、目標與直行船起點為依據，加上 margin）
        xs = [self.yield_pos[0], self.yield_goal[0], self.direct_pos[0]]
        ys = [self.yield_pos[1], self.yield_goal[1], self.direct_pos[1]]
        margin = 20
        self.min_x = min(xs) - margin
        self.max_x = max(xs) + margin
        self.min_y = min(ys) - margin
        self.max_y = max(ys) + margin

        # 預測直行船路徑（含減速）
        self.direct_predicted_path = self.predict_direct_path()

        self.analysis = {}

    @staticmethod
    def h_cost_distance(a, b):
        (x1, y1) = a
        (x2, y2) = b
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        # 使用八方向距離作為啟發式估計
        return (math.sqrt(2) - 1) * min(dx, dy) + max(dx, dy)

    # ---------------------------
    # FCC 相關函式（不做修改）
    # ---------------------------
    def calc_u_theta(self, theta1, theta2):
        angle_diff = abs(theta1 - theta2)
        return (17 / 44) * (
            math.cos(math.radians(angle_diff - 19))
            + math.sqrt(440 / 289 + math.cos(math.radians(angle_diff - 19)) ** 2)
        )

    def calc_u_dist(self, dist, theta1, theta2):
        if dist > self.TWICE_SECTER_RADIUS_MAX:
            return 0
        elif dist < self.TWICE_SECTER_RADIUS_MIN:
            return 1
        delta = abs(theta1 - theta2)
        if 67.5 <= delta < 180:
            return (
                (self.INTERVAL_PARAMETER_67_180 - dist) / self.INTERVAL_PARAMETER_67_180
            ) ** 2
        elif 247.5 <= delta < 360:
            return (
                (self.INTERVAL_PARAMETER_245_360 - dist)
                / self.INTERVAL_PARAMETER_245_360
            ) ** 2
        else:
            return (
                (self.INTERVAL_PARAMETER_0_67 - dist) / self.INTERVAL_PARAMETER_0_67
            ) ** 2

    def calc_fcc(self, dist, theta1, theta2):
        return 0.5 * self.calc_u_theta(theta1, theta2) + 0.5 * self.calc_u_dist(
            dist, theta1, theta2
        )

    # ---------------------------
    # 直行船路徑預測（不變）
    # ---------------------------
    def predict_direct_path(self):
        pos = [self.direct_pos[0], self.direct_pos[1]]
        goal = self.direct_goal
        path = [tuple(pos)]
        v = self.direct_speed / self.yield_speed
        k_dec_default = 10
        while True:
            dx = goal[0] - pos[0]
            dy = goal[1] - pos[1]
            D_rem = math.hypot(dx, dy)
            if D_rem < 1e-6:
                break
            direction = (dx / D_rem, dy / D_rem)
            if D_rem > v * k_dec_default:
                pos[0] += v * direction[0]
                pos[1] += v * direction[1]
                path.append(tuple(pos))
            else:
                k_dec = int(math.ceil(2 * D_rem / v))
                if k_dec < 1:
                    k_dec = 1
                for i in range(k_dec):
                    d_i = (2 * D_rem * (1 - (i + 0.5) / k_dec)) / k_dec
                    pos[0] += d_i * direction[0]
                    pos[1] += d_i * direction[1]
                    path.append(tuple(pos))
                break
        return path

    # ---------------------------
    # FCC-A* 啟發式函數（修改：使用八方向距離）
    # ---------------------------
    def _heuristic(self, state):
        x, y, t, _ = state
        # 用八方向距離估計 yield 船至目標的成本
        dx_goal = abs(x - self.yield_goal[0])
        dy_goal = abs(y - self.yield_goal[1])
        h_dist = (math.sqrt(2) - 1) * min(dx_goal, dy_goal) + max(dx_goal, dy_goal)

        # 若有 FCC 部分則計算其成本
        if t == 0:
            return h_dist
        if t < len(self.direct_predicted_path):
            direct_pos = self.direct_predicted_path[t]
            if t == 0:
                predicted_heading = self.direct_heading
            else:
                prev = self.direct_predicted_path[t - 1]
                curr = self.direct_predicted_path[t]
                dxx = curr[0] - prev[0]
                dyy = curr[1] - prev[1]
                norm = math.hypot(dxx, dyy)
                if norm == 0:
                    predicted_heading = self.direct_heading
                else:
                    predicted_heading = math.degrees(math.atan2(dxx, dyy)) % 360
        else:
            direct_pos = self.direct_predicted_path[-1]
            if len(self.direct_predicted_path) > 1:
                prev = self.direct_predicted_path[-2]
                curr = self.direct_predicted_path[-1]
                dxx = curr[0] - prev[0]
                dyy = curr[1] - prev[1]
                norm = math.hypot(dxx, dyy)
                if norm == 0:
                    predicted_heading = self.direct_heading
                else:
                    predicted_heading = math.degrees(math.atan2(dxx, dyy)) % 360
            else:
                predicted_heading = self.direct_heading
        dx = x - direct_pos[0]
        dy = y - direct_pos[1]
        D = math.hypot(dx, dy)
        if D != 0:
            theta1 = math.degrees(math.atan2(dx, dy)) % 360
        else:
            theta1 = 0
        fcc_cost = self.calc_fcc(D, theta1, predicted_heading)
        return h_dist + self.fcc_scale * fcc_cost

    # ---------------------------
    # 鄰近節點函式：允許8方向移動（取消轉彎限制）
    # ---------------------------
    def _neighbors(self, state):
        x, y, t, curr_h = state
        nbrs = []
        # 考慮 8 個方向
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                # 計算此方向的航向（以正北為 0°，順時針增加）
                cand_h = math.degrees(math.atan2(dx, dy)) % 360
                nx = x + dx
                ny = y + dy
                nbrs.append((nx, ny, t + 1, cand_h))
        return nbrs

    # ---------------------------
    # FCC-A* 主演算法（修改：計算移動成本時區分直行與斜行）
    # ---------------------------
    def _a_star(self):
        start_state = (self.yield_pos[0], self.yield_pos[1], 0, self.yield_heading)
        goal_xy = (self.yield_goal[0], self.yield_goal[1])
        open_heap = []
        g_cost = {start_state: 0}
        parent = {}
        f_start = self._heuristic(start_state)
        heapq.heappush(open_heap, (f_start, start_state))
        closed = set()
        while open_heap:
            current_f, current = heapq.heappop(open_heap)
            if (current[0], current[1]) == goal_xy:
                path = []
                state = current
                while state in parent:
                    path.append(state)
                    state = parent[state]
                path.append(start_state)
                path.reverse()
                return path
            closed.add(current)
            for neighbor in self._neighbors(current):
                if neighbor in closed:
                    continue
                # 計算從 current 移動到 neighbor 的步驟成本：
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                step_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1
                tentative_g = g_cost[current] + step_cost
                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    parent[neighbor] = current
                    f = tentative_g + self._heuristic(neighbor)
                    heapq.heappush(open_heap, (f, neighbor))
        return []

    # ---------------------------
    # 主介面：計算 FCC-A* 路徑
    # ---------------------------
    def calculate_path(self):
        yield_path_states = self._a_star()  # 每個元素為 (x,y,t,h)
        yield_path_grid = [(state[0], state[1]) for state in yield_path_states]

        analysis_positions = []  # 單位 m
        analysis_steps = []
        for state in yield_path_states:
            x, y, t, _ = state
            analysis_positions.append((x * self.grid_scale, y * self.grid_scale))
            analysis_steps.append(t)

        # 利用首尾向量法計算 yield 船的「真實」航向（使用前 k 個點）
        k_heading = 3
        computed_headings = []
        for i, pos in enumerate(analysis_positions):
            j = max(0, i - k_heading)
            start_pos = analysis_positions[j]
            dx = pos[0] - start_pos[0]
            dy = pos[1] - start_pos[1]
            if math.hypot(dx, dy) > 1e-6:
                head_angle = math.degrees(math.atan2(dx, dy)) % 360
            else:
                head_angle = computed_headings[i - 1] if i > 0 else self.yield_heading
            computed_headings.append(head_angle)

        # 計算 FCC 值：根據 yield 船與直行船預測位置（均轉換為 m 單位）
        direct_path_real = [
            (x * self.grid_scale, y * self.grid_scale)
            for (x, y) in self.direct_predicted_path
        ]
        computed_fcc = []
        for i, pos in enumerate(analysis_positions):
            if i < len(direct_path_real):
                dp = direct_path_real[i]
            else:
                dp = direct_path_real[-1]
            D = math.hypot(pos[0] - dp[0], pos[1] - dp[1])
            if i == 0:
                predicted_heading = self.direct_heading
            else:
                prev_dp = (
                    direct_path_real[i - 1]
                    if i - 1 < len(direct_path_real)
                    else direct_path_real[-1]
                )
                dxx = dp[0] - prev_dp[0]
                dyy = dp[1] - prev_dp[1]
                norm = math.hypot(dxx, dyy)
                if norm == 0:
                    predicted_heading = self.direct_heading
                else:
                    predicted_heading = math.degrees(math.atan2(dxx, dyy)) % 360
            fcc_val = (
                self.calc_fcc(D, computed_headings[i], predicted_heading)
                * self.fcc_scale
            )
            computed_fcc.append(fcc_val)

        self.analysis = {
            "steps": analysis_steps,
            "positions": analysis_positions,
            "headings": computed_headings,
            "fcc": computed_fcc,
        }

        yield_path_real = [
            (x * self.grid_scale, y * self.grid_scale) for (x, y) in yield_path_grid
        ]
        direct_path_real = [
            (x * self.grid_scale, y * self.grid_scale)
            for (x, y) in self.direct_predicted_path
        ]
        return direct_path_real, yield_path_real


# =============================================================================
# 測試及視覺化（包含 Pygame 船的路徑圖與嵌入 Pygame 的 Matplotlib 分析圖）
# =============================================================================
if __name__ == "__main__":
    import pygame
    import sys
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
    import os

    # 設定 Pygame 視窗位置
    os.environ["SDL_VIDEO_WINDOW_POS"] = "100,200"  # 向下移動 100 像素

    # ---------------------------
    # 輔助函式：畫箭頭（以三角形表示）
    # center 為箭頭中心 (像素座標)
    # angle 為方向（0° 表示北，順時針增加）
    # size 為箭頭長度
    # ---------------------------
    def draw_arrow(surface, color, center, angle, size):
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

    # ---------------------------
    # 測試參數：
    # 這邊可以調整船的位置，例如從 (50,50) 調整到 (100,100)
    # ---------------------------
    ship1_speed = 2
    ship1_pos = (50, 50)  # 可改為 (50,50) 或其他
    ship2_speed = 5
    ship2_pos = (0, 0)
    ship1_goal = (0, 0)
    ship2_goal = (50, 50)  # 若改 ship1_pos，目標也調整
    yield_ship = 1  # yield 船為船2
    grid_scale = 1
    planner = fcc_a(
        ship1_speed,
        ship1_pos,
        ship2_speed,
        ship2_pos,
        ship1_goal,
        ship2_goal,
        yield_ship,
        grid_scale=grid_scale,
    )
    direct_path, yield_path = planner.calculate_path()
    analysis = planner.analysis
    yield_steps = analysis["steps"]
    yield_positions = analysis["positions"]
    yield_headings = analysis["headings"]
    fcc_values = analysis["fcc"]

    # ---------------------------
    # Pygame 視覺化：船的路徑圖與 Matplotlib 分析圖整合顯示
    # 這裡我們先不使用固定 vis_scale，而是根據路徑邊界計算動態的縮放比例，
    # 使得無論船的位置範圍如何，繪製的地圖區域大小都固定（例如 600×600 像素）。
    # ---------------------------
    pygame.init()
    # 這裡先設定一個「固定繪製區域」的尺寸（單位：像素）
    desired_draw_area_width = 1200
    desired_draw_area_height = 1200

    # 計算 yield_path 與 direct_path 的邊界（單位：m）
    all_x = [pt[0] for pt in yield_path] + [pt[0] for pt in direct_path]
    all_y = [pt[1] for pt in yield_path] + [pt[1] for pt in direct_path]
    margin_m = 5
    min_x = int(min(all_x)) - margin_m
    max_x = int(max(all_x)) + margin_m
    min_y = int(min(all_y)) - margin_m
    max_y = int(max(all_y)) + margin_m

    m_range_x = max_x - min_x
    m_range_y = max_y - min_y
    # 動態計算每公尺的像素數，使得整體寬高不超過固定的繪製區域
    vis_scale = min(
        desired_draw_area_width / m_range_x, desired_draw_area_height / m_range_y
    )

    ship_path_width = m_range_x * vis_scale
    ship_path_height = m_range_y * vis_scale

    # 右側留一區域用來顯示 Matplotlib 圖（例如 400 像素寬）
    plot_width = 1000
    total_width = int(ship_path_width + plot_width)
    total_height = int(ship_path_height)

    screen = pygame.display.set_mode((total_width, total_height))
    pygame.display.set_caption("Ship Paths and Analysis (Pygame)")
    font = pygame.font.SysFont(None, 16)
    clock = pygame.time.Clock()

    # 產生 Matplotlib 圖：FCC vs Steps 與 Heading vs Steps
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    axs[0].plot(yield_steps, fcc_values, "c.-")
    axs[0].set_title("FCC vs Steps (Yield Ship)")
    axs[0].set_xlabel("Step")
    axs[0].set_ylabel("FCC")
    axs[0].grid(True)
    axs[1].plot(yield_steps, yield_headings, "b.-")
    axs[1].set_title("Heading vs Steps (Yield Ship)")
    axs[1].set_xlabel("Step")
    axs[1].set_ylabel("Heading (°)")
    axs[1].grid(True)
    plt.tight_layout()
    canvas = FigureCanvas(fig)
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()
    size = fig.canvas.get_width_height()
    plot_surface = pygame.image.fromstring(raw_data, size, "RGB")

    # 調整 Matplotlib 圖貼至右側區域 (維持寬高比)
    fig_w, fig_h = size
    aspect_ratio = fig_w / fig_h
    desired_w = plot_width
    desired_h = int(desired_w / aspect_ratio)
    plot_surface = pygame.transform.scale(plot_surface, (desired_w, desired_h))

    running = True
    while running:
        clock.tick(10)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        # 畫網格：垂直線
        for x in range(min_x, max_x + 1):
            xpos = int((x - min_x) * vis_scale)
            pygame.draw.line(
                screen, (200, 200, 200), (xpos, 0), (xpos, ship_path_height)
            )
        # 畫水平線（注意：y 軸在 pygame 中向下增大，因此反轉 y 座標）
        for y in range(min_y, max_y + 1):
            ypos = int(ship_path_height - ((y - min_y) * vis_scale))
            pygame.draw.line(
                screen, (200, 200, 200), (0, ypos), (ship_path_width, ypos)
            )
        # 塗色：讓路船經過的每個格子填淺藍色
        for pt in yield_path:
            cell_x = int(pt[0])
            cell_y = int(pt[1])
            rect = pygame.Rect(
                int((cell_x - min_x) * vis_scale),
                int(ship_path_height - ((cell_y - min_y + 1) * vis_scale)),
                int(vis_scale),
                int(vis_scale),
            )
            pygame.draw.rect(screen, (173, 216, 230), rect)
        # 繪製 yield 船路徑箭頭及步數（藍色）
        for i, pt in enumerate(yield_path):
            sx = int((pt[0] - min_x) * vis_scale + vis_scale / 2)
            sy = int(ship_path_height - ((pt[1] - min_y) * vis_scale + vis_scale / 2))
            draw_arrow(screen, (0, 0, 255), (sx, sy), yield_headings[i], 10)
            txt = font.render(str(yield_steps[i]), True, (0, 0, 255))
            screen.blit(txt, (sx + 2, sy + 2))
        # 繪製直行船路徑箭頭及步數（紅色）
        direct_steps = list(range(len(direct_path)))
        direct_headings = []
        prev = None
        for i, pt in enumerate(direct_path):
            if i == 0:
                direct_headings.append(planner.direct_heading)
                prev = pt
                continue
            dx = pt[0] - prev[0]
            dy = pt[1] - prev[1]
            dist = math.hypot(dx, dy)
            if dist == 0:
                direct_headings.append(direct_headings[-1])
            else:
                head = math.degrees(math.atan2(dx, dy)) % 360
                direct_headings.append(head)
            prev = pt
        for i, pt in enumerate(direct_path):
            sx = int((pt[0] - min_x) * vis_scale + vis_scale / 2)
            sy = int(ship_path_height - ((pt[1] - min_y) * vis_scale + vis_scale / 2))
            draw_arrow(screen, (255, 0, 0), (sx, sy), direct_headings[i], 10)
            txt = font.render(str(direct_steps[i]), True, (255, 0, 0))
            screen.blit(txt, (sx + 2, sy + 2))
        # 將 Matplotlib 的圖貼到右側
        screen.blit(plot_surface, (int(ship_path_width), 0))
        pygame.display.flip()
    pygame.quit()
