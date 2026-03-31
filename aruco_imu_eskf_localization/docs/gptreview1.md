네, 그 제약은 반드시 써야 합니다.
그리고 방향도 맞습니다. 다만 구현은 **“PnP를 풀기 전에 임의로 잘라낸다”보다, 후보 pose를 만든 다음 board의 정면 가시성 조건으로 해를 고른다”**가 더 안전합니다.

핵심은 이겁니다.

`solvePnP`가 주는 pose는 **object(board) → camera** 변환입니다. 즉, 네가 정의한 board 좌표계의 점들이 카메라 좌표계에서 어디에 오는지를 주는 것이고, 카메라 좌표계는 OpenCV 기준으로 `x` 오른쪽, `y` 아래, `z` 전방입니다. 그래서 최종 해를 검사할 때는 “보드의 법선(normal)이 카메라를 향하는가”, “카메라가 board의 앞면 반공간에 있는가” 같은 **물리적 가시성 조건**을 직접 체크할 수 있습니다. ([OpenCV Documentation][1])

또 네 경우처럼 3개 마커의 상대 위치를 이미 알고 있으면, 그건 독립 마커 3장이 아니라 **하나의 rigid board**입니다. OpenCV도 board는 일부 마커만 보여도 같은 board pose를 추정할 수 있고, 독립 마커보다 더 많은 코너를 이용하므로 보통 더 정확하다고 설명합니다. ([OpenCV Documentation][2])

그래서 네가 말한 “팔로워는 항상 리더 후면의 보드 정면만 본다. 보드 뒷면으로 갈 일은 없다”는 것은 단순한 경험칙이 아니라, **강한 도메인 제약**입니다. 이 제약을 쓰면 단일 마커 fallback이나 planar ambiguity에서 나오는 잘못된 해를 많이 잘라낼 수 있습니다. 특히 OpenCV는 `solvePnPGeneric()`이 `IPPE`, `IPPE_SQUARE` 같은 방법에서 **복수 해를 반환할 수 있다**고 문서화하고 있으니, 그 후보들 중에서 네 물리 제약을 만족하는 해만 채택하는 것이 정석에 가깝습니다. ([OpenCV Documentation][1])

구체적으로는 이렇게 보면 됩니다.

네 board 정의에서 모든 마커는 `z=0` 평면 위에 있으니, board의 앞/뒤는 결국 **board 평면 법선의 방향**으로 정해집니다. 예를 들어 board 좌표계에서 앞면 법선을
[
n_B = [0,0,1]^T
]
또는 설계상 반대로
[
n_B = [0,0,-1]^T
]
로 정하면, pose 후보 (^{C}T_B=[R\ |\ t])에 대해 카메라 좌표계에서 board 법선은
[
n_C = R,n_B
]
가 됩니다.

이때 “카메라가 board 앞면을 보고 있다”는 조건은 여러 방식으로 쓸 수 있습니다.

가장 쉬운 건 **board 원점이 카메라 앞에 있어야 한다**는 조건입니다.

[
t_z > 0
]

이건 최소 조건입니다. 하지만 이것만으로는 부족합니다. 잘못된 해도 종종 (t_z>0)는 만족할 수 있기 때문입니다.

더 강한 조건은 **카메라가 board 앞면 반공간에 있어야 한다**는 것입니다.
카메라 중심을 board 좌표계로 옮기면

[
p^{B}_{cam} = -R^T t
]

이고, 이 점이 앞면 쪽에 있어야 하므로

[
n_B^T p^{B}_{cam} > 0
]

같은 조건을 둘 수 있습니다.
이 부호는 네가 “board의 정면”을 (+z_B)로 둘지 (-z_B)로 둘지에 따라 바뀌지만, 어쨌든 한쪽 부호로 고정됩니다.

직관적으로는 이겁니다.

* 카메라는 항상 board 앞쪽 공간에 있어야 한다
* board 뒷면 쪽 공간에 있는 해는 버린다

이 제약 하나만 넣어도 “갑자기 뒤로 넘어간 pose”는 많이 제거됩니다.

그리고 네 유즈케이스에서는 여기서 한 단계 더 강하게 가도 됩니다.

팔로워는 리더 뒤를 따라가므로, board 기준 follower는 보통

* lateral (y)가 너무 크지 않고
* distance (x) 또는 depth가 일정 범위 안이며
* yaw 차이가 180도 근처로 뒤집히지 않고
* frame-to-frame 변화율이 차량이 낼 수 있는 범위를 넘지 않아야 합니다

즉, **단순한 앞/뒤 판별만이 아니라 주행가능 집합(feasible set)** 으로 게이팅하는 게 좋습니다.

예를 들어 board 기준 pose를 (^{B}T_F)로 바꾼 뒤에:

* 종방향 거리: (0.2 \text{ m} < x < 3.0 \text{ m})
* 횡방향 오프셋: (|y| < 0.8 \text{ m})
* 높이: (|z - z_0| < 0.3 \text{ m})
* yaw: (|\psi| < 45^\circ) 또는 현재 예측치 주변
* 프레임 간 jump: (\Delta x,\Delta y,\Delta \psi) 제한

이런 식으로 걸면 됩니다.

중요한 건, 이걸 **PnP 이전의 하드 컷**으로 넣지 말고, **PnP 후보 해 선택 단계 + measurement gating 단계** 두 군데에 넣는 겁니다.

내가 추천하는 순서는 이렇습니다.

1. **3개 또는 2개 마커가 보이면**
   custom board 전체 object points로 `solvePnP`를 풉니다.
   가능하면 이전 프레임 해를 initial guess로 넣고 refine합니다. OpenCV board tutorial도 보드 pose는 `solvePnP()` 기반으로 잡는 흐름을 사용합니다. ([OpenCV Documentation][2])

2. **1개만 보이면**
   그 마커에 대해 `solvePnPGeneric(..., IPPE_SQUARE)`로 **후보 해를 둘 이상** 받습니다. OpenCV는 `IPPE_SQUARE`가 marker pose에 적합하며, `solvePnPGeneric()`이 복수 해를 반환할 수 있다고 설명합니다. ([OpenCV Documentation][1])

3. 각 후보를 **board pose로 환산**합니다.
   즉 `marker_i` pose를 그대로 쓰지 않고, 네가 아는 `^B T_{M_i}`를 이용해 모두 `^C T_B`로 바꿉니다.

4. 각 후보에 대해 점수를 매깁니다.
   점수 항목은:

   * reprojection error
   * board front-facing 조건
   * follower가 board 앞면 반공간에 있는가
   * 예측 상태와의 innovation norm
   * 거리/횡오프셋/yaw feasible set 만족 여부

5. 제약을 만족하는 후보 중 최저 비용 해만 채택합니다.
   만족하는 게 없으면 이번 프레임 vision update는 버립니다.

이렇게 하면 네가 말한 “보드 뒷면으로 순간이동한 해”는 구조적으로 차단됩니다.

그리고 네 board 정의는 이 제약을 넣기에 아주 좋습니다.

* ID 2가 원점
* ID 0,1이 위쪽 좌우에 있고
* 세 점 배치가 비대칭성은 약하지만 최소한 “윗줄 큰 마커 2개 + 아래 작은 마커 1개” 구조를 줍니다

즉, 단일 마커가 보이더라도 “이 마커가 board 안에서 어느 위치인지”가 정해져 있으니, marker pose를 board pose로 되돌린 다음 물리 제약으로 걸러내기 좋습니다. 이게 바로 board를 쓰는 장점입니다. ([OpenCV Documentation][2])

실무적으로는 아래처럼 생각하면 됩니다.

* **PnP는 후보를 만든다**
* **도메인 제약은 후보를 고른다**
* **ESKF는 마지막으로 연속성을 확인한다**

즉, “애초에 팔로워가 board 뒷면으로 갈 수 없다”는 지식은 반드시 써야 하고,
그 최적 위치는 **single-marker fallback 해 선택기와 board-pose measurement gate**입니다.

한 줄로 정리하면,

**맞습니다. 네 경우에는 board 앞면 가시성 제약을 반드시 넣어야 하고, 특히 1개 마커만 보일 때는 `solvePnPGeneric`의 후보 해를 모두 board pose로 환산한 뒤, ‘카메라는 항상 board 앞면 반공간에 있다’는 물리 제약과 예측치 연속성으로 해를 선택하는 구조가 가장 안전합니다.**

원하시면 다음 답변에서는 이걸 바로 이어서
**네 YAML board 정의를 기준으로 “앞면 반공간 판정식 + 후보 해 점수 함수 + reject 규칙”을 수식과 의사코드로 딱 맞게 적어드리겠습니다.

[1]: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html "OpenCV: Perspective-n-Point (PnP) pose computation"
[2]: https://docs.opencv.org/4.x/db/da9/tutorial_aruco_board_detection.html "OpenCV: Detection of ArUco boards"

가능합니다. 그리고 네 경우에는 그 방법이 거의 필수에 가깝습니다.

다만 한 가지만 먼저 분명히 하겠습니다.
**지금 YAML의 `board` 좌표계는 `+x=오른쪽`, `+y=위`, `+z=보드 평면 법선`으로 써야** 합니다. 즉, 네 YAML 기준에서는 “앞/뒤” 게이팅 축이 `x`가 아니라 **`z_B`** 입니다. OpenCV의 `solvePnP()`는 **object(board) → camera** 변환을 주고, `solvePnPGeneric()`은 `IPPE`, `IPPE_SQUARE` 등에서 가능한 해를 여러 개 돌려줄 수 있습니다. 또 단일 4-corner coplanar marker pose는 본질적으로 ambiguity가 생길 수 있어서, OpenCV FAQ도 큰 마커, multiple markers/boards, `SOLVEPNP_IPPE_SQUARE`를 권합니다. ([OpenCV Documentation][1])

내 추천은 아래 구조입니다.

---

## 1) 먼저 `board`의 “정면”을 명시적으로 정의

네 board YAML에는 `+x`, `+y`만 적혀 있고 `+z`가 안 적혀 있습니다.
이건 꼭 명시하세요.

나는 이렇게 두는 것을 권합니다.

* `+x_B`: 보드에서 오른쪽
* `+y_B`: 보드에서 위
* `+z_B`: **인쇄된 앞면에서 팔로워 쪽으로 나오는 법선**

즉, 팔로워가 보드를 정상적으로 보고 있으면 **팔로워의 카메라/베이스는 항상 `z_B > 0` 반공간에 있어야** 합니다.

네 보드의 corner는 `TL, TR, BR, BL` 순서로 이렇게 잡으면 됩니다. `IPPE_SQUARE`도 square marker object points를 그 순서로 요구합니다. ([OpenCV Documentation][1])

[
M_2\ (s=62,\ h=31):
(-31,31,0),(31,31,0),(31,-31,0),(-31,-31,0)
]

[
M_0\ (s=124,\ h=62,\ c=(-119,93)):
(-181,155,0),(-57,155,0),(-57,31,0),(-181,31,0)
]

[
M_1\ (s=124,\ h=62,\ c=(119,93)):
(57,155,0),(181,155,0),(181,31,0),(57,31,0)
]

---

## 2) 게이팅은 “PnP 이전”보다 “후보 해 생성 후”에 넣는 게 맞다

앞/뒤 제약은 아주 강한 물리 제약이지만, **2D 코너만 보고 PnP 전에 자르는 것보다**
**후보 pose를 만든 뒤에 그 pose가 앞면 반공간에 있는지 검사하는 쪽이 더 안전**합니다.

왜냐하면 OpenCV도 planar target에서 복수 해를 인정하고 있고, `solvePnPGeneric()`이 그 후보들을 꺼내주는 구조이기 때문입니다. 즉,

* **3개/2개 마커**: board 전체 visible corners로 pose 후보 생성
* **1개 마커**: 그 마커에 대해 `IPPE_SQUARE`로 후보 2개 생성
* 그 다음 **board front-facing 제약 + 예측 pose 연속성**으로 선택

이 순서가 가장 좋습니다. ([OpenCV Documentation][1])

---

## 3) 네 보드에 바로 넣을 하드 게이트

### 3-1. 먼저 solvePnP 결과를 base_link 기준으로 바꿔라

`solvePnP` 결과를 (^{C}T_{B})라 두면, 필터/제어가 쓸 것은 보통 (^{B}T_{\text{base}})입니다.

[
{}^{B}T_{\text{base}} = ({}^{C}T_{B})^{-1} \cdot {}^{C}T_{\text{base}}
]

여기서 (^{C}T_{\text{base}})는 카메라-베이스 정적 extrinsic입니다.

그리고 이제 **모든 게이팅은 `marker frame`이 아니라 `board frame`에서** 합니다.

---

### 3-2. 앞면 반공간 게이트

[
p_{\text{base}}^{B} = \text{translation}({}^{B}T_{\text{base}})
= [x_B,\ y_B,\ z_B]^T
]

네 YAML 정의대로면 앞/뒤는 `z_B`입니다.
그러므로 1차 하드 게이트는

[
z_B > z_{\min}
]

입니다.

시작값은

* (z_{\min}=0.05\text{ m})

정도로 두면 됩니다.

즉, **베이스가 board 뒷면 반공간 ((z_B \le 0))** 에 있으면 바로 버립니다.

---

### 3-3. 시야각 게이트

카메라/베이스가 너무 사선이면 1-marker ambiguity가 심해집니다.

[
\cos\alpha = \frac{z_B}{|p_{\text{base}}^{B}|}
]

[
\alpha < \alpha_{\max}
]

시작값은

* (\alpha_{\max}=75^\circ)

정도를 권합니다.

즉,

[
\frac{z_B}{|p_{\text{base}}^{B}|} > \cos 75^\circ
]

를 만족하지 않으면 reject입니다.

---

### 3-4. 깊이/측면/높이 feasible set 게이트

리더 트렁크를 보며 따라가는 상황이면, base_link가 board 기준에서 갈 수 있는 영역이 사실상 좁습니다.

시작값 예시는:

[
0.15 < z_B < 4.0
]

[
|x_B| < 0.8
]

[
|y_B - y_0| < 0.4
]

여기서

* (z_B): board 법선 방향 거리
* (x_B): 좌우 오프셋
* (y_B): 높이 차

입니다.

정확한 수치는 카메라 장착 높이와 차량 폭에 맞게 잡으면 되고, 핵심은
**“실제 follower가 board 기준에서 존재 가능한 3D 박스”를 미리 정해두는 것**입니다.

---

### 3-5. positive depth / cheirality 게이트

후보 pose에서 visible corners를 다시 camera frame으로 변환했을 때,
모든 corner depth가 camera 앞쪽이어야 합니다.

[
\min_j Z_{c,j} > z_{\text{cam,min}}
]

시작값:

* (z_{\text{cam,min}} = 0.02\text{ m})

이 조건을 깬 후보는 버립니다.

---

## 4) 후보 선택 점수 함수

하드 게이트를 통과한 후보들만 점수화하세요.

내 추천은:

[
J_i =
w_r \left(\frac{e^{(i)}*{\text{reproj}}}{e*{\text{ref}}(n_i)}\right)^2
+
w_m, d^{2}*{\text{maha}}(i)
+
w*\alpha \left(\frac{\alpha_i}{\alpha_{\max}}\right)^2
+
w_f,\Pi_{\text{fallback}}(n_i)
]

여기서

* (e_{\text{reproj}}): reprojection RMSE [px]
* (n_i): 사용된 마커 개수
* (d^2_{\text{maha}}): 예측 pose와의 Mahalanobis distance
* (\alpha_i): 시야각
* (\Pi_{\text{fallback}}): 1-marker일수록 penalty

시작값으로는

* (w_r = 0.5)
* (w_m = 1.0)
* (w_\alpha = 0.2)
* (w_f = 0.5)

정도로 두면 충분합니다.

그리고 기준 reprojection은 대충 이렇게 두면 됩니다.

* 3 markers: (e_{\text{ref}}=2.0) px
* 2 markers: (e_{\text{ref}}=3.0) px
* 1 marker: (e_{\text{ref}}=5.0) px

fallback penalty는

* 3 markers: (0)
* 2 markers: (0.3)
* 1 marker: (1.0)

정도로 시작하면 됩니다.

핵심은 **3→2→1 marker로 갈수록 “같은 board pose”는 유지하되, 점수와 covariance만 보수적으로 바꾸는 것**입니다. Board는 원래 일부 가려져도 pose를 계속 추정하도록 쓰는 도구이고, 더 많은 코너를 쓸수록 정확해집니다. ([OpenCV Documentation][2])

---

## 5) 1-marker일 때는 이렇게 처리

1개만 보이면 그 마커 frame을 바로 쓰지 말고, **반드시 board pose로 환산한 뒤** 후보를 평가하세요.

### 절차

1. 보이는 마커가 `id = k`
2. 그 마커 4코너로 `solvePnPGeneric(..., IPPE_SQUARE)` 수행
3. 후보 (^{C}T_{M_k}^{(1)}, ^{C}T_{M_k}^{(2)})를 얻음
4. 보드 내 고정 변환 (^{B}T_{M_k})를 써서 각 후보를 (^{C}T_B^{(i)})로 환산
5. 각 후보를 (^{B}T_{\text{base}}^{(i)})로 바꿔서 하드 게이트 + 점수 계산
6. 최저 점수 해만 채택

그리고 **best와 second-best score 차이가 너무 작으면 아예 update를 하지 마세요.**

예를 들면
[
J_2 - J_1 < 0.8
]
같으면, 그 프레임은 “모호함 해소 실패”로 보고 vision update skip이 낫습니다.

이게 중요합니다.
억지로 잘못된 후보를 넣는 것보다, 그 한 프레임은 IMU predict만 쓰는 편이 전체적으로 훨씬 안정적입니다.

---

## 6) 의사코드

```python
def estimate_board_pose_measurement(detections, K, D, T_C_base, pred_state, pred_cov):
    candidates = []

    # 0) board corner set 구성
    visible_ids = detections.ids

    if len(visible_ids) >= 2:
        obj_pts, img_pts = collect_visible_board_corners(detections)
        # planar board -> all possible IPPE solutions
        sols = solvePnPGeneric_IPPE(obj_pts, img_pts, K, D)
        for sol in sols:
            T_C_B = sol.T_C_B
            reproj = sol.reproj_rmse
            candidates.append(make_candidate(T_C_B, reproj, len(visible_ids), mode="board"))
    elif len(visible_ids) == 1:
        k = visible_ids[0]
        obj_pts, img_pts = single_marker_points_for_ippe_square(k, detections)
        sols = solvePnPGeneric_IPPE_SQUARE(obj_pts, img_pts, K, D)
        for sol in sols:
            T_C_Mk = sol.T_C_obj
            T_C_B = T_C_Mk @ inverse(T_B_Mk[k])   # candidate in board frame
            reproj = sol.reproj_rmse
            candidates.append(make_candidate(T_C_B, reproj, 1, mode="single"))
    else:
        return NO_MEAS

    feasible = []
    for c in candidates:
        T_B_base = inverse(c.T_C_B) @ T_C_base
        p = translation(T_B_base)  # [x_B, y_B, z_B]

        # hard gates
        if p[2] <= 0.05:   # front half-space
            continue
        if norm(p) < 0.15 or norm(p) > 4.0:
            continue
        if abs(p[0]) > 0.8:
            continue
        if abs(p[1] - Y_EXPECTED) > 0.4:
            continue

        alpha = acos(p[2] / norm(p))
        if alpha > deg2rad(75):
            continue

        if min_corner_depth_in_camera(c.T_C_B) <= 0.02:
            continue

        # temporal prior
        r = pose_residual(T_B_base, pred_state.T_B_base)  # [dp(3), dtheta(3)]
        d2 = mahalanobis(r, pred_cov_pose)

        score = (
            0.5 * (c.reproj_rmse / reproj_ref(c.num_markers))**2 +
            1.0 * d2 +
            0.2 * (alpha / deg2rad(75))**2 +
            fallback_penalty(c.num_markers)
        )

        feasible.append((score, T_B_base, c))

    if not feasible:
        return NO_MEAS

    feasible.sort(key=lambda x: x[0])

    # ambiguous single-marker case -> skip update
    if len(feasible) >= 2 and feasible[0][2].num_markers == 1:
        if feasible[1][0] - feasible[0][0] < 0.8:
            return NO_MEAS

    best = feasible[0]
    R_meas = covariance_from_quality(best[2], best[0])
    return Measurement(T_B_base=best[1], R=R_meas, quality=best[2])
```

---

## 7) ESKF 쪽에서는 이렇게 받아라

필터는 측정치를 항상 **board 기준 base_link pose**로만 받습니다.

즉 measurement residual은

[
r =
\begin{bmatrix}
p^{meas}*{B\to base} - \hat p^-*{B\to base}\
\log!\left(R^{meas}*{B\to base}\hat R*{B\to base}^{-\top}\right)
\end{bmatrix}
]

로 두고,
1-marker면 (R)을 크게, 2-marker면 중간, 3-marker면 작게 둡니다.

간단한 시작식은

[
R = s \cdot \mathrm{diag}
(\sigma_x^2,\sigma_y^2,\sigma_z^2,\sigma_{roll}^2,\sigma_{pitch}^2,\sigma_{yaw}^2)
]

[
s = 1 + 0.6\mathbf{1}*{1tag} + 0.2,e*{\text{reproj}} + 0.5\left(1-\frac{z_B}{|p_B|}\right)
]

정도면 충분합니다.

---

## 8) 네 경우에 더 좋은 방법들

### A. 가장 추천: 지금 보드 유지 + 후보해 게이팅 + temporal prior

이게 제일 현실적입니다.

추가로 꼭 넣을 것은

* `refineDetectedMarkers()`
* `CORNER_REFINE_APRILTAG` 또는 subpixel refinement
* 이전 프레임 pose를 initial guess로 넣고 `solvePnPRefineVVS()` 또는 `solvePnPRefineLM()`로 마무리

입니다. OpenCV FAQ는 board를 쓸 때 `refineDetectedMarkers()`를 권하고, OpenCV는 `CORNER_REFINE_APRILTAG` 옵션과 pose refinement 함수도 제공합니다. ([OpenCV Documentation][3])

### B. 하드웨어 변경이 가능하면: **보드를 살짝 3D로**

이게 구조적으로 가장 강합니다.

예를 들어

* 좌우 큰 마커를 10~15도 정도 바깥으로 꺾거나
* 가운데 작은 마커를 약간 앞/뒤로 띄우면

더 이상 모든 점이 한 평면에 있지 않아서, **planar ambiguity 자체를 약하게 만들 수 있습니다.** OpenCV의 ArUco `Board`는 원래 3D layout도 허용합니다. ([OpenCV Documentation][3])

실무적으로는 이게 “알고리즘으로 억지로 막는 것”보다 훨씬 강합니다.

### C. 보드 디자인을 더 비대칭으로

OpenCV FAQ도 single-marker ambiguity가 심할 때 larger marker, multi-marker board, non-symmetrical marker/board 구성을 권합니다. 네 보드는 이미 작은 center marker로 약한 비대칭성이 있지만, 아직 좌우 큰 마커가 대칭이고 전부 평면입니다. 가능하면

* 좌/우 큰 마커 크기를 다르게 하거나
* center marker를 약간 옆으로 치우치게 하거나
* 하나를 깊이 방향으로 띄우는 편이 더 낫습니다. ([OpenCV Documentation][3])

### D. 스택 교체가 가능하면: AprilGrid / AprilTag 계열

이건 꽤 강력한 대안입니다.

Kalibr는 calibration target로 **Aprilgrid를 추천**하면서, **partial visibility가 가능하고 target pose가 fully resolved, 즉 no flips**라고 명시합니다. 또 AprilTag 3는 검출 속도 향상과 작은 태그에서의 검출률 개선, flexible layout을 내세웁니다. 다만 이건 OpenCV ArUco만 쓰는 현재 스택보다 개발 전환 비용이 있습니다. ([GitHub][4])

즉,

* 지금 당장 빨리 안정화: ArUco board 유지
* 장기적으로 뒤집힘 자체를 더 강하게 없애고 싶음: Aprilgrid/AprilTag 고려

이렇게 보면 됩니다.

### E. 정밀도가 최우선이면: ChArUco

ChArUco는 partial views를 허용하면서도 chessboard corner를 보간하므로 corner 정확도가 더 좋습니다. 다만 layout이 고정적이고, 보통 마커가 더 작아져 검출 난이도가 올라갈 수 있습니다. 그래서 네처럼 **추종용 런타임 보드**에는 무조건 우위라고 보긴 어렵고, “정밀도 우선 + 보드가 충분히 잘 보임”일 때 후보입니다. ([OpenCV Documentation][5])

---

## 9) 내가 고르면 이렇게 갑니다

지금 네 상황에서 최적안은 이겁니다.

1. **현재 3-marker board 유지**
2. `+z_B = board front normal`을 YAML/코드에 명시
3. **모든 출력은 항상 board pose 기준**
4. **3/2/1 marker 모두 후보를 만든 뒤**

   * `z_B > 0`
   * view-angle gate
   * feasible box gate
   * temporal prior gate
     로 고름
5. 1-marker에서 애매하면 **그 프레임은 update skip**
6. detector에

   * `visible_markers`
   * `reprojection_rmse`
   * `single_marker_fallback`
   * `confidence`
     를 같이 내보냄
7. 가능하면 다음 버전에서 **얕은 V자 3D board**로 바꿈

그리고 한 가지 더.
지금 `board` 좌표계는 기하학용으로는 좋지만 제어용으로는 직관이 떨어집니다. 그래서 나는 별도로 `leader_rear` 같은 child frame을 하나 더 두는 것을 권합니다.

예를 들어 ROS 차량축으로 쓰려면

[
x_{leader_rear} = z_B,\quad
y_{leader_rear} = -x_B,\quad
z_{leader_rear} = y_B
]

처럼 바꾸면,

* (x): 차간거리
* (y): 횡오프셋
* (z): 높이

가 되어 제어와 디버깅이 훨씬 편해집니다.

네가 원하면 다음에는 이걸 바로 이어서
**ROS2 메시지 구조 / detector node 출력 / ESKF update 함수 인터페이스**까지 코드 뼈대 수준으로 적어드리겠습니다.

[1]: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html "OpenCV: Perspective-n-Point (PnP) pose computation"
[2]: https://docs.opencv.org/4.x/db/da9/tutorial_aruco_board_detection.html "OpenCV: Detection of ArUco boards"
[3]: https://docs.opencv.org/3.4/d1/dcb/tutorial_aruco_faq.html "OpenCV: Aruco module FAQ"
[4]: https://github.com/ethz-asl/kalibr/wiki/calibration-targets "Calibration targets · ethz-asl/kalibr Wiki · GitHub"
[5]: https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html "OpenCV: Detection of ChArUco Boards"
