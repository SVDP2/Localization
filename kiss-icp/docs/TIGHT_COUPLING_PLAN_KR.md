# KISS-ICP × ESKF Tight Coupling 계획 (Phase 2)

> 작성일: 2026-01-30  
> 대상 브랜치: `kiss icp tight couple` (사용자 생성)  
> 목적: GNSS 음영/튀는 구간에서 **일관된 DR + 심리스 전환**을 위해 KISS-ICP를 ESKF 내부로 내장(tight coupling)하여 지연/동기/상태관리를 통합한다.

---

## 0) 한 줄 요약

- **KISS-ICP는 “상대 모션(Δpose)”를 잘 만들고**,  
  **ESKF는 “절대 앵커(GNSS) + 상태 일관성/가드”를 잘 한다.**
- Tight coupling에서는 KISS-ICP의 registration 결과를 ESKF 업데이트 형태로 흡수하고,
  ESKF 예측을 KISS-ICP initial guess/deskew에 제공하여 **서로를 부스트**한다.

---

## 1) 목표 / 비목표

### 목표
- GNSS 음영 진입/진출 시 **seamless transition**
  - 포즈 점프 최소화(연속성), 관성적으로 자연스러운 궤적
- 음영 내부 **robust dead reckoning**
  - 장거리 누적 오차는 존재해도, 단기 구간에서 “일관된” 오도메트리 제공
- 운영/디버깅 관점에서
  - GNSS 상태/ICP 품질에 따른 가중치/게이팅이 **한 노드 안에서 일관되게** 동작
  - 재현 가능한 로깅/진단 지표 확보

### 비목표(초기 범위에서 제외)
- 전역 최적화(루프클로저/맵매칭)까지 포함한 SLAM 완성형
- OOS(Out-of-sequence) 정식 처리(상태 히스토리 + 재전파) “완전체”
  - 단, Phase 2.2에서 최소한의 시간정렬 정책은 포함

---

## 2) 현재 베이스라인(현 repo 상태)

- `eskf_localization`
  - 전파: **gyro 기반**, accel은 전파 입력에서 실질적으로 0이 되도록 처리(정책)  
  - 업데이트: GNSS pos/vel, GNSS heading yaw, 차량 제약(NHC/ZUPT 등)  
  - 출력 TF: `map -> base_link`
- `kiss_icp` (ROS wrapper)
  - 출력 TF/odom을 `map -> base_link_icp`로 분리하여 충돌 회피
  - ESKF TF(`map -> base_link`)가 뜨면 이를 초기 pose로 사용(초기 정렬)

이 구조는 Phase 1(loosely coupled)에는 적합하지만,
Phase 2에서 원하는 “지연 없는 동기 처리/상태 일관성/전환 품질”은 한계가 있다.

---

## 3) Tight Coupling 아키텍처(제안)

### 핵심 원칙
1) ESKF가 **유일한 전역 상태(명목 상태 + P)**를 가진다.
2) KISS-ICP는 ESKF 노드 내부에서 실행되며,
   - 입력: pointcloud(+timestamps), ESKF 예측/TF(extrinsic)
   - 출력: `ΔT_icp` 또는 `T_icp`, 그리고 품질지표(품질 기반 R 구성용)
3) ESKF 업데이트는 “절대 pose 강제”가 아니라
   - **상대 모션/속도/요(heading) 형태** 또는
   - **prediction 대비 residual** 형태로 넣는다.

### 데이터 흐름(권장)
```
PointCloud(t) ──▶ KISS-ICP RegisterFrame( initial_guess = ESKF_pred_delta )
                 ├─ outputs: ΔT_icp(t), T_icp(t), quality(t)
                 ▼
ESKF Update @ t:
  - yaw update (from ΔT_icp yaw)  [옵션]
  - body-velocity update (from ΔT_icp / dt) [옵션]
  - (선택) relative pose residual update (추가 API 필요)
```

> 초기 구현은 “yaw/velocity 업데이트”가 가장 비용 대비 효과가 좋고,
> 포즈(위치)까지 강하게 묶는 건 품질지표/시간정렬이 안정화된 뒤에 확장한다.

---

## 4) 왜 `base_link_icp`가 2~3m 차이나 보이나?

Tight coupling에서는 이 문제를 “제거”하는 쪽으로 간다.

- KISS-ICP 단독 절대 포즈(`map -> base_link_icp`)는 누적 오차로 인해
  GNSS 앵커 기준(`map -> base_link`)과 벌어지는 것이 자연스럽다.
- 해결 전략:
  - **KISS-ICP 절대 포즈를 ESKF에 그대로 강제하지 않는다.**
  - 대신 `ΔT_icp`(상대 모션)으로 ESKF의 DR 품질을 올리고,
    GNSS가 좋을 때는 ESKF가 절대 위치를 앵커링한다.

---

## 5) 측정 모델 설계(Phase 2에서의 “융합 형태”)

### 옵션 A (추천 시작점): `ΔT_icp` → yaw + body-velocity 업데이트
- 장점: ESKF에 이미 존재하는 업데이트 API를 재사용 가능(리스크↓)
- 업데이트 설계:
  - yaw: `z_yaw = yaw(ΔT_icp)`를 누적하거나, 혹은 `yaw_rate`로 변환해 bias에 제약
  - velocity: `v_base ≈ (R^T * Δp_map)/dt` 또는 `Δp_base/dt`
- 단점: 위치 drift는 여전히 누적(하지만 음영에서 “일관된 DR”은 확보 가능)

### (중요) 축별 관측성: `v_x` vs `v_y` vs `yaw`
도로/차량 환경을 고려하면, LiDAR ICP는 상황에 따라 특정 축이 퇴화(degeneracy)하기 쉽다.

- 경험적으로(특히 벽/가드레일이 있는 도로):
  - **`yaw`와 `v_y`(횡방향)**: 구조물이 만들어주는 제약이 많아 상대적으로 강함
  - **`v_x`(종방향)**: 반복되는 구조/긴 직선 구간에서 “미끄러짐(sliding)”으로 약해질 수 있음
- 따라서 초기 Phase 2에서는 아래 조합을 기본 정책으로 권장한다.
  - `v_x`: OBD speed(+필요 시 GNSS speed) 중심, ICP는 약하게/품질 좋을 때만 보조
  - `v_y`/`yaw`: ICP 기반 업데이트의 가중치를 상대적으로 더 높게 가져가되, 품질 게이팅은 필수

### 옵션 B: relative pose residual update(추가 API)
- ESKF에 `update_relative_pose_6d(...)` 형태의 측정 업데이트를 추가
- 측정은 “예측한 Δpose(ESKF) vs 측정 Δpose(KISS)”의 residual로 구성
- 장점: 위치/요/자세를 동시에 정합 → 더 강한 결합
- 단점: 자코비안/리셋 안정성/게이팅 설계 부담↑

### 옵션 C: ICP 자체를 ESKF state에 condition(최적화 결합)
- 사실상의 tightest coupling(비선형 최적화 성격)
- 초기 Phase 2에서는 범위 초과(추후 발전)

---

## 6) 품질지표(Quality)와 공분산(R) 정책

Tight coupling 품질의 핵심은 “측정을 언제/얼마나 믿을지”다.

### 최소 요구(초기)
- `R`은 고정 시작값으로 시작하되, 아래 조건에서 inflate/skip:
  - correspondence 수가 너무 적음
  - ICP 수렴 실패/반복수 과다
  - adaptive threshold(σ)가 비정상적으로 큼
  - Δpose가 물리적으로 불가능(가속/요레이트 한계 초과)

### 권장 발전
- `σ`(adaptive threshold) 기반으로 `R ∝ σ^2` 형태로 연속 스케일링
- “GNSS 상태 기반 램핑”:
  - 음영 진입: LiDAR R을 점진적으로 낮추고(신뢰↑)
  - 복구: LiDAR R을 점진적으로 높이고(신뢰↓)
  - 스위치가 아니라 **연속 변화**로 transition 품질 확보

### (중요) “GNSS가 거의 GT” 가정은 설정에 따라 깨질 수 있음
- GNSS 토픽이 20Hz로 들어온다는 사실과, “해당 구간에서 ESKF가 GNSS를 강하게 믿는다”는 것은 다르다.
- 실제 운영 파라미터에 따라 status==1(SBAS) 구간을 크게 다운웨이트할 수 있으므로,
  “GNSS good 구간/GT 구간”의 정의는 반드시 다음을 함께 고려한다:
  - GNSS status(0/1/2)
  - NavSatFix covariance 크기
  - ESKF 내부에서 적용된 covariance scale/inflate 및 NIS 인플레이트 여부

### 로컬맵 “건강도”와 리앵커/리셋 정책
로컬맵이 “건강하지 않다”는 것은 단순히 드리프트 누적만이 아니라,
현재 스캔이 로컬맵에 **정상적으로 정합되지 않는 상태**를 의미한다(원인은 다양).

가능한 원인 예시:
- 초기 guess/시간정렬 문제(지연으로 잘못된 시각의 pose로 정합 시작)
- 환경 퇴화(터널/장벽 반복 패턴/개활지), 동적 객체 비중 증가
- deskew/타임스탬프 필드/TF extrinsic 문제로 스캔이 “찌그러짐”
- 누적 드리프트로 인해 로컬맵이 실제 환경과 벌어짐

권장 정책(2단계):
1) **Soft anchor(저빈도)**: KISS-ICP의 내부 pose를 ESKF pose로 “당겨서” 초기 guess를 안정화(맵은 유지)
2) **Hard reset(필요 시)**: 로컬맵까지 `Reset()`하고 ESKF pose에서 재시작

> 리앵커를 GNSS 20Hz에 맞춰 매번(20Hz) 강제로 하는 것은 일반적으로 비권장이다.  
> 잦은 강제 정렬은 연속성을 깨고 로컬맵을 흔들어 오히려 ICP 품질을 떨어뜨릴 수 있다.

### “누가 맞나” 판단(ESKF vs ICP)과 리앵커 조건
단순히 `|p(base_link_icp) - p(base_link)|` 임계치만으로 리앵커하면,
“ESKF가 틀린 순간에 ICP를 망가뜨리는” 문제가 생긴다.

따라서 리앵커/리셋은 다음의 **신뢰도 기반 게이팅**을 포함해야 한다:
- ESKF 신뢰도(예):
  - GNSS status==2 + GNSS covariance 작음
  - 최근 GNSS update에서 NIS 인플레이트가 과하지 않음
  - P(pos/yaw)가 과도하게 커지지 않음
- ICP 신뢰도(예):
  - correspondence/σ/수렴 여부/반복 횟수
  - `Δpose`가 물리 한계(가속/요레이트/속도) 내

권장 동작(예시):
- **ESKF↑ & ICP↓**: ICP soft anchor 또는 hard reset
- **ICP↑ & ESKF↓(GNSS 튐/멀티패스)**: GNSS 영향 다운웨이트(기존 정책 활용) + ICP 상대모션 update 비중↑
- **둘 다↓**: 업데이트를 보수적으로(스킵/강한 인플레이트), 안전 모드

### 임계값(시작점 예시)
임계값은 데이터/차종/환경에 따라 달라지며, “지속시간 + 히스테리시스”가 중요하다.

- 에러 정의: `T_err = T_map_base_link * inv(T_map_base_link_icp)`
  - `e_p = ||trans(T_err)||`, `e_yaw = yaw(T_err)`
- Soft anchor 후보(예):
  - `e_p > 0.5~1.0m` 또는 `|e_yaw| > 3~5deg`가 **0.5~2.0초 지속**
- Hard reset 후보(예):
  - `e_p > 3~5m` 또는 `|e_yaw| > 10~15deg`가 지속 + ICP 품질 저하 동반
  - 또는 ICP 실패 연속 발생

---

## 7) 시간정렬/지연 처리(운영 리스크 1순위)

### 현실적인 제약
- pointcloud 처리(ICP)는 지연이 발생할 수 있고, stamp 기준으로 보면 ESKF state보다 과거 측정이 될 수 있다.

### Phase 2.1(최소 정책)
- ESKF 업데이트는 `|t_icp - t_state| <= tol`일 때만 적용(기본 tol을 운영에 맞게 확대)
- tol을 넘으면:
  - (A) skip
  - (B) “현재 시각으로 재태깅” 같은 편법은 금지(일관성 깨짐)

### Phase 2.2(권장)
- 짧은 state history(예: 0.5~1.0s) 유지
- 측정이 과거면:
  - 히스토리에서 해당 시각 state를 꺼내 업데이트
  - 이후 IMU/차량 제약을 재적용(repropagate)

### (운영 팁) 리앵커 빈도와 타이밍
- GNSS가 20Hz라고 해서 ICP를 20Hz로 “하드 리앵커”하는 방식은 피한다.
- 권장:
  - 매 LiDAR 프레임: ESKF 예측을 initial guess로 제공(연속성 유지)
  - soft anchor/hard reset: “상태 전환 이벤트(음영→복구)” 또는 “발산 징후”일 때만 저빈도로 수행

---

## 8) 구현 마일스톤(제안)

### M0: 지표/관측 기반 정리(0.5~1일)
- 동일 시각 기준으로 다음을 비교하는 툴/rosbag 재현 절차 확립
  - `map->base_link`(ESKF) vs `map->base_link_icp`(KISS)
  - GNSS status(0/1/2) 구간별 오차 통계
- 목표: “3m 오프셋”이 드리프트인지, 초기정렬 문제인지, 지연/시각 불일치인지 분리

### M1: 단일 프로세스 내장(1~2일)
- `eskf_localization` 내부에 KISS-ICP pipeline 객체 포함
- pointcloud subscription을 ESKF 노드에서 직접 받아 KISS-ICP `RegisterFrame()` 호출
- 기존 `kiss_icp_node`는 비활성화 가능하도록 런치 분리

### M2: Phase 2 최소 융합(2~4일)
- `ΔT_icp`로부터:
  - body-velocity 업데이트(우선 x, 이후 y 옵션)
  - yaw 업데이트 또는 yaw-rate/bias 제약
- quality 기반 게이팅/인플레이트 추가
- GNSS 상태 기반 램핑(진입/진출 transition 스무딩)

### M3: 상대 포즈 residual 업데이트(선택, 3~6일)
- `update_relative_pose_6d` API 추가 및 H/R 설계
- 게이팅(NIS)과 최대 보정량 제한 정책 확립

### M4: OOS 최소 지원(선택, 3~7일)
- 짧은 state history + repropagate
- rosbag 기반 회귀 테스트 추가(가능하면)

---

## 9) TODO 체크리스트 (착수용)

> 원칙: “바로 코드를 치기” 전에, 먼저 M0에서 **현상/지연/신뢰도**를 계측해 기준선을 만든다.  
> 이후 M1~M2는 “작게 켜고, 안전하게 확장”한다.

### M0 — 재현/계측 (Start Here)
- [ ] 대상 rosbag 1~2개 선정 (필수 포함: GNSS 음영 진입/진출, GNSS 튐/멀티패스 의심 구간)
- [ ] 동일 timestamp 기준 에러 정의/계측 확정
  - `T_err = T_map_base_link * inv(T_map_base_link_icp)`
  - `e_p = ||trans(T_err)||`, `e_yaw = yaw(T_err)`
- [ ] ICP 품질지표 로깅 항목 확정(최소)
  - `sigma`(adaptive threshold), ICP 수렴 여부/반복 횟수, 대응점 수(가능하면)
- [ ] “지연” 계측(필수)
  - pointcloud stamp vs ESKF state stamp 차이 분포(평균/최대/95%)
  - GNSS stamp vs ESKF state stamp 차이 분포(참고)
- [ ] 출력물(문서/아티팩트)
  - [ ] rosbag별 구간 라벨링(구간 시작/종료 시각)
  - [ ] e_p/e_yaw 시계열 플롯 + 구간별 통계(평균/RMS/최대)
  - [ ] “3m 차이”의 원인 가설 1~2개로 압축(드리프트 vs 지연 vs 초기정렬 vs 환경퇴화)

**M0 Done(완료조건)**:
- “현재 구조에서” 음영/복구 구간의 대표 지표가 숫자로 나온다(예: transition 구간 e_p max, e_yaw max).
- 시간정렬 문제가 있는지/없는지(필요 tol 범위)가 데이터로 확인된다.

### M1 — 단일 프로세스 내장(기술 부채 정리)
- [ ] `eskf_localization`에 KISS-ICP pipeline 라이브러리 링크/빌드 연결
- [ ] ESKF 노드에서 pointcloud subscribe + `RegisterFrame()` 호출 경로 추가
- [ ] 파라미터 플래그(필수)
  - [ ] `enable_kiss_icp` (전체 on/off)
  - [ ] `enable_kiss_icp_publish_debug` (선택: 디버그 토픽/메트릭 publish)
- [ ] 런치/운영 구성
  - [ ] 기존 `kiss_icp_node`를 끄고도 동일 rosbag에서 ESKF 단독이 정상 동작(롤백 경로)

**M1 Done**:
- `enable_kiss_icp=false`일 때 기존과 출력이 동일(회귀 없음).
- `enable_kiss_icp=true`일 때 KISS-ICP가 내부에서 돌아가며, 성능/지연이 측정 가능하다.

### M2 — 최소 융합(안전하게, 효과 빠르게)
> 목표: “seamless transition”과 “음영 DR 일관성”을 먼저 체감 가능하게 만들기.

- [ ] ICP update는 “절대 pose 강제”가 아니라 **상대모션 기반 update**로 시작
- [ ] 1단계: `yaw` 업데이트만 먼저 연결(품질 게이트 포함)
  - [ ] `enable_kiss_icp_yaw_update`
  - [ ] yaw 측정은 `ΔT_icp` 기반, `R_yaw`는 σ/품질에 따라 inflate 가능
- [ ] 2단계: `v_y` 업데이트 추가(선택)
  - [ ] `enable_kiss_icp_vy_update`
  - [ ] `R_vy` 품질 기반 스케일
- [ ] `v_x`는 기본적으로 OBD(+GNSS speed) 유지
  - [ ] `v_x`에 대한 ICP update는 기본 off 또는 “품질 매우 좋을 때만” 보조
- [ ] GNSS 상태 기반 램핑(필수)
  - [ ] 음영 진입/진출에서 ICP update 가중치를 “스위치”가 아니라 “램핑”으로 변화

**M2 Done(최소 성공 기준)**:
- 음영 진입/진출 구간에서 포즈 점프가 M0 대비 유의미하게 감소(예: e_p max 또는 e_yaw max 감소).
- GNSS 튐 구간에서 궤적 흔들림이 감소(주관+지표 모두).
- ICP 실패/품질 저하 시 ESKF가 망가지지 않는다(스킵/인플레이트로 방어).

### M3 — 리앵커/리셋(정렬 유지 장치)
- [ ] `T_err` 기반 soft anchor/hard reset 정책 구현(저빈도)
  - [ ] “임계값 + 지속시간 + 히스테리시스” 적용
  - [ ] GNSS 20Hz마다 하드 리앵커 금지(기본 정책)
- [ ] “누가 맞나” 신뢰도 게이팅(필수)
  - [ ] ESKF 신뢰도: GNSS status/cov, NIS 인플레이트, P 통계 등
  - [ ] ICP 신뢰도: σ/수렴/물리 한계/연속성 등

**M3 Done**:
- “ESKF가 틀린 순간에 ICP를 망가뜨리는” 케이스가 재현되더라도, 게이팅으로 방어된다.
- 발산 징후에서 hard reset이 동작하고, 이후 복구가 가능하다.

### M4 — NHC 가변 정책(충돌 제거)
- [ ] ICP로 `v_y`를 넣는 동안 NHC(`v_y≈0`)가 싸우지 않도록 옵션화
  - [ ] ICP `v_y` 활성 시: NHC(v_y) 약화(= var 증가) 또는 v_z만 유지하는 모드 제공
  - [ ] ICP 품질 저하 시: NHC 원복(안전망)

**M4 Done**:
- ICP `v_y` update 활성 상태에서도 횡방향이 과도하게 0으로 끌리는 현상이 사라진다.

### M5 — 시간정렬/OOS(선택, 필요 시)
- [ ] Phase 2.1 tol 정책 튜닝(“스킵이 너무 많지 않게”)
- [ ] 필요 시 짧은 state history + repropagate로 OOS 최소 지원

**M5 Done**:
- ICP update 적용률이 충분하고(스킵 과다 해소), 성능/지연이 운영 가능 범위다.

---

## 10) 테스트/검증 체크리스트

### 기능
- [ ] GNSS 음영 진입/진출에서 포즈 점프가 줄었는가?
- [ ] 음영 구간에서 속도/요가 “일관”되게 유지되는가?
- [ ] GNSS 튐(멀티패스) 구간에서 궤적이 덜 흔들리는가?

### 안전장치
- [ ] ICP 실패 시 ESKF가 망가지지 않는가? (skip / R inflate)
- [ ] 비정상 Δpose(급격 이동/급회전)에서 보호가 동작하는가?

### 성능
- [ ] 50Hz publish 목표에서 CPU 사용량/지연이 허용 범위인가?
- [ ] Multi-LiDAR 입력 동기화가 실제 데이터에서 안정적인가?

---

## 11) 롤백/운영 플래그

- 파라미터로 기능을 단계적으로 켜고 끌 수 있어야 한다.
  - `enable_kiss_icp` (pipeline on/off)
  - `enable_kiss_icp_yaw_update`
  - `enable_kiss_icp_vel_update`
  - `kiss_icp_quality_gate_*`
  - `kiss_icp_R_*` 및 GNSS 상태 기반 램핑 계수

### (추가) NHC(Non-Holonomic Constraint) 정책
- NHC는 `v_y≈0, v_z≈0`를 강제하는 제약이며, ICP에서 `v_y`를 측정으로 넣기 시작하면 **서로 싸울 수 있다**.
- 권장:
  - ICP `v_y`/`yaw` 업데이트를 강하게 쓰는 구간에는 `v_y` NHC를 약화(= `vehicle.nhc_var` 증가)하거나,
    `v_z`만 유지하고 `v_y`만 선택적으로 약화/비활성화하는 옵션을 둔다.
  - ICP 품질이 나쁘면 NHC를 다시 강하게(원복) 해서 안전망으로 사용한다.

문제 발생 시:
- KISS-ICP를 끄면 즉시 기존 ESKF 운영 모드로 복귀 가능해야 한다.
