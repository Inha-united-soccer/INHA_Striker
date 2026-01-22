# Striker 알고리즘 참고문헌 (Algorithm References)

이 문서는 INHA Striker Brain에서 사용되는 알고리즘과 참고문헌을 **실제 구현에 기반하여** 정리합니다.

---

## ⚠️ 중요 사항

**대부분의 구현은 표준 알고리즘의 변형이 아니라, 로봇 축구 도메인에 특화된 휴리스틱입니다.** 따라서 다음과 같이 구분합니다:

1. **인용이 필요한 경우**: 표준 알고리즘/프레임워크를 직접 사용하거나 명확히 기반한 경우
2. **인용이 불필요한 경우**: 기본 수학/제어 개념 또는 도메인 특화 휴리스틱

---

## 1. Behavior Tree (행동 트리) ✅ 인용 필요

**사용 위치**: 전체 의사결정 구조 (`behavior_trees/`, `brain_tree.h`)

**설명**: BehaviorTree.CPP 라이브러리를 사용하여 계층적 행동 트리 구조를 구현합니다.

**참고문헌**:
- **BehaviorTree.CPP Library**: https://www.behaviortree.dev/
  - 실제 사용 중인 라이브러리이므로 공식 문서 참조
- Colledanchise, M., & Ogren, P. (2018). *Behavior Trees in Robotics and AI: An Introduction*. arXiv preprint arXiv:1709.00084.
  - Behavior Tree 개념에 대한 이론적 배경

**구현 특징**:
- `ReactiveSequence`: 중단 가능한 행동 실행
- `_while` 조건: 상태 기반 조건부 실행
- 서브트리 모듈화

**코드 위치**: `src/brain/behavior_trees/`, `src/brain/include/brain_tree.h`

---

## 2. 장애물 회피 (Obstacle Avoidance) ⚠️ 기본 개념

**사용 위치**: `brain.cpp::calcAvoidDir()`, `brain.cpp::findSafeDirections()`, `chase.cpp`

**설명**: 특정 방향에서의 장애물 거리를 계산하고, 좌/우로 안전한 회피 방향을 탐색하는 **단순한 휴리스틱**입니다.

**구현 분석**:
- `distToObstacle(angle)`: 특정 각도 방향의 최단 장애물 거리 계산 (기본 기하학)
- `findSafeDirections()`: 좌/우로 각도를 스캔하며 안전 거리 이상인 첫 방향 찾기
- `calcAvoidDir()`: 목표 방향에 가까운 안전 방향 선택

**인용 필요성**: ❌ **불필요**
- VFH나 다른 복잡한 알고리즘이 아님
- 단순한 거리 기반 회피 로직
- 기본 기하학 계산 + 휴리스틱

**코드 위치**:
```cpp
// src/brain/src/brain.cpp:1169-1267
double Brain::distToObstacle(double angle)  // 기본 거리 계산
vector<double> Brain::findSafeDirections(double startAngle, double safeDist)  // 좌/우 스캔
double Brain::calcAvoidDir(double startAngle, double safeDist)  // 방향 선택
```

---

## 3. Point-to-Segment Distance ⚠️ 기본 수학

**사용 위치**: `chase.cpp::DribbleToGoal` (경로 장애물 평가)

**설명**: 선분(경로)에 대한 점(장애물)의 최단 거리를 계산합니다.

**인용 필요성**: ❌ **불필요**
- 기본 기하학 공식 (벡터 투영)
- 표준 수학 교과서 수준

**수학적 공식** (표준):
```
t = clamp((P - A) · (B - A) / ||B - A||², 0, 1)
closest_point = A + t * (B - A)
distance = ||P - closest_point||
```

**코드 위치**:
```cpp
// src/brain/src/chase.cpp:347-353
double t = ((obs.posToField.x - ballPos.x) * dx + (obs.posToField.y - ballPos.y) * dy) / (dx*dx + dy*dy);
t = max(0.0, min(1.0, t));
double closestX = ballPos.x + t * dx;
double closestY = ballPos.y + t * dy;
double d = hypot(obs.posToField.x - closestX, obs.posToField.y - closestY);
```

---

## 4. Score-based Path Selection (점수 기반 경로 선택) ⚠️ 도메인 특화

**사용 위치**: `DribbleToGoal` (골대 방향 선택), `offtheball.cpp` (오프더볼 위치 선택)

**설명**: 후보 위치들에 대해 다중 요소 점수를 계산하여 최적의 목표 지점을 선택하는 **도메인 특화 휴리스틱**입니다.

**인용 필요성**: ❌ **불필요**
- ROS Costmap이나 DWA와는 다른 단순 점수 계산
- 로봇 축구 도메인에 특화된 휴리스틱
- 표준 알고리즘이 아님

**구현 특징**:
- 후보 위치 그리드 생성
- 다중 요소 점수 계산:
  - 장애물 거리 (clearance)
  - 골대 중앙 선호 (centerPenalty)
  - 골 유효 범위 보너스 (goalBonus)
- 최고 점수 위치 선택

**코드 위치**:
```cpp
// src/brain/src/chase.cpp:333-388 (DribbleToGoal 경로 선택)
// src/brain/src/offtheball.cpp (오프더볼 위치 선택)
```

---

## 5. Radial-Tangential Control ⚠️ 기본 제어 개념

**사용 위치**: `chase.cpp::DribbleToGoal` (CircleBack 단계)

**설명**: 공을 중심으로 한 극좌표계에서 반지름 방향과 접선 방향 제어를 분리합니다.

**인용 필요성**: ❌ **불필요**
- 극좌표계 변환은 기본 수학/제어 개념
- 표준 제어 이론 교과서 수준

**구현 특징**:
- `v_radial`: 거리 오차에 비례한 반지름 방향 속도 (P 제어)
- `v_tangential`: 각도 오차에 비례한 접선 방향 속도 (P 제어)
- 두 성분을 합성하여 필드 좌표계 속도 계산

**코드 위치**:
```cpp
// src/brain/src/chase.cpp:467-499
double distError = tightCircleBackDist - distToBall;
double v_radial = distError * 2.0;  // P 제어
double angleError = toPInPI(desiredAngle - angleBallToRobot);
double v_tangential = angleError * 1.5;  // P 제어
// 극좌표계 → 필드 좌표계 → 로봇 좌표계 변환
```

---

## 6. Proportional (P) Control ⚠️ 기본 제어 개념

**사용 위치**: `chase.cpp::Chase`, `chase.cpp::SimpleChase`

**설명**: 목표 위치까지의 오차에 비례하여 속도를 결정합니다.

**인용 필요성**: ❌ **불필요**
- 기본 제어 이론 (PID의 P 성분)
- 표준 제어 교과서 수준

**코드 위치**:
```cpp
// src/brain/src/chase.cpp:162-172
double p_gain = 1.0;
vx = target_r.x * p_gain;
vy = target_r.y * p_gain;
```

---

## 7. Hysteresis-based State Machine ⚠️ 기본 제어 개념

**사용 위치**: `chase.cpp::DribbleToGoal` (CircleBack ↔ Push 전환), `striker_decision.cpp` (Kick Lock)

**설명**: 상태 전환 시 히스테리시스를 사용하여 노이즈로 인한 불필요한 상태 토글을 방지합니다.

**인용 필요성**: ❌ **불필요**
- 일반적인 제어 이론의 히스테리시스 패턴
- Schmitt Trigger와 유사한 개념이지만 표준 제어 교과서 수준

**코드 위치**:
```cpp
// src/brain/src/chase.cpp:448-453
double enterThresh = deg2rad(30);  // 진입 임계값
double exitThresh = deg2rad(20);   // 탈출 임계값 (히스테리시스)
if (alignmentError > enterThresh) isCircleBack = true;
else if (alignmentError < exitThresh) isCircleBack = false;
```

---

## 8. Sigmoid-based Speed Modulation ⚠️ 기본 수학 함수

**사용 위치**: `chase.cpp::SimpleChase`

**설명**: 공과의 거리와 각도에 따라 부드럽게 속도를 조절하는 시그모이드 함수 사용.

**인용 필요성**: ❌ **불필요**
- 시그모이드는 기본 수학 함수
- 로봇 제어에서 거리 기반 속도 조절은 일반적인 패턴

---

## 📋 최종 정리

### ✅ 인용이 필요한 경우

| 항목 | 참고문헌 | 이유 |
|------|---------|------|
| **Behavior Tree** | BehaviorTree.CPP Library (https://www.behaviortree.dev/) | 실제 사용 중인 라이브러리 |

### ❌ 인용이 불필요한 경우 (기본 개념/도메인 특화)

- **장애물 회피**: 단순 거리 기반 휴리스틱 (VFH 아님)
- **Point-to-Segment Distance**: 기본 기하학
- **Score-based Path Selection**: 도메인 특화 휴리스틱
- **Radial-Tangential Control**: 기본 극좌표계 변환
- **P Control**: 기본 제어 이론
- **Hysteresis State Machine**: 기본 제어 패턴
- **Sigmoid Function**: 기본 수학 함수

---

## 💡 논문 작성 시 권장사항

1. **Behavior Tree**: BehaviorTree.CPP 라이브러리 사용 명시
2. **나머지 알고리즘**: 
   - "기본 제어 이론" 또는 "도메인 특화 휴리스틱"으로 설명
   - 특별한 알고리즘 논문 인용 불필요
   - 대신 **로봇 축구 도메인에서의 적용 방법**을 강조
3. **RoboCup 관련 논문**: 
   - RoboCup Humanoid League 관련 최신 논문 참조 권장
   - 하지만 특정 알고리즘 인용보다는 **전체 시스템 접근 방식** 비교가 더 적절할 수 있음

---

**결론**: 대부분의 구현은 표준 알고리즘의 변형이 아니라, 로봇 축구 도메인에 특화된 휴리스틱입니다. Behavior Tree 라이브러리만 명시적으로 인용하고, 나머지는 "기본 제어/수학 개념" 또는 "도메인 특화 휴리스틱"으로 설명하는 것이 적절합니다.

