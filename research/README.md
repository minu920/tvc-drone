# TVC 연구용 개발환경 / 2단계 및 모델 규약 / 3단계 A

원본 기준: `5cff917900f27b2e5b21c164b132abaccfa6da86`, 작업 브랜치 `bench-test`.
이 폴더는 원본 기체를 그대로 제작하는 설명서가 아니라 재현·비교 실험을 위한 추가 도구다.
비행 펌웨어/게인/보정값은 변경하지 않았다. 모든 Python 검사는 오프라인이며 기체와 연결하지 않는다.

## 지금 실행하기

저장소 루트에서 실행한다. 현재 Windows에서는 `.venv`에 Python 3.12 분석/GCS 의존성을 설치했다.
이 가상환경의 기반 Python은 앱에 포함된 런타임이다. 앱 런타임 경로가 바뀌거나 다른 PC로 옮기면
일반 Python 3.12로 가상환경을 새로 만들어야 하며 `.venv` 자체를 복사하지 않는다.

```powershell
.\.venv\Scripts\python.exe research/parameter_check.py --profile research/parameters/my_airframe.json
.\.venv\Scripts\python.exe research/baseline_check.py
.\.venv\Scripts\python.exe research/model_check.py
.\.venv\Scripts\python.exe -m unittest discover -s research -p 'test_*.py' -v
.\.venv\Scripts\python.exe research/actuator_study.py --profile research/parameters/upstream_reference.json
.\.venv\Scripts\python.exe research/propulsion_check.py --output artifacts/propulsion
.\.venv\Scripts\python.exe research/pid_attitude.py --profile research/parameters/upstream_reference.json --output artifacts/pid-attitude
.\.venv\Scripts\python.exe research/gcs_smoke.py
.\.venv\Scripts\python.exe -m pip check
```

Ubuntu 또는 다른 PC에서 새로 설치할 때, Python 3.12와 venv 지원을 준비한 후:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-research.txt
.venv/bin/python research/baseline_check.py
.venv/bin/python research/model_check.py
.venv/bin/python -m unittest discover -s research -p 'test_*.py' -v
.venv/bin/python research/actuator_study.py
# GCS가 필요할 때만 Qt/시리얼 의존성 추가
.venv/bin/python -m pip install -r requirements-gcs.txt
.venv/bin/python research/gcs_smoke.py
```

위 Ubuntu 명령은 이 PC에서 실행 검증하지 않았다. Qt의 시스템 라이브러리/화면 환경은 별도로 필요할 수 있다.
직접 의존성은 버전을 고정했고, Windows에서 실제 해결된 전체 버전은
`artifacts/baseline/environment-freeze.txt`에 저장된다. 이것은 범용 Ubuntu lock 파일이 아니다.

## 도구와 산출물

- `parameters/upstream_reference.json`: 원본 재현용 참고 파라미터. 내 기체의 실측값이 아님.
- `parameters/my_airframe.json`: 내 기체용 초안. 중력 관례값 외에는 미정이며 원본 값을 자동 상속하지 않음.
- `parameter_profiles.py`, `parameter_check.py`: SI 단위·출처·상태·값 검증, 계산별 입력 점검 및 스냅샷/해시 기록.
  파라미터 관리는 [3단계 B 안내](../docs/kr/파라미터-관리.md)를 참고한다.
- `tvc_model.py`: NED/FRD 힘·토크, ROS 경계 좌표 변환, SI 축약 호버 LQI 모델. 실기 보정/완전한 비선형 시뮬레이터는 아님.
- `model_check.py`, `test_tvc_model.py`: 원본 수식과 좌표/단위 변환 후 등가성 및 물리 규약 검사.
  출력은 `artifacts/model-contract/`. 미정인 Izz/최대추력 등을 임의로 채우지 않음.
- `baseline_check.py`: 원본 `apps/python/lqr_compute.py`를 수정 없이 실행. 콘솔, 원본 그래프,
  행렬/상태 NPZ, 버전/체크 JSON, 패키지 버전 목록을 `artifacts/baseline/`에 저장.
- `actuator_study.py`: 가상의 1축 자유비행 모델에서 이상 액추에이터 LQI와 지연 상태 확장 LQI 비교.
  결과는 `artifacts/actuator-study/`. 명령 지연 0/20/60 ms와 서보 시정수 20/40/80/120 ms의 24개 조합 포함.
  위 시간은 원본 프로파일 기준이다. `--profile`로 입력을 선택하고 `--output`으로 실험 폴더를 분리한다.
  입력이 부족한 내 기체 프로파일은 실행을 중단한다. 결과에는 실행 당시 값과 SHA-256을 보존한다.
- `test_actuator_study.py`: 설계 극점, 유한한 상태값, 각도/속도 제한, 지연 큐, 잘못된 입력 검사.
- `parameters/propulsion_candidate.json`, `propulsion_check.py`: 동축 추진계 후보를 제조사 공표
  정지추력표로 서류 판정. APC의 계열별 RPM 상한, 부하 RPM 추정, 동축 손실 가정에 따른 추력·T/W·질량
  상한을 계산하고 `artifacts/propulsion/`에 기록한다. 공표 데이터 범위 밖은 외삽하지 않고 중단한다.
  전류·체공시간은 계산하지 않는다. 측정된 모터 효율 곡선이 필요하며 이 저장소에는 없다.
  `--strict`는 최저 운용 전압에서 목표를 만족하지 못하면 종료 코드 2를 낸다.
- `test_propulsion_check.py`: 표 값 재현, 외삽 거부, 미정값 거부, 계열별 상한, 손실 민감도 단조성 검사.
  12×3.8SF가 이 속도에서 상한을 넘는다는 사실을 회귀 시험으로 고정한다.
- `pid_attitude.py`, `test_pid_attitude.py`: 원본 내부 루프 LQI를 대체하는 종속 PID 자세 제어기와
  TVC 할당. 할당식은 `tvc_model.body_wrench`의 정확한 역변환이며, 왕복 시험으로 고정한다.
  원본 `pid_t`의 규약(적분 상태 클램프, 측정값 미분, 미분 필터, 외부 anti-windup)을 그대로 따라
  게인이 펌웨어로 그대로 넘어간다. 요는 닫지 않는다 — 차동추력 계수가 미측정이며 단독 모터
  데이터로 구할 수 없으므로 요청 시 오류를 낸다. `Izz`도 요구하지 않는다: 요 각속도를 0으로 두면
  자이로 항의 롤·피치 성분이 사라진다.
- `gcs_smoke.py`: Qt 화면 두 개 생성과 경로 보간만 검사. 시리얼을 열지 않으며 화면도 표시하지 않는다.
- `build_stm32.ps1`: 기존 STM32CubeIDE로 새 복사본을 빌드. 원본 프로젝트와 이전 결과를 덮어쓰지 않는다.
  실행: `powershell -File research/build_stm32.ps1`. 보드에 업로드하지 않는다.

생성 결과는 반복 실행 시 같은 결과 파일을 갱신한다. 보존할 실험은 Python 도구의 `--output`에 새 폴더를 지정한다.
`.venv/`, `.build/`, `artifacts/`는 Git에서 제외했다. 보고서에 쓸 확정 실험 데이터는 별도 버전 관리 전략이 필요하다.

## 결과 해석 제한

`baseline_check.py`의 통과는 의존성과 계산 재현을 의미한다. 비행 안전, 비선형 안정성,
센서 융합 정확도, 펌웨어와의 완전한 일치를 증명하지 않는다.

`actuator_study.py`는 연구 질문을 구체화하기 위한 예제다. 질량/관성/레버암은 원본 계산값이고,
서보 시정수·속도·외란은 실측값이 아니다. 두 제어기는 동일한 50 Hz와 상태 비용으로 설계했지만,
상태 확장형은 실제 서보 각도를 정확히 알고 있다고 가정한다. 원본 서보에서 이 값이 읽히는 것은 아니며,
실제 구현에는 엔코더나 검증된 관측기가 필요하다. 원본 100 Hz 다축 펌웨어를 그대로 모사한 것도 아니다.

첫 8초 실험에서 지연 상태 확장 LQI는 120 ms 조건의 흔들림을 줄였지만 전체 구간 자세 RMS는
1.676°에서 1.693°로 소폭 증가했다. 40 ms에서는 1.525°와 1.623°다. 따라서 "새 제어기가 더 좋다"고
결론내릴 수 없다. 같은 목표 응답 속도/입력 사용량, 상태 추정 오차, 다양한 외란을 포함해 비교해야 한다.

연구 방향·환경 선택·다음 실험은 [연구 로드맵](../docs/kr/연구-로드맵.md),
현재 검증 범위는 [환경 검증 기록](../docs/kr/개발환경-검증.md)을 참고한다.
3단계 A의 규약과 아직 해결하지 않은 문제는 [모델 규약](../docs/kr/모델-규약.md)에 정리했다.
