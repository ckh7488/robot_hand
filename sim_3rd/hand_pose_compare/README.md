# 손 인식 모델 비교

로봇 변환과 CAD를 제외하고, 사용자가 저장한 동일 사진에서 MediaPipe와 WiLoR의 원본 3D 손을 비교한다. `hand_follow`의 실행 환경·관절 보정·형상은 변경하지 않는다.

## 실행 경로

- 비교 후보: [WiLoR 공식 구현](https://github.com/rolpotamias/WiLoR)이 연결한 [WiLoR-mini](https://github.com/warmshao/WiLoR-mini), 소스 `ebec42f94c389070cdd7dda6fd1bf0b4a659c960`.
- Chumpy 소스: [mattloper/chumpy](https://github.com/mattloper/chumpy), `580566eafc9ac68b2614b64d6f7aaa84eebb70da`. 이 버전은 Python 3.12에서 제거된 getargspec에 대한 패치가 필요 없었다.
- 가중치는 공개 `warmshao/WiLoR-mini` 저장소에서 버전을 고정해 받고 SHA-256을 기록한다. 환경·가중치·개인 사진은 D:의 별도 작업 경로에 둔다. 코드 저장소에는 넣지 않는다.
- Python 3.12의 독립 환경에서 `requirements.txt`를 사용한다. torch/torchvision은 CUDA 11.8 인덱스의 Windows 빌드다. 기존 웹캠 의존성 폴더에 설치하지 않는다.
- 공식 저장소에서 mini 실행판을 데모용으로 명시한다. 이 실험은 교체 가능성 평가이며 제품용 구현·배포 승인이 아니다. WiLoR·MANO·Ultralytics의 이용 조건은 각각 적용된다. 모델 파일을 재배포하지 않는다.

`pythonw.exe run_hidden.pyw job.json`으로 실행한다. Windows에서는 `Start-Process -WindowStyle Hidden`을 사용한다. 작업 설정 예:

```json
{
  "upstream": "D:/local-models/WiLoR-mini-ebec42f94c389070cdd7dda6fd1bf0b4a659c960",
  "chumpy": "D:/local-models/chumpy-580566eafc9ac68b2614b64d6f7aaa84eebb70da",
  "weights": "D:/local-models/wilor-weights",
  "cases": "D:/private-hand-session/cases",
  "output": "D:/private-hand-comparison/results",
  "cache": "D:/private-hand-comparison/cache",
  "device": "cuda:0",
  "repeats": 3
}
```

`weights/pretrained_models/`에 `mano_mean_params.npz`, `MANO_RIGHT.pkl`, `wilor_final.ckpt`, `detector.pt`가 있어야 한다. `cases/case_*/`에는 기존 앱에서 명시적으로 저장한 `camera.png`와 `case.json`이 있어야 한다. 로그는 `output/run.log`, 성공 표시는 `complete.json`, 오류는 `failed.txt`다. 새 출력 폴더를 사용한다.

## 비교 계약

1. 저장된 PNG를 색·좌우·해상도 변경 없이 OpenCV BGR로 읽는다. mini 내부 forward가 BGR→RGB를 수행한다. README의 별도 RGB 변환 예를 중복 적용하지 않는다. 공식 WiLoR의 OpenCV 입력 및 crop RGB 변환과 일치한다.
2. WiLoR의 YOLO가 독립적으로 검출하고 면적이 가장 큰 손을 선택한다. 모든 박스·점수·좌우 분류도 저장한다. MediaPipe의 박스·좌우 분류·관절 좌표를 새 모델 입력에 사용하지 않는다. 검출 실패는 실패로 남긴다.
3. GPU 메모리 중복을 줄이기 위해 체크포인트는 CPU에 먼저 읽는다. 누락된 모델 키가 있으면 중단하고, 불필요한 체크포인트 키는 보고한다. 원래 네트워크 구조·가중치·crop 배율은 그대로다. float32·한 손으로 실행한다.
4. 3D 표시에는 각 손바닥 기준의 강체 회전·이동만 적용한다. 두 모델에 같은 미터당 표시 배율을 사용한다. 개별 뼈 길이, 관절각, 가림 관계, 시간 필터는 수정하지 않는다. 좌우손의 입체 방향도 반사하지 않는다.
5. `projection.png`는 원본 사진 위에 MediaPipe의 저장된 2D 점과 WiLoR 3D의 투영을 따로 표시한다. 두 모델의 점 의미·손목 정의는 완전히 같지 않으므로 점 사이 차이를 정확도 점수로 부르지 않는다.
6. 예열 1회 후 장면당 3회 실행한다. GPU 동기화 시간을 포함한 검출·3D 복원·전후처리 시간을 기록하며 파일 읽기/모델 적재는 제외한다. 첫 실행 시간은 따로 남긴다. 실시간 카메라 지연·프레임률은 이 정지 사진 시험에서 검증하지 않는다.

`prediction.json`은 전체 모델 출력과 입력 해시, `summary.json`은 장면별 시간·검출·GPU 메모리다. 두 파일 모두 개인 사례 데이터로 취급한다. 반복 결과가 같아도 이는 동일 사진 재현성이지 시간축 떨림 검증이 아니다. `mesh.png`는 모델이 낸778정점을 정면·사선의 직교 투영과 z-buffer로 표시한다. 표면 정점/접촉/충돌을 수정하지 않는다.

## 검사와 다음 판단

`test_display.py`는 좌우손·두 보기에서 거리·입체 방향·입력 배열 보존, 퇴화 입력 거부를 검사한다. 저장 폴더의 다섯 장면을 모두 실행했다. 결과는 [VISION001](../../checkpoints/VISION001.md)에 기록한다. 사진 투영과 엄지/검지 관계, 처리 시간을 함께 검토한다.

개인화 학습은 틀린 MediaPipe 출력을 정답으로 복제하는 방식으로 진행하지 않는다. 우선 다른 사전학습 모델도 실패하는 자세를 분리하고, 그 자세의 정답을 수작업·복수 시점 등으로 검증할 방법을 마련해야 한다. 정지 사진 네 장만으로 학습 개선이나 떨림 해결을 주장하지 않는다.
