# WEBCAM006 검토 자료

- `local/validation.json`: 75검사와 입력 보존 검사 범위.
- `local/capture_validation.json`: 합성 앱4프레임에서2번의 명시적 저장, 표시 좌표/픽셀 동일성. 내부 case 해시는 로컬 합성 실행의 입력 식별자다.
- `local/preview_layout.png`: 새 S 버튼·저장 건수·확인 표시. 합성 카메라이므로 사진 영역은 비어 있다.
- `local/mediapipe_raw_synthetic.png`: 같은 합성 저장의 원본 좌표3방향. 실제 사용자 추종 정확도 평가가 아니다.
- `local/form_report.json`: 동결된 FORM002의132입력·124메시·20축·33자세 보존.

재현: `python -m unittest discover -s sim_3rd/hand_follow/tests -v`, `python sim_3rd/hand_follow/evaluate_pose_cases.py --output <별도폴더>`, `python cad/accepted_form_v1/verify.py --output <별도폴더>`.

실제 사용자 사진과 좌표는 여기에 포함하지 않는다. 모드가 shape인 실제 실행은 확인했지만 사용자 표시 사례는 이 체크포인트 작성 시0건이다. 코드 검사와 실시간 품질 평가를 구분한다.
