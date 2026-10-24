# WEBCAM005 검토 자료

`local/replay_report.json`: 같은 저장 입력의 전후 변화량, 손가락 유지율, 움직임 범위, 필터 합성 반응. 실제 자세 정확도/종단 지연은 아니다.

`local/tracking_comparison.gif`: 기록31–34초,43–46초,231–234초의 이전 출력/수정 코드 재생. 같은 FORM002 형상, 동일 저장 좌우 라벨. 비교 GIF는 표본마다150ms 표시하므로 실시간 속도 증거가 아니다.

`local/ui_benchmark.json`: 같은 픽셀로 UI 구성만 비교. 개선이 확인되지 않은 수치도 그대로 남겼다. `local/preview_layout.png`는 검정 합성 카메라 영역이 있는 레이아웃 검사 이미지다.

`local/validation.json`: 코드 검사 수와 합성 앱 종료 결과. 제작/충돌 검사는 포함하지 않는다. 정확 커밋 SHA의 새 사본 재검사는 커밋 후 별도 기록한다.
