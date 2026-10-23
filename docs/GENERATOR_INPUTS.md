# 기존 생성기의 호환 입력

V9 build_candidate.py는 원본 sources/XL330.stp, cad/index_joint_v1의 ROBOTIS 프레임 STEP, 루트 robot_config.json을 읽습니다. 모두 원본 바이트로 보존했습니다.

robot_config.json의 작은 손 치수/구동 가정은 역사적인 입력입니다. 현재 CAD 생성기가 사용하는 것은 arm.motors의 모터 외형과 arm.joints의 이름 순서입니다. 현재 손 치수·자세의 원본은 cad/modular_roots_v9/design.json입니다. 오래된 config의 손 질량/동역학 수치를 V9/V12 검증에 사용하면 안 됩니다.

현재 후보 조립은 원본 CAD에서 수정 부품만 교체합니다. V9 생성기를 동결 폴더에서 다시 실행하면 감사 증거를 덮어쓸 수 있으므로, 재생성 실험은 복사본에서 수행합니다.

공식 모터 데이터의 세대·전압별 사양은 일치시켜야 합니다. 기존 config의 AK45-10 V3.0과 구동 대안 표의 baseline AK45-10은 동일한 SKU 검증이 완료되지 않았습니다. 최종 모터를 확정할 때 해당 도면/토크–속도 표로 다시 계산합니다.
