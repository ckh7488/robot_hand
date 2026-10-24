# Remote drive V1 — 20축 배치 후보

기존 손바닥 232 × 195 mm, 손가락 뿌리 좌표·마디 길이·손목 연결부를 유지하고, 원위 마디의 모터 10개를 뿌리 가까이 옮기는 **실제 솔리드 CAD 후보**입니다. 모터 명령에서 실제 관절 추종까지 50 ms를 목표, 100 ms를 상한으로 삼습니다. 성능 목표를 달성한 제작 도면은 아닙니다.

![전체 조립과 손바닥 내부](images/assembly.png)

| 자료 | 내용 |
| --- | --- |
| [assembly.step.zip](assembly.step.zip) | 압축을 풀면 `assembly.step`. 이름과 색상을 가진 전체 다섯 손가락 조립 STEP |
| [assembly.glb](assembly.glb) | 가벼운 색상 포함 조립 미리보기. 제작용 도면이 아님 |
| [design.json](design.json) · [generate.py](generate.py) | 수정 가능한 파라미터와 CadQuery/OCC 생성기 |
| [simulation_contract.json](simulation_contract.json) | 20개 관절, 부모 관계, 축, 범위, CAD 기반 질량·무게중심·관성 텐서 |
| [generated/placements_mass_inertia.json](generated/placements_mass_inertia.json) | 각 부품의 배치와 질량·관성 근거 |
| [reviews/geometry.json](reviews/geometry.json) | BRep 유효성 및 중립 자세의 새 부품 교차 검사와 미해결 사항 |

## 배치와 제어 자유도

- 비엄지 네 손가락의 벌림·MCP 모터는 기존 뿌리 구조에 남습니다. MCP 모터는 벌림 때 움직입니다.
- PIP와 DIP 모터 8개는 손바닥에 고정합니다. PIP/DIP는 독립 제어를 유지하며 16축 연동은 적용하지 않습니다.
- 엄지의 CMC 두 축은 유지합니다. MCP/IP 모터 두 개는 CMC 뒤의 움직이는 받침대에 배치하므로 엄지 뿌리 관성에 포함됩니다.
- 손목 장착 부품을 원본 STEP에서 가져왔습니다. 팔·전완 모터 설계를 새로 검증한 것은 아닙니다.

기존 원본의 `thumb_spread / thumb_mcp / thumb_pip / thumb_dip`는 새 계약의 `thumb_base_sweep / thumb_opposition / thumb_mcp / thumb_ip`에 대응합니다. CAD 부품 이름에는 원본 이름이 남아 있으므로 `body` 속성을 기준으로 질량을 합산합니다.

## 실제 형상과 개념 표시의 경계

새 마디는 두께 2 mm의 속이 빈 쉘, 두 갈래 지지대와 그 안의 중앙 축받이, 금속 핀, 반경 8 mm 풀리로 생성합니다. 뿌리에는 기존 H101의 실제 체결면을 남겼습니다. 모터에는 양방향 감김용 스풀과 고정판을 배치합니다. 제조사 모터의 정확한 STEP을 사용하며 원본은 수정하지 않습니다.

보라색 두 줄은 굽힘·폄 케이블의 **경로 공간 표시**입니다. 감김, 접선, 베어링, 고정단, 장력 조절, 벌림축 보상까지 성립하는 완성 전동계가 아닙니다. 줄은 질량과 정밀 충돌 검사에서 제외합니다. 원위 관절 센서, 하네스, 새로운 체결 나사와 장력 조절 부품도 아직 상세화하지 않았습니다.

**현재 제작을 막는 항목:** 엄지 MCP의 운동축에는 기존 35° 기울기를 유지했지만, 엄지 중수골 끝의 새 지지대는 아직 그 축 방향에 맞게 회전하지 않았습니다. 다음 수정에서는 이 지지대를 기울어진 관절축에 맞추고 다시 질량·간섭·동역학을 계산해야 합니다. 중립 자세에서 보고된 다른 새 부품의 겹침도 미해결이며, 유효한 솔리드라는 결과만으로 조립 가능하다고 판단하지 않습니다.

시뮬레이션의 일정 반경 근사에서는 PIP 모터 회전이 `MCP + PIP`, DIP 모터 회전이 `MCP + PIP + DIP`에 비례합니다. 이 때문에 여러 마디가 동시에 굽으면 모터 속도가 관절 하나의 속도보다 커집니다. 1:1 반경은 시작 후보이며 목표를 충족한다는 뜻이 아닙니다. 실제 3차원 줄 경로로 검증되지 않은 벌림 계수 0 가정도 계약에 명시했습니다.

## 재생성

Python 3.12와 [고정된 CAD 패키지](requirements-cad.txt)를 사용합니다. FreeCAD GUI 없이 CadQuery/OCC로 STEP을 만듭니다. Windows에서는 저장소 작업 규칙에 따라 `pythonw.exe` 숨김 프로세스와 로그를 사용합니다.

```text
generate.py --deps <workspace-local-packages> --output <outside-checkout-output-directory>
```

`--output`을 생략하면 이 후보 폴더에 생성합니다. 원본 V9 파일은 읽기만 합니다. 입력 해시는 `generated/source_hashes.json`, 생성기·설계 해시는 `generated/build_summary.json`에 기록합니다. STEP 생성시각에 따른 바이트 차이는 물리 형상·질량·관성 재현과 구분합니다.

검토는 중립 자세에서만 수행합니다. 모든 연속 자세의 간섭, 줄 이탈, 공구 접근, 공차와 반복 구동 검증은 남아 있습니다. 새 부품은 동일 몸체끼리도 배치 충돌 목록에 포함합니다. 원본 부품끼리의 기존 접촉과 명시한 동일 모터의 회전자 결합만 제외합니다. 보고된 겹침은 조립 검토에서 각각 해소하거나 실제 체결 접촉임을 입증해야 합니다.
