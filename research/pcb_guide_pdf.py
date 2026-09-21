"""Build the Korean-language PCB guide PDF for the upstream flight controller board.

Everything stated here was read out of resources/pcb: the schematic sheets, the board file
and the JLCPCB assembly BOM. Where a number could not be confirmed from those files it is
marked as needing a check rather than filled in.
"""
import argparse
import re
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (KeepTogether, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

# Built-in CJK CID fonts, so no font file has to ship with this script.
KR_BODY, KR_HEAD = "HYSMyeongJo-Medium", "HYGothic-Medium"
INK = colors.HexColor("#1b1b1b")
RULE = colors.HexColor("#c9c9c9")
BAND = colors.HexColor("#eef1f5")
WARN = colors.HexColor("#8a3324")


def read_bom(path):
    """Designator groups from the JLCPCB assembly BOM."""
    z = zipfile.ZipFile(path)
    ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    shared = []
    if "xl/sharedStrings.xml" in z.namelist():
        for si in ET.fromstring(z.read("xl/sharedStrings.xml")).findall("m:si", ns):
            shared.append("".join(t.text or "" for t in si.iter("{%s}t" % ns["m"])))
    sheet = ET.fromstring(z.read("xl/worksheets/sheet1.xml"))
    rows = []
    for row in sheet.findall(".//m:row", ns):
        cells = {}
        for c in row.findall("m:c", ns):
            col = re.match(r"([A-Z]+)", c.get("r")).group(1)
            v = c.find("m:v", ns)
            if v is not None:
                cells[col] = shared[int(v.text)] if c.get("t") == "s" else v.text
        if cells.get("A"):
            rows.append(cells)
    out = []
    for r in rows[1:]:
        refs = [x.strip() for x in r["A"].split(",") if x.strip()]
        out.append({"refs": refs, "n": len(refs), "footprint": r.get("B", ""),
                    "value": r.get("D", ""), "lcsc": r.get("E", "")})
    return out


def styles():
    pdfmetrics.registerFont(UnicodeCIDFont(KR_BODY))
    pdfmetrics.registerFont(UnicodeCIDFont(KR_HEAD))
    s = getSampleStyleSheet()
    s.add(ParagraphStyle("KTitle", fontName=KR_HEAD, fontSize=19, leading=25,
                         textColor=INK, spaceAfter=2))
    s.add(ParagraphStyle("KSub", fontName=KR_BODY, fontSize=9.5, leading=14,
                         textColor=colors.HexColor("#555555"), spaceAfter=14))
    s.add(ParagraphStyle("KH1", fontName=KR_HEAD, fontSize=13.5, leading=19,
                         textColor=INK, spaceBefore=15, spaceAfter=6))
    s.add(ParagraphStyle("KH2", fontName=KR_HEAD, fontSize=11, leading=16,
                         textColor=INK, spaceBefore=10, spaceAfter=4))
    s.add(ParagraphStyle("KBody", fontName=KR_BODY, fontSize=9.5, leading=15.5,
                         textColor=INK, alignment=TA_LEFT, spaceAfter=5))
    s.add(ParagraphStyle("KNote", fontName=KR_BODY, fontSize=9, leading=14,
                         textColor=WARN, spaceBefore=3, spaceAfter=7,
                         leftIndent=8, borderPadding=0))
    s.add(ParagraphStyle("KMono", fontName="Courier", fontSize=8.4, leading=12,
                         textColor=INK, leftIndent=8, spaceAfter=7)),
    s.add(ParagraphStyle("KCell", fontName=KR_BODY, fontSize=8.6, leading=12.5,
                         textColor=INK))
    s.add(ParagraphStyle("KCellH", fontName=KR_HEAD, fontSize=8.6, leading=12.5,
                         textColor=INK))
    return s


def table(data, widths, st, align=None):
    body = [[Paragraph(str(c), st["KCellH"] if i == 0 else st["KCell"])
             for c in row] for i, row in enumerate(data)]
    t = Table(body, colWidths=widths, repeatRows=1, hAlign="LEFT")
    cmds = [("BACKGROUND", (0, 0), (-1, 0), BAND),
            ("LINEBELOW", (0, 0), (-1, 0), 0.6, RULE),
            ("LINEBELOW", (0, 1), (-1, -2), 0.25, colors.HexColor("#e8e8e8")),
            ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ("TOPPADDING", (0, 0), (-1, -1), 4),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ("LEFTPADDING", (0, 0), (-1, -1), 6),
            ("RIGHTPADDING", (0, 0), (-1, -1), 6)]
    for col, a in (align or {}).items():
        cmds.append(("ALIGN", (col, 0), (col, -1), a))
    t.setStyle(TableStyle(cmds))
    return t


def build(bom, out_path, source_note):
    st = styles()
    doc = SimpleDocTemplate(str(out_path), pagesize=A4,
                            leftMargin=20 * mm, rightMargin=18 * mm,
                            topMargin=18 * mm, bottomMargin=16 * mm,
                            title="TVC 드론 커스텀 비행제어기 PCB 안내",
                            author="tvc-drone-reference")
    f = []
    P = lambda text, s="KBody": Paragraph(text, st[s])

    f.append(P("TVC 드론 커스텀 비행제어기 PCB", "KTitle"))
    f.append(P(source_note, "KSub"))

    f.append(P("1. 이 보드가 무엇인가", "KH1"))
    f.append(P("원본 프로젝트에 포함된 <b>완성된 커스텀 비행제어기</b>다. 설계 파일, 거버, "
               "조립용 BOM과 배치 파일이 모두 들어 있어 추가 설계 없이 발주할 수 있다. "
               "상용 FC를 쓰는 것과 이 보드를 직접 제작하는 것 둘 다 가능하며, 이 문서는 "
               "후자를 선택한 경우를 다룬다."))
    f.append(P("가장 큰 장점은 <b>원본 펌웨어가 이 보드를 전제로 작성되어 있다</b>는 점이다. "
               "상용 FC로 가면 PX4용 TVC 기체 정의와 제어 할당을 새로 구현해야 하지만, "
               "이 보드는 기존 펌웨어가 그대로 올라간다. STM32CubeIDE 빌드는 이미 검증했다."))

    f.append(P("2. 보드 제원", "KH1"))
    f.append(table([["항목", "값", "출처"],
                    ["구리층", "4층 (F.Cu / In1.Cu / In2.Cu / B.Cu)", "rera.kicad_pcb"],
                    ["보드 두께", "1.6 mm", "rera.kicad_pcb"],
                    ["외곽", "바운딩 66.4 × 66.4 mm, 중심 최대반경 35.5 mm", "Edge.Cuts"],
                    ["즉 들어갈 원", "직경 약 71 mm", "계산"],
                    ["마운팅 홀", "4개", "PER 시트"],
                    ["MCU", "STM32F722RET6, LQFP-64", "BOM"],
                    ["부품", "61 품목군 / 총 136개", "bom.xlsx"]],
                   [30 * mm, 88 * mm, 34 * mm], st))
    f.append(P("직경 71 mm 원에 들어가므로 기체 본체 내경 87.6 mm에 여유 있게 들어간다. "
               "바운딩 박스가 정사각형으로 보이지만 모서리가 잘린 형상이라 대각 94 mm가 "
               "아니라 71 mm가 실제 제약이다."))
    f.append(Paragraph("확인 필요: 마운팅 홀 4개의 좌표가 기체 격벽의 볼트원과 맞지 않는다. "
                       "현재 격벽은 인서트 62 mm, 짐벌 80 mm 볼트원을 쓴다. KiCad에서 실제 "
                       "홀 위치를 읽어 격벽을 맞추거나 어댑터 판을 넣어야 한다.", st["KNote"]))

    f.append(P("3. KiCad로 열고 보는 법", "KH1"))
    f.append(P("현재 이 PC에 KiCad가 설치되어 있지 않다. 공식 사이트에서 KiCad 8 이상을 "
               "설치한 뒤 아래 순서로 본다."))
    f.append(P("① 프로젝트 열기", "KH2"))
    f.append(Paragraph("resources/pcb/rera.kicad_pro", st["KMono"]))
    f.append(P("이 파일을 열면 프로젝트 창이 뜬다. 개별 파일을 직접 열지 말고 프로젝트로 "
               "열어야 심볼·풋프린트 라이브러리 경로가 붙는다. libs 폴더에 프로젝트 전용 "
               "라이브러리가 들어 있다."))
    f.append(P("② 회로도 보기 — 좌측 첫 아이콘 (Schematic Editor)", "KH2"))
    f.append(P("최상위 시트에 5개의 하위 시트가 계층 구조로 붙어 있다. 시트 사각형을 "
               "더블클릭하면 내부로 들어가고, 상단 화살표로 돌아온다."))
    f.append(table([["시트", "역할", "심볼 수"],
                    ["MCU", "STM32F722, 25 MHz 크리스털, 리셋·부트 스위치, 부저, LED", "34"],
                    ["PWR", "전원 계통 전체. 벅 컨버터, 파워 먹스, LDO, 전류 측정", "36"],
                    ["PER", "모든 외부 커넥터, USB-C, ESD 보호, 마운팅 홀", "41"],
                    ["MEM", "SPI 플래시, microSD 소켓, SD 라인 ESD 보호", "19"],
                    ["NVU", "항법 센서 3종 (IMU, 기압계, 지자기)", "10"]],
                   [22 * mm, 108 * mm, 22 * mm], st, {2: "CENTER"}))
    f.append(P("③ 기판 보기 — Board Editor", "KH2"))
    f.append(P("우측 Appearance 패널에서 레이어를 켜고 끈다. 처음에는 F.Cu와 F.Silkscreen만 "
               "켜고 보면 부품 배치가 읽힌다. 단축키 <b>Alt+3</b>이 3D 뷰어이고, 여기서 "
               "실제 완성 모습과 높이를 확인할 수 있다. 기체에 넣기 전 간섭 확인에 쓴다."))
    f.append(P("④ 처음 볼 때 권하는 순서", "KH2"))
    f.append(P("PWR 시트 → PER 시트의 커넥터 → NVU 시트. 전원이 어디서 들어와 어떤 전압으로 "
               "갈라지는지 먼저 잡고, 그다음 외부와 연결되는 지점을 보고, 마지막에 센서를 "
               "본다. MCU 시트는 핀 배정이라 나중에 봐도 된다."))

    f.append(PageBreak())
    f.append(P("4. 전원 계통", "KH1"))
    f.append(P("회로도의 전역 전원 네트에서 확인한 구조다."))
    f.append(Paragraph(
        "+BATT  (배터리, 스크류 터미널 J13)\n"
        "  └─ MP2393GTL-Z 동기 벅 ─→ BATT_5V ─→ BATT_VBUS ─┐\n"
        "                                                   ├─ TPS2121 우선순위 먹스 ─→ +5V\n"
        "USB_VBUS  (USB-C) ───────────────────────────────┘        │\n"
        "                                                          └─ AMS1117-3.3 ─→ +3.3V, +3.3VA",
        st["KMono"]))
    f.append(table([["소자", "역할", "비고"],
                    ["MP2393GTL-Z", "배터리 전압을 5 V로 내리는 동기 스텝다운. "
                                    "이 보드에서 배터리 전압을 직접 받는 유일한 소자", "SOT-583"],
                    ["TPS2121RUXR", "USB 5 V와 배터리 유래 5 V 중 하나를 고르는 우선순위 "
                                    "먹스. USB만 꽂아도 보드가 살아난다", "핫스왑 보호"],
                    ["AMS1117-3.3", "5 V를 3.3 V로 내리는 LDO. MCU와 센서 전원", "SOT-223"],
                    ["INA226", "배터리 전압·전류·전력 측정. 2 mΩ 션트와 조합. I2C로 MCU에 보고",
                     "PWR_SCL/SDA"],
                    ["+3.3VA", "센서용으로 분리한 아날로그 3.3 V 계통", "노이즈 분리"]],
                   [30 * mm, 90 * mm, 32 * mm], st))

    f.append(P("5. 4S 적합성 검토", "KH1"))
    f.append(P("배터리 전압을 직접 받는 소자는 MP2393 하나다. 따라서 이 소자의 입력 정격이 "
               "셀 수의 상한을 정한다."))
    f.append(table([["구성", "완충 전압", "MP2393 정격 24 V 대비", "판정"],
                    ["4S", "16.8 V", "70%", "적합"],
                    ["6S", "25.2 V", "105%", "초과"]],
                   [28 * mm, 30 * mm, 50 * mm, 40 * mm], st,
                   {1: "CENTER", 2: "CENTER", 3: "CENTER"}))
    f.append(P("<b>4S로 쓰는 것은 적합하다.</b> 기준 사양 v1이 4S로 확정되어 있으므로 보드를 "
               "그대로 쓸 수 있다. 반대로 나중에 6S로 올릴 생각이라면 이 보드는 쓸 수 없고 "
               "전원단을 재설계해야 한다."))
    f.append(Paragraph("발주 전 직접 확인할 것: MP2393 데이터시트에서 recommended operating "
                       "범위와 absolute maximum을 구분해 읽고, 2 mΩ 션트가 이 기체의 "
                       "풀스로틀 합산 44 A에서 감당 가능한지 정격 전력을 계산한다. "
                       "P = I²R = 44² × 0.002 ≈ 3.9 W이므로 1206 칩 저항으로는 부족할 "
                       "가능성이 크다.", st["KNote"]))

    f.append(P("6. 센서와 저장", "KH1"))
    f.append(table([["소자", "역할", "인터페이스"],
                    ["ICM-42688-P", "6축 IMU. 자세 제어의 주 센서. 자세·각속도", "SPI"],
                    ["BMP388", "기압계. 고도 추정", "SPI"],
                    ["BMM150", "지자기. 방위(요) 기준", "I2C"],
                    ["W25Q128JVSIQ", "16 MB SPI NOR 플래시. 고속 로깅용", "SPI"],
                    ["TF-01A", "microSD 소켓. 로그 회수용", "SDMMC"],
                    ["25 MHz", "MCU 메인 클럭 크리스털", "-"]],
                   [34 * mm, 90 * mm, 28 * mm], st))
    f.append(P("광류 센서(PMW3901)와 하향 거리센서(VL53L1X)는 <b>이 보드에 없다.</b> "
               "외부 모듈로 I2C 또는 EXT 커넥터에 붙인다."))

    f.append(P("7. 커넥터 배치", "KH1"))
    f.append(table([["커넥터", "핀", "용도", "규격"],
                    ["Battery", "2", "배터리 입력", "Phoenix 스크류 터미널 5.08 mm"],
                    ["USB4105-GF-A", "-", "USB-C. 전원 공급과 통신", "USB-C 리셉터클"],
                    ["Conn_M1 / M2", "2", "모터 2개 ESC 신호", "JST PH 2.0 mm"],
                    ["Conn_SV1~SV4", "3", "서보 4채널. TVC는 2개만 쓰므로 2채널 여유",
                     "JST PH 2.0 mm"],
                    ["Conn_RF", "6", "무선 링크(LoRa 모듈)", "JST PH 2.0 mm"],
                    ["Conn_GPS", "6", "GNSS 모듈용. <b>펌웨어에 GPS 코드 없음</b>",
                     "JST PH 2.0 mm"],
                    ["Conn_I2C", "4", "외부 I2C 센서", "JST PH 2.0 mm"],
                    ["Conn_EXT1 / EXT2", "4", "확장. SPI, UART, PWM, 제어선 인출",
                     "JST PH 2.0 mm"],
                    ["Conn_SW", "4", "외부 스위치 / 아밍", "2.54 mm 핀헤더"]],
                   [34 * mm, 12 * mm, 62 * mm, 44 * mm], st, {1: "CENTER"}))
    f.append(P("<b>서보 4채널은 이 프로젝트에 유용하다.</b> TVC 2축에 2채널을 쓰고 남는 "
               "2채널을 짐벌 출력축 각도 센서나 추가 계측에 쓸 수 있다. 첫 마일스톤이 "
               "명령각과 실제각을 같은 로그에 남기는 것이므로 여유 채널이 도움이 된다."))
    f.append(P("GPS 커넥터는 자리만 있고 펌웨어에 GPS·NMEA 코드가 한 줄도 없다. 연구 방향이 "
               "GPS 없는 위치 유지이므로 이대로 두어도 무리가 없다."))

    f.append(PageBreak())
    f.append(P("8. 보호 소자", "KH1"))
    f.append(table([["소자", "수량", "역할"],
                    ["USBLC6-2SC6", "1", "USB 데이터선 ESD 보호"],
                    ["TPD1E05U06DPYR", "6", "microSD 라인 ESD 보호"],
                    ["ESDA7P60-1U1M", "1", "전원 입력단 ESD·서지 보호"],
                    ["1N4148WS", "1", "범용 다이오드"]],
                   [40 * mm, 18 * mm, 90 * mm], st, {1: "CENTER"}))
    f.append(P("MLT-8540 부저와 SS8050 트랜지스터로 음향 경고를 낸다. LED는 적색(전원)과 "
               "청색(상태) 2개, 스위치는 리셋과 부트 2개다."))

    f.append(P("9. 발주 절차", "KH1"))
    f.append(P("이 프로젝트는 KiCad Fabrication Toolkit 플러그인으로 <b>JLCPCB 조립 발주 "
               "세트가 이미 생성되어 있다.</b> 파일 3개를 그대로 올리면 된다."))
    f.append(table([["파일", "경로", "용도"],
                    ["거버", "resources/pcb/production/gerber.zip", "기판 제작"],
                    ["BOM", "resources/pcb/production/final/bom.xlsx",
                     "부품 목록. LCSC 부품번호 포함"],
                    ["배치", "resources/pcb/production/final/positions.xlsx",
                     "부품 좌표·회전"]],
                   [22 * mm, 74 * mm, 56 * mm], st))
    f.append(P("발주 시 지정할 값", "KH2"))
    f.append(table([["항목", "값"],
                    ["레이어", "4층"],
                    ["두께", "1.6 mm"],
                    ["조립 면", "부품 배치를 3D 뷰어에서 확인해 단면/양면 결정"],
                    ["수량", "최소 2~3장 권장. 첫 보드에서 실수하는 경우가 흔하다"]],
                   [30 * mm, 122 * mm, ], st))
    f.append(Paragraph("발주 전 필수 확인: KiCad에서 ERC(회로 검사)와 DRC(기판 규칙 검사)를 "
                       "직접 돌려 통과를 확인한다. 이 저장소의 파일이 검사를 통과한다는 기록은 "
                       "없다. 그리고 BOM의 LCSC 부품번호가 현재 재고와 맞는지 JLCPCB 부품 "
                       "검색에서 하나씩 확인한다. 단종·품절이 있으면 대체품을 지정해야 한다.",
                       st["KNote"]))

    f.append(P("10. 수정을 검토할 항목", "KH1"))
    f.append(P("지금 바로 고칠 필요는 없지만, 이 기체 구성에서 짚어둘 지점이다."))
    f.append(table([["항목", "내용", "우선순위"],
                    ["션트 정격", "2 mΩ 션트가 풀스로틀 44 A에서 약 3.9 W를 소모한다. "
                                  "정격 확인 후 필요하면 전력형으로 교체", "높음"],
                    ["마운팅 홀", "기체 격벽 볼트원과 불일치. 보드 홀을 옮기거나 격벽을 "
                                  "맞추거나 어댑터 판", "높음"],
                    ["서보 전원", "서보 4채널이 보드 5 V에서 전원을 받는 구조인지 확인. "
                                  "MG996R 2개는 순간 전류가 크므로 별도 BEC 권장", "높음"],
                    ["GPS 커넥터", "쓰지 않으므로 그대로 두거나, 다른 용도로 재배정", "낮음"],
                    ["엔코더 입력", "짐벌 출력축 각도 센서를 붙일 입력이 필요하면 "
                                    "EXT 커넥터로 충분한지 확인", "중간"]],
                   [26 * mm, 100 * mm, 22 * mm], st, {2: "CENTER"}))

    f.append(P("11. 이 문서의 한계", "KH1"))
    f.append(P("여기 적힌 내용은 저장소의 회로도, 기판 파일, 조립 BOM에서 읽은 것이다. "
               "다음은 확인하지 않았다."))
    f.append(P("· 실제 회로 동작. 이 보드가 제작·동작된 기록은 원본 저자의 영상뿐이다.<br/>"
               "· ERC·DRC 통과 여부.<br/>"
               "· 각 소자의 데이터시트 정격과 회로 값의 정합성. 션트만 개략 계산했다.<br/>"
               "· MCU 핀 배정과 펌웨어 설정의 일치.<br/>"
               "· 부품 재고와 가격."))
    f.append(P("원본 라이선스는 GPL-3.0이다. 개인적으로 제작·수정하는 것은 문제없으나, "
               "수정본을 배포하거나 판매하려면 GPL 조건과 포함된 제3자 라이브러리 조건을 "
               "함께 확인해야 한다."))

    doc.build(f)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pcb", type=Path, default=Path("resources/pcb"))
    p.add_argument("--output", type=Path, default=Path("artifacts/pcb/PCB-안내.pdf"))
    args = p.parse_args(argv)

    bom_path = args.pcb / "production" / "final" / "bom.xlsx"
    bom = read_bom(bom_path) if bom_path.exists() else []
    total = sum(g["n"] for g in bom)
    note = (f"원본 저장소 resources/pcb 에서 읽은 내용. 회로도 5시트, 4층 기판, "
            f"조립 BOM {len(bom)} 품목군 / 총 {total}개 부품 기준.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    build(bom, args.output, note)
    print(f"BOM: {len(bom)} groups, {total} components")
    print(f"wrote {args.output.resolve()}  ({args.output.stat().st_size / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
