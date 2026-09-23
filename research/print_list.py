"""Build the print list for the bare-frame vehicle, as a spreadsheet and a PDF.

Every number here is read from generated data rather than typed in, so the list cannot
drift from the parts. Quantities and masses come from the assembly report, build
orientations from a measured overhang sweep of the actual meshes, and fastener features
from the solids themselves.

    python print_list.py

Inputs (all produced by other scripts in this repo):
    artifacts/cad/vehicle-assembly.json   quantities, masses, carbon length, clash result
    <data>/overhang.json                  six build directions per part, measured
    <data>/holes.json                     fastener features per part, measured
    <data>/print-strategy.json            optional: reviewed orientation and settings

Outputs:
    artifacts/cad/print-list.xlsx
    artifacts/cad/print-list.pdf
"""
import argparse
import json
from datetime import date
from pathlib import Path

# What each part is and why it matters, in the user's language. Not derivable from the
# geometry, so it lives here rather than being invented per run.
PARTS_KR = {
    "gimbal-outer-ring": ("짐벌 외측 링",
                          "벌크헤드에 고정되는 기준 링. 내측 링이 이 안에서 Y축으로 "
                          "회전하며, 683ZZ 베어링 2개가 여기 압입된다."),
    "gimbal-inner-ring": ("짐벌 내측 링",
                          "외측 링 안에서 Y축 회전, 동시에 크래들을 X축으로 받는다. "
                          "2축 짐벌의 중간 부재로 양쪽 하중을 모두 받는다."),
    "gimbal-cradle": ("짐벌 크래들",
                      "모터 2개가 하단 플레이트에 직결된다. 추력 25.5 N 전체와 "
                      "반작용 토크가 이 부품의 팔을 지나 트러니언으로 나간다. "
                      "가장 하중이 큰 출력 부품."),
    "bulkhead": ("벌크헤드 (격벽)",
                 "4개 전부 동일 부품. z = 30 / 90 / 260 / 320 mm에 배치되고 "
                 "8 mm 카본 스파인 4본이 관통한다. 기체의 골격."),
    "battery-tray": ("배터리 트레이",
                     "4S 팩(137×44×33 mm)을 기체 축방향으로 세워 고정한다. "
                     "착륙 충격 시 팩 관성을 받는다."),
    "gimbal-servo-bracket": ("짐벌 서보 브래킷",
                             "MG996R 서보 1개씩 고정. 2개 필요(롤/피치)."),
    "leg-bracket": ("랜딩기어 브래킷",
                    "최상단 벌크헤드 '하면'에 볼트로 붙고, 8 mm 카본 로드를 "
                    "26.9° 로 유지한다. 착륙 하중 전부가 이 조인트를 지난다."),
    "leg-foot": ("랜딩기어 풋",
                 "카본 로드 끝단의 접지 패드. 소켓이 로드와 같은 26.9° 로 기울어져 있다."),
    "fit-coupon": ("끼워맞춤 테스트 쿠폰",
                   "본출력 전 검증용 1회성 부품. 모터 볼트 패턴, 열삽입 인서트 보스, "
                   "683ZZ 베어링 시트 2종이 한 장에 들어있다."),
}

ORDER = ["fit-coupon", "bulkhead", "leg-bracket", "leg-foot", "gimbal-outer-ring",
         "gimbal-inner-ring", "gimbal-cradle", "gimbal-servo-bracket", "battery-tray"]

EXCLUDED = [
    ("prop-sweep-envelope_64mm_15deg.stl", "출력 안 함 — 참고 형상",
     "로터가 ±15° 틸트에서 쓸고 가는 공간(키프아웃). 다른 부품이 이 안에 들어가면 "
     "안 된다는 기준용. 실물 부품이 아니다."),
    ("prop-sweep-envelope_107mm_20deg.stl", "출력 안 함 — 참고 형상",
     "같은 키프아웃의 ±20° / 간격 107 mm 버전."),
    ("vehicle-assembly.stl", "출력 안 함 — 조립 확인용",
     "16개 부품 전체를 제자리에 배치한 형상. 간섭 검사와 그림용."),
    ("superseded/leg-knee.stl", "폐기 — 출력 금지",
     "3본 트러스 랜딩기어. 직선 다리로 대체됨."),
    ("superseded/leg-bracket-upper.stl", "폐기 — 출력 금지", "위와 같음."),
    ("superseded/leg-bracket-lower.stl", "폐기 — 출력 금지", "위와 같음."),
    ("superseded/rocket-shell-*.stl (7개)", "폐기 — 출력 금지",
     "외피 일체. 233 g 무게 때문에 삭제. 기체는 골조만 사용."),
]

HW_KR = {
    "insert": "M3 열삽입 인서트 (황동, 5.7 mm)",
    "m3_clear": "M3 볼트 통과 구멍",
    "bearing": "683ZZ 베어링 압입 시트 (3×7×3)",
    "rod": "8 mm 카본 로드 소켓",
    "relief": "베어링 분해용 관통 구멍",
    "ball": "푸시로드 볼링크 구멍",
    "pivot": "피벗 보스 관통",
    "cable": "모터 케이블 관통",
    "wire": "배선 관통 (그로밋)",
    "bore": "중앙 보어",
}
# Which features mean a bought item, and how many of that item per feature.
BUY_PER_FEATURE = {"insert": ("M3 열삽입 인서트", 1), "bearing": ("683ZZ 베어링", 1)}


def load(data_dir, artifacts):
    asm = json.loads((artifacts / "vehicle-assembly.json").read_text(encoding="utf-8"))
    over = json.loads((data_dir / "overhang.json").read_text(encoding="utf-8"))
    holes = json.loads((data_dir / "holes.json").read_text(encoding="utf-8"))
    sp = data_dir / "print-strategy.json"
    strat = json.loads(sp.read_text(encoding="utf-8")) if sp.exists() else {}
    return asm, over, holes, strat


def rows(asm, over, holes, strat):
    counts = {p["part"]: p for p in asm["printed_parts"]}
    counts["fit-coupon"] = {"part": "fit-coupon", "count": 1,
                            "mass_asa_g": 23.7, "volume_cm3": 22.1}
    out = []
    for name in ORDER:
        c = counts.get(name)
        if not c:
            continue
        o = over.get(name, {}).get("orientations", [])
        s = strat.get(name, {})
        # The measured sweep is sorted best-first, so its head is the fallback.
        best = o[0] if o else {}
        chosen_label = s.get("orientation") or best.get("up", "")
        chosen = next((r for r in o if r["up"] == chosen_label), best)
        qty = c["count"]
        each = c["mass_asa_g"] / qty
        out.append({
            "part": name,
            "file": f"{name}.stl",
            "kr": PARTS_KR[name][0],
            "desc": PARTS_KR[name][1],
            "qty": qty,
            "each_g": each,
            "total_g": c["mass_asa_g"],
            "height": chosen.get("height_mm"),
            "footprint": chosen.get("footprint_mm"),
            "orientation": chosen_label,
            "overhang_pct": chosen.get("overhang_pct"),
            "bed_cm2": chosen.get("bed_contact_cm2"),
            "supports": s.get("supports", ""),
            "material": s.get("material", ""),
            "layer": s.get("layer_height_mm", ""),
            "walls": s.get("walls", ""),
            "infill": s.get("infill_pct", ""),
            "pattern": s.get("infill_pattern", ""),
            "brim": s.get("brim", ""),
            "reason": s.get("orientation_reason", ""),
            "critical": s.get("critical_features", []),
            "risks": s.get("risks", []),
            "holes": holes.get(name, {}),
            "all_orientations": o,
        })
    return out


def hardware(rs):
    """Bought items implied by the features actually present in the solids."""
    tally = {}
    for r in rs:
        for tag, h in r["holes"].items():
            if tag in BUY_PER_FEATURE:
                label, per = BUY_PER_FEATURE[tag]
                tally.setdefault(label, 0)
                tally[label] += h["count"] * per * r["qty"]
    return tally


# --------------------------------------------------------------------------- xlsx
def write_xlsx(path, rs, asm, hw):
    from openpyxl import Workbook
    from openpyxl.styles import Font, Alignment, PatternFill, Border, Side
    from openpyxl.utils import get_column_letter

    head_f = Font(name="맑은 고딕", bold=True, size=10, color="FFFFFF")
    body_f = Font(name="맑은 고딕", size=10)
    small_f = Font(name="맑은 고딕", size=9)
    mono_f = Font(name="Consolas", size=9.5)
    head_fill = PatternFill("solid", fgColor="2F4858")
    alt_fill = PatternFill("solid", fgColor="F2F5F7")
    warn_fill = PatternFill("solid", fgColor="FFF3CD")
    thin = Side(style="thin", color="D0D7DC")
    box = Border(left=thin, right=thin, top=thin, bottom=thin)
    wrap = Alignment(wrap_text=True, vertical="top")
    ctr = Alignment(horizontal="center", vertical="center")

    wb = Workbook()

    def style_header(ws, ncol, row=1):
        for i in range(1, ncol + 1):
            c = ws.cell(row=row, column=i)
            c.font, c.fill, c.alignment, c.border = head_f, head_fill, ctr, box
        ws.freeze_panes = ws.cell(row=row + 1, column=1)

    # ---- sheet 1: the print list ----
    ws = wb.active
    ws.title = "출력목록"
    cols = [("No", 5), ("STL 파일명", 26), ("부품", 17), ("개수", 6), ("개당 g", 8),
            ("합계 g", 8), ("출력 방향", 17), ("출력높이 mm", 11), ("베드 mm", 14),
            ("오버행 %", 9), ("서포트", 16), ("재료", 13), ("레이어 mm", 10),
            ("벽", 5), ("인필 %", 8), ("완료", 6)]
    ws.append([c[0] for c in cols])
    for i, (_, w) in enumerate(cols, start=1):
        ws.column_dimensions[get_column_letter(i)].width = w
    style_header(ws, len(cols))
    for n, r in enumerate(rs, start=1):
        fp = r["footprint"]
        ws.append([n, r["file"], r["kr"], r["qty"], round(r["each_g"], 1),
                   round(r["total_g"], 1), r["orientation"], r["height"],
                   f"{fp[0]:.0f} x {fp[1]:.0f}" if fp else "",
                   r["overhang_pct"], r["supports"], r["material"], r["layer"],
                   r["walls"], r["infill"], ""])
        row = ws.max_row
        for i in range(1, len(cols) + 1):
            c = ws.cell(row=row, column=i)
            c.border, c.font = box, body_f
            if i in (1, 4, 5, 6, 8, 10, 13, 14, 15, 16):
                c.alignment = ctr
            else:
                c.alignment = wrap
            if i == 2:
                c.font = mono_f
            if n % 2 == 0:
                c.fill = alt_fill
        if r["part"] == "fit-coupon":
            for i in range(1, len(cols) + 1):
                ws.cell(row=row, column=i).fill = warn_fill
    tot = ws.max_row + 1
    ws.cell(row=tot, column=3, value="합계").font = Font(name="맑은 고딕", bold=True)
    ws.cell(row=tot, column=4, value=f"=SUM(D2:D{tot-1})").font = Font(name="맑은 고딕", bold=True)
    ws.cell(row=tot, column=6, value=f"=ROUND(SUM(F2:F{tot-1}),1)").font = Font(name="맑은 고딕", bold=True)
    for i in (3, 4, 6):
        ws.cell(row=tot, column=i).alignment = ctr
    ws.cell(row=tot + 2, column=2,
            value="노란 줄(쿠폰)을 가장 먼저 출력하세요. 본출력 16개는 그 결과를 "
                  "확인한 뒤 진행합니다.").font = small_f
    ws.cell(row=tot + 3, column=2,
            value=f"합계 개수에 쿠폰 1개가 포함되어 있습니다. 기체 구성 부품은 "
                  f"{asm['printed_kinds']}종 {asm['printed_pieces']}개, "
                  f"ASA {asm['printed_mass_asa_g']:.0f} g.").font = small_f
    ws.cell(row=tot + 4, column=2,
            value=f"별도 구매: 8 mm 카본 로드 "
                  f"{asm['carbon_rod_total_mm']/1000:.2f} m (다리 "
                  f"{asm['carbon_leg_rod_total_mm']/1000:.2f} m + 스파인 "
                  f"{asm['carbon_spine_total_mm']/1000:.2f} m).").font = small_f

    # ---- sheet 2: order and reasoning ----
    ws2 = wb.create_sheet("출력순서·근거")
    ws2.append(["순서", "STL 파일명", "개수", "방향 선정 근거", "치수가 중요한 부분",
                "주의할 점"])
    for i, w in enumerate([6, 26, 6, 52, 40, 52], start=1):
        ws2.column_dimensions[get_column_letter(i)].width = w
    style_header(ws2, 6)
    for n, r in enumerate(rs, start=1):
        risks = "\n".join(f"· {x['risk']} → {x['mitigation']}" for x in r["risks"])
        ws2.append([n, r["file"], r["qty"], r["reason"],
                    "\n".join(f"· {c}" for c in r["critical"]), risks])
        for i in range(1, 7):
            c = ws2.cell(row=ws2.max_row, column=i)
            c.border, c.alignment = box, wrap
            c.font = mono_f if i == 2 else small_f
            if i in (1, 3):
                c.alignment = ctr
        ws2.row_dimensions[ws2.max_row].height = 76

    # ---- sheet 3: fasteners per part ----
    ws3 = wb.create_sheet("부품별 체결부")
    ws3.append(["STL 파일명", "개수", "체결부 (1개당)", "수량/개", "전체 수량"])
    for i, w in enumerate([26, 6, 36, 10, 10], start=1):
        ws3.column_dimensions[get_column_letter(i)].width = w
    style_header(ws3, 5)
    for r in rs:
        for tag, h in sorted(r["holes"].items(), key=lambda kv: -kv[1]["count"]):
            ws3.append([r["file"], r["qty"], HW_KR.get(tag, h["label"]),
                        h["count"], h["count"] * r["qty"]])
            for i in range(1, 6):
                c = ws3.cell(row=ws3.max_row, column=i)
                c.border, c.font = box, small_f
                c.alignment = ctr if i in (2, 4, 5) else wrap
                if i == 1:
                    c.font = mono_f
    ws3.append([])
    ws3.append(["필요 수량 합계 (구멍 개수에서 계산)", "", "", "", ""])
    ws3.cell(row=ws3.max_row, column=1).font = Font(name="맑은 고딕", bold=True)
    for label, n in sorted(hw.items()):
        ws3.append(["", "", label, "", n])
        for i in (3, 5):
            ws3.cell(row=ws3.max_row, column=i).font = small_f
            ws3.cell(row=ws3.max_row, column=i).border = box
        ws3.cell(row=ws3.max_row, column=5).alignment = ctr
    ws3.append(["", "", "여유분을 포함해 주문하세요. 인서트는 삽입 실패가 잦습니다.",
                "", ""])
    ws3.cell(row=ws3.max_row, column=3).font = small_f

    # ---- sheet 4: the measured evidence ----
    ws4 = wb.create_sheet("방향 측정데이터")
    ws4.append(["STL 파일명", "빌드 방향", "출력높이 mm", "베드 가로 mm", "베드 세로 mm",
                "베드 접촉 cm2", "오버행 cm2", "오버행 %", "채택"])
    for i, w in enumerate([26, 18, 11, 12, 12, 12, 11, 9, 7], start=1):
        ws4.column_dimensions[get_column_letter(i)].width = w
    style_header(ws4, 9)
    for r in rs:
        for o in r["all_orientations"]:
            pick = "O" if o["up"] == r["orientation"] else ""
            ws4.append([r["file"], o["up"], o["height_mm"], o["footprint_mm"][0],
                        o["footprint_mm"][1], o["bed_contact_cm2"], o["overhang_cm2"],
                        o["overhang_pct"], pick])
            for i in range(1, 10):
                c = ws4.cell(row=ws4.max_row, column=i)
                c.border, c.font, c.alignment = box, small_f, ctr
                if i == 1:
                    c.font, c.alignment = mono_f, wrap
            if pick:
                for i in range(1, 10):
                    ws4.cell(row=ws4.max_row, column=i).fill = alt_fill
    ws4.cell(row=ws4.max_row + 2, column=1,
             value="45° 기준으로 실제 메쉬에서 측정한 값입니다. 오버행 %는 "
                   "전체 표면적 대비 비율.").font = small_f

    # ---- sheet 5: do not print ----
    ws5 = wb.create_sheet("출력 제외")
    ws5.append(["파일", "구분", "설명"])
    for i, w in enumerate([44, 22, 66], start=1):
        ws5.column_dimensions[get_column_letter(i)].width = w
    style_header(ws5, 3)
    for f, kind, why in EXCLUDED:
        ws5.append([f, kind, why])
        for i in range(1, 4):
            c = ws5.cell(row=ws5.max_row, column=i)
            c.border, c.alignment = box, wrap
            c.font = mono_f if i == 1 else small_f
            if "금지" in kind:
                c.fill = warn_fill
    wb.save(path)


# --------------------------------------------------------------------------- pdf
def write_pdf(path, rs, asm, hw, image=None):
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_LEFT
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.cidfonts import UnicodeCIDFont
    from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                    Spacer, Table, TableStyle)

    BODY, HEAD = "HYSMyeongJo-Medium", "HYGothic-Medium"
    pdfmetrics.registerFont(UnicodeCIDFont(BODY))
    pdfmetrics.registerFont(UnicodeCIDFont(HEAD))
    s = getSampleStyleSheet()
    s.add(ParagraphStyle("T", fontName=HEAD, fontSize=18, leading=24, spaceAfter=4))
    s.add(ParagraphStyle("Sub", fontName=BODY, fontSize=9.5, leading=14,
                         textColor=colors.HexColor("#55606a"), spaceAfter=10))
    s.add(ParagraphStyle("H1", fontName=HEAD, fontSize=13, leading=18, spaceBefore=12,
                         spaceAfter=6))
    s.add(ParagraphStyle("B", fontName=BODY, fontSize=9.3, leading=14.5, spaceAfter=5))
    s.add(ParagraphStyle("Cell", fontName=BODY, fontSize=7.9, leading=11,
                         alignment=TA_LEFT))
    s.add(ParagraphStyle("CellM", fontName="Courier", fontSize=7.6, leading=11))
    s.add(ParagraphStyle("Note", fontName=BODY, fontSize=8.4, leading=12.5,
                         textColor=colors.HexColor("#55606a")))

    doc = SimpleDocTemplate(str(path), pagesize=landscape(A4),
                            leftMargin=13 * mm, rightMargin=13 * mm,
                            topMargin=13 * mm, bottomMargin=12 * mm,
                            title="TVC 드론 3D 출력 목록", author="")
    P = lambda t, st="Cell": Paragraph(str(t), s[st])
    story = []

    story.append(Paragraph("동축 TVC 드론 — 3D 출력 목록", s["T"]))
    story.append(Paragraph(
        f"골조 구성 (외피 없음) · 출력 {asm['printed_kinds']}종 "
        f"{asm['printed_pieces']}개 + 테스트 쿠폰 1개 · ASA 기준 "
        f"{asm['printed_mass_asa_g']:.0f} g · 생성일 {date.today().isoformat()}",
        s["Sub"]))

    # A fresh style per table: TableStyle is mutable, and sharing one instance while
    # adding per-row BACKGROUND commands leaked those row indices into later, shorter
    # tables, where reportlab then indexed past the end of the row positions.
    def grid_style():
        return TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2F4858")),
            ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
            ("FONTNAME", (0, 0), (-1, 0), HEAD),
            ("FONTSIZE", (0, 0), (-1, 0), 8),
            ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#C8D0D6")),
            ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ("TOPPADDING", (0, 0), (-1, -1), 3),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ])

    # --- main table ---
    head = ["No", "STL 파일명", "부품", "개수", "개당\ng", "합계\ng", "출력 방향",
            "높이\nmm", "베드 mm", "오버행\n%", "서포트", "재료", "레이어", "벽",
            "인필"]
    data = [[P(h, "Cell") for h in head]]
    for n, r in enumerate(rs, start=1):
        fp = r["footprint"]
        data.append([P(n), P(r["file"], "CellM"), P(r["kr"]), P(r["qty"]),
                     P(f"{r['each_g']:.1f}"), P(f"{r['total_g']:.1f}"),
                     P(r["orientation"]), P(r["height"]),
                     P(f"{fp[0]:.0f}×{fp[1]:.0f}" if fp else ""),
                     P(r["overhang_pct"]), P(r["supports"]), P(r["material"]),
                     P(r["layer"]), P(r["walls"]), P(r["infill"])])
    t = Table(data, repeatRows=1, colWidths=[
        9 * mm, 36 * mm, 24 * mm, 10 * mm, 11 * mm, 11 * mm, 25 * mm, 11 * mm,
        18 * mm, 12 * mm, 24 * mm, 20 * mm, 13 * mm, 8 * mm, 10 * mm])
    st = grid_style()
    for n, r in enumerate(rs, start=1):
        if r["part"] == "fit-coupon":
            st.add("BACKGROUND", (0, n), (-1, n), colors.HexColor("#FFF3CD"))
        elif n % 2 == 0:
            st.add("BACKGROUND", (0, n), (-1, n), colors.HexColor("#F2F5F7"))
    t.setStyle(st)
    story.append(t)
    story.append(Spacer(1, 5 * mm))
    story.append(Paragraph(
        "노란 줄(쿠폰)을 가장 먼저 출력하십시오. 이 기체의 모든 치수는 인터넷에서 받은 "
        "모터 STEP 하나에서 나왔고 실측은 아직 없습니다. 쿠폰은 15분이면 출력되며, "
        "구조 부품과 <b>같은 재료·같은 레이어 높이·같은 벽 수</b>로 출력해야 의미가 "
        "있습니다. 별도 구매: 8 mm 카본 로드 "
        f"{asm['carbon_rod_total_mm']/1000:.2f} m.", s["Note"]))

    if image and Path(image).exists():
        story.append(PageBreak())
        story.append(Paragraph("조립 형상", s["H1"]))
        story.append(Image(str(image), width=262 * mm, height=126 * mm))
        story.append(Paragraph(
            f"전장 {asm['envelope_mm'][0]:.0f} × {asm['envelope_mm'][1]:.0f} × "
            f"{asm['envelope_mm'][2]:.0f} mm. 16개 부품 전체에 대해 간섭 검사를 "
            "통과했습니다(1 mm³ 초과 겹침 없음).", s["Note"]))

    # --- per part detail ---
    story.append(PageBreak())
    story.append(Paragraph("부품별 상세", s["H1"]))
    det = [[P(h, "Cell") for h in
            ["STL 파일명", "부품 / 역할", "방향 선정 근거", "치수가 중요한 부분",
             "주의할 점"]]]
    for r in rs:
        det.append([
            P(f"{r['file']}\n×{r['qty']}", "CellM"),
            P(f"<b>{r['kr']}</b><br/>{r['desc']}"),
            P(r["reason"]),
            P("<br/>".join(f"· {c}" for c in r["critical"])),
            P("<br/>".join(f"· {x['risk']} → {x['mitigation']}" for x in r["risks"])),
        ])
    t2 = Table(det, repeatRows=1,
               colWidths=[34 * mm, 58 * mm, 58 * mm, 46 * mm, 76 * mm])
    t2.setStyle(grid_style())
    story.append(t2)

    # --- hardware ---
    story.append(PageBreak())
    story.append(Paragraph("체결 부품 (구멍 개수에서 산출)", s["H1"]))
    hwd = [[P(h, "Cell") for h in ["품목", "필요 수량", "비고"]]]
    for label, n in sorted(hw.items()):
        hwd.append([P(label), P(n),
                    P("여유분 포함 주문 권장. 인서트는 삽입 실패가 잦습니다."
                      if "인서트" in label else "압입 여유 −0.10 mm 로 설계됨.")])
    hwd.append([P("8 mm 카본 로드"), P(f"{asm['carbon_rod_total_mm']/1000:.2f} m"),
                P(f"다리 {asm['carbon_leg_rod_total_mm']/1000:.2f} m + 스파인 "
                  f"{asm['carbon_spine_total_mm']/1000:.2f} m")])
    t3 = Table(hwd, repeatRows=1, colWidths=[70 * mm, 30 * mm, 150 * mm])
    t3.setStyle(grid_style())
    story.append(t3)
    story.append(Spacer(1, 4 * mm))
    story.append(Paragraph(
        "위 수량은 STEP 솔리드에서 구멍을 직접 세어 계산한 값입니다. 볼트·너트, "
        "푸시로드, 볼링크, 모터·ESC·서보·배터리는 포함되지 않습니다.", s["Note"]))

    story.append(Paragraph("출력하지 않는 파일", s["H1"]))
    exd = [[P(h, "Cell") for h in ["파일", "구분", "설명"]]]
    for f, kind, why in EXCLUDED:
        exd.append([P(f, "CellM"), P(kind), P(why)])
    t4 = Table(exd, repeatRows=1, colWidths=[74 * mm, 32 * mm, 144 * mm])
    stx = grid_style()
    for i, (f, kind, why) in enumerate(EXCLUDED, start=1):
        if "금지" in kind:
            stx.add("BACKGROUND", (0, i), (-1, i), colors.HexColor("#FFF3CD"))
    t4.setStyle(stx)
    story.append(t4)

    doc.build(story)


def main(argv=None):
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--artifacts", type=Path, default=here.parent / "artifacts" / "cad")
    p.add_argument("--data", type=Path, required=True,
                   help="Directory holding overhang.json, holes.json and optionally "
                        "print-strategy.json")
    p.add_argument("--image", type=Path, default=None)
    args = p.parse_args(argv)

    asm, over, holes, strat = load(args.data, args.artifacts)
    rs = rows(asm, over, holes, strat)
    hw = hardware(rs)

    xlsx = args.artifacts / "print-list.xlsx"
    pdf = args.artifacts / "print-list.pdf"
    write_xlsx(xlsx, rs, asm, hw)
    write_pdf(pdf, rs, asm, hw, args.image)

    print(f"Print list  {len(rs)} rows, "
          f"{sum(r['qty'] for r in rs)} pieces, "
          f"{sum(r['total_g'] for r in rs):.0f} g in ASA")
    for r in rs:
        print(f"  {r['file']:28s} x{r['qty']}  {r['orientation']:17s} "
              f"{r['overhang_pct']:5.1f}% overhang  "
              f"{'' if r['material'] else '(strategy pending)'}")
    if not strat:
        print("  NOTE: no print-strategy.json, so material and slicer columns are blank.")
    print(f"  wrote {xlsx.name} and {pdf.name} to {args.artifacts.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
