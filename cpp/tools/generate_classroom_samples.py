"""Regenerate the classroom page from edu and verified ctrl C++ examples."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

__all__: list[str] = []

ROOT = Path(__file__).resolve().parent.parent
PAGE = ROOT / "examples" / "classroom.html"
BEGIN = "// BEGIN GENERATED SAMPLES"
END = "// END GENERATED SAMPLES"

# Only samples confirmed by the operator are included. Distances are the
# original conservative clearance requirements; they are not measured paths.
CARDS: tuple[tuple[str, str, str, str], ...] = (
    ("卡 1：走正方形", "square", "后方和左侧各 1.5 米，前方和右侧各 0.5 米。", ""),
    ("卡 2：十字走位", "cross", "四周各 1 米。", ""),
    ("卡 4：能歌善舞", "singAndDance", "四周各 0.5 米。", ""),
    ("卡 13：8 字形运动", "figureEight", "前后各 0.7 米、左右各 0.9 米。", ""),
    (
        "卡 16：云台角度控制",
        "motionControl",
        "四周各 0.5 米及上方无障碍。",
        "30 10\n-30 0\n0 0\n\n",
    ),
    ("卡 17：弹钢琴", "piano", "不移动机器人。", ""),
    ("卡 18：算数学题", "arithmetic", "不移动机器人。", ""),
    ("卡 19：向左转", "turnLeft", "四周各 0.5 米。", ""),
    ("卡 25：S 形倒退", "sReverse", "后方 1 米、左右各 0.6 米、前方 0.5 米。", ""),
    ("卡 28：标签演奏会", "markerConcert", "准备官方数字 1、2 标签；机器人不移动。", ""),
    ("卡 29：加速前进", "speedUp", "前方 5 米平整直道，其他方向各 0.5 米。", ""),
)
PRESETS: tuple[tuple[str, str, str, bool], ...] = (
    ("好奇哨兵", "curiousSentry", "四周各 0.5 米及云台上方无障碍。", True),
    ("萤火虫花园", "fireflyGarden", "不移动机器人。", False),
    ("招财猫营业中", "luckyCat", "四周各 0.5 米及云台上方无障碍。", False),
    ("月光华尔兹", "moonlightWaltz", "右侧 0.7 米、其他方向各 0.5 米。", False),
    ("点球守门员", "penaltyKeeper", "左右各 0.6 米、前后各 0.5 米。", False),
    ("环游小火车", "stationTrain", "前方和右侧各 0.7 米、后方和左侧各 0.5 米。", False),
    ("深夜铁板烧", "streetChef", "不移动机器人。", False),
    (
        "周五狂欢舞（无音轨）",
        "fridayDisco",
        "约 2 米×2 米空地；左侧至少 1 米，快速自转，远离人和易碎物。",
        True,
    ),
)


def function(source: str, name: str) -> str:
    match = re.search(
        r"^void " + re.escape(name) + r"\(RobotCtrl& rm\) \{.*?^\}", source, re.M | re.S
    )
    if not match:
        raise ValueError(f"missing example: {name}")
    return match.group(0)


def wrap(
    name: str, clearance: str, prefix: str, body: str, recenter: bool = False, lights: bool = False
) -> str:
    description = (
        f"// {name}\n// 场地：{clearance}\n// 运行前关闭其他控制端；人须在旁，随时可断电。\n"
    )
    finish = "rm.led_ctrl.turn_off(rm_define::armor_all);" if lights else ""
    if recenter:
        finish += " rm.gimbal_ctrl.recenter();"
    invoke = f"        {body}(rm);"
    if finish:
        invoke = (
            "        try {\n"
            + invoke
            + "\n        } catch (...) {\n"
            + f"            try {{ {finish} }} catch (...) {{}}\n            throw;\n        }}\n        {finish}"
        )
    return (
        description
        + prefix
        + f"\n\nint main() {{\n    try {{\n        RobotCtrl rm;\n{invoke}\n"
        + '    } catch (const std::exception& e) {\n        std::fprintf(stderr, "error: %s\\n", e.what());\n'
        + "        return 1;\n    }\n    return 0;\n}\n"
    )


def examples() -> list[dict[str, str]]:
    cards = (ROOT / "examples" / "challenges.cpp").read_text(encoding="utf-8")
    presets = (ROOT / "examples" / "presets.cpp").read_text(encoding="utf-8")
    card_prefix = cards[: cards.index("namespace {")].replace(
        '"hanppie/hanppie.hpp"', '"hanppie.h"'
    )
    constants = cards[cards.index("const std::uint32_t SCALE") : cards.index("// 1. Walk")]
    preset_prefix = presets[: presets.index("namespace {")].replace(
        '"hanppie/hanppie.hpp"', '"hanppie.h"'
    )
    helpers = presets[presets.index("const double BEAT") : presets.index("// 拆弹倒计时")]
    sound = presets[presets.index("class Soundtrack {") : presets.index("\nstruct Note {")]
    samples = [
        {
            "name": "求和（读取输入）",
            "input": "3\n10 20 30\n",
            "code": "#include <bits/stdc++.h>\nusing namespace std;\n\nint main() {\n    int n;\n    cin >> n;\n    long long sum = 0;\n    for (int i = 0; i < n; ++i) { int x; cin >> x; sum += x; }\n    cout << sum << endl;\n}\n",
        }
    ]
    for title, filename in (
        ("前进、转弯（edu 入门接口）", "forward_turn.cpp"),
        ("走正方形（edu 入门接口）", "square.cpp"),
    ):
        source = (ROOT / "examples" / "edu" / filename).read_text(encoding="utf-8")
        samples.append(
            {
                "name": title,
                "input": "",
                "code": "// 已完成实机演示验证；场地和操作要求见下方注释。\n"
                "// 转向会在停车后确认姿态稳定，未能到位时结束程序并返回失败。\n"
                "// edu 按开局朝向累计转向目标，下一次转向会补偿前进时的偏航。\n"
                + source.replace('"hanppie/hanppie.hpp"', '"hanppie.h"'),
            }
        )
    drive = (
        (ROOT / "examples" / "drive.cpp")
        .read_text(encoding="utf-8")
        .replace('"hanppie/hanppie.hpp"', '"hanppie.h"')
    )
    samples.append(
        {
            "name": "基础驱动：前进",
            "input": "",
            "code": "// 前方 0.7 米，其他方向各 0.5 米；人在旁可断电。\n" + drive,
        }
    )
    for title, fn, clearance, stdin in CARDS:
        samples.append(
            {
                "name": title,
                "input": stdin,
                "code": wrap(
                    title, clearance, card_prefix + constants + "\n" + function(cards, fn), fn
                ),
            }
        )
    for title, fn, clearance, recenter in PRESETS:
        extra = "\n" + sound if fn == "fridayDisco" else ""
        samples.append(
            {
                "name": title,
                "input": "",
                "code": wrap(
                    title,
                    clearance,
                    preset_prefix
                    + "namespace {\n"
                    + helpers
                    + extra
                    + "\n"
                    + function(presets, fn)
                    + "\n}  // namespace",
                    fn,
                    recenter=recenter,
                    lights=True,
                ),
            }
        )
    return samples


def main() -> int:
    page = PAGE.read_text(encoding="utf-8")
    generated = (
        BEGIN
        + "\nconst SAMPLES = "
        + json.dumps(examples(), ensure_ascii=False, indent=2)
        + ";\n"
        + END
    )
    updated = page[: page.index(BEGIN)] + generated + page[page.index(END) + len(END) :]
    if "--check" in sys.argv:
        if updated != page:
            print("classroom samples are stale", file=sys.stderr)
            return 1
    else:
        PAGE.write_text(updated, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
