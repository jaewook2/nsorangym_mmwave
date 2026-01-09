#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, re, sys
from typing import Dict, Any, List
from influxdb import InfluxDBClient
import ollama  # pip install ollama

# ========= InfluxDB 1.x 연결 정보 =========
INFLUX_HOST = os.getenv("INFLUX_HOST", "localhost")
INFLUX_PORT = int(os.getenv("INFLUX_PORT", "8086"))
INFLUX_USER = os.getenv("INFLUX_USER", "admin")
INFLUX_PASS = os.getenv("INFLUX_PASS", "admin")
INFLUX_DB   = os.getenv("INFLUX_DB",   "influx")

MEAS = "UE_Position"  # 측정명

client = InfluxDBClient(host=INFLUX_HOST, port=INFLUX_PORT,
                        username=INFLUX_USER, password=INFLUX_PASS,
                        database=INFLUX_DB)

# ========= Influx 실행 유틸 =========
def run_influx(query: str):
    # 안전 로그
    print(f"\n[InfluxQL] {query}")
    res = client.query(query)
    # 결과를 표준 파이썬 객체로
    tables = []
    if hasattr(res, 'raw') and res.raw and 'series' in (res.raw or {}):
        for s in res.raw['series']:
            columns = s.get('columns', [])
            values = s.get('values', [])
            name   = s.get('name', '')
            tags   = s.get('tags', {})
            rows = [dict(zip(columns, row)) for row in values]
            tables.append({"measurement": name, "tags": tags, "rows": rows})
    return tables

# ========= 툴 정의 =========
def latest_position(ue: str):
    q = f'SELECT LAST(x) AS x, LAST(y) AS y FROM {MEAS} WHERE "ue"=\'{ue}\''
    return run_influx(q)

def position_history(ue: str, limit: int = 20):
    q = f'SELECT x,y FROM {MEAS} WHERE "ue"=\'{ue}\' ORDER BY time DESC LIMIT {int(limit)}'
    return run_influx(q)

def latest_all():
    q = f'SELECT LAST(x) AS x, LAST(y) AS y FROM {MEAS} GROUP BY "ue"'
    return run_influx(q)

def near_xy(x: float, y: float, radius: float = 200.0):
    # 최신 좌표를 전부 가져와서 파이썬에서 거리 계산
    tables = latest_all()
    out = []
    for series in tables:
        ue = series.get("tags", {}).get("ue")
        for row in series["rows"]:
            px, py = float(row.get("last", row.get("x", 0))), float(row.get("last_1", row.get("y", 0)))
            # 위 SELECT LAST(x) AS x, LAST(y) AS y 로 가져오는 경우 열 이름은 x,y가 됨
            px = float(row.get("x", px))
            py = float(row.get("y", py))
            dist = ((px - x)**2 + (py - y)**2)**0.5
            if dist <= radius:
                out.append({"ue": ue, "x": px, "y": py, "dist": dist})
    # dist 오름차순
    out.sort(key=lambda r: r["dist"])
    return [{"measurement":"UE_Position", "tags":{}, "rows":out}]

TOOLS = {
    "latest_position": latest_position,   # args: { "ue": "9" }
    "position_history": position_history, # args: { "ue": "9", "limit": 20 }
    "latest_all": latest_all,             # args: {}
    "near_xy": near_xy,                   # args: { "x": 1000.0, "y": 2000.0, "radius": 300.0 }
}

# ========= 시스템 프롬프트 (Llama에게: JSON만 출력해서 툴 선택) =========
SYSTEM_PROMPT = """당신은 InfluxDB 도우미입니다.
다음 툴 중 하나만 선택하여 JSON으로 출력하세요. 자연어 문장은 출력하지 마세요.

- latest_position: 특정 UE의 최신 x,y 좌표. args: {"ue": "<string>"}
- position_history: 특정 UE의 최근 좌표 히스토리. args: {"ue":"<string>", "limit": <int>}
- latest_all: 모든 UE의 최신 좌표. args: {}
- near_xy: 주어진 (x,y)에서 radius 이내의 UE들. args: {"x": <float>, "y": <float>, "radius": <float>}

출력 형식(반드시 이 형태, 키 순서 상관없음):
{"tool": "<tool_name>", "args": { ... }}

예시1) {"tool":"latest_position","args":{"ue":"9"}}
예시2) {"tool":"position_history","args":{"ue":"8","limit":10}}
예시3) {"tool":"latest_all","args":{}}
예시4) {"tool":"near_xy","args":{"x":2000,"y":2000,"radius":500}}
"""

def decide_tool(user_msg: str) -> Dict[str, Any]:
    rsp = ollama.chat(
       # model=os.getenv("LLAMA_MODEL", "llama3:8b"),
        model=os.getenv("LLAMA_MODEL", "llama3.1:8b-instruct-q4_0"),
        messages=[
            {"role":"system", "content": SYSTEM_PROMPT},
            {"role":"user",   "content": user_msg},
        ],
        options={"temperature": 0.1}
    )
    txt = rsp["message"]["content"].strip()
    # 모델이 장난치지 않도록 JSON만 파싱
    m = re.search(r"\{.*\}", txt, re.DOTALL)
    if not m:
        raise ValueError(f"Llama 응답에서 JSON을 찾지 못함: {txt}")
    obj = json.loads(m.group(0))
    tool = obj.get("tool")
    args = obj.get("args", {})
    if tool not in TOOLS:
        raise ValueError(f"알 수 없는 tool: {tool}")
    return {"tool": tool, "args": args}

def pretty_print_tables(tables: List[Dict[str,Any]]):
    if not tables:
        print("\n(결과 없음)")
        return
    for s in tables:
        meas = s.get("measurement", "")
        tags = s.get("tags", {})
        rows = s.get("rows", [])
        if tags:
            print(f"\n[{meas}] tags={tags}")
        else:
            print(f"\n[{meas}]")
        if not rows:
            print("(empty)")
            continue
        # 컬럼 유추
        cols = sorted({k for r in rows for k in r.keys()})
        print(" | ".join(cols))
        for r in rows[:50]:  # 너무 길면 50행까지만
            print(" | ".join(str(r.get(c,"")) for c in cols))

def main():
    print("Llama × InfluxDB (UE_Position). 한글로 물어보세요. 예:")
    print("- 'UE 9 최신 좌표 알려줘'")
    print("- 'UE 8 최근 10개 좌표'"
          "\n- '모든 UE 최신 위치'"
          "\n- 'x=2500,y=2000 반경 500 안에 누가 있어?'")
    print("종료: Ctrl+C\n")

    while True:
        try:
            q = input("Q> ").strip()
            if not q:
                continue
            plan = decide_tool(q)
            tool = plan["tool"]; args = plan["args"]
            print(f"\n[Plan] {tool} {args}")
            result = TOOLS[tool](**args)
            pretty_print_tables(result)
        except KeyboardInterrupt:
            print("\n종료")
            break
        except Exception as e:
            print(f"\n[오류] {e}")

if __name__ == "__main__":
    main()
