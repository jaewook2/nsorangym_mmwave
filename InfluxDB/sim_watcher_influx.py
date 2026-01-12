#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
from typing import Dict, List, Set, Tuple
from watchdog.events import PatternMatchingEventHandler
from watchdog.observers import Observer
import threading
import re
import os
import time

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

lock = threading.Lock()


class SimWatcher(PatternMatchingEventHandler):
    # 감시 파일 패턴
    patterns = ['cu-up-cell-*.txt', 'cu-cp-cell-*.txt', "du-cell-*.txt", 'ue_trace.txt']

    # KPM 저장
    kpm_map: Dict[Tuple[float, int, int], List] = {}

    def __init__(self, directory: str):
        super().__init__(
            patterns=self.patterns,
            ignore_patterns=[],
            ignore_directories=True,
            case_sensitive=False
        )
        self.directory = directory
        self.consumed_keys: Set[Tuple[float, int, int]] = set()

        # ========== InfluxDB 2.x ENV ==========
        self.influx_url = os.getenv("INFLUX_URL", "http://influxdb:8086")
        self.influx_token = os.getenv("INFLUX_TOKEN", "mytoken")
        self.influx_org = os.getenv("INFLUX_ORG", "oran")
        self.influx_bucket = os.getenv("INFLUX_BUCKET", "ns3")

        # InfluxDB 2.x 클라이언트
        self.client = InfluxDBClient(
            url=self.influx_url,
            token=self.influx_token,
            org=self.influx_org
        )
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

        print(" Start Watchdog (InfluxDB 2.x)...")
        print(f" - Watch directory: {directory}")
        print(f" - Influx URL: {self.influx_url}")
        print(f" - Org: {self.influx_org}, Bucket: {self.influx_bucket}")

    def on_modified(self, event):
        super().on_modified(event)

        lock.acquire()
        try:
            with open(event.src_path, 'r') as file:
                fname = os.path.basename(file.name)

                # ---------------------------
                # UE 위치 파일
                # ---------------------------
                if fname == 'ue_trace.txt':
                    reader = csv.DictReader(file)
                    self._send_positions_to_influx(reader)
                    return

                # ---------------------------
                # KPM 로그 파일
                # ---------------------------
                reader = csv.DictReader(file)
                for row in reader:
                    if not row:
                        continue

                    try:
                        timestamp = float(row['timestamp'])
                        ue_imsi = int(row['ueImsiComplete'])
                        ue = str(row['ueImsiComplete']).strip()
                    except Exception:
                        continue

                    # 파일명에서 cell id 파싱
                    # 예: cu-up-cell-2.txt → 2
                    Fnames = file.name.replace('.txt', '').split('-')
                    cellid = Fnames[-1]

                    # file_type 결정 (기존 로직 그대로)
                    if re.search('cu-up-cell-[2-8].txt', file.name):
                        key = (timestamp, ue_imsi, 0)
                    elif re.search('cu-cp-cell-[2-8].txt', file.name):
                        key = (timestamp, ue_imsi, 1)
                    elif re.search('du-cell-[2-8].txt', file.name):
                        key = (timestamp, ue_imsi, 2)
                    elif re.search('cu-up-cell-1.txt', file.name):
                        key = (timestamp, ue_imsi, 3)
                    elif re.search('cu-cp-cell-1.txt', file.name):
                        key = (timestamp, ue_imsi, 4)
                    else:
                        continue

                    # 중복 방지
                    if key in self.consumed_keys:
                        continue

                    if key not in self.kpm_map:
                        self.kpm_map[key] = []

                    fields = []
                    values = []

                    # fieldnames 처리
                    for column_name in reader.fieldnames:
                        if column_name not in row:
                            continue
                        if row[column_name] == '':
                            continue
                        try:
                            values.append(float(row[column_name]))
                            fields.append(column_name)
                        except Exception:
                            continue

                    # file id number
                    regex = re.search(r"\w*-(\d+)\.txt", file.name)
                    file_id_number = regex.group(1) if regex else "0"

                    self.consumed_keys.add(key)

                    print("Write received KPM data to InfluxDB 2.x")
                    self._send_kpm_to_influxdb(
                        ue=ue,
                        serv_cellid=str(cellid),
                        timestamp_s=timestamp,
                        values=values,
                        fields=fields,
                        file_type=key[2],
                        file_id_number=file_id_number
                    )

        finally:
            lock.release()

    # ---------------------------
    # KPM Write (InfluxDB 2.x)
    # ---------------------------
    def _send_kpm_to_influxdb(
        self,
        ue: str,
        serv_cellid: str,
        timestamp_s: float,
        values: List[float],
        fields: List[str],
        file_type: int,
        file_id_number: str
    ):
        # InfluxDB 2.x: nanoseconds
        #timestamp_ns = int(timestamp_s * 1e9)
        timestamp_ns = int(time.time() * 1e9)
        
        points: List[Point] = []

        i = 0
        cellId = "0"

        for field in fields:
            stat = field

            # convert pdcp_latency
            if field == 'DRB.PdcpSduDelayDl.UEID (pdcpLatency)':
                values[i] = values[i] * pow(10, -1)

            servecell = False

            # cellId tracking
            if 'L3' in field and 'cellId' in field:
                cellId = str(int(values[i]))

            if 'L3 serving Id(m_cellId)' in field:
                stat = 'Serv_Cellid_ue_'

            elif 'L3 serving SINR' in field and '3gpp' not in field:
                stat = 'SINR_cell_' + str(int(cellId))
                stat_serv = 'Serv_SINR_cell_' + str(int(cellId))
                servecell = True

            elif 'L3 neigh SINR' in field and '3gpp' not in field:
                stat = 'SINR_cell_' + str(int(cellId))

            elif 'L3 serving SINR 3gpp' in field:
                stat = '3GPP_SINR_cell_' + str(int(cellId))
                stat_serv = '3GPP_Serv_SINR_cell_' + str(int(cellId))
                servecell = True

            elif 'L3 neigh SINR 3gpp' in field:
                stat = '3GPP_SINR_cell_' + str(int(cellId))

            # -------------------------
            # Cell trace (UEID / L3 제외)
            # -------------------------
            if 'UEID' not in field and 'L3' not in field:
                stat = (field + '_cell_' + serv_cellid).replace(' ', '')
                p = (
                    Point(stat)
                    .tag("file_type", str(file_type))
                    .tag("file_id_number", str(file_id_number))
                    .field("value", float(values[i]))
                    .field("sim_t", timestamp_s)                 # 시뮬레이션 시간(초)도 같이 저장(추천)
                    .time(timestamp_ns)                )
                points.append(p)
                i += 1
                continue

            # -------------------------
            # UE trace
            # -------------------------
            stat = (stat + '_ue_' + ue).replace(' ', '')

            if file_type in (0, 3):
                stat += '_up'
            elif file_type in (1, 4):
                stat += '_cp'
            elif file_type == 2:
                stat += '_du'

            p = (
                Point(stat)
                .tag("serv_cellid", str(serv_cellid))
                .tag("file_type", str(file_type))
                .tag("file_id_number", str(file_id_number))
                .field("value", float(values[i]))
                .field("sim_t", timestamp_s)                 # 시뮬레이션 시간(초)도 같이 저장(추천)
                .time(timestamp_ns)
            )
            points.append(p)

            if servecell:
                stat_serv = (stat_serv + '_ue_' + ue).replace(' ', '')
                if file_type in (0, 3):
                    stat_serv += '_up'
                elif file_type in (1, 4):
                    stat_serv += '_cp'
                elif file_type == 2:
                    stat_serv += '_du'

                p2 = (
                    Point(stat_serv)
                    .tag("serv_cellid", str(serv_cellid))
                    .tag("file_type", str(file_type))
                    .tag("file_id_number", str(file_id_number))
                    .field("value", float(values[i]))
                    .field("sim_t", timestamp_s)                 # 시뮬레이션 시간(초)도 같이 저장(추천)
                    .time(timestamp_ns)
                )
                points.append(p2)

            i += 1

        # write
        if points:
            self.write_api.write(bucket=self.influx_bucket, org=self.influx_org, record=points)

    # ---------------------------
    # UE Position Write (InfluxDB 2.x)
    # ---------------------------
    def _send_positions_to_influx(self, reader: csv.DictReader):
        points: List[Point] = []

        for row in reader:
            # 헤더는: timestamp, ueImsiComplete, position_x, position_y
            if not row.get('timestamp') or not row.get('ueImsiComplete'):
                continue
            try:
                timestamp_s = float(row['timestamp'])
                ue = str(row['ueImsiComplete']).strip()
                x = float(row['position_x'])
                y = float(row['position_y'])
            except Exception:
                continue

            #timestamp_ns = int(timestamp_s * 1e9)
            timestamp_ns = int(time.time() * 1e9)

            p = (
                Point("UE_Position")
                .tag("ue", ue)
                .field("x", x)
                .field("y", y)
                .field("sim_t", timestamp_s)                 # 시뮬레이션 시간(초)도 같이 저장(추천)
                .time(timestamp_ns)
            )
            points.append(p)

        if points:
            self.write_api.write(bucket=self.influx_bucket, org=self.influx_org, record=points)
            print(f"Wrote {len(points)} UE positions to InfluxDB 2.x")


if __name__ == "__main__":
    directory = os.getenv("WATCH_DIR", "/workspace/ns3-mmwave-oran/")
    event_handler = SimWatcher(directory)

    observer = Observer()
    observer.schedule(event_handler, directory, recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()

    observer.join()
