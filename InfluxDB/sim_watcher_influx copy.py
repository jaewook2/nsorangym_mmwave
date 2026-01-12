import csv
from typing import Dict, List, Set, Tuple
import numpy as np
from watchdog.events import PatternMatchingEventHandler
from watchdog.observers import Observer
import threading
import re
import os
import time
from influxdb import InfluxDBClient

lock = threading.Lock()


class SimWatcher(PatternMatchingEventHandler):
    patterns = ['cu-up-cell-*.txt', 'cu-cp-cell-*.txt', "du-cell-*.txt",  'ue_positions.txt']
    kpm_map: Dict[Tuple[int, int, int], List] = {}
    consumed_keys: Set[Tuple[int, int, int]]
    influx_host = "localhost"
    influx_port = 8086
    influx_user = 'admin'
    influx_password = 'admin'
    db_name = 'influx'

    client = InfluxDBClient(
        host=influx_host,
        port=influx_port,
        username=influx_user,
        password=influx_password,
        database=db_name)

    client.create_database(db_name)

    def __init__(self, directory):
        PatternMatchingEventHandler.__init__(self, patterns=self.patterns,
                                             ignore_patterns=[],
                                             ignore_directories=True, case_sensitive=False)
        self.directory = directory
        self.consumed_keys = set()
        print("Start Watchdog....")

    def on_created(self, event):
        super().on_created(event)

    def on_modified(self, event):
        super().on_modified(event)

        lock.acquire()
        try:
            with open(event.src_path, 'r') as file:
                if os.path.basename(file.name) == 'ue_positions.txt':
                    reader = csv.DictReader(file)
                    self._send_positions_to_influx(reader)
                    return
                
                
                reader = csv.DictReader(file)
                for row in reader:
                    timestamp = float(row['timestamp'])
                    ue_imsi = int(row['ueImsiComplete'])
                    ue = row['ueImsiComplete']
                    Fnames = file.name.replace('.txt', '')
                    Fnames = Fnames.split('-')
                    cellid = Fnames[-1]

                    if re.search('cu-up-cell-[2-8].txt', file.name):
                        key = (timestamp, ue_imsi, 0)
                    if re.search('cu-cp-cell-[2-8].txt', file.name):
                        key = (timestamp, ue_imsi, 1)
                    if re.search('du-cell-[2-8].txt', file.name):
                        key = (timestamp, ue_imsi, 2)
                    if re.search('cu-up-cell-1.txt', file.name):
                        key = (timestamp, ue_imsi, 3)  # to see data for eNB cell
                    if re.search('cu-cp-cell-1.txt', file.name):
                        key = (timestamp, ue_imsi, 4)  # same here

                    if key not in self.consumed_keys:
                        if key not in self.kpm_map:
                            self.kpm_map[key] = []

                        fields = list()

                        for column_name in reader.fieldnames:
                            if row[column_name] == '':
                                continue
                            self.kpm_map[key].append(float(row[column_name]))
                            fields.append(column_name) # column_name : insert

                        regex = re.search(r"\w*-(\d+)\.txt", file.name)
                        fields.append('file_id_number')
                        self.kpm_map[key].append(regex.group(1))  # last item of list will be file_id_number

                        self.consumed_keys.add(key)
                        print("Write the recived data at xAPP to Influx DB")
                        self._send_to_influxDB(ue=ue, serv_cellid = cellid, values=self.kpm_map[key], fields=fields, file_type=key[2])
        finally:
            lock.release()

    def on_closed(self, event):
        super().on_closed(event)

    def _send_to_influxDB(self, ue: int, serv_cellid: int, values: List, fields: List, file_type: int):
        # convert timestamp in nanoseconds (InfluxDB)
        timestamp = int(values[0] * (pow(10, 6)))

        i = 0
        influx_points = []
        cellId = '0'
        # 한줄씩 처리 ==> UE_ID,

        for field in fields:
            stat = field # field Name
            if field == 'file_id_number':
                continue

            # convert pdcp_latency
            if field == 'DRB.PdcpSduDelayDl.UEID (pdcpLatency)':
                values[i] = values[i] * pow(10, -1)

            # UETrace
            # SINR 처리 : SINR_cell_a_ue_b, Serv_SINR_cell_a_ue_b
            ### L3 serving SINR,L3 neigh SINR #
            # 3GPP-SINR 처리 : 3GPP_SINR_cell_a_ue_b, Serv_3GPP_SINR_cell_a_ue_b
            ### L3 serving SINR 3gpp ,L3 neigh SINR 3gpp # (convertedSinr)
            # Serving Cell ID UE :Serv_Cellid_ue_b
            ### L3 serving Id(m_cellId)

            # Cell Trace
            # numActive UEs
            #
            servecell = False
            if 'L3' in field and 'cellId' in field:
                cellId = values[i]

            if 'L3 serving Id(m_cellId)' in field:
                stat = 'Serv_Cellid_ue_'

            elif 'L3 serving SINR' in field and '3gpp' not in field:
                #stat = stat + '_cell_' + str(int(cellId))
                stat = 'SINR_cell_' + str(int(cellId))
                stat_serv = 'Serv_SINR_cell_' + str(int(cellId))
                servecell = True

            elif 'L3 neigh SINR' in field and '3gpp' not in field:
                stat = 'SINR_cell_' + str(int(cellId))

            elif 'L3 serving SINR 3gpp' in field:
                # stat = stat + '_cell_' + str(int(cellId))
                stat = '3GPP_SINR_cell_' + str(int(cellId))
                stat_serv = '3GPP_Serv_SINR_cell_' + str(int(cellId))
                servecell = True

            elif 'L3 neigh SINR 3gpp' in field:
                stat = '3GPP_SINR_cell_' + str(int(cellId))

            if 'UEID' not in field and 'L3' not in field:
                # Cell num
                stat = field + '_cell_' + serv_cellid
                stat = stat.replace(' ', '')
                influx_point = {
                    "measurement": stat,
                    "tags": {
                        'timestamp': timestamp
                    },
                    "fields": {
                        "value": values[i]
                    }
                }
                influx_points.append(influx_point)
                i += 1
                continue

            stat = stat + '_ue_' + ue
            if file_type == 0 or file_type == 3:
                stat += '_up'
            if file_type == 1 or file_type == 4:
                stat += '_cp'
            if file_type == 2:
                stat += '_du'
            stat = stat.replace(' ', '')
            print(stat)

            influx_point = {
                "measurement": stat,
                "tags": {
                    'timestamp': timestamp
                },
                "fields": {
                    "value": values[i]
                }
            }

            influx_points.append(influx_point)
            if servecell:
                stat_serv = stat_serv + '_ue_' + ue
                if file_type == 0 or file_type == 3:
                    stat_serv += '_up'
                if file_type == 1 or file_type == 4:
                    stat_serv += '_cp'
                if file_type == 2:
                    stat_serv += '_du'
                stat_serv = stat_serv.replace(' ', '')
                influx_point = {
                    "measurement": stat_serv,
                    "tags": {
                        'timestamp': timestamp
                    },
                    "fields": {
                        "value": values[i]
                    }
                }
                influx_points.append(influx_point)

            i += 1
        # pipe.send()
        self.client.write_points(influx_points)

    def _send_positions_to_influx(self, reader: csv.DictReader):
        points = []
        for row in reader:
            # 헤더는 정확히: timestamp, ueImsiComplete, position_x, position_y
            if not row.get('timestamp') or not row.get('ueImsiComplete'):
                continue
            try:
                ts = float(row['timestamp'])           # ns-3에서 seconds로 찍었으면 float seconds
                ue = str(row['ueImsiComplete']).strip()
                x = float(row['position_x'])
                y = float(row['position_y'])
            except Exception:
                continue  # 파싱 실패한 라인 스킵

            points.append({
                "measurement": "UE_Position",
                "tags": {
                    "ue": ue
                },
                # InfluxDB 1.x: time 필드 + time_precision='n' 로 ns 단위 기록
                "time": int(ts * 1e9),
                "fields": {
                    "x": x,
                    "y": y
                }
            })

        if points:
            # ns 단위
            self.client.write_points(points, time_precision='n')
            print(f"Wrote {len(points)} UE positions to InfluxDB")
        
        
if __name__ == "__main__":
    directory = '/workspace/ns3-mmwave-oran/' ## replace with your directory to watch
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
