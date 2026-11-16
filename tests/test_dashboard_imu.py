"""Exercise UDP→CSV type3 and DWT wrap without real radio/hardware."""
import csv, socket, struct, tempfile, time
from pathlib import Path
from threading import Thread
from uav_blackbox_decoder import crc16_ccitt
from realtime_dashboard import Collector

def frame(source: bytes, index: int, cycles: int):
    b=bytearray(source)
    struct.pack_into('<H',b,16,index)
    struct.pack_into('<I',b,24,cycles)
    struct.pack_into('<H',b,86,crc16_ccitt(b[:86]))
    return bytes(b)

def main():
    with tempfile.TemporaryDirectory() as tmp:
        c=Collector('127.0.0.1',16612,Path(tmp)/'uav_test.csv')
        th=Thread(target=c.run,daemon=True); th.start(); time.sleep(.25)
        src=(Path('/tmp/test_imu_packet.bin').read_bytes())
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        start=0xfffe0000
        for i in range(4):
            s.sendto(frame(src,i,(start+170000*i)&0xffffffff),('127.0.0.1',16612))
            time.sleep(.04)
        time.sleep(.35)
        c.stop_event.set(); th.join(2)
        assert not th.is_alive()
        with c.imu_path.open(newline='') as fp: rows=list(csv.DictReader(fp))
        assert len(rows)==4,rows
        assert [int(r['capture_index']) for r in rows]==[0,1,2,3]
        assert [round(float(r['relative_us'])) for r in rows]==[0,1000,2000,3000],rows
        assert c.imu_packets==4,c.imu_packets
        print('UDP→CSV + rollover timing test PASS')

if __name__=='__main__':main()
