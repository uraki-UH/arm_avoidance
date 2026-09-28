"""Exact wire regression: DDS padding, truncation and arbitrary suffix rejection."""
import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from compact_frame import HEADER,MAGIC,parse_snapshot,SnapshotFormatError

class Transport(unittest.TestCase):
    def packet(self,frame):
        payload=HEADER.pack(MAGIC,1234,1,1,0,len(frame),1,1,2,*([0.]*6))+frame+struct.pack('<6fI',1,2,3,4,5,6,7)
        return b'\0\1\0\0'+struct.pack('<III',0,0,len(payload))+payload
    def test_all_alignments(self):
        for name in (b'base_footprint',b'a',b'ab',b'abc',b'abcd'):
            raw=self.packet(name);padded=raw+bytes((-len(raw))%4)
            for data in (raw,padded):
                got=parse_snapshot(data)
                self.assertEqual(got.frame_id,name.decode());self.assertEqual(got.points.tolist(),[[1.,2.,3.]])
            for bad in (raw[:-1],raw+b'\x01',raw+b'\0'*4,padded+b'\0'):
                with self.assertRaises(SnapshotFormatError):parse_snapshot(bad)

if __name__=='__main__':unittest.main()
