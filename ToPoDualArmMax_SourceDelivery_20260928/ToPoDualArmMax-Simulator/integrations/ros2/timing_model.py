"""Same-frame timing join and HUD text, independent of ROS and rendering FPS."""
import collections
import math
import statistics
import time


class TimingModel:
    def __init__(self):
        self.pending = {'ais':collections.OrderedDict(),'fvg':collections.OrderedDict()}
        self.latest = None
        self.samples = collections.deque(maxlen=100)
        self.updated_ns = None
        self.joined = self.rejected = 0
        self.fps = None
        self.fps_updated_ns = None
        self.input_intervals = collections.deque(maxlen=100)
        self.previous_input_start = None
        self.previous_sequence = None
        self.render_timing = None
        self.render_timing_updated_ns = None

    def ingest(self, kind, metric, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        if kind not in self.pending:
            raise ValueError('Unknown stage')
        stamp = int(metric['stamp_ns'])
        sequence = metric.get('frame_sequence')
        key = (stamp, int(sequence)) if sequence is not None else stamp
        self.pending[kind][key] = (dict(metric),now_ns)
        for cache in self.pending.values():
            for old in list(cache):
                if now_ns-cache[old][1] > 5_000_000_000:
                    del cache[old]
            while len(cache)>100:
                cache.popitem(last=False)
        if not all(key in cache for cache in self.pending.values()):
            return None
        ais,ta = self.pending['ais'].pop(key)
        fvg,tf = self.pending['fvg'].pop(key)
        start = int(ais['callback_start_steady_ns'])
        end = int(fvg['worker_end_steady_ns'])
        # Reject old equal bag stamps across playback cycles or mixed executions.
        if (ais.get('frame_id')!=fvg.get('frame_id') or not ais.get('frame_id') or
            abs(ta-tf)>2_000_000_000 or end<start or end-start>5_000_000_000 or
            end>now_ns+100_000_000):
            self.rejected += 1
            return None
        ais_ms=float(ais['callback_before_metrics_ms']); fvg_ms=float(fvg['fvg_total_ms'])
        checks=(ais_ms,fvg_ms,float(ais['gng_core_ms']),float(fvg['inference_ms']),
                float(fvg['worker_queue_ms']),float(fvg['input_arrival_hz']),float(fvg['map_arrival_hz']))
        if any(not math.isfinite(x) or x<0 for x in checks):
            self.rejected += 1
            return None
        result={'stamp_ns':stamp, 'frame_sequence':sequence, 'processing_sum_ms':ais_ms+fvg_ms,
                'ais_ms':ais_ms, 'fvg_ms':fvg_ms,
                'pipeline_latency_ms':(end-start)/1e6,
                'gng_core_ms':float(ais['gng_core_ms']),
                'fvg_inference_ms':float(fvg['inference_ms']),
                'queue_ms':float(fvg['worker_queue_ms']),
                'input_hz':float(fvg['input_arrival_hz']),
                'map_hz':float(fvg['map_arrival_hz']),
                'worker_drops':int(fvg['worker_dropped_pairs']),
                'sync_cache_evictions':int(fvg['sync_cache_evictions']),
                'wall_time':time.time(),'ais':ais,'fvg':fvg}
        if (self.previous_sequence is not None and sequence is not None and sequence <= self.previous_sequence):
            self.input_intervals.clear(); self.previous_input_start = None
        if self.previous_input_start is not None and start > self.previous_input_start:
            self.input_intervals.append((start-self.previous_input_start)/1e6)
        self.previous_input_start = start; self.previous_sequence = sequence
        result['input_interval_ms'] = self.input_intervals[-1] if self.input_intervals else None
        result['input_interval_max100_ms'] = max(self.input_intervals) if self.input_intervals else None
        self.latest=result; self.updated_ns=now_ns; self.joined+=1
        self.samples.append(result['processing_sum_ms'])
        return result

    def text(self,now_ns=None):
        now_ns=time.monotonic_ns() if now_ns is None else now_ns
        if self.latest is None:
            return ('AiS-GNG-FVG: waiting for matched frame\n'
                    'Processing ms is NOT RViz FPS.\n'
                    'Waiting for stamped AiS-GNG + FVG metrics.'),False
        m=self.latest; age=max(0.,(now_ns-self.updated_ns)/1e9,
                              (now_ns-int(m['fvg']['worker_end_steady_ns']))/1e9)
        values=sorted(self.samples); p95=values[max(0,math.ceil(.95*len(values))-1)]
        stale=age>1.
        prefix='STALE / last ' if stale else ''
        fps=f'{self.fps:.1f}' if self.fps is not None and now_ns-self.fps_updated_ns<3_000_000_000 else '--'
        stream=(f"Frames {m['input_hz']:.1f} Hz | Qdrop {m['worker_drops']} | loss {m['fvg'].get('snapshot_sequence_gaps',0)}"
                if m['fvg'].get('receive_mode') in ('same_callback_compact_snapshot','same_callback_exact_roi_compact_snapshot_v3','same_callback_fullrange_compact_snapshot_v3') else
                f"Input {m['input_hz']:.1f} Hz | Qdrop {m['worker_drops']} | unpaired {m['sync_cache_evictions']}")
        interval=m.get('input_interval_ms'); maximum=m.get('input_interval_max100_ms')
        cadence=(f'Input interval {interval:.1f} / max100 {maximum:.1f} ms' if interval is not None else 'Input interval -- ms')
        if m['fvg'].get('full_range'):
            scope=f"FULL RANGE | valid points {m['ais'].get('valid_learning_points','--')} | nodes {m['fvg'].get('input_nodes','--')}"
        else: scope='Spatial ROI mode (not full range)'
        text=(f"AiS-GNG-FVG: {prefix}{m['processing_sum_ms']:.2f} ms\n"
              f"AiS {m['ais_ms']:.2f} + FVG {m['fvg_ms']:.2f} ms (stage sum)\n"
              f"Mean {statistics.mean(values):.2f} / P95 {p95:.2f} ms [{len(values)} frames]\n"
              f"Pipeline {m['pipeline_latency_ms']:.2f} ms | FVG queue {m['queue_ms']:.2f} ms\n"
              f"{stream}\n"
              f"{cadence}\n"
              f"{scope}\n"
              f"RViz {fps} fps (redraw, not new data)")
        if self.render_timing is not None and now_ns-self.render_timing_updated_ns<3_000_000_000:
            r=self.render_timing.get('frame_interval_ms',{})
            if r.get('p95') is not None and r.get('max') is not None:
                text+=f"\nDraw gap P95 {r['p95']:.1f} / max {r['max']:.1f} ms [1s]"
        return text,stale
