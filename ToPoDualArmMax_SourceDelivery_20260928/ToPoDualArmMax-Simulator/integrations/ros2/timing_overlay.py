"""JSK-derived ROS2 OverlayText publisher: independent actual processing times."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float64,ColorRGBA
from rviz_2d_overlay_msgs.msg import OverlayText
from timing_model import TimingModel


class Overlay(Node):
    def __init__(self):
        super().__init__('ais_gng_processing_overlay',start_parameter_services=False,enable_rosout=False)
        self.model=TimingModel()
        self.publisher=self.create_publisher(OverlayText,'/ais_gng/processing_overlay',1)
        self.metrics=self.create_publisher(String,'/ais_gng_fvg/processing_metrics',10)
        self.subs=[self.create_subscription(String,'/ais_gng/timing',lambda m:self.stage('ais',m),20),
                   self.create_subscription(String,'/fvg_observer/metrics',lambda m:self.stage('fvg',m),20),
                   self.create_subscription(Float64,'/fvg_observer/render_fps',self.fps,5),
                   self.create_subscription(String,'/rviz/frame_timing',self.render_timing,4)]
        self.timer=self.create_timer(.2,self.draw)
        live=Path(__file__).resolve().parent/'live';live.mkdir(exist_ok=True)
        self.log=(live/'processing_metrics.jsonl').open('a',buffering=1)

    def stage(self,kind,message):
        try:
            result=self.model.ingest(kind,json.loads(message.data))
            if result:
                encoded=json.dumps(result)
                self.metrics.publish(String(data=encoded))
                self.log.write(encoded+'\n')
        except (ValueError,KeyError,TypeError) as exc:
            self.log.write(json.dumps({'error':str(exc),'kind':kind,'wall_time':time.time()})+'\n')

    def fps(self,message):
        self.model.fps=message.data;self.model.fps_updated_ns=time.monotonic_ns()

    def render_timing(self,message):
        try:
            self.model.render_timing=json.loads(message.data)
            self.model.render_timing_updated_ns=time.monotonic_ns()
        except (ValueError,TypeError):pass

    def draw(self):
        text,stale=self.model.text()
        msg=OverlayText()
        msg.action=OverlayText.ADD
        msg.width=720;msg.height=245
        msg.horizontal_distance=12;msg.vertical_distance=12
        msg.horizontal_alignment=OverlayText.LEFT;msg.vertical_alignment=OverlayText.TOP
        msg.text_size=16.;msg.font='DejaVu Sans Mono';msg.line_width=1
        msg.bg_color=ColorRGBA(r=.025,g=.03,b=.04,a=.8)
        over=self.model.latest and self.model.latest['processing_sum_ms']>33.333333
        msg.fg_color=ColorRGBA(r=1. if stale or over else .75,g=.67 if stale or over else 1.,b=.3 if stale or over else .85,a=1.)
        msg.text=text
        self.publisher.publish(msg)


def main():
    rclpy.init();node=Overlay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
