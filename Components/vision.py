import wpilib

import limelight
import limelightresults

import time
import json


class Vision:
    """Main vision class, responsible for close-in navigation and global positioning with AprilTags"""

    def __init__(self):

        self.good = False
        self.discovered_limelights = limelight.discover_limelights(debug=True, timeout=1)

        print("[Vision] discovered limelights:", self.discovered_limelights)

        self.ll = None
        
        if self.discovered_limelights:
            self.limelight_address = self.discovered_limelights[0]
            self.ll = limelight.Limelight(self.limelight_address)
            self.ll.enable_websocket()
            self.good = True

    def poll(self):
        if not self.good:
            return
        
        result = self.ll.get_latest_results()
        
        parsed_result = limelightresults.parse_results(result)

        if parsed_result is not None:
            print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
            print(parsed_result)
        else:
            print("no target")
