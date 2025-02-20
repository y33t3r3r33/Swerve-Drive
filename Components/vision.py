import wpilib
import wpiutil

import limelight
import limelightresults

import time
import json

import sys


class Vision:
    """Main vision class, responsible for close-in navigation and global positioning with AprilTags"""

    def __init__(self):

        print("\nVision.__init__ executed, initializing vision class...")
        
        self.good = False
        self.discovered_limelights = limelight.discover_limelights(debug=True, timeout=1)

        print("[Vision.__init__] discovered limelights:", self.discovered_limelights)

        self.ll = None

        self.cycle = 0
        
        if self.discovered_limelights:
            print("[Vision.__init__] Limelights discovered, opening first one...")
            self.limelight_address = self.discovered_limelights[0]
            self.ll = limelight.Limelight(self.limelight_address)
            timestamp = time.perf_counter_ns()
            self.ll.enable_websocket()
            timestamp_end = time.perf_counter_ns()
            timestamp_end -= timestamp
            print(f"[Vision.__init__] Took {timestamp_end}ns to enable websocket")
            print("[Vision.__init__] Flagging vision class as good, current limelight = " + self.limelight_address)
            self.good = True
        else:
            print("[Vision.__init__] No limelights found.")

        print("Vision.__init__ completed.")

    def __del__(self):
        print("\nRunning Vision.__del__, destructing...")
        if self.good:
            self.good = False
            print("[Vision.__del__] Vision flagged as good, closing...")
            self.ll.disable_websocket()
        else:
            print("[Vision.__del__] Vision not flagged as good, nothing to do.")

        print("Vision.__del__ completed.")

    def poll(self):
        if not self.good:
            return

        # self.cycle += 1
        
        # if self.cycle < 10:
            # return

        # self.cycle = 0

        results = self.ll.get_latest_results()

        parsed_results = limelightresults.parse_results(results)

        if parsed_results is not None:
            fiducial_results = parsed_results.fiducialResults
            for i in fiducial_results:
                print(vars(i))

        # parsed_result = limelightresults.parse_results(result)

        # if parsed_result is not None:
        #     print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
        #     bot_pose = parsed_result.botpose
        #     if bot_pose is not None:
        #         print(f"position from cam: {bot_pose[0]}, {bot_pose[1]}, {bot_pose[2]}")
        #         print(f"rotation from cam: {bot_pose[3]}, {bot_pose[4]}, {bot_pose[5]}")
        # else:
        #     print("no target")
