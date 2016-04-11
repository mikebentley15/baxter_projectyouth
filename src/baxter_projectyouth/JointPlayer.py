import baxter_dataflow
import baxter_interface
import math

class JointPlayer(object):
    def __init__(self, limb):
        self.limb = limb
        self._joint_names = limb.joint_names()

    def play_path(self, path):
        for positions in path[:-1]:
            self._move_to_positions(positions, False)
        self._move_to_positions(path[-1], True)
    
    def _move_to_positions(self, positions, is_end):
        if is_end:
            self.limb.move_to_joint_positions(positions)
            return

        cmd = self.limb.joint_angles()
        def filtered_cmd():
            '''
            First Order Filter - 0.2 Hz Cutoff

            Taken from baxter_interface/limb.py, then modified.
            '''
            for joint, angle in positions.iteritems():
                angle_diff = min(math.pi/2, abs(cmd[joint] - positions[joint]))
                cmd[joint] = 0.012488 * positions[joint] + 0.98751 * cmd[joint]
                # 0.012488 at pi/2 or greater
                # 0.12488 at 0
                # Linearly scale between
                #t = 0.12488 / (1 + 18 * angle_diff / math.pi)
                #cmd[joint] = t * positions[joint] + (1-t) * cmd[joint]
            return cmd

        diffs = [lambda: abs(a - self.limb.joint_angle(j)) for j, a in positions.iteritems()]

        threshold = baxter_interface.settings.JOINT_ANGLE_TOLERANCE
        timeout = 15.0 # seconds

        # Otherwise, go there without the filter
        self.limb.set_joint_positions(filtered_cmd())
        baxter_dataflow.wait_for(
            lambda: (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            rate=100,
            raise_on_error=False,
            body=lambda: self.limb.set_joint_positions(filtered_cmd())
            )

