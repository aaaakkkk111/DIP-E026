from __future__ import annotations

import unittest

from route_a.firmware_profile import DEFAULT_PID, FIRMWARE_CONSTANTS, PIDValues
from route_a.protocol import (
    BYTES_PER_SECOND_8N1, FrameStreamParser, ProtocolError, build_checked_frame,
    build_motion_frame, build_pid_query_frame, build_pid_update_frame,
    parse_pid_report, verify_checked_frame,
)
from route_a.telemetry import FastTelemetry, SlowTelemetry, parse_extended_telemetry
from route_a.virtual_stm32 import FirmwareKalmanPitch, SensorInputs, VirtualSTM32


class FirmwareProtocolTests(unittest.TestCase):
    def test_firmware_kalman_tracks_static_pitch(self):
        import math
        filter_=FirmwareKalmanPitch(); angle=math.radians(6.0)
        estimates=[filter_.step(-math.sin(angle),math.cos(angle),0.0) for _ in range(200)]
        self.assertAlmostEqual(math.degrees(estimates[-1]),6.0,delta=0.15)

    def test_default_values_scaling_and_exact_full_update(self):
        mcu=VirtualSTM32()
        self.assertEqual(mcu.pid,DEFAULT_PID)
        self.assertEqual(mcu.internal_pid,{"AP":9600,"AD":48,"VP":6200,"VI":31,"TP":1400,"TD":20})
        expected=b"$0,0,0,0,1,1,1,AP96.00,AD48.00,VP62.00,VI31.00,TP14.00,TD20.00#"
        self.assertEqual(build_pid_update_frame(DEFAULT_PID),expected)
        self.assertLess(len(expected),80)

    def test_query_replies_twice_and_three_groups_ack_three_times(self):
        mcu=VirtualSTM32()
        query=mcu.receive(build_pid_query_frame())
        self.assertEqual(len(query),2); self.assertEqual(query[0],query[1]); self.assertEqual(parse_pid_report(query[0]),DEFAULT_PID)
        candidate=PIDValues(97,49,63,32,15,21)
        replies=mcu.receive(build_pid_update_frame(candidate))
        self.assertEqual(replies,[b"$OK#",b"$OK#",b"$OK#"]); self.assertEqual(mcu.pid,candidate)

    def test_restore_receive_error_and_query_error(self):
        from route_a.protocol import build_restore_frame
        mcu=VirtualSTM32(PIDValues(-1,48,62,31,14,20))
        self.assertEqual(mcu.receive(build_pid_query_frame()),[b"$GetPIDError#"])
        replies=mcu.receive(build_restore_frame()); self.assertEqual(replies[-1],b"$OK#"); self.assertEqual(mcu.pid,DEFAULT_PID)
        self.assertEqual(mcu.receive(b"$bad#"),[b"$ReceivePackError#"])

    def test_motion_semantics_are_firmware_targets(self):
        mcu=VirtualSTM32(); sensor=SensorInputs()
        mcu.receive(build_motion_frame("forward")); mcu.control_step(sensor)
        self.assertEqual(mcu.last.movement,25)
        mcu.receive(build_motion_frame("backward")); mcu.control_step(sensor); self.assertEqual(mcu.last.movement,-25)
        mcu.receive(build_motion_frame("left")); mcu.control_step(sensor); self.assertEqual(mcu.last.turn_target,-30)
        mcu.receive(build_motion_frame("pivot_right")); mcu.control_step(sensor); self.assertEqual(mcu.last.turn_target,50)

    def test_five_ms_control_and_protections(self):
        mcu=VirtualSTM32(); mcu.control_step(SensorInputs(pitch_deg=2)); self.assertEqual(mcu.tick_ms,5)
        pwm,_=mcu.control_step(SensorInputs(pitch_deg=41)); self.assertEqual(pwm,(0,0)); self.assertTrue(mcu.last.flags&1)
        pwm,_=mcu.control_step(SensorInputs(battery_v=9.5)); self.assertEqual(pwm,(0,0)); self.assertTrue(mcu.last.flags&2)
        self.assertEqual(FIRMWARE_CONSTANTS["pwm_deadzone"],1300); self.assertEqual(FIRMWARE_CONSTANTS["pwm_limit"],2600)

    def test_fragmentation_sticky_frames_and_checksum(self):
        parser=FrameStreamParser(); first=build_motion_frame("stop"); second=build_pid_query_frame()
        output=[]
        for chunk in (first[:3],first[3:]+second[:4],second[4:]): output.extend(parser.feed(chunk))
        self.assertEqual(output,[first,second])
        frame=build_checked_frame("E1S,500,12000,0,0,0"); self.assertEqual(verify_checked_frame(frame),"E1S,500,12000,0,0,0")
        bad=bytearray(frame); bad[5]^=1
        with self.assertRaises(ProtocolError): verify_checked_frame(bytes(bad))

    def test_80_byte_boundary(self):
        parser=FrameStreamParser(); self.assertEqual(parser.feed(b"$"+b"A"*77+b"#"),[b"$"+b"A"*77+b"#"])
        self.assertEqual(parser.feed(b"$"+b"A"*78+b"#"),[]); self.assertGreaterEqual(parser.discarded_frames,1)

    def test_extended_frames_fit_9600_baud_and_are_deployable(self):
        mcu=VirtualSTM32(); frames=[]
        for _ in range(100):
            _,new=mcu.control_step(SensorInputs(pitch_deg=39,pitch_rate_deg_s=150,yaw_rate_deg_s=50,encoder_left=20,encoder_right=-20))
            frames.extend(new)
        fast=next(frame for frame in frames if frame.startswith(b"$E1F")); slow=next(frame for frame in frames if frame.startswith(b"$E1S"))
        self.assertIsInstance(parse_extended_telemetry(fast),FastTelemetry); self.assertIsInstance(parse_extended_telemetry(slow),SlowTelemetry)
        self.assertLess(len(fast),80); self.assertLess(len(slow),80)
        self.assertLess(len(fast)*10+len(slow)*2,BYTES_PER_SECOND_8N1)
        forbidden={"contact_force_n","friction","center_of_mass","true_angle","motor_torque","external_push"}
        self.assertFalse(forbidden & set(parse_extended_telemetry(fast).as_dict()))


if __name__=="__main__": unittest.main()
