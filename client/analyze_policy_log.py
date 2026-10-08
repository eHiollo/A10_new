from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {"count": len(values), "median": statistics.median(values),
            "p95": values[min(len(values) - 1, int((len(values) - 1) * 0.95))], "max": values[-1]}


def analyze_retargets(inference, sends, events):
    inference_by_actions = {json.dumps(record["actions"]): record for record in inference}
    sends_by_sequence = {record["ack"]["batch_seq"]: record for record in sends if "batch_seq" in record.get("ack", {})}
    matched = []
    for event in events:
        if event.get("event") not in ("segment", "retarget"):
            continue
        sent = sends_by_sequence.get(event["batch_seq"])
        prediction = inference_by_actions.get(json.dumps(sent["actions"])) if sent else None
        if prediction is None:
            continue
        state = prediction["state"][:6]
        target_delta = [goal - origin for goal, origin in zip(event["q1"][:6], state)]
        observation_drift = [command - origin for command, origin in zip(event["q0"][:6], state)]
        applied_delta = [goal - command for goal, command in zip(event["q1"][:6], event["q0"][:6])]
        matched.append({"batch_seq": event["batch_seq"], "prefix_skip": event["prefix_skip"],
                        "model_delta_from_observation": target_delta, "command_drift_since_observation": observation_drift,
                        "applied_delta": applied_delta, "first_segment_s": event["n_steps"] / 500.0})
    return {
        "matched_batches": len(matched),
        "significance_threshold_rad": 0.001,
        "opposite_to_model_delta_per_axis": [sum(abs(row["applied_delta"][axis]) > 0.001 and
                                                   row["model_delta_from_observation"][axis] * row["applied_delta"][axis] < 0
                                                   for row in matched) for axis in range(6)],
        "nontrivial_moves_per_axis": [sum(abs(row["applied_delta"][axis]) > 0.001 for row in matched) for axis in range(6)],
        "first_segment_s": distribution([row["first_segment_s"] for row in matched]),
        "recent_batches": matched[-12:],
    }


def summarize(path: Path) -> dict:
    records = []
    incomplete_lines = 0
    with path.open(encoding="utf-8") as source:
        for line in source:
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                incomplete_lines += 1
    inference = [record for record in records if record.get("event") == "inference"]
    sends = [record for record in records if record.get("event") == "batch_sent"]
    async_sends = [record for record in records if record.get("event") == "async_send"]
    statuses = [record["status"] for record in records if record.get("event") == "robot_status"]
    events = [event for status in statuses for event in status.get("events", [])]
    samples = [event for event in events if event.get("event") == "sample"]
    following_error = [max(abs(target - actual) for target, actual in zip(event["q0"][:6], event["actual"][:6])) for event in samples]
    alignment = [event for event in events if event.get("event") == "align"]
    empty_by_sequence = {}
    for record in records:
        if record.get("event") != "robot_status":
            continue
        status = record["status"]
        if status.get("remaining") == 0 and status.get("policy_running") is True:
            empty_by_sequence.setdefault(status.get("batch_seq"), record["wall_time_ns"])
    gaps_ms = []
    execution_s = []
    for sent, next_sent in zip(sends, sends[1:]):
        emptied_at = empty_by_sequence.get(sent.get("ack", {}).get("batch_seq"))
        if emptied_at is not None and sent["wall_time_ns"] <= emptied_at <= next_sent["wall_time_ns"]:
            execution_s.append((emptied_at - sent["wall_time_ns"]) / 1e9)
            gaps_ms.append((next_sent["wall_time_ns"] - emptied_at) / 1e6)
    return {
        "log": str(path), "records": len(records), "incomplete_lines": incomplete_lines,
        "session": next((record for record in records if record.get("event") == "session_start"), None),
        "controller_builds": sorted({status["controller_build"] for status in statuses if "controller_build" in status}),
        "raw_action_shapes": dict(Counter(str(record.get("raw_action_shape")) for record in inference)),
        "inference_roundtrip_ms": distribution([record["inference_roundtrip_ms"] for record in inference]),
        "transport_timing_ms": {name: distribution([record["transport_timing"][name] for record in inference
                                                     if name in record.get("transport_timing", {})])
                                for name in ("pack_ms", "send_ms", "receive_wait_ms", "unpack_ms")},
        "server_infer_ms": distribution([record["server_timing"]["infer_ms"] for record in inference
                                          if isinstance(record.get("server_timing"), dict) and "infer_ms" in record["server_timing"]]),
        "policy_infer_ms": distribution([record["policy_timing"]["infer_ms"] for record in inference
                                          if isinstance(record.get("policy_timing"), dict) and "infer_ms" in record["policy_timing"]]),
        "retarget_analysis": analyze_retargets(inference, sends, events),
        "sent_observation_age_s": distribution([record["obs_age_s"] for record in sends if record.get("obs_age_s") is not None]),
        "batch_ack_rtt_ms": distribution([record["ack_rtt_ms"] for record in sends]),
        "sent_batches": len(sends),
        "batch_execution_s": distribution(execution_s),
        "queue_empty_to_next_send_ms": distribution(gaps_ms),
        "pipeline_sends": sum(record.get("event") == "pipeline_send" for record in records),
        "pipeline_discards": dict(Counter(record["reason"] for record in records if record.get("event") == "pipeline_discard")),
        "async_overlap_sends": sum(record["remaining_before"] > 0 for record in async_sends),
        "async_empty_queue_sends_after_first": sum(record["remaining_before"] == 0 and record["cycle"] > 0 for record in async_sends),
        "async_unknown_queue_sends": sum(record["remaining_before"] < 0 for record in async_sends),
        "aligned_batches": len(alignment),
        "fully_expired_batches": len({event["batch_seq"] for event in alignment if event["remaining"] == 0 and event["prefix_skip"] > 0}),
        "prefix_skip": distribution([event["prefix_skip"] for event in alignment]),
        "controller_events": dict(Counter(event["event"] for event in events)),
        "max_axis_following_error_rad": distribution(following_error),
        "control_period_max_us_per_sample": distribution([event["max_control_period_us"] for event in samples]),
        "control_work_max_us_per_sample": distribution([event["max_control_work_us"] for event in samples]),
        "arm_vel_values": sorted({event["arm_vel"] for event in samples}),
        "minimum_action_segment_s_values": sorted({status["minimum_action_segment_s"] for status in statuses if "minimum_action_segment_s" in status}),
        "smoothing_samples": sum(bool(event.get("smoothing")) for event in samples),
        "command_velocity_max_rad_s": distribution([event["max_command_vel_rad_s"] for event in samples if "max_command_vel_rad_s" in event]),
        "command_acceleration_max_rad_s2": distribution([event["max_command_acc_rad_s2"] for event in samples if "max_command_acc_rad_s2" in event]),
        "reference_lag_rad": distribution([event["reference_lag_rad"] for event in samples if "reference_lag_rad" in event]),
        "camera_age_s": {name: distribution([record["cameras"][name]["age_s"] for record in inference if record["cameras"].get(name)]) for name in ("right", "top")},
        "state_age_s": distribution([status["state_age_s"] for status in statuses if "state_age_s" in status]),
        "trace_dropped": max((status.get("trace_dropped", 0) for status in statuses), default=0),
        "stop_acknowledged": any(record.get("event") == "stop_ack" for record in records),
        "last_policy_running": statuses[-1].get("policy_running") if statuses else None,
        "errors": [record for record in records if record.get("event") in ("inference_error", "status_error", "transport_error", "pipeline_error", "session_error", "stop_failed")],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="汇总本次异步实机日志；不连接机器人。")
    parser.add_argument("session", nargs="?", type=Path)
    args = parser.parse_args()
    session = args.session
    if session is None:
        candidates = list((Path(__file__).resolve().parent / "logs").glob("run_*/events.jsonl"))
        if not candidates:
            parser.error("未找到新格式实机日志，请传入日志目录。")
        session = max(candidates, key=lambda path: path.stat().st_mtime)
    path = session / "events.jsonl" if session.is_dir() else session
    print(json.dumps(summarize(path), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
