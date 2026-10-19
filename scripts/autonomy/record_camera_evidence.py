#!/usr/bin/env python3
"""Read-only simulation camera/pose samples tied to a live evaluation process.

No control lease, movement calls, model calls, target labels or simulator truth.
Frames can be joined to action intervals by their recorded capture timestamps.
"""
import argparse
import json
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'build/generated/python'))


def main():
    import grpc
    from google.protobuf.json_format import MessageToDict
    from PIL import Image
    from icarus.v1 import drone_api_pb2 as api, drone_api_pb2_grpc as rpc
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--process', required=True, type=int)
    parser.add_argument('--interval', type=float, default=5)
    parser.add_argument('--maximum-frames', type=int, default=1000)
    args = parser.parse_args()
    if args.interval <= 0 or args.maximum_frames <= 0:
        parser.error('Interval and maximum frames must be positive')
    process_path = Path(f'/proc/{args.process}/stat')
    identity = process_path.read_text().rsplit(')', 1)[1].split()[19]
    stopping = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stopping.set())
    args.directory.mkdir(parents=True, exist_ok=True)
    channel = grpc.insecure_channel('127.0.0.1:50051')
    sessions = rpc.SessionServiceStub(channel)
    state_api = rpc.StateServiceStub(channel)
    session = sessions.Connect(api.ConnectRequest(client_id='camera-evidence-observer',
        client_name='read-only camera acceptance', requested_api_major='1'), timeout=5).session_id
    frames = 0
    try:
        while frames < args.maximum_frames and not stopping.is_set():
            try:
                if process_path.read_text().rsplit(')', 1)[1].split()[19] != identity:
                    break
            except OSError:
                break
            try:
                request = api.GetStateRequest(session_id=session, vehicle_id='icarus-01')
                before = state_api.GetState(request, timeout=2)
                if before.armed and not before.landed:
                    with tempfile.TemporaryDirectory(prefix='icarus-camera-proof-') as temporary:
                        raw = Path(temporary) / 'frame.ppm'
                        subprocess.run([str(ROOT / 'scripts/capture-camera-frame'), '--output', str(raw)],
                            cwd=ROOT, check=True, capture_output=True, timeout=15)
                        metadata = json.loads(raw.with_suffix('.ppm.json').read_text())
                        after = state_api.GetState(request, timeout=2)
                        output = args.directory / f"{metadata['captured_at_unix_ms']}-forward.png"
                        Image.open(raw).convert('RGB').save(output)
                        metadata.update(observer_only=True, simulation_only=True,
                            state_before=MessageToDict(before, preserving_proto_field_name=True),
                            state_after=MessageToDict(after, preserving_proto_field_name=True))
                        output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
                        frames += 1
            except (OSError, ValueError, subprocess.SubprocessError, grpc.RpcError) as error:
                with (args.directory / 'errors.jsonl').open('a') as stream:
                    stream.write(json.dumps({'unix_ms': int(time.time()*1000), 'error': str(error)}) + '\n')
            stopping.wait(args.interval)
    finally:
        try:
            sessions.CloseSession(api.CloseSessionRequest(session_id=session), timeout=2)
        except grpc.RpcError:
            pass
        channel.close()
    print(json.dumps({'frames': frames, 'flight_authority': False}), flush=True)


if __name__ == '__main__':
    main()
