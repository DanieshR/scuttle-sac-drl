#!/usr/bin/env python3
"""
Progressive SAC curriculum training: stages 1 to 9
Advances when rolling success rate over last 20 episodes >= 0.70 (3 consecutive checks)
or when MAX_EPISODES is hit.

Usage (inside container):
  python3 /home/turtlebot3_drlnav/progressive_train.py [--start-stage N]

Run via docker-compose (recommended, builds workspace first):
  docker-compose run --rm drl_training python3 /home/turtlebot3_drlnav/progressive_train.py
"""

import argparse
import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

# ─── Config ────────────────────────────────────────────────────────────────────
BASE_PATH       = os.getenv('DRLNAV_BASE_PATH', '/home/turtlebot3_drlnav')
MODEL_BASE      = os.path.join(BASE_PATH, 'src', 'turtlebot3_drl', 'model')
MODEL_DIR       = os.path.join(MODEL_BASE, socket.gethostname())
STATE_FILE      = os.path.join(BASE_PATH, 'output', 'progressive_state.json')

ALGO            = 'sac'
START_STAGE     = 1
END_STAGE       = 9

SUCCESS_OUTCOME = 1   # from settings.py
SUCCESS_THRESH  = 0.70
WINDOW          = 20  # rolling window for success rate
MIN_EPISODES    = 100 # min episodes before checking advancement
MAX_EPISODES    = 600 # force-advance if this many episodes reached
CONSEC_CHECKS   = 3   # consecutive 30-second checks above threshold
POLL_SEC        = 30  # seconds between checks

GAZEBO_WAIT     = 50  # seconds for Gazebo to fully start
GOALS_WAIT      = 8   # seconds for gazebo_goals to be ready
ENV_WAIT        = 8   # seconds for environment node to be ready
AGENT_WAIT      = 25  # seconds for agent to create session dir + log

NET_NAMES       = ['actor', 'critic', 'critic_target']  # must match SAC network names

# ─── Globals ───────────────────────────────────────────────────────────────────
_procs = []   # tracked subprocesses for cleanup


def log(msg):
    ts = time.strftime('%H:%M:%S')
    print(f"[{ts}] {msg}", flush=True)


# ─── Process management ────────────────────────────────────────────────────────

def kill_all():
    """Kill all ROS2 training processes."""
    patterns = ['gzserver', 'gzclient', 'gazebo_goals', 'environment',
                'train_agent', 'robot_state_publisher', 'gznode']
    for pat in patterns:
        subprocess.run(['pkill', '-f', pat], capture_output=True)
    time.sleep(5)
    # Force kill if still alive
    for pat in patterns:
        subprocess.run(['pkill', '-9', '-f', pat], capture_output=True)
    global _procs
    _procs = []


def start_proc(cmd, label, stdout=None, stderr=None):
    env = os.environ.copy()
    log(f"  > {' '.join(cmd)}")
    p = subprocess.Popen(
        cmd, env=env,
        stdout=stdout or subprocess.DEVNULL,
        stderr=stderr or subprocess.DEVNULL
    )
    _procs.append(p)
    return p


# ─── Log parsing ───────────────────────────────────────────────────────────────

def find_session_dir(stage):
    """Return the latest session dir for given stage, or None."""
    if not os.path.exists(MODEL_DIR):
        return None
    dirs = sorted([
        d for d in os.listdir(MODEL_DIR)
        if d.startswith(ALGO + '_') and d.endswith(f'_stage_{stage}')
    ])
    return os.path.join(MODEL_DIR, dirs[-1]) if dirs else None


def find_training_log(session_dir, stage):
    """Find training log file in session_dir for given stage."""
    pattern = os.path.join(session_dir, f'_train_stage{stage}_*.txt')
    files = glob.glob(pattern)
    return max(files, key=os.path.getmtime) if files else None


def parse_log(log_path):
    """Returns (success_rate over last WINDOW, total_episodes, last_episode_num)."""
    outcomes = []
    last_ep = 0
    try:
        with open(log_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('episode'):
                    continue
                parts = line.split(',')
                if len(parts) >= 3:
                    try:
                        ep = int(parts[0].strip())
                        outcome = int(parts[2].strip())
                        outcomes.append(outcome)
                        last_ep = ep
                    except ValueError:
                        pass
    except Exception:
        return 0.0, 0, 0

    if not outcomes:
        return 0.0, 0, 0

    recent = outcomes[-WINDOW:]
    rate = recent.count(SUCCESS_OUTCOME) / len(recent)
    return rate, len(outcomes), last_ep


def get_latest_saved_episode(session_dir, stage):
    """Return highest saved episode number in session_dir for given stage."""
    pattern = os.path.join(session_dir, f'actor_stage{stage}_episode*.pt')
    files = glob.glob(pattern)
    if not files:
        return 0
    eps = []
    for f in files:
        try:
            ep = int(os.path.basename(f).split('_episode')[-1].replace('.pt', ''))
            eps.append(ep)
        except ValueError:
            pass
    return max(eps) if eps else 0


# ─── Curriculum transfer ───────────────────────────────────────────────────────

def prepare_curriculum(prev_stage, next_stage):
    """
    Create a new session dir for next_stage with weights copied from prev_stage.
    Returns (session_name, episode_to_load) or (None, 0) to start fresh.
    """
    src_dir = find_session_dir(prev_stage)
    if not src_dir:
        log(f"[Curriculum] No session for stage {prev_stage} – starting stage {next_stage} fresh")
        return None, 0

    latest_ep = get_latest_saved_episode(src_dir, prev_stage)
    if latest_ep == 0:
        log(f"[Curriculum] No saved weights in {src_dir} – starting fresh")
        return None, 0

    # Pick new session index
    i = 0
    while os.path.exists(os.path.join(MODEL_DIR, f'{ALGO}_{i}_stage_{next_stage}')):
        i += 1
    new_dir = os.path.join(MODEL_DIR, f'{ALGO}_{i}_stage_{next_stage}')
    os.makedirs(new_dir)
    log(f"[Curriculum] Transfer session: {os.path.basename(new_dir)} from ep {latest_ep}")

    # Copy + rename weight files
    for net in NET_NAMES:
        src = os.path.join(src_dir, f'{net}_stage{prev_stage}_episode{latest_ep}.pt')
        dst = os.path.join(new_dir, f'{net}_stage{next_stage}_episode{latest_ep}.pt')
        if os.path.exists(src):
            shutil.copy2(src, dst)
            log(f"  Copied {net} weights")
        else:
            log(f"  WARNING: {src} not found, skipping")

    # Copy agent pkl (model architecture)
    for fname in [f'stage{prev_stage}_agent.pkl']:
        src = os.path.join(src_dir, fname)
        dst = os.path.join(new_dir, fname.replace(f'stage{prev_stage}', f'stage{next_stage}'))
        if os.path.exists(src):
            shutil.copy2(src, dst)

    # Copy graphdata pkl (contains total_steps to skip OBSERVE_STEPS)
    src_pkl = os.path.join(src_dir, f'stage{prev_stage}_episode{latest_ep}.pkl')
    dst_pkl = os.path.join(new_dir, f'stage{next_stage}_episode{latest_ep}.pkl')
    if os.path.exists(src_pkl):
        shutil.copy2(src_pkl, dst_pkl)
        log(f"  Copied graphdata (skips {25000} observe steps)")

    # Copy replay buffer if present (can be large)
    src_buf = os.path.join(src_dir, f'stage{prev_stage}_latest_buffer.pkl')
    dst_buf = os.path.join(new_dir, f'stage{next_stage}_latest_buffer.pkl')
    if os.path.exists(src_buf):
        buf_gb = os.path.getsize(src_buf) / 1e9
        log(f"  Copying replay buffer ({buf_gb:.2f} GB)...")
        shutil.copy2(src_buf, dst_buf)

    session_name = os.path.basename(new_dir)
    return session_name, latest_ep


# ─── State persistence ─────────────────────────────────────────────────────────

def save_state(stage, session_name, load_episode):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    with open(STATE_FILE, 'w') as f:
        json.dump({'stage': stage, 'session': session_name, 'episode': load_episode}, f)


def load_state():
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE) as f:
                return json.load(f)
        except Exception:
            pass
    return None


# ─── Stage training ────────────────────────────────────────────────────────────

def run_stage(stage, session_name=None, load_episode=0):
    """
    Launch full training stack for one stage, monitor until advancement criteria met.
    Returns (final_success_rate, session_dir_used).
    """
    log(f"\n{'='*60}")
    log(f"STAGE {stage}  {'(loading: ' + session_name + ' ep' + str(load_episode) + ')' if session_name else '(fresh start)'}")
    log(f"{'='*60}")

    kill_all()

    # 1. Gazebo
    log(f"[Stage {stage}] Starting Gazebo...")
    start_proc(
        ['ros2', 'launch', 'turtlebot3_gazebo', f'turtlebot3_drl_stage{stage}.launch.py'],
        'gazebo'
    )
    time.sleep(GAZEBO_WAIT)

    # 2. Goal manager
    log(f"[Stage {stage}] Starting gazebo_goals...")
    start_proc(['ros2', 'run', 'turtlebot3_drl', 'gazebo_goals'], 'goals')
    time.sleep(GOALS_WAIT)

    # 3. Environment
    log(f"[Stage {stage}] Starting environment...")
    start_proc(['ros2', 'run', 'turtlebot3_drl', 'environment'], 'env')
    time.sleep(ENV_WAIT)

    # 4. Agent
    log(f"[Stage {stage}] Starting train_agent...")
    agent_cmd = ['ros2', 'run', 'turtlebot3_drl', 'train_agent', ALGO]
    if session_name:
        agent_cmd += [session_name, str(load_episode)]

    agent_proc = start_proc(agent_cmd, 'agent', stdout=sys.stdout, stderr=sys.stderr)
    time.sleep(AGENT_WAIT)

    # 5. Monitor
    session_dir = None
    log_file = None
    consecutive = 0
    final_rate = 0.0
    last_n = 0

    while True:
        time.sleep(POLL_SEC)

        if agent_proc.poll() is not None:
            log(f"[Stage {stage}] Agent exited unexpectedly (rc={agent_proc.returncode})")
            break

        # Locate session dir and log file
        if session_dir is None:
            if session_name:
                # We know the dir we created for curriculum
                session_dir = os.path.join(MODEL_DIR, session_name)
            else:
                session_dir = find_session_dir(stage)

        if session_dir and log_file is None:
            log_file = find_training_log(session_dir, stage)

        if log_file is None:
            log(f"[Stage {stage}] Waiting for training log...")
            continue

        rate, n_eps, last_ep = parse_log(log_file)

        if n_eps != last_n:
            log(f"[Stage {stage}] Ep {last_ep:4d} | {n_eps:4d} total | "
                f"success/{WINDOW}ep: {rate:.2f}  "
                f"{'[ABOVE THRESHOLD]' if rate >= SUCCESS_THRESH and n_eps >= MIN_EPISODES else ''}")
            last_n = n_eps

        final_rate = rate

        if n_eps < MIN_EPISODES:
            continue

        if rate >= SUCCESS_THRESH:
            consecutive += 1
            log(f"[Stage {stage}] Threshold met ({consecutive}/{CONSEC_CHECKS})")
            if consecutive >= CONSEC_CHECKS:
                log(f"[Stage {stage}] ADVANCING → stage {stage + 1}")
                break
        else:
            consecutive = 0

        if n_eps >= MAX_EPISODES:
            log(f"[Stage {stage}] MAX_EPISODES {MAX_EPISODES} reached – forcing advance")
            break

    for p in _procs[:]:
        try:
            p.terminate()
        except Exception:
            pass
    kill_all()

    return final_rate, session_dir


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-stage', type=int, default=None,
                        help='Override start stage (default: resume from state file or 1)')
    parser.add_argument('--end-stage', type=int, default=END_STAGE,
                        help=f'Stop after this stage (default: {END_STAGE})')
    args = parser.parse_args()

    # Try to resume from saved state
    state = load_state()
    if args.start_stage:
        start = args.start_stage
        session_name = None
        load_episode = 0
        log(f"Forced start at stage {start}")
    elif state:
        start = state['stage']
        session_name = state.get('session')
        load_episode = state.get('episode', 0)
        log(f"Resuming from state file: stage {start}, session={session_name}, ep={load_episode}")
    else:
        start = START_STAGE
        session_name = None
        load_episode = 0
        log(f"Starting fresh from stage {start}")

    os.makedirs(MODEL_DIR, exist_ok=True)

    end = args.end_stage
    for stage in range(start, end + 1):
        save_state(stage, session_name, load_episode)

        rate, session_dir = run_stage(stage, session_name, load_episode)
        log(f"\n[Stage {stage}] Done. Final success rate: {rate:.2f}")

        if stage < end:
            session_name, load_episode = prepare_curriculum(stage, stage + 1)
            save_state(stage + 1, session_name, load_episode)

    log(f"\n{'='*60}")
    log(f"ALL STAGES 1-9 COMPLETE")
    log(f"{'='*60}")
    if os.path.exists(STATE_FILE):
        os.remove(STATE_FILE)


if __name__ == '__main__':
    def _sigint(sig, frame):
        log("Interrupted – cleaning up")
        kill_all()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)
    main()
