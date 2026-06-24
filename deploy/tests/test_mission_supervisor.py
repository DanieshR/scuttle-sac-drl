from mission_supervisor import decide_action, ProcessManager


def test_decide_action_matrix():
    assert decide_action(running=False, requested='frontier') == 'start'
    assert decide_action(running=True,  requested='frontier') == 'noop'
    assert decide_action(running=True,  requested='manual')   == 'stop'
    assert decide_action(running=False, requested='manual')   == 'noop'
    assert decide_action(running=False, requested='idle')     == 'noop'  # any non-frontier


def test_process_manager_start_and_stop():
    pm = ProcessManager(['sleep', '1000'])
    assert pm.running is False
    pm.start()
    assert pm.running is True
    proc_before = pm.proc
    pm.start()                       # idempotent — no second process
    assert pm.proc is proc_before
    assert pm.running is True
    pm.stop(sigint_timeout=5.0, sigterm_timeout=2.0)
    assert pm.running is False


def test_process_manager_stop_when_not_running_is_safe():
    pm = ProcessManager(['sleep', '1000'])
    pm.stop()                        # should not raise
    assert pm.running is False
