#!/usr/bin/env python3
"""gmps_driver の検知の時刻の遅れ(detect_lag_sec)を、ビルドしたノードに作り物の CAN の枠を流して確かめる(2026-10-19 に書いた確かめのプログラム)。

何をするか: 隔離した ROS_DOMAIN_ID で gmps_driver を起動し、読み取り機の検知の枠(CAN id 0x10: 検知ビット、横偏差 +0.123 m、S 極、遅れの距離 0.14 m)を
stamp T で送り、出てくる GmpsDetect の stamp が T − detect_lag_sec になること、他の項目は変わらないことを見る。
入力: ビルド済みの gmps_driver(環境変数 GMPS_TEST_INSTALL の install、無ければ今の環境のもの)。設定ファイルは使わず、既定値とコマンドラインの値で起動する。
出力: pytest の合否。
実機では使わない(実機のトピック名には触れず、/t/... に付け替える。CAN には何も送らない)。

確かめること:
  - 既定(detect_lag_sec を渡さない)→ stamp は 4.6 ms 前(コードの既定 0.0046。config/gmsp_driver.param.yaml も同じ値)
  - detect_lag_sec 0.0 → stamp はそのまま(従来どおり)
  - detect_lag_sec −0.01 → 0 として扱う(stamp はそのまま)。起動ログに負の値の WARN
  - stamp が 0 の枠 → 0 のまま(負の時刻にしない)
  - 横偏差・極・遅れの距離・検知カウンタはそのまま
変異の確かめ: 遅れを引かない版(2026-10-19 より前の実機の版)の install を GMPS_TEST_INSTALL に指すと、既定の場合が落ちる(stamp がずれない)。

使い方(ビルドした ws の install を source してから):
  source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash
  python3 -m pytest -s -v test/test_detect_lag.py
  GMPS_TEST_INSTALL=<別の ws>/install python3 -m pytest ...     # 別のビルドで(変異の確かめ)
  GMPS_TEST_DOMAIN=98(既定)。先住のノードがあれば止まる。
"""
import os
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

DOMAIN = int(os.environ.get('GMPS_TEST_DOMAIN', '98'))
INSTALL = os.environ.get('GMPS_TEST_INSTALL')
DEFAULT_LAG = 0.0046
T0 = 1700000000


def ros_env():
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    env.pop('ROS_LOCALHOST_ONLY', None)
    return env


def shell_prefix():
    s = 'set +u; source /opt/ros/humble/setup.bash; '
    if INSTALL:
        s += f'source {INSTALL}/setup.bash; '
    return s


@pytest.fixture(scope='session')
def rclpy_ctx():
    n = subprocess.run(['bash', '-c', f'{shell_prefix()} ros2 node list --no-daemon --spin-time 2 | wc -l'],
                       env=ros_env(), capture_output=True, text=True).stdout.strip()
    if n != '0':
        pytest.fail(f'ROS_DOMAIN_ID={DOMAIN} に先住のノードが {n} ある。GMPS_TEST_DOMAIN で空いている番号を指定すること')
    import rclpy
    rclpy.init(domain_id=DOMAIN)
    yield rclpy
    rclpy.shutdown()


def run_driver(rclpy, tmpdir, lag_arg, stamps):
    """gmps_driver を起動し、検知の枠を stamps(秒)の順に送って、出てきた GmpsDetect の一覧とノードのログを返す"""
    from rclpy.node import Node
    from can_msgs.msg import Frame
    from gmps_msgs.msg import GmpsDetect
    logfile = Path(tmpdir) / 'driver.node.log'
    extra = f' -p detect_lag_sec:={lag_arg}' if lag_arg is not None else ''
    cmd = (f'{shell_prefix()} exec ros2 run gmps_driver gmps_driver --ros-args -p watchdog_timeout:=10000.0{extra}'
           ' -r in_gmps_can_frame:=/t/can_rx -r out_gmps_can_frame:=/t/can_tx -r out_gmps_detect:=/t/gmps_detect'
           ' -r in_velocity:=/t/vel -r in_soft_reset:=/t/reset -r out_gmps_error:=/t/err')
    proc = subprocess.Popen(['bash', '-c', cmd], env=ros_env(), stdout=open(logfile, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    nd = Node('gmps_driver_test')
    got = []
    nd.create_subscription(GmpsDetect, '/t/gmps_detect', got.append, 10)
    pub = nd.create_publisher(Frame, '/t/can_rx', 10)
    try:
        t_end = time.time() + 2.5
        while time.time() < t_end:
            rclpy.spin_once(nd, timeout_sec=0.02)
        assert proc.poll() is None, f'gmps_driver が起動していない:\n{open(logfile).read()}'
        for i, t in enumerate(stamps):
            f = Frame(); f.header.frame_id = 'gmps'; f.header.stamp.sec = int(t); f.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
            f.id = 0x10; f.dlc = 8; lat = 123
            f.data = [0x01 | 0x04, lat & 0xFF, (lat >> 8) & 0xFF, 0, 0, 0, 0, 14]   # bit0 検知, bit2 = S 極, 横偏差 123 mm, 遅れ 14 cm
            pub.publish(f)
            t_wait = time.time() + 2.0
            while len(got) < i + 1 and time.time() < t_wait:
                rclpy.spin_once(nd, timeout_sec=0.02)
        assert len(got) == len(stamps), f'GmpsDetect が {len(got)}/{len(stamps)} 通:\n{open(logfile).read()}'
        return got, open(logfile).read()
    finally:
        os.killpg(proc.pid, 2)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 9)
        nd.destroy_node()


def as_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


@pytest.mark.parametrize('lag_arg, expected_lag', [(None, DEFAULT_LAG), ('0.0', 0.0), ('-0.01', 0.0)],
                         ids=['default_0.0046', 'zero', 'negative_clamped_to_zero'])
def test_detect_stamp_is_shifted_by_lag(rclpy_ctx, lag_arg, expected_lag):
    with tempfile.TemporaryDirectory(prefix='gmps_lag_test_') as d:
        stamps = [T0 + 0.5, T0 + 1.5, T0 + 2.5]
        got, log = run_driver(rclpy_ctx, d, lag_arg, stamps)
        for i, (m, t) in enumerate(zip(got, stamps)):
            shift_ms = (as_sec(m.header.stamp) - t) * 1e3
            assert shift_ms == pytest.approx(-expected_lag * 1e3, abs=1e-3), f'検知 {i}: stamp のずれ {shift_ms:+.3f} ms'
            assert m.lateral_deviation == pytest.approx(0.123, abs=1e-4) and m.pole == 2 and m.delay_dist == pytest.approx(0.14, abs=1e-6)
            assert m.counter == i + 1 and m.header.frame_id == 'gmps'
        assert f'detect_lag_sec {expected_lag:.4f}' in log, log
        assert 'version ' in log, log
        if lag_arg is not None and float(lag_arg) < 0:
            assert 'is negative; treated as 0' in log, log
        else:
            assert 'is negative' not in log, log


def test_zero_stamp_is_not_made_negative(rclpy_ctx):
    with tempfile.TemporaryDirectory(prefix='gmps_lag_test_') as d:
        got, _ = run_driver(rclpy_ctx, d, None, [0.0, T0 + 0.5])
        assert got[0].header.stamp.sec == 0 and got[0].header.stamp.nanosec == 0, 'stamp 0 の枠は 0 のまま'
        assert (as_sec(got[1].header.stamp) - (T0 + 0.5)) * 1e3 == pytest.approx(-DEFAULT_LAG * 1e3, abs=1e-3)
