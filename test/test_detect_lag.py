#!/usr/bin/env python3
"""gmps_driver の検知の時刻の遅れ(detect_lag_sec)を、ビルドしたノードに作り物の CAN の枠を流して確かめる(2026-10-19 に書いた確かめのプログラム)。

何をするか: 隔離した ROS_DOMAIN_ID で gmps_driver を起動し、読み取り機の検知の枠(CAN id 0x10: 検知ビット、横偏差 +0.123 m、S 極、遅れの距離 0.14 m)を
stamp T で送り、出てくる GmpsDetect の stamp が T − detect_lag_sec になること、他の項目は変わらないことを見る。
入力: ビルド済みの gmps_driver(環境変数 GMPS_TEST_INSTALL の install、無ければ今の環境のもの)。設定ファイルは使わず、既定値とコマンドラインの値で起動する
  (yaml に整数で書いた場合の確かめだけ、一時の設定ファイルを作って --params-file で渡す)。
出力: pytest の合否。起動ログの版の名乗り(version <枝>@<hash>)と install の場所を print する(-s で見える)。
  環境変数 GMPS_TEST_EXPECT_VERSION があれば、版の名乗りがそれと同じことも確かめる(どのビルドを試したかを結果に残すため)。
実機では使わない(実機のトピック名には触れず、/t/... に付け替える。CAN には何も送らない。読み取り機の装置は要らない)。

確かめること:
  - 既定(detect_lag_sec を渡さない)→ stamp は 4.6 ms 前(コードの既定 0.0046。config/gmsp_driver.param.yaml も同じ値)
  - detect_lag_sec 0.0 → stamp はそのまま(従来どおり)
  - detect_lag_sec −0.01 → 0 として扱う(stamp はそのまま)。起動ログに負の値の WARN
  - detect_lag_sec 0(整数。コマンドラインと yaml の両方)→ 起動して stamp はそのまま(型を固定した宣言では起動時に死んでいた。2026-10-19 の確かめ役の指摘)
  - detect_lag_sec 0.46(桁の間違い)→ 起動ログに「0.05 s より大きい」の WARN。値はそのまま使う(stamp は 460 ms 前)
  - stamp が 0 の枠 → 0 のまま(負の時刻にしない)
  - 横偏差・極・遅れの距離・検知カウンタはそのまま
変異の確かめ: 遅れを引かない版(2026-10-19 より前の実機の版)の install を GMPS_TEST_INSTALL に指すと、既定の場合が落ちる(stamp がずれない)。
  整数の 0 の 2 件も落ちる(起動しない)。

使い方(ビルドした ws の install を source してから):
  source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -s -v -p no:cacheprovider test/test_detect_lag.py
  GMPS_TEST_INSTALL=<別の ws>/install python3 -m pytest ...     # 別のビルドで(変異の確かめ)
  GMPS_TEST_DOMAIN=98(既定)。先住のノードがあれば止まる。
  GMPS_TEST_KEEP_LOCALHOST_ONLY=1 なら環境の ROS_LOCALHOST_ONLY をそのまま使う(既定では外す。local の ubuntu-4060-3 では 1 のままだとノードが起動しない)。
"""
import os
import re
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

DOMAIN = int(os.environ.get('GMPS_TEST_DOMAIN', '98'))
INSTALL = os.environ.get('GMPS_TEST_INSTALL')
EXPECT_VERSION = os.environ.get('GMPS_TEST_EXPECT_VERSION')
DEFAULT_LAG = 0.0046
T0 = 1700000000


def ros_env():
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    if os.environ.get('GMPS_TEST_KEEP_LOCALHOST_ONLY') != '1':
        env.pop('ROS_LOCALHOST_ONLY', None)
    return env


def shell_prefix():
    s = 'set +u; source /opt/ros/humble/setup.bash; '
    if INSTALL:
        s += f'source {INSTALL}/setup.bash; '
    return s


@pytest.fixture(scope='session')
def rclpy_ctx():
    prefix = subprocess.run(['bash', '-c', f'{shell_prefix()} ros2 pkg prefix gmps_driver'],
                            env=ros_env(), capture_output=True, text=True).stdout.strip()
    print(f'\n[test_detect_lag] gmps_driver の install: {prefix or "?"}(GMPS_TEST_INSTALL={INSTALL or "なし"}、domain {DOMAIN})')
    n = subprocess.run(['bash', '-c', f'{shell_prefix()} ros2 node list --no-daemon --spin-time 2 | wc -l'],
                       env=ros_env(), capture_output=True, text=True).stdout.strip()
    if n != '0':
        pytest.fail(f'ROS_DOMAIN_ID={DOMAIN} に先住のノードが {n} ある。GMPS_TEST_DOMAIN で空いている番号を指定すること')
    import rclpy
    rclpy.init(domain_id=DOMAIN)
    yield rclpy
    rclpy.shutdown()


def run_driver(rclpy, tmpdir, extra_args, stamps):
    """gmps_driver を起動し(extra_args = 追加の ros-args の文字列)、検知の枠を stamps(秒)の順に送って、出てきた GmpsDetect の一覧とノードのログを返す"""
    from rclpy.node import Node
    from can_msgs.msg import Frame
    from gmps_msgs.msg import GmpsDetect
    logfile = Path(tmpdir) / 'driver.node.log'
    cmd = (f'{shell_prefix()} exec ros2 run gmps_driver gmps_driver --ros-args -p watchdog_timeout:=10000.0{extra_args}'
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
        assert proc.poll() is None, f'gmps_driver が起動していない(終了コード {proc.poll()}):\n{open(logfile).read()}'
        for i, t in enumerate(stamps):
            f = Frame(); f.header.frame_id = 'gmps'; f.header.stamp.sec = int(t); f.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
            f.id = 0x10; f.dlc = 8; lat = 123
            f.data = [0x01 | 0x04, lat & 0xFF, (lat >> 8) & 0xFF, 0, 0, 0, 0, 14]   # bit0 検知, bit2 = S 極, 横偏差 123 mm, 遅れ 14 cm
            pub.publish(f)
            t_wait = time.time() + 2.0
            while len(got) < i + 1 and time.time() < t_wait:
                rclpy.spin_once(nd, timeout_sec=0.02)
        assert len(got) == len(stamps), f'GmpsDetect が {len(got)}/{len(stamps)} 通:\n{open(logfile).read()}'
        log = open(logfile).read()
        check_version(log)
        return got, log
    finally:
        os.killpg(proc.pid, 2)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 9)
        nd.destroy_node()


def check_version(log):
    """起動ログの版の名乗り(version <枝>@<hash>[+dirty])を print し、GMPS_TEST_EXPECT_VERSION があればそれと同じことを確かめる"""
    m = re.search(r'\[gmps_driver\]: version (\S+)', log)
    v = m.group(1) if m else None
    print(f'[test_detect_lag] 起動ログの version: {v}')
    assert v, f'起動ログに version の行が無い:\n{log}'
    if EXPECT_VERSION:
        assert v == EXPECT_VERSION, f'版の名乗り {v} が GMPS_TEST_EXPECT_VERSION={EXPECT_VERSION} と違う(別のビルドを試している)'


def as_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def check_frames(got, stamps, expected_lag):
    for i, (m, t) in enumerate(zip(got, stamps)):
        shift_ms = (as_sec(m.header.stamp) - t) * 1e3
        assert shift_ms == pytest.approx(-expected_lag * 1e3, abs=1e-3), f'検知 {i}: stamp のずれ {shift_ms:+.3f} ms(期待 {-expected_lag * 1e3:+.3f})'
        assert m.lateral_deviation == pytest.approx(0.123, abs=1e-4) and m.pole == 2 and m.delay_dist == pytest.approx(0.14, abs=1e-6)
        assert m.counter == i + 1 and m.header.frame_id == 'gmps'


@pytest.mark.parametrize('extra, expected_lag', [('', DEFAULT_LAG), (' -p detect_lag_sec:=0.0', 0.0), (' -p detect_lag_sec:=-0.01', 0.0),
                                                 (' -p detect_lag_sec:=0', 0.0)],
                         ids=['default_0.0046', 'zero', 'negative_clamped_to_zero', 'integer_zero_accepted'])
def test_detect_stamp_is_shifted_by_lag(rclpy_ctx, extra, expected_lag):
    with tempfile.TemporaryDirectory(prefix='gmps_lag_test_') as d:
        stamps = [T0 + 0.5, T0 + 1.5, T0 + 2.5]
        got, log = run_driver(rclpy_ctx, d, extra, stamps)
        check_frames(got, stamps, expected_lag)
        assert f'detect_lag_sec {expected_lag:.4f}' in log, log
        if '-0.01' in extra:
            assert 'is negative; treated as 0' in log, log
        else:
            assert 'is negative' not in log, log
        assert 'is larger than' not in log, log


def test_integer_zero_in_params_file_is_accepted(rclpy_ctx):
    """yaml に detect_lag_sec: 0(小数点なし)と書いて戻したときにノードが起動し、stamp がずれないこと
    (型を固定した宣言では「Wrong parameter type」で終了コード 255 で死に、検知が止まっていた。2026-10-19 の確かめ役の指摘)"""
    with tempfile.TemporaryDirectory(prefix='gmps_lag_test_') as d:
        y = Path(d) / 'int_zero.param.yaml'
        y.write_text('/**:\n  ros__parameters:\n    detect_lag_sec: 0\n')
        stamps = [T0 + 0.5, T0 + 1.5]
        got, log = run_driver(rclpy_ctx, d, f' --params-file {y}', stamps)
        check_frames(got, stamps, 0.0)
        assert 'detect_lag_sec 0.0000' in log and 'is negative' not in log and 'is larger than' not in log, log


def test_large_lag_warns_but_is_applied(rclpy_ctx):
    """桁の間違い(0.46 = 100 倍)は起動ログに WARN を出す。値は変えない(頭打ちにしない。頭打ちにしても誤った位置を出すだけなので、気づけるようにする)"""
    with tempfile.TemporaryDirectory(prefix='gmps_lag_test_') as d:
        stamps = [T0 + 0.5, T0 + 1.5]
        got, log = run_driver(rclpy_ctx, d, ' -p detect_lag_sec:=0.46', stamps)
        check_frames(got, stamps, 0.46)
        assert 'detect_lag_sec 0.4600 is larger than 0.05 s' in log, log
        assert 'detect_lag_sec 0.4600 (' in log, log


def test_zero_stamp_is_not_made_negative(rclpy_ctx):
    with tempfile.TemporaryDirectory(prefix='gmps_lag_test_') as d:
        got, _ = run_driver(rclpy_ctx, d, '', [0.0, T0 + 0.5])
        assert got[0].header.stamp.sec == 0 and got[0].header.stamp.nanosec == 0, 'stamp 0 の枠は 0 のまま'
        assert (as_sec(got[1].header.stamp) - (T0 + 0.5)) * 1e3 == pytest.approx(-DEFAULT_LAG * 1e3, abs=1e-3)
