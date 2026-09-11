"""Exercise actual command builders without credentials or hardware access."""
import importlib.machinery
import importlib.util
from pathlib import Path
import shlex
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
loader = importlib.machinery.SourceFileLoader('fleet_ctl_tested', str(ROOT / 'fleet_ctl'))
spec = importlib.util.spec_from_loader(loader.name, loader)
fleet = importlib.util.module_from_spec(spec)
loader.exec_module(fleet)


class PlanningCommands(unittest.TestCase):
    def setUp(self):
        self.no_network = patch.object(fleet.subprocess, 'run', side_effect=AssertionError('No subprocess expected'))
        self.no_credentials = patch.object(fleet, 'password', side_effect=AssertionError('No credentials expected'))
        self.no_network.start()
        self.no_credentials.start()
        self.addCleanup(self.no_network.stop)
        self.addCleanup(self.no_credentials.stop)

    def test_existing_field_defaults(self):
        vid, command = fleet.launch_line('ghost', [])
        self.assertEqual(vid, 3)
        tokens = shlex.split(command)
        self.assertIn('drone_id:=3', tokens)
        self.assertIn('pose_source:=cuvslam', tokens)
        self.assertIn('trust_prior_alignment:=false', tokens)
        self.assertIn('debug_skip_arm_check:=false', tokens)
        self.assertFalse(any(t.startswith(('odom_topic:=', 'esdf_service_name:=')) for t in tokens))

    def test_gui_uses_same_team_geofence_stages(self):
        for name, (vid, _) in fleet.FLEET.items():
            fis = shlex.split(fleet.planning_tab_command(name, '3', []))
            planner = shlex.split(fleet.planning_tab_command(name, '5', []))
            self.assertIn('fis_stage.launch.py', fis)
            self.assertIn('planner_stage.launch.py', planner)
            for tokens in [fis, planner]:
                self.assertIn('drone_id:=%d' % vid, tokens)
                self.assertIn('alignment_yaml:=%s' % fleet.YAML, tokens)
            self.assertIn('debug_skip_arm_check:=false', planner)

    def test_explicit_options_match_gui_and_headless(self):
        args = ['--shared-frontiers', '--planner-odom=/review/odometry', '--ekf2', '--trust-prior']
        _, command = fleet.launch_line('delta', args)
        tokens = shlex.split(command)
        fis = shlex.split(fleet.planning_tab_command('delta', '3', args))
        planner = shlex.split(fleet.planning_tab_command('delta', '5', args))
        service = 'esdf_service_name:=/nvblox_shared_node/get_esdf_and_gradient'
        odom = 'odom_topic:=/review/odometry'
        self.assertIn(service, tokens)
        self.assertIn(service, fis)
        self.assertNotIn(service, planner)
        self.assertIn(odom, tokens)
        self.assertIn(odom, planner)
        self.assertNotIn(odom, fis)
        self.assertIn('pose_source:=ekf2', tokens)

    def test_ekf2_does_not_implicitly_switch_planner_odometry(self):
        self.assertNotIn('odom_topic:=', fleet.launch_line('ghost', ['--ekf2'])[1])

    def test_debug_remains_explicit(self):
        command = fleet.planning_tab_command('ghost', '5', ['--debug'])
        self.assertIn('debug_skip_arm_check:=true', shlex.split(command))

    def test_reject_invalid_or_shell_active_topic_names(self):
        for topic in ['', '/bad topic', '/bad;cmd', '$(cmd)', '`cmd`', '/a\ncmd', '/a//b']:
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                fleet.launch_line('ghost', ['--planner-odom=' + topic])

    def test_only_planning_tabs_are_rendered(self):
        with self.assertRaises(ValueError):
            fleet.planning_tab_command('ghost', '2', [])

    def test_malformed_new_options_do_not_silently_select_defaults(self):
        for args in [['--planner-odom'], ['--planner-odom', '/review/odometry'],
                     ['--shared-frontiers=true'], ['--planner-odomx=/odom']]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                fleet.planning_options(args)

    def test_parent_option_validation_is_non_actuating(self):
        fleet.cmd_validate_planning_options(['--shared-frontiers', '--planner-odom=/odom'])
        with self.assertRaises(SystemExit):
            fleet.cmd_validate_planning_options(['--shared-frontiers=false'])


if __name__ == '__main__':
    unittest.main()
