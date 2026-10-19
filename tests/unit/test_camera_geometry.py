import copy
import math
import unittest
from python.dcm.camera_geometry import camera_calibration, orbit_camera_feasibility, project_point, resolve_reference


def state():
    return {'local_position_ned': {'north_m': 10., 'east_m': 20., 'down_m': -8., 'origin_id': 'ekf'},
            'attitude': {'roll_rad': 0., 'pitch_rad': 0., 'yaw_rad': 0.},
            'position': {'geo': {'latitude_deg': 0., 'longitude_deg': 0., 'altitude_m': 108., 'frame': 1}},
            'home': {'geo': {'latitude_deg': 0., 'longitude_deg': 0., 'altitude_m': 100., 'frame': 1}}}


class CameraGeometryTests(unittest.TestCase):
    def test_calibration_matches_forward_sensor(self):
        cal = camera_calibration()
        self.assertEqual((cal['width'], cal['height']), (848, 480))
        self.assertAlmostEqual(math.degrees(cal['vertical_fov_rad']), 55.8, delta=1)
        self.assertFalse(cal['tilt_control_available'])

    def test_home_altitude_transform(self):
        home = resolve_reference(state())
        self.assertTrue(home['available'])
        self.assertEqual(home['down_m'], 0)
        self.assertEqual(home['north_m'], 10)

    def test_home_is_not_current_position(self):
        s = state()
        s['home']['geo']['latitude_deg'] = 0.0001
        result = resolve_reference(s)
        self.assertAlmostEqual(result['north_m'], 21.0574, places=3)
        self.assertEqual(result['down_m'], 0)

    def test_native_home_reference_survives_async_pose_delivery(self):
        s = state()
        s.update(home_local_position_ned={'north_m': -3., 'east_m': 4.,
                 'down_m': -2., 'origin_id': 'ekf'},
                 home_position_received_at_unix_ms=1000, observed_at_unix_ms=9000)
        result = resolve_reference(s)
        self.assertEqual(result['source'], 'autopilot_HOME_POSITION')
        self.assertEqual(result['north_m'], -3.)
        s['home_local_position_ned']['origin_id'] = 'previous-origin'
        self.assertFalse(resolve_reference(s)['available'])

    def test_missing_home_frame_is_not_assumed_amsl(self):
        s = state()
        del s["home"]["geo"]["frame"]
        self.assertFalse(resolve_reference(s)["available"])

    def test_missing_home_stays_unknown(self):
        s = state()
        del s['home']
        self.assertFalse(resolve_reference(s)['available'])

    def test_launch_is_saved_pose_not_current(self):
        launch = state()
        current = copy.deepcopy(launch)
        current['local_position_ned']['north_m'] = 30
        self.assertEqual(resolve_reference(current, 'launch', launch)['north_m'], 10)
        self.assertFalse(resolve_reference(current, 'launch')['available'])

    def test_origin_change_uses_launch_global_measurement(self):
        launch = state()
        current = state()
        current['local_position_ned']['origin_id'] = 'new'
        current['position']['geo']['latitude_deg'] = 0.0001
        result = resolve_reference(current, 'launch', launch)
        self.assertTrue(result['available'])
        self.assertEqual(result['origin_id'], 'new')
        self.assertAlmostEqual(result['north_m'], -1.0574, places=3)

    def test_local_home_origin_mismatch_unavailable(self):
        s = state()
        s['home'] = {'local_ned': {**s['local_position_ned'], 'origin_id': 'old'}}
        self.assertFalse(resolve_reference(s)['available'])

    def test_invalid_coordinates_unknown(self):
        for value in (None, float('nan'), True):
            s = state()
            s['position']['geo']['latitude_deg'] = value
            self.assertFalse(resolve_reference(s)['available'])

    def test_forward_and_behind(self):
        s = state()
        target = {**s['local_position_ned'], 'north_m': 13}
        self.assertTrue(project_point(s, target)['in_frame'])
        target['north_m'] = 7
        self.assertFalse(project_point(s, target)['in_frame'])

    def test_yaw_rotation(self):
        s = state()
        s['attitude']['yaw_rad'] = math.pi/2
        target = {**s['local_position_ned'], 'east_m': 23}
        self.assertTrue(project_point(s, target)['in_frame'])
        s['attitude']['yaw_rad'] = 0
        self.assertFalse(project_point(s, target)['in_frame'])

    def test_downward_camera_sees_ground(self):
        s = state()
        target = {**s['local_position_ned'], 'down_m': 0}
        self.assertTrue(project_point(s, target, 'downward_brio')['in_frame'])
        self.assertFalse(project_point(s, target, 'forward_d435i')['in_frame'])

    def test_steep_home_orbit_infeasible_forward(self):
        result = orbit_camera_feasibility(3, 8)
        self.assertFalse(result['feasible'])
        self.assertAlmostEqual(result['target_depression_deg'], 69.44, places=1)
        self.assertFalse(result['image_proof'])
        self.assertTrue(orbit_camera_feasibility(3, 8, camera='downward_brio')['feasible'])

    def test_same_height_building_point_feasible(self):
        result = orbit_camera_feasibility(3, 8, 8)
        self.assertTrue(result['feasible'])
        self.assertFalse(result['occlusion_checked'])

    def test_unknown_camera_target_height_and_origin(self):
        self.assertFalse(orbit_camera_feasibility(3, 8, None)['available'])
        self.assertFalse(orbit_camera_feasibility(3, 8, camera='gimbal')['available'])
        s = state()
        self.assertFalse(project_point(s, {**s['local_position_ned'], 'origin_id': 'other'})['available'])

    def test_roll_changes_image_projection(self):
        s = state()
        target = {**s['local_position_ned'], 'north_m': 15, 'east_m': 21}
        upright = project_point(s, target)
        s['attitude']['roll_rad'] = math.pi/2
        rolled = project_point(s, target)
        self.assertGreater(upright['pixel_x'], 424)
        self.assertLess(rolled['pixel_y'], 240)

if __name__ == '__main__':
    unittest.main()


def test_yaw_cannot_bring_a_directly_below_point_into_a_fixed_forward_camera():
    for roll in (0.,.1,-.1):
        for yaw in (0.,math.pi/2,math.pi,3*math.pi/2):
            s=state();s['attitude'].update(roll_rad=roll,yaw_rad=yaw)
            point={**s['local_position_ned'],'down_m':s['local_position_ned']['down_m']+3}
            front=project_point(s,point,'forward_d435i')
            down=project_point(s,point,'downward_brio')
            assert front['in_frame'] is False
            assert front['yaw_alone_can_resolve'] is False
            assert 'elevation' in front['reason']
            assert down['in_frame'] is True
            assert down['yaw_alone_can_resolve'] is True
            assert front['image_proof'] is down['image_proof'] is False


def test_unknown_yaw_reachability_is_not_an_impossibility_claim():
    s=state();point={**s['local_position_ned'],'north_m':s['local_position_ned']['north_m']-5}
    result=project_point(s,point)
    assert result['in_frame'] is False
    assert result['yaw_alone_can_resolve'] is None
    s['attitude']['yaw_rad']=math.pi
    visible=project_point(s,point)
    assert visible['in_frame'] is True and visible['yaw_alone_can_resolve'] is True


def test_proven_elevation_limits_remain_outside_the_frustum_across_yaw():
    proven=0
    for camera in ('forward_d435i','downward_brio'):
        for roll,pitch in ((0.,0.),(.2,-.15),(-.15,.2)):
            for north,east,down in ((0,0,3),(0,0,-3),(.5,.2,5),(10,8,-1),(4,-6,2)):
                s=state();s['attitude'].update(roll_rad=roll,pitch_rad=pitch)
                p={k:s['local_position_ned'][k]+v for k,v in zip(('north_m','east_m','down_m'),(north,east,down))}
                p['origin_id']=s['local_position_ned']['origin_id']
                if project_point(s,p,camera)['yaw_alone_can_resolve'] is False:
                    proven+=1
                    for degree in range(0,360,10):
                        s['attitude']['yaw_rad']=math.radians(degree)
                        assert project_point(s,p,camera)['in_frame'] is False
    assert proven>0
