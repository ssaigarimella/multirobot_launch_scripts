#!/usr/bin/env python3
"""make_rviz_configs.py - generate the checked-in .rviz configs.

Generated rather than hand-edited so the per-drone colour scheme and the topic
bindings stay consistent, and so regenerating after a topic rename is one edit.
Run:  python3 viz/make_rviz_configs.py   (writes viz/*.rviz)

PER-DRONE COLOUR CODING (stated explicitly, as the figures depend on it):
    delta  (d1)  AMBER   255;140;0     reference drone, id 1
    ghost  (d3)  CYAN    0;190;255
    peers' shared map is drawn in a desaturated version of the owning drone's
    colour, so "what I mapped" vs "what my peer told me" is visually separable.

FIXED FRAME is `world` - the common root produced by viz/fleet_frame_prefix.py.
Every display below reads a /d<N>/... topic whose header.frame_id has already
been rewritten to d<N>/odom, which resolves to world through
world -> d<N>/map -> d<N>/odom.
"""
import os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

DRONES = [
    # (id, name, colour, peer-map colour)
    (1, 'delta', '255; 140; 0', '160; 100; 20'),
    (3, 'ghost', '0; 190; 255', '20; 110; 150'),
]

# Flight volume: the planner runs a 35x35 m team geofence, and nvblox's
# workspace AABB is 20x20x3 m (seen in the tab-1 log). A 40 m grid at 1 m
# cells covers both without dwarfing the vehicles.
GRID_CELLS, GRID_SIZE = 40, 1.0


def grid():
    return {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid (1 m cells, 40 m)',
            'Enabled': True, 'Alpha': 0.25, 'Color': '160; 160; 160',
            'Cell Size': GRID_SIZE, 'Plane': 'XY', 'Plane Cell Count': GRID_CELLS,
            'Line Style': {'Line Width': 0.03, 'Value': 'Lines'},
            'Offset': {'X': 0, 'Y': 0, 'Z': 0}, 'Reference Frame': '<Fixed Frame>'}


def tf_display(frames_enabled=True):
    return {'Class': 'rviz_default_plugins/TF', 'Name': 'TF (world root, both drones)',
            'Enabled': True, 'Show Names': True, 'Show Axes': True,
            'Show Arrows': False, 'Marker Scale': 0.6, 'Update Interval': 0,
            'Frame Timeout': 30, 'Filter (blacklist)': '', 'Filter (whitelist)': '',
            'Tree': {}, 'Frames': {'All Enabled': frames_enabled}}


def cloud(name, topic, colour, size=0.05, style='Boxes', decay=0.0):
    return {'Class': 'rviz_default_plugins/PointCloud2', 'Name': name,
            'Enabled': True, 'Topic': {'Value': topic, 'Depth': 5,
                                       'Durability Policy': 'Volatile',
                                       'Reliability Policy': 'Reliable',
                                       'History Policy': 'Keep Last',
                                       'Filter size': 10},
            'Style': style, 'Size (m)': size, 'Alpha': 1.0,
            'Color Transformer': 'FlatColor', 'Color': colour,
            'Decay Time': decay, 'Position Transformer': 'XYZ',
            'Queue Size': 10, 'Selectable': True, 'Use rainbow': False}


def marker(name, topic):
    return {'Class': 'rviz_default_plugins/Marker', 'Name': name, 'Enabled': True,
            'Topic': {'Value': topic, 'Depth': 5, 'Durability Policy': 'Volatile',
                      'Reliability Policy': 'Reliable', 'History Policy': 'Keep Last',
                      'Filter size': 10},
            'Namespaces': {}, 'Queue Size': 100}


def marker_array(name, topic):
    d = marker(name, topic)
    d['Class'] = 'rviz_default_plugins/MarkerArray'
    return d


def occupancy(name, topic, colour_scheme='map'):
    return {'Class': 'rviz_default_plugins/Map', 'Name': name, 'Enabled': False,
            'Topic': {'Value': topic, 'Depth': 5, 'Durability Policy': 'Volatile',
                      'Reliability Policy': 'Reliable', 'History Policy': 'Keep Last',
                      'Filter size': 10},
            'Alpha': 0.7, 'Color Scheme': colour_scheme, 'Draw Behind': True,
            'Update Topic': {'Value': ''}, 'Use Timestamp': False, 'Binary representation': False}


def odom_trail(name, topic, colour):
    # Keep=100000 + a small Position Tolerance is what turns the Odometry
    # display into an accumulating TRAJECTORY rather than a single flickering
    # arrow. Covariance off: it clutters a publication figure.
    return {'Class': 'rviz_default_plugins/Odometry', 'Name': name, 'Enabled': True,
            'Topic': {'Value': topic, 'Depth': 100, 'Durability Policy': 'Volatile',
                      'Reliability Policy': 'Reliable', 'History Policy': 'Keep Last',
                      'Filter size': 10},
            'Keep': 100000, 'Position Tolerance': 0.02, 'Angle Tolerance': 0.05,
            'Shape': {'Value': 'Arrow', 'Color': colour, 'Alpha': 1.0,
                      'Shaft Length': 0.12, 'Shaft Radius': 0.02,
                      'Head Length': 0.06, 'Head Radius': 0.04},
            'Covariance': {'Value': False,
                           'Position': {'Alpha': 0.2, 'Color': '204; 51; 204', 'Scale': 1},
                           'Orientation': {'Alpha': 0.5, 'Color': '255; 255; 127',
                                           'Frame': 'Local', 'Offset': 1, 'Scale': 1},
                           'Orientation Frame': 'Local'}}


def pose_array(name, topic, colour):
    return {'Class': 'rviz_default_plugins/PoseArray', 'Name': name, 'Enabled': True,
            'Topic': {'Value': topic, 'Depth': 5, 'Durability Policy': 'Volatile',
                      'Reliability Policy': 'Reliable', 'History Policy': 'Keep Last',
                      'Filter size': 10},
            'Shape': 'Arrow (Flat)', 'Color': colour, 'Alpha': 1.0,
            'Arrow Length': 0.4, 'Head Length': 0.1, 'Head Radius': 0.05,
            'Shaft Length': 0.3, 'Shaft Radius': 0.02, 'Axes Length': 0.3,
            'Axes Radius': 0.03}


def group(name, displays, enabled=True):
    return {'Class': 'rviz_common/Group', 'Name': name, 'Enabled': enabled,
            'Displays': displays}


def viz_manager(displays, focal=(1.0, -1.75, 0.0), distance=11.0,
                yaw=1.57, pitch=0.95):
    """A SAVED, DETERMINISTIC CAMERA POSE - this is what makes figures
    reproducible across sessions. Focal point sits midway between the two
    drones' origins (delta at world 0,0 and ghost at world 0,-3.5)."""
    return {
        'Class': '',
        'Displays': displays,
        'Enabled': True,
        'Global Options': {'Background Color': '255; 255; 255',
                           'Fixed Frame': 'world',
                           'Frame Rate': 30,
                           'Default Light': True},
        'Name': 'root',
        'Tools': [{'Class': 'rviz_default_plugins/MoveCamera'},
                  {'Class': 'rviz_default_plugins/FocusCamera'},
                  {'Class': 'rviz_default_plugins/Measure', 'Line color': '128; 128; 0'}],
        'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
        'Value': True,
        # RViz only restores the saved pose when this block is EXACTLY the
        # shape it writes itself: the view must be named 'Current View', it
        # needs the stereo sub-block, and 'Saved' must be null rather than [].
        # With 'Name' set to anything else RViz silently discards Distance /
        # Yaw / Pitch / Focal Point and auto-frames the scene instead -
        # observed 2026-08-28 (config said 22 m, RViz opened at 4.99 m).
        # Target Frame is left empty on purpose: with it set to 'world' the
        # Orbit view re-centres on the frame and overrides Focal Point.
        'Views': {'Current': {'Class': 'rviz_default_plugins/Orbit',
                              'Name': 'Current View',
                              'Distance': distance,
                              'Enable Stereo Rendering': {
                                  'Stereo Eye Separation': 0.06,
                                  'Stereo Focal Distance': 1,
                                  'Swap Stereo Eyes': False,
                                  'Value': False},
                              'Focal Point': {'X': focal[0], 'Y': focal[1], 'Z': focal[2]},
                              'Focal Shape Fixed Size': False,
                              'Focal Shape Size': 0.05,
                              'Invert Z Axis': False,
                              'Near Clip Distance': 0.01,
                              'Pitch': pitch, 'Yaw': yaw,
                              'Target Frame': '<Fixed Frame>',
                              'Value': 'Orbit (rviz)'},
                  'Saved': None},
    }


def panels():
    return [{'Class': 'rviz_common/Displays', 'Name': 'Displays',
             'Help Height': 0, 'Tree Height': 700, 'Property Tree Widget': {
                 'Expanded': [], 'Splitter Ratio': 0.5}},
            {'Class': 'rviz_common/Views', 'Name': 'Views',
             'Expanded': [], 'Splitter Ratio': 0.5}]


def write(path, displays, **kw):
    cfg = {'Panels': panels(),
           'Visualization Manager': viz_manager(displays, **kw),
           'Window Geometry': {'Height': 1080, 'Width': 1920,
                               'Displays': {'collapsed': False},
                               'Views': {'collapsed': False},
                               'Hide Left Dock': False, 'Hide Right Dock': True,
                               'QMainWindow State': ''}}
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False, width=120)
    print('wrote', path)


# ------------------------------------------------------------- fleet overview
fleet = [grid(), tf_display()]
for did, name, col, peercol in DRONES:
    p = f'/d{did}'
    fleet.append(group(f'{name} (d{did})', [
        marker(f'{name} map (nvblox colour layer)', f'{p}/nvblox_node/color_layer_marker'),
        cloud(f'{name} ESDF cloud', f'{p}/nvblox_node/static_esdf_pointcloud', col),
        occupancy(f'{name} occupancy grid (2-D, off by default)',
                  f'{p}/nvblox_node/static_occupancy_grid'),
        odom_trail(f'{name} trajectory', f'{p}/visual_slam/tracking/odometry', col),
        marker_array(f'{name} frontiers', f'{p}/fis/frontier_markers'),
        pose_array(f'{name} frontier centroids', f'{p}/fis/frontier_centroids', col),
        pose_array(f'{name} frontier viewpoints', f'{p}/fis/frontier_viewpoints', col),
        marker_array(f'{name} exploration / claim markers', f'{p}/exploration/markers'),
        cloud(f'{name} peer map (shared nvblox)',
              f'{p}/nvblox_shared_node/static_esdf_pointcloud', peercol, size=0.04),
        marker(f'{name} workspace bounds', f'{p}/nvblox_node/workspace_bounds'),
    ]))
write(os.path.join(HERE, 'fleet_overview.rviz'), fleet)

# --------------------------------------------------------- single drone detail
# Defaults to ghost (d3); change the topic prefix to inspect delta.
did, name, col, peercol = DRONES[1]
p = f'/d{did}'
single = [grid(), tf_display(),
          group(f'{name} (d{did}) mapping', [
              marker(f'{name} map (nvblox colour layer)', f'{p}/nvblox_node/color_layer_marker'),
              cloud(f'{name} ESDF cloud', f'{p}/nvblox_node/static_esdf_pointcloud', col, size=0.04),
              occupancy(f'{name} occupancy grid (2-D)', f'{p}/nvblox_node/static_occupancy_grid'),
              marker(f'{name} ESDF slice bounds', f'{p}/nvblox_node/esdf_slice_bounds'),
              marker(f'{name} workspace bounds', f'{p}/nvblox_node/workspace_bounds'),
          ]),
          group(f'{name} planning', [
              odom_trail(f'{name} trajectory', f'{p}/visual_slam/tracking/odometry', col),
              marker_array(f'{name} frontiers', f'{p}/fis/frontier_markers'),
              pose_array(f'{name} frontier centroids', f'{p}/fis/frontier_centroids', col),
              pose_array(f'{name} frontier viewpoints', f'{p}/fis/frontier_viewpoints', col),
              marker_array(f'{name} exploration / claim markers', f'{p}/exploration/markers'),
          ]),
          group(f'{name} peer / shared map', [
              cloud(f'{name} peer map (shared nvblox)',
                    f'{p}/nvblox_shared_node/static_esdf_pointcloud', peercol, size=0.04),
              marker(f'{name} peer map colour layer',
                     f'{p}/nvblox_shared_node/color_layer_marker'),
          ])]
write(os.path.join(HERE, 'single_drone_detail.rviz'), single,
      focal=(1.0, -3.5, 0.0), distance=7.0)
