"""Compass/view parameters over measured yaw; no mission-language parsing."""
import math

COMPASS_HEADINGS = {'north':0, 'northeast':45, 'east':90, 'southeast':135,
                    'south':180, 'southwest':225, 'west':270, 'northwest':315}
VIEW_SCHEMA = {'anyOf':[
    {'enum':['current',*COMPASS_HEADINGS]},
    {'type':'number','minimum':0,'exclusiveMaximum':360}],
    'description':'Camera yaw at the current vehicle position: compass name or absolute degrees; current retains measured heading. Use compass values for explicit viewing directions. Camera tilt is fixed.'}


def resolve_view(view, state):
    if isinstance(view,str):
        if view in COMPASS_HEADINGS:return COMPASS_HEADINGS[view]
        if view=='current':
            yaw=state.get('attitude',{}).get('yaw_rad')
            if type(yaw) in (int,float) and math.isfinite(yaw):return math.degrees(yaw)%360
            raise ValueError('Current view requires measured yaw')
    elif type(view) in (int,float) and math.isfinite(view) and 0<=view<360:
        return view
    raise ValueError('View requires a compass direction, current, or absolute degrees in [0,360)')


def canonical_view_arguments(name, arguments):
    if not isinstance(arguments,dict):return arguments
    result=dict(arguments)
    key='heading_deg' if name=='face_heading' else 'view_heading_deg'
    if name in ('face_heading','localize_target','inspect_scene','detect') and key in result:
        if 'view' in result:raise ValueError('Supply view once, not both heading aliases')
        result['view']=result.pop(key)
    return result
