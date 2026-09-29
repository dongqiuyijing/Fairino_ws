"""Pure offline-testable checks for the explicitly limited actual-only trial.

Software abort thresholds are NOT independent protective limits.
"""
import math


def check_feedback(health, now_ns, maximum_age_ms=100):
    if health.get('scope') != 'actual-feedback-only' or health.get('error'):
        raise ValueError('feedback monitor invalid')
    if not 0 <= now_ns-health.get('writer_mono_ns', 0) <= maximum_age_ms*1e6:
        raise ValueError('feedback writer stale')
    result={}
    for ip in ('192.168.58.2','192.168.58.5'):
        e=health.get('arms',{}).get(ip,{})
        if not e.get('actual_fields_valid') or e.get('robot_time_changed') is not True:
            raise ValueError(f'{ip}: actual values invalid or robot time not advancing')
        if not 0 <= now_ns-e.get('sample_mono_ns',0) <= maximum_age_ms*1e6:
            raise ValueError(f'{ip}: actual feedback stale')
        v=e['values']
        for name in ('actual_joint_pos','actual_TCP_pos'):
            if len(v.get(name,[])) != 6 or not all(math.isfinite(x) for x in v[name]):
                raise ValueError(f'{ip}: bad {name}')
        for name in ('main_code','sub_code','emergency_stop','collision_state','safety_stop0_state','safety_stop1_state'):
            if v.get(name) != [0]:
                raise ValueError(f'{ip}: unsafe or missing {name}')
        if v.get('rbt_enable_state') != [1]:
            raise ValueError(f'{ip}: robot not enabled')
        result[ip]=v
    return result


def tool_x_direction(pose):
    # Fixed-axis XYZ RPY convention: first column of Rz(rz) Ry(ry) Rx(rx).
    ry,rz=map(math.radians,pose[4:6])
    return (math.cos(rz)*math.cos(ry),math.sin(rz)*math.cos(ry),-math.sin(ry))


def check_displacement(baseline, current, moving=False):
    a,b='192.168.58.2','192.168.58.5'
    qa,qb=baseline[a]['actual_joint_pos'],baseline[b]['actual_joint_pos']
    if max(abs(x-y) for x,y in zip(qb,current[b]['actual_joint_pos'])) > 0.05:
        raise ValueError('Arm B joint movement')
    if math.dist(baseline[b]['actual_TCP_pos'][:3],current[b]['actual_TCP_pos'][:3]) > 0.2:
        raise ValueError('Arm B TCP movement')
    p0,p=baseline[a]['actual_TCP_pos'],current[a]['actual_TCP_pos']
    d=[y-x for x,y in zip(p0[:3],p[:3])]
    along=sum(x*y for x,y in zip(d,tool_x_direction(p0)))
    lateral=math.sqrt(max(0,sum(x*x for x in d)-along*along))
    if moving:
        if math.dist(p0[:3],p[:3])>7 or along<-.3 or lateral>.8:
            raise ValueError('Arm A distance/direction deviation')
        if max(abs(x-y) for x,y in zip(qa,current[a]['actual_joint_pos']))>1.0:
            raise ValueError('Arm A joint excursion over 1 degree')
        if max(abs((y-x+180)%360-180) for x,y in zip(p0[3:],p[3:]))>.5:
            raise ValueError('Arm A orientation excursion')
    elif math.dist(p0[:3],p[:3])>.2:
        raise ValueError('Arm A not stationary during preflight')
    return dict(along_mm=along,lateral_mm=lateral,distance_mm=math.sqrt(sum(x*x for x in d)))
