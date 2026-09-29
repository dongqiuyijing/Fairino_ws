from copy import deepcopy
import pytest
from fr3_teleop.actual_feedback_guard import check_feedback, check_displacement


def health():
    v=dict(actual_joint_pos=[0.]*6,actual_TCP_pos=[0.]*6,rbt_enable_state=[1])
    v.update({k:[0] for k in ('main_code','sub_code','emergency_stop','collision_state','safety_stop0_state','safety_stop1_state')})
    return dict(scope='actual-feedback-only',error=None,writer_mono_ns=1000000000,arms={
        ip:dict(actual_fields_valid=True,valid=False,target_available=False,
                robot_time_changed=True,sample_mono_ns=1000000000,values=deepcopy(v))
        for ip in ('192.168.58.2','192.168.58.5')})


def test_actual_only_without_target_is_accepted():
    assert len(check_feedback(health(),1050000000))==2


@pytest.mark.parametrize('change', ['stale','frozen','nan','fault','disabled','missing'])
def test_necessary_checks_remain(change):
    h=health(); e=h['arms']['192.168.58.2']
    if change=='stale': e['sample_mono_ns']=0
    if change=='frozen': e['robot_time_changed']=False
    if change=='nan': e['values']['actual_TCP_pos'][0]=float('nan')
    if change=='fault': e['values']['main_code']=[1]
    if change=='disabled': e['values']['rbt_enable_state']=[0]
    if change=='missing': del e['values']['safety_stop0_state']
    with pytest.raises(ValueError): check_feedback(h,1050000000)


def test_direction_and_b_stillness():
    base=check_feedback(health(),1050000000); current=deepcopy(base)
    current['192.168.58.2']['actual_TCP_pos'][0]=5
    assert check_displacement(base,current,True)['along_mm']==5
    current['192.168.58.2']['actual_TCP_pos'][0]=-1
    with pytest.raises(ValueError): check_displacement(base,current,True)
    current=deepcopy(base);current['192.168.58.5']['actual_TCP_pos'][0]=.3
    with pytest.raises(ValueError): check_displacement(base,current,True)
