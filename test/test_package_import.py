from robotx_mission_state_machine.state_machine import MainState


def test_all_eight_main_states_are_defined():
    assert {int(state) for state in MainState} == set(range(8))
