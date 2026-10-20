"""Schema errors remain actionable without repeating full model payloads."""
import copy
import json

import pytest
from python.dcm.agent_tools import validate_call


def rejected(tool, arguments):
    before = copy.deepcopy(arguments)
    with pytest.raises(ValueError) as raised:
        validate_call(tool, arguments)
    assert arguments == before
    return str(raised.value), json.loads(str(raised.value).split(': ', 1)[1])


def test_large_invalid_array_reports_actual_length_and_limit_without_dumping_values():
    message, result = rejected('detect', {'classes': ['person'] * 1000})
    assert result['validator'] == 'maxItems'
    assert result['path'] == ['classes']
    assert result['actual'] == {'type': 'array', 'length': 1000}
    assert result['expected'] == 80
    assert len(message) < 200 and 'person' not in message


def test_duplicate_classes_identified_without_replaying_array():
    message, result = rejected('detect', {'classes': ['person', 'car', 'person']})
    assert result['validator'] == 'uniqueItems'
    assert result['duplicate_values'] == ['person']
    assert result['actual']['length'] == 3


def test_unknown_enum_value_is_shown_without_all_supported_labels():
    message, result = rejected('detect', {'classes': ['not_a_detector_class']})
    assert result['validator'] == 'enum' and result['path'] == ['classes', 0]
    assert result['actual']['value'] == 'not_a_detector_class'
    assert result['expected']['allowed_value_count'] == 80
    assert 'allowed_values_reference' in result['expected'] and len(message) < 300


def test_object_reference_alternatives_remain_visible():
    message, result = rejected('orbit_target', {'standoff_m': 3, 'camera': 'none'})
    assert result['validator'] == 'oneOf'
    alternatives = result['expected']['alternatives']
    assert any('target_id' in a['required'] for a in alternatives)
    assert any('description' in a['required'] for a in alternatives)
    assert len(message) < 450


def test_tagged_union_reports_selected_action_missing_field_not_all_other_actions():
    message, result = rejected('execute_sequence', {'actions': [
        {'name': 'takeoff', 'arguments': {}}, {'name': 'arm', 'arguments': {}}]})
    assert result['validator'] == 'required'
    assert result['path'] == ['actions', 0, 'arguments']
    assert result['expected']['missing_required'] == ['target_altitude_agl_m']
    assert len(message) < 300


@pytest.mark.parametrize('arguments,validator,expected,actual', [
    ({'angle_deg': 30, 'rate_deg_s': 90}, 'maximum', 60, 90),
    ({'angle_deg': 'ninety'}, 'type', 'number', 'ninety'),
])
def test_numeric_bounds_and_types_preserve_constraint_and_actual_value(arguments, validator, expected, actual):
    message, result = rejected('yaw', arguments)
    assert result['validator'] == validator
    assert result['expected'] == expected and result['actual']['value'] == actual


def test_missing_and_unknown_fields_are_explicit():
    _, missing = rejected('takeoff', {})
    assert missing['expected']['missing_required'] == ['target_altitude_agl_m']
    _, extra = rejected('arm', {'fictional_switch': True})
    assert extra['expected']['unknown_keys'] == ['fictional_switch']


def diagnostic(schema, value):
    from jsonschema import Draft202012Validator
    from python.dcm.validation_diagnostics import schema_diagnostic
    return schema_diagnostic(next(Draft202012Validator(schema).iter_errors(value)))


def test_arbitrary_discriminators_report_only_selected_representation():
    schema = {'oneOf': [
        {'properties': {'format': {'const': 'vector'}, 'basis': {'const': 'cartesian'},
                        'x': {'type': 'number'}, 'y': {'type': 'number'}},
         'required': ['format', 'basis', 'x', 'y']},
        {'properties': {'format': {'const': 'vector'}, 'basis': {'const': 'polar'}},
         'required': ['format', 'basis', 'radius', 'angle']}]}
    result = diagnostic(schema, {'format': 'vector', 'basis': 'cartesian', 'x': 3})
    assert result['validator'] == 'required'
    assert result['expected']['missing_required'] == ['y']
    assert 'radius' not in json.dumps(result)


def test_ambiguous_discriminators_remain_explicit_choices():
    schema = {'oneOf': [
        {'properties': {'kind': {'const': 'vector'}, 'basis': {'const': 'local'}},
         'required': ['kind', 'basis', 'x']},
        {'properties': {'kind': {'const': 'vector'}, 'basis': {'const': 'global'}},
         'required': ['kind', 'basis', 'latitude']}]}
    result = diagnostic(schema, {'kind': 'vector'})
    assert result['validator'] == 'oneOf'
    assert {v['discriminators']['basis'] for v in result['expected']['alternatives']} == {'local', 'global'}


def test_boolean_discriminator_is_not_confused_with_numeric_one():
    schema = {'oneOf': [
        {'properties': {'tag': {'const': 1}}, 'required': ['tag', 'numeric_payload']},
        {'properties': {'tag': {'const': True}}, 'required': ['tag', 'boolean_payload']}]}
    result = diagnostic(schema, {'tag': True})
    assert result['expected']['missing_required'] == ['boolean_payload']


def test_selected_branch_reports_multiple_real_faults_without_other_branches():
    schema = {'oneOf': [
        {'properties': {'kind': {'const': 'record'}, 'payload': {'type': 'string'}},
         'required': ['kind', 'payload'], 'additionalProperties': False},
        {'properties': {'kind': {'const': 'sequence'}}, 'required': ['kind', 'items']}]}
    result = diagnostic(schema, {'kind': 'record', 'extra': 3})
    faults = [result, *result.get('related_errors', [])]
    assert any(f['expected'].get('missing_required') == ['payload'] for f in faults)
    assert any(f['expected'].get('unknown_keys') == ['extra'] for f in faults)
    assert 'items' not in json.dumps(result)
