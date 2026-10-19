"""Concise factual schema errors without replaying entire arguments or schemas."""
import json


def _actual(value):
    if isinstance(value, list):
        return {'type': 'array', 'length': len(value)}
    if isinstance(value, dict):
        return {'type': 'object', 'keys': list(value)[:12], 'key_count': len(value)}
    if isinstance(value, str) and len(value) > 160:
        return {'type': 'string', 'length': len(value), 'value_prefix': value[:160]}
    return {'type': ('null' if value is None else 'boolean' if isinstance(value, bool)
                     else 'string' if isinstance(value, str) else 'number'), 'value': value}


def schema_diagnostic(error):
    """Preserve the failing path, constraint and relevant actual measurement."""
    validator = error.validator
    value = error.instance
    expected = error.validator_value
    if validator in ('oneOf', 'anyOf'):
        # For a tagged action union, report that action's real argument failure,
        # rather than every other action's unrelated discriminator mismatch.
        if isinstance(value, dict) and 'name' in value:
            for index, branch in enumerate(expected):
                if branch.get('properties', {}).get('name', {}).get('const') == value['name']:
                    relevant = [child for child in error.context
                                if child.schema_path and child.schema_path[0] == index]
                    if relevant:
                        return schema_diagnostic(relevant[0])
        alternatives = []
        for branch in expected[:16]:
            variant = {'required': branch.get('required', [])}
            name = branch.get('properties', {}).get('name', {}).get('const')
            if name is not None:
                variant['name'] = name
            alternatives.append(variant)
        expected = {'alternatives': alternatives, 'alternative_count': len(expected)}
    elif validator == 'required':
        expected = {'missing_required': [key for key in expected if key not in value]}
    elif validator == 'enum':
        expected = {'allowed_values_reference': 'tool parameter schema enum',
                    'allowed_value_count': len(expected)}
    elif validator == 'additionalProperties' and isinstance(value, dict):
        unknown = [key for key in value if key not in error.schema.get('properties', {})]
        expected = {'additional_properties': False, 'unknown_keys': unknown[:12],
                    'unknown_key_count': len(unknown)}
    diagnostic = {'validator': validator, 'path': list(error.absolute_path),
                  'expected': expected, 'actual': _actual(value)}
    if validator == 'uniqueItems' and isinstance(value, list):
        seen, duplicates = set(), []
        for item in value:
            encoded = json.dumps(item, sort_keys=True, ensure_ascii=False)
            if encoded in seen and len(duplicates) < 8 and item not in duplicates:
                duplicates.append(item if not isinstance(item, (dict, list)) else _actual(item))
            seen.add(encoded)
        diagnostic['duplicate_values'] = duplicates
    return diagnostic


def format_schema_error(tool, error):
    return f'{tool}: ' + json.dumps(schema_diagnostic(error), separators=(',', ':'), ensure_ascii=False)
