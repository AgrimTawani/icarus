"""Concise factual schema errors without replaying entire arguments or schemas."""
import json
from jsonschema import Draft202012Validator


def _actual(value):
    if isinstance(value, list):
        return {'type': 'array', 'length': len(value)}
    if isinstance(value, dict):
        return {'type': 'object', 'keys': list(value)[:12], 'key_count': len(value)}
    if isinstance(value, str) and len(value) > 160:
        return {'type': 'string', 'length': len(value), 'value_prefix': value[:160]}
    return {'type': ('null' if value is None else 'boolean' if isinstance(value, bool)
                     else 'string' if isinstance(value, str) else 'number'), 'value': value}


def _union_candidates(value, branches):
    """Select only with explicit constant tags, never guessed intent."""
    if not isinstance(value, dict):
        return None
    candidates = []
    for index, branch in enumerate(branches):
        if not isinstance(branch, dict):
            continue
        tags = {key: node['const'] for key, node in branch.get('properties', {}).items()
                if isinstance(node, dict) and 'const' in node and key in value}
        if tags and all(Draft202012Validator({'const': tag}).is_valid(value[key])
                        for key, tag in tags.items()):
            candidates.append((index, len(tags)))
    if not candidates:
        return None
    score = max(weight for _, weight in candidates)
    return [index for index, weight in candidates if weight == score]


def schema_diagnostic(error):
    """Preserve the failing path, constraint and relevant actual measurement."""
    validator = error.validator
    value = error.instance
    expected = error.validator_value
    if validator in ('oneOf', 'anyOf'):
        # Any explicit constant discriminator can identify a union branch.
        # Required fields and numeric constraints remain unchanged; only the
        # diagnostic is narrowed to the caller's actual representation.
        selected = _union_candidates(value, expected)
        if selected and len(selected) == 1 and error.context:
            relevant = [child for child in error.context
                        if child.schema_path and child.schema_path[0] == selected[0]]
            if relevant:
                result = schema_diagnostic(relevant[0])
                extra = [schema_diagnostic(child) for child in relevant[1:4]]
                if extra:
                    result.setdefault('related_errors', []).extend(extra)
                return result
        variants = [(index, branch) for index, branch in enumerate(expected)
                    if not selected or index in selected]
        alternatives = []
        for _, branch in variants[:16]:
            branch = branch if isinstance(branch, dict) else {}
            variant = {'required': branch.get('required', [])}
            tags = {key: node['const'] for key, node in branch.get('properties', {}).items()
                    if isinstance(node, dict) and 'const' in node}
            if tags:
                variant['discriminators'] = tags
            alternatives.append(variant)
        expected = {'alternatives': alternatives, 'alternative_count': len(variants)}
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
