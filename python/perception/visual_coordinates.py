"""Explicit model-input geometry; never infer a coordinate frame from a box."""
import base64
import hashlib
import io
import math


def grounding_image_policy(artifact):
    policy = artifact.get('grounding_image')
    if policy is None:
        return None
    if not isinstance(policy, dict) or set(policy) != {'alignment', 'minimum_tokens', 'maximum_tokens'}:
        raise ValueError('grounding_image needs alignment, minimum_tokens and maximum_tokens')
    alignment, minimum, maximum = (policy[k] for k in ('alignment', 'minimum_tokens', 'maximum_tokens'))
    if (any(type(v) is not int for v in (alignment, minimum, maximum)) or
            not 1 <= alignment <= 128 or not 1 <= minimum <= maximum <= 4096):
        raise ValueError('Invalid grounding image resolution policy')
    return dict(policy)


def prepare_grounding_image(path, artifact):
    """Align and bound pixels before encoding, retaining the exact inverse map.

    The rounding matches the pinned mtmd smart-resize implementation. Passing
    an already aligned input within the same token limits avoids a hidden
    second resize. Camera intrinsics still describe the original sensor frame.
    """
    source = path.read_bytes()
    policy = grounding_image_policy(artifact)
    if policy is None:
        return base64.b64encode(source).decode(), None
    from PIL import Image
    with Image.open(io.BytesIO(source)) as original:
        width, height = original.size
        a = policy['alignment']
        w = max(a, math.floor(width / a + .5) * a)
        h = max(a, math.floor(height / a + .5) * a)
        low, high = (policy[k] * a * a for k in ('minimum_tokens', 'maximum_tokens'))
        if w * h > high:
            scale = math.sqrt(width * height / high)
            w, h = (max(a, math.floor(size / scale / a) * a) for size in (width, height))
        elif w * h < low:
            scale = math.sqrt(low / (width * height))
            w, h = (math.ceil(size * scale / a) * a for size in (width, height))
        if not low <= w * h <= high:
            raise ValueError('Image aspect ratio cannot satisfy the declared resolution policy')
        image = original.convert('RGB').resize((w, h), Image.Resampling.BICUBIC)
        buffer = io.BytesIO()
        image.save(buffer, format='PNG')
    encoded = buffer.getvalue()
    transform = {'source_width': width, 'source_height': height,
                 'model_width': w, 'model_height': h,
                 'coordinate_frame': 'model_input_pixels',
                 'model_image_sha256': hashlib.sha256(encoded).hexdigest(), **policy}
    return base64.b64encode(encoded).decode(), transform


def source_pixel_box(box, transform):
    if box is None or transform is None:
        return box
    return [value * transform[source] / transform[model] for value, source, model in zip(
        box, ('source_width', 'source_height') * 2, ('model_width', 'model_height') * 2)]
