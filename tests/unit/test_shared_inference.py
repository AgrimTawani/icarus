"""Shared accelerator lifecycle, including failure/cancellation races."""
import dataclasses
from unittest.mock import Mock, patch
import pytest
from test_llama_runtime import descriptor
from python.dcm.llama_runtime import LlamaCppRuntime


def runtime(layers):
    return LlamaCppRuntime(dataclasses.replace(descriptor(), gpu_layers=layers),
                           binary=__file__, verify_artifact=False)


def test_gpu_lease_releases_owned_model_before_vision_and_restarts_lazily():
    model = runtime('auto')
    model.process = Mock()
    with patch.object(model, 'stop', side_effect=lambda: setattr(model, 'process', None)) as stop:
        with model.visual_inference() as layers:
            assert layers == 99
            stop.assert_called_once()
            assert model.process is None
            with pytest.raises(RuntimeError, match='previous model inference'):
                model._chat('not allowed to overlap')
    # No eager restart while vision may still be winding down; the next chat
    # uses the existing lazy-start path in _chat_owned.
    assert model.process is None
    assert model._inference_lock.acquire(blocking=False)
    model._inference_lock.release()


def test_cpu_policy_is_preserved_without_unloading_cpu_model():
    model = runtime(0)
    with patch.object(model, 'stop') as stop:
        with model.visual_inference() as layers:
            assert layers == 0
        stop.assert_not_called()


def test_visual_failure_releases_lock_without_overlapping_model_recovery():
    model = runtime('auto')
    with patch.object(model, 'stop'):
        with pytest.raises(RuntimeError, match='vision failed'):
            with model.visual_inference():
                raise RuntimeError('vision failed')
    assert model._inference_lock.acquire(blocking=False)
    model._inference_lock.release()


def test_pending_model_inference_prevents_visual_accelerator_takeover():
    model = runtime('auto')
    model._inference_lock.acquire()
    try:
        with patch.object(model, 'stop') as stop, pytest.raises(RuntimeError, match='previous model inference'):
            with model.visual_inference():
                pass
        stop.assert_not_called()
    finally:
        model._inference_lock.release()
