# DCM Model and Quantization Selection

This records *why* the first Phase 11 model artifact was chosen, so the choice
can be audited and revisited rather than rediscovered. It covers the
development host in use today and the intended Jetson target. It does not
claim any model has been evaluated: at the time of writing no model has been
connected, and the only runtime is the `mock-no-action` stub described in
[`DCM-OBSERVE-V1.md`](DCM-OBSERVE-V1.md).

Model choice is configuration, not code. Every artifact below is selected
through the `model:` block in the Phase 11 config and must be pinned by
absolute path plus SHA-256 checksum.

## Selection criteria

The DCM emits exactly one JSON object naming one of six actions with at most
one numeric argument. That is a low-entropy task at `temperature: 0.0`, so raw
model capability is not the binding constraint. Three other things are:

1. **Numeric argument fidelity.** The observer validates
   `target_altitude_agl_m` within `[0.5, 10]`. Quantization degrades numeric
   handling before it degrades fluency, and a drifted altitude is recorded as
   an invalid action.
2. **Not confounding the experiment.** Phase 12 measures invalid-action rate
   and argument validity. If the quantization is aggressive enough to cause
   errors, those metrics measure the quantizer, not the model, and the
   model comparison the phase exists to produce becomes meaningless.
3. **Reproducibility.** A pinned checksum is only meaningful if the artifact is
   a deterministic function of the published weights.

## Development host (current)

Measured on the machine in use, 2026-09-19:

| Component | Value |
| --- | --- |
| GPU | NVIDIA RTX 4050 Laptop, 6141 MiB VRAM |
| VRAM already used by the desktop session | 576 MiB |
| VRAM actually available | ~5.07 GiB |
| System RAM | 15 GiB total, ~8.3 GiB available |
| CPU | AMD Ryzen 7 7435HS, 16 threads |
| Free disk | 384 GiB |

VRAM, not disk, is the constraint. Disk is irrelevant at this scale, so model
artifacts are stored on the internal NVMe. External or removable storage is
explicitly rejected: llama.cpp memory-maps the weights, so bus speed becomes a
permanent inference cost rather than a one-time copy cost.

### The KV cache is part of the budget

Qwen3-4B has 36 layers, 8 key/value heads and head dimension 128, so an
f16 KV cache costs:

```
2 (K and V) x 36 layers x 8 kv-heads x 128 dim x 2 bytes = 144 KiB per token
                                       8192 tokens = 1.13 GiB
```

That gigabyte is frequently omitted from "will it fit" estimates and is the
reason the largest artifact that fits on paper does not fit in practice.

### Measured budget for Qwen3-4B-Instruct-2507

File sizes are the real published artifact sizes from
`bartowski/Qwen_Qwen3-4B-Instruct-2507-GGUF`, not estimates. The overhead
column assumes the 8192-token f16 KV cache above plus ~0.4 GiB of compute and
context buffers.

| Quantization | File | Total with 8k KV | Spare of 5.07 GiB |
| --- | --- | --- | --- |
| Q4_K_M | 2.33 GiB | 3.86 GiB | 1.21 GiB |
| **Q5_K_M (selected)** | **2.69 GiB** | **4.22 GiB** | **0.85 GiB** |
| Q6_K | 3.08 GiB | 4.61 GiB | 0.46 GiB |
| Q8_0 | 3.99 GiB | 5.52 GiB | does not fit |

### Decision

**Primary: `Qwen3-4B-Instruct-2507-Q5_K_M`.** It is the highest-fidelity
artifact that retains meaningful headroom. Q6_K fits today but its 0.46 GiB
margin is consumed by ordinary desktop use or any growth in the observation
window. Q5_K_M keeps quantization damage well clear of the numeric-argument
threshold, so Phase 12 metrics reflect the model.

**Secondary: `Qwen3-4B-Instruct-2507-Q4_K_M`.** Kept for two reasons. It is
the quantization already named in the Phase 11 roadmap config, and it is the
realistic Jetson deployment candidate. It also provides the cheapest possible
first test of the evaluation harness: Q5 versus Q4 of identical weights is an
A/B with a strong prior, so if the harness cannot separate them, that is
evidence about the harness's sensitivity to collect *before* trusting it to
rank different model families. Switching between the two must require only a
config change, which exercises the Phase 11 exit gate directly.

**The non-thinking Instruct release is required.** Qwen3 also publishes a
thinking variant whose reasoning preamble violates the strict single-JSON-object
contract enforced by `validate_proposal`. Every proposal would be recorded as
invalid.

### Rejected, with reasons

| Rejected | Reason |
| --- | --- |
| Q8_0, F16 | Exceed available VRAM once the KV cache is counted |
| Q3_K_*, Q2_K, IQ1/IQ2 | Numeric degradation lands exactly on the validated altitude argument |
| Q4_0, Q4_1, Q5_0, Q5_1 | Legacy formats; same file size as K-quants at lower quality |
| IQ4_XS, IQ4_NL and other imatrix quants | Quality depends on an undocumented calibration corpus chosen by the uploader. Not a quality objection — a reproducibility one. A pinned checksum should identify an artifact derived deterministically from published weights |
| Ollama as the runtime | Manages a tag-keyed blob store rather than a pinned file, and interposes a daemon between the controller and the model process, preventing the hard decision deadline Phase 11 requires |

## Jetson target: AGX Orin 64GB

**Confirmed 2026-09-20: Jetson AGX Orin 64GB.** The Orin Nano section below is
retained only as a contrast, since it was considered earlier; it is not the
target.

Two properties change the analysis relative to the development host.

**Memory is unified.** There is no separate VRAM pool. The model competes with
the operating system *and* with the perception engine, obstacle map, MAVLink
gateway and Drone API services — all of which must keep running while the DCM
is thinking. The budget is therefore not "total RAM" but what remains after the
existing autonomy stack is resident.

**Token generation is bandwidth-bound, not compute-bound.** A useful ceiling is
memory bandwidth divided by model size. This makes advertised TOPS figures a
poor predictor of decision latency.

| Module | Memory | Bandwidth (approx) |
| --- | --- | --- |
| Orin Nano 8GB | 8 GB unified | ~68 GB/s (~102 GB/s in Super mode) |
| Orin NX 16GB | 16 GB unified | ~102 GB/s |
| AGX Orin 64GB | 64 GB unified | ~204.8 GB/s |

Confirm these against the datasheet for the module actually purchased.

### Not the target: Orin Nano 8GB, for contrast

Use **Qwen3-4B-Instruct-2507-Q4_K_M** (2.33 GiB). After the OS and the resident
autonomy stack, roughly 4–4.5 GB is realistically available; Q4_K_M plus a
1.13 GiB KV cache fits with little to spare. Q5_K_M is likely too tight once
perception is running. If memory is short, quantize the KV cache
(`--cache-type-k q8_0 --cache-type-v q8_0`, roughly halving it) before dropping
the weight quantization further. At ~68 GB/s the theoretical generation ceiling
is around 25–27 tok/s, so realistically 15–20 tok/s. Decision latency must be
budgeted against that, not against development-host timings.

### The target: AGX Orin 64GB

Memory stops being the constraint and the choice becomes a latency-versus-
capability trade-off worth measuring rather than assuming.

| Candidate | File | 8k KV cache | Notes |
| --- | --- | --- | --- |
| Qwen3-4B-Instruct Q5_K_M | 2.69 GiB | 1.13 GiB | Same artifact as the dev host; the latency baseline |
| Qwen3-14B Q5_K_M | 9.79 GiB | 1.25 GiB | Capability candidate |
| Qwen3-14B Q6_K | 11.29 GiB | 1.25 GiB | Comfortable at 64 GB |

Qwen3-14B has 40 layers and the same 8 KV heads and head dimension 128, giving
160 KiB per token, or 1.25 GiB at 8192 tokens.

Note that the AGX Orin's ~204.8 GB/s is close to the development laptop's
~192 GB/s. Its advantage over this host is **capacity, not speed**: it can hold
a substantially larger model, but it will not run the *same* 4B model much
faster. A 14B model is roughly 3.6x the weights of a 4B at the same
quantization and should be expected to generate roughly 3.6x slower. Whether
that latency is acceptable inside a control loop is a Phase 12 measurement, not
an assumption to make here.

### Storage

The official NVIDIA AGX Orin 64GB Developer Kit ships with **64 GB of eMMC 5.1
and no SSD**. NVIDIA sells no SSD variants; the bundles that include one are
third-party, and they generally fit a 1 TB drive. The carrier board has one
M.2 Key M slot, PCIe Gen4 x4, 2280 form factor, so a drive can be added later.

An SSD is required, but a large one is not, and the reason is worth recording
because it is counter-intuitive. Episodes are small: the corpus on the
development host measures **5.3 MB for 11 episodes**, about 600 KB each, so ten
thousand flights would occupy roughly 6 GB. Flight data is not the storage
driver.

What consumes space is the toolchain and the model artifacts:

| Item | Size |
| --- | --- |
| JetPack and CUDA | ~25-30 GB (reported, not measured here) |
| llama.cpp build | 0.75 GB (measured) |
| Qwen3-4B Q4_K_M | 2.33 GB |
| Qwen3-14B Q6_K | 11.29 GB |
| Icarus build and runtime | ~0.2 GB |
| 10,000 episodes | ~6 GB |

On 64 GB eMMC, JetPack alone leaves roughly 34 GB. A single 4B model fits. A
14B model is tight. **Holding several model artifacts at once does not fit**,
and Phase 12 requires comparing at least two models plus a deterministic
baseline, so the eMMC cannot serve the evaluation work even though it could
serve a single deployed model.

Recommendation: buy the plain official developer kit without a bundled SSD, and
add a **512 GB M.2 2280 NVMe Gen4 x4** drive. That is ample for several model
artifacts, the toolchain and a corpus far larger than anything this project has
produced. The commonly recommended 1 TB is sized for datasets and video, which
this project keeps elsewhere.

## Required provenance

Whatever is selected, the Phase 11 adapter must record and pin:

- Absolute artifact path and SHA-256 checksum, verified at load so a truncated
  or swapped file fails loudly rather than silently changing results
- Model ID, family, quantization and publishing repository
- Runtime and its build revision
- Prompt version, context length and sampling settings
- Per-decision latency, and timeouts enforced at the process boundary

Artifacts live outside the repository. They are never committed to git.

## Status

Nothing in this document is an evaluation result. It records a pre-registered
selection and its rationale so that later measurements can be attributed to the
model rather than to an unexamined setup choice. The Jetson section is planning
only; no artifact has been run on Jetson hardware, and hardware autonomy
remains gated behind Phase 13.
