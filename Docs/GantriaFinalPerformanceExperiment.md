# Final soft-body performance candidate: source and evidence

## Source boundary

The frozen qualified B native source is
`d8b980a9f4c997ff9b715d6f17d48a15ecc6de37`. The final experimental
native source is `7e59df5fd7f3e23504f2e696e2b095bf706b26b4`. Five bounded
functional changes sit between them: pooled temporary graph adjacency,
incremental frontier scores during graph grouping, solver sweep/mode read
hoisting, an opt-in vertex-group size, and first-sweep retained-multiplier
clearing. The diff touches seven files (+620/−97 lines); the three Jolt runtime
files are `SoftBodyMotionProperties.cpp` and `SoftBodySharedSettings.cpp/.h`.
The remaining changes are native tests, their CMake registration, and a
qualification-script correction. Windows Release, Linux Release, and Linux
ASan/UBSan native unit CI passed at `7e59df5f`. A compile-only generic Engine
provider check also passed against pristine upstream Jolt `e77f1755` with all
fork features disabled. The worker then restored the final `7e59df5f` source;
this compatibility check is not a pristine physical qualification.

The original `Optimize(results)` symbol and `Optimize()` wrapper still select
256 vertices per group. The new `bool Optimize(results, batch)` accepts only
128, 256, 512, or 1024. It rejects another value before changing either the
settings or remap results. Read-only accessors report group count and final
nonparallel edge/volume counts. No creation-settings field or stream-layout
change was added by this performance patch set. Choosing a nondefault group
size changes constraint order, update groups, and the serialized optimized
shared-settings graph; old settings streams still need the external versioned
wrapper already described for frozen B. Runtime collision batching,
U2/C8/I5/N80/Q2 solver caps, material/contact code, and the default group
choice are unchanged.

The Engine use of group size 1024 remains a private experiment for cloth with
at least 16,384 vertices. Smaller cloth and rubber keep the default 256 group
size. This candidate has not changed the production Engine.

## Why the trial targeted grouping

The external Jolt scope profiler attributed 7.305 s of 7.880 s inclusive
`SoftBodySimulate` work to `ApplyEdgeConstraints` over 40 frames of one
16K cloth at one worker. At eight workers, `ApplyEdgeConstraints` counted
9.394 s inclusive worker work, while the broader `SoftBodySimulate` scope
counted 22.580 s. These nested sums are neither wall time nor a critical
path. The existing `FinishIteration` scope, which executes after all parallel
groups and before the next sweep, counted 6,400 calls and 1.519 s inclusive
work at eight workers. It includes the final nonparallel group, contact
velocity work on final sweeps, and the next integration. This measured serial
tail motivated a bounded group-size trial; it does not attribute all of the
scope to crossing edges.

`Optimize` places a constraint spanning two vertex groups in the final
nonparallel group. A separate instrumented admission pass found 7,406 serial
edges with 33 update groups at size 512 and 3,810 serial edges with 17 groups
at size 1024 for the 16K cloth. The larger group reduces the serial edge
count but leaves fewer parallel groups. The source does not establish an
optimal batch size outside the measured scene and worker caps.

A final diagnostic profile of the exact `7e59df5f` native and guarded Engine
candidate compared the earlier 256-group hoist source over the same 40-frame
16K-cloth W8 case. Edge-scope calls fell from 416,000 to 108,800, nested
`FinishIteration` worker work from 1,519.3 to 892.8 ms (−41.2%), barrier work
from 2,815.2 to 2,249.6 ms (−20.1%), and integration work from 344.8 to
221.0 ms (−35.9%). Edge work fell only 3.6%, from 8,978.9 to 8,654.2 ms.
Rubber4K retained 108,800 edge-scope calls and its `FinishIteration` work
rose 3.2%; it still uses 256-group optimization. All 40 final cloth W8 state
hashes matched both the 600-frame C long-run prefix and the final postguard
trace; all 40 rubber W8 hashes matched its C long-run prefix. The profile
includes the pooled-graph, frontier, and lazy-clear changes, so these work
deltas do not isolate grouping. They are diagnostic nested worker sums, never
wall-time rankings or a critical-path measurement.

## Unprofiled timing evidence

The frozen B and final C Provider and Engine matrices each completed their
planned processes. Each matrix was counterbalanced against A in a separate
cohort; the B/C percentages below align the same scene, cap, tail, and repeat
across cohorts and are not direct counterbalanced B/C pairs. The local reducer
checked stdout/stderr SHA-256,
exit status, measured and warmup frame counts, an uninstrumented Provider
marker or unprofiled Engine CMake cache, and the frozen Jolt N80/Q2 caps.
Numbers below are medians of process P50s; percentages are medians of the
aligned B/C changes. They are timing evidence, not a physical winner.

| 16K cloth, no collider | Frozen B | Final C | Aligned P50 change |
| --- | ---: | ---: | ---: |
| Provider W8 cold | 67.73 ms | 55.37 ms | −18.24% |
| Provider W8 warm | 67.99 ms | 55.88 ms | −17.80% |
| Full Engine physics call W8 | 68.57 ms | 56.22 ms | −18.01% |
| Provider W1 cold | 187.88 ms | 189.29 ms | +0.76% |
| Provider W1 warm | 186.73 ms | 190.54 ms | +2.04% |
| Full Engine physics call W1 | 186.40 ms | 190.33 ms | +2.11% |

Provider total and full Engine physics-call timings come from different
executables and measurement boundaries. They independently show the W8
direction; their difference is not an Engine overhead measurement. Per-process
P50/P95/P99, CPU, working set, admission, retirement, and aligned deltas for
all scene/worker/tail cells are in the `PerformanceReduction` artifact. Its
`Review.md` displays the full native and Engine B/C matrix and XPBD timing
controls. The Provider harness has no matched cross-backend quality envelope,
so it declares no overall A/C winner.

Two isolated group-size comparisons with three fresh-process matched pairs
each found a median W8 P50 change of −2.45% at size 512 and −20.62% at size
1024 versus frozen B. At W1, the size-1024 median was +0.89%. Every nondefault
cloth terminal trajectory hash differed from B. The grouping result therefore
requires numerical and physical qualification, not a source-level exactness
assumption.

A separate final-guard confirmation directly counterbalanced B and C at the
same 16K-cloth W8 cell after Engine cache, joint, and sleep guards. The three
uninstrumented process P50 pairs were 69.739→56.108 ms (−19.55%),
70.299→55.460 ms (−21.11%), and 70.128→55.833 ms (−20.38%). Their median
paired change was −20.38%, and the changed terminal hashes remained subject
to the same physical gate. These direct pairs confirm the large-cloth W8
direction without treating separate matrix cohorts as paired runs.

The lazy multiplier-clearing change was tested separately at the default 256
group size. It removes bulk zero fills before each temporal integration.
Every nondegenerate edge or volume first sweep overwrites its multiplier; a
constraint skipped on that sweep because its denominator is below `1e-12`
now clears only its own multiplier. This preserves the old zero if a later
sweep can solve it. Tests cover edge and volume skip/recovery after a previous
nonzero multiplier and exact states across consecutive N2/Q2 integrations.
Three unprofiled pairs on 16K cloth W8 showed a −2.20% median matched P50
change, all three faster and all terminal state hashes exact. W1 cloth signs
were mixed. The three-pair rubber4K W8 sample was mixed with a +2.19% median;
five further rubber W8 pairs at the final compiled large-cloth setting had a
−1.56% median with mixed signs. Rubber4K uses default 256 in both binaries,
so these rubber comparisons isolate the lazy-clear source effect. No general
rubber speedup follows.

## Physical domain and adoption boundary

CSR pooling retains per-vertex first-seen neighbor order and duplicate counts;
the frontier cache retains the greedy candidate order and group distance
anchor. Default-256 optimized-graph fingerprints across the tested constraint
families and 40-frame solver state streams matched their frozen controls.
The optional 1024 grouping deliberately changes the large-cloth trajectory.

The final candidate's 600-frame free 16K-cloth comparison stayed within
0.646 mm RMS and 1.818 mm maximum position difference from B; tested small
long runs retained exact state. This finite-scene result does not establish
equivalence for arbitrary driven cloth. The bounded driven large-cloth fixture
failed its predeclared quality gate in both frozen B and C. It remains an
rejected stress case and cannot justify relaxing contact, pin, geometry,
energy, or long-run gates. Single-worker cloth also consistently regressed in
the full matrix. The private experiment holds group size 1024 fixed across
worker caps, so changing the cap does not itself select a different constraint
graph and order. Switching to 256 at one worker would introduce a separate
trajectory and qualification case; it was not part of the measured candidate.
A representative driven 16K configuration must pass the original physical
envelope before promoting size 1024 beyond the private experiment. The
compile-only pristine-consumer compatibility check has passed. The
upstream-compatible default grouping remains 256.
