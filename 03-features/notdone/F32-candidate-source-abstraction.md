# F32 — Candidate Source Abstraction (Frontier is one of many)

**Priority**: Medium
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no

**Description**: Goal selection has two separable stages: **candidate generation**
(map → set of candidate points) and **candidate ranking** (filters + weighted
scorers). Today they are fused inside `FrontierAlgorithm`: `find_frontier_clusters`
generates and the F31 pipeline ranks. Frontier clustering is only *one* generation
strategy. This feature extracts a `CandidateSource` protocol so alternate
generators — uniform grid, random/Poisson-disk sampling, PRM/RRT-style samplers,
or a hybrid — can feed the same ranking pipeline. Depends on F31 (the pipeline is
already generator-agnostic; `CellCtx` carries only geometry, nothing frontier-specific).

## The split

```python
class CandidateSource(Protocol):
    def generate(self, ctx: ExplorationContext) -> list[CandidatePoint]: ...
```

- `generate` returns candidate points (world xy + cell index); the F31 pipeline
  builds `CellCtx` and ranks them, unchanged, regardless of source.
- `FrontierSource` wraps `find_frontier_clusters` — current behavior, the default.
- An algorithm = one `CandidateSource` + the shared scoring pipeline. `next_goal`
  becomes: `cells = source.generate(ctx); return pipeline.select(cells, ctx)`.

## Why frontier is not just an arbitrary generator

Frontier cells sit on the known/unknown boundary, so travelling to one
**structurally guarantees information gain** — it reveals unmapped space. Random
or grid points land mostly in already-known free space and reveal nothing. So any
non-frontier source **must** be paired with an information-gain scorer (F15
novelty is a weak proxy: unknown cells crossed on the straight-line path) or it
wastes trips revisiting known area. Frontier gets info-gain for free; samplers
have to earn it in the ranking. This is the load-bearing caveat for the whole
feature — a naive random source with only a distance scorer explores worse, not
better.

## Candidate sources worth having

- **FrontierSource** (default): built-in info-gain; but yields nothing in some map
  states (free strips narrower than `2*buffer_cells+1`; sparse or clustered
  candidates).
- **GridSource**: candidates on a uniform lattice over free space — even coverage
  where frontiers are sparse; needs the novelty/clearance scorers to prioritize.
- **RandomSource**: Poisson-disk / blue-noise samples over free space — coverage
  without lattice artifacts.
- **HybridSource**: frontier candidates plus sampled fill — robust exactly when
  frontier detection comes up empty (a real failure we hit: narrow strip → no
  frontier → premature "done").

## Relation to other features

- **F31** is the enabler: its filters + weighted scorers already rank any
  `CellCtx` without knowing the source. Land F31 first; then this split is cheap.
- **F26** (indoor-survey paper): "how are candidates generated" is precisely the
  algorithm-comparison axis the survey varies. `CandidateSource` makes each
  strategy a swappable, benchmarkable unit — the experimental scaffold for F26.
- **F30** (path-distance): a reachability filter/`d`-source that applies to any
  candidate source, not just frontier.
- **F23** decoupling holds: sources stay pure (read only `/map` from
  `ExplorationContext`), no ROS/costmap coupling.

## How to Demo

**Setup**: sim stack, one session per source
(`--candidate_source frontier|grid|hybrid`), novelty + clearance scorers on.

**Steps**:
1. Run exploration to completion in the multi_room world per source.
2. Compare coverage-over-time, total goals, redundant (low-info) goals, and
   behavior in the narrow-strip map state that starves the frontier source.

**Expected output**: `frontier` reproduces today's behavior exactly; `hybrid`
still produces goals where the pure frontier source reports none; a `grid`/random
source without an info-gain scorer visibly wastes trips (demonstrating the
info-gain caveat), and with novelty+clearance on it achieves comparable coverage.
