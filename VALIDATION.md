# Validating the thesis against the project

The thesis is a set of claims. The project is the evidence. "Validating" means: for every
claim in the thesis, there is one artefact in this repository that proves it, and one command
that re-checks it. If a command fails, either the thesis is wrong or the system changed — and
you find out before your supervisor does.

Nothing here is opinion-based. Every check either passes or names the cell that disagrees.

---

## The one idea

A thesis sentence can go wrong in exactly four ways, and each way has its own check:

| # | Kind of claim | Example | What proves it | How it is checked |
|---|---|---|---|---|
| A | **A number** | "the NS scores 53.81" | the raw episode CSV | recompute it from the CSV |
| B | **A statement about the system** | "rule H1 forbids actions that break the reserve" | the source code | run the unit test for that rule |
| C | **A configuration value** | "the reserve is 15 %" | `params.yaml` | regenerate the appendix and diff |
| D | **A statement about other people's work** | "Macenski et al. introduced Nav2" | the cited paper | verify the entry, then *read the paper* |

A, B and C are fully automated below. **D cannot be automated** — a script can confirm a paper
exists and that its title and year are right, but only reading the paper confirms it says what
Chapter 2 claims. That reading has now been done for all 26 citations against their abstracts
(`CITATION_AUDIT.md` in the thesis repository); what remains is confirming the handful of claims
that sit in paywalled full text rather than in an abstract.

---

## Running everything

```bash
cd /home/robojazzy/robofetch_ws_copy/robofetch_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source venv/bin/activate

# A — every number in chapter 7 recomputed from the raw data
python3 tools/thesis/check_claims.py --thesis ../autonomous-industrial-mobility-thesis

# B — every rule and model behaviour the thesis describes
python3 -m pytest src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test -q

# C — the appendices must regenerate byte for byte from the configuration
cd ../autonomous-industrial-mobility-thesis
python3 tools/gen_appendix.py --project ../robofetch_ws && git diff --stat generated/

# D — every reference checked against the publisher's record
python3 tools/check_refs.py
```

Last run (2026-09-29): **232 claim checks passed, 0 failed**; **163 unit tests passed**;
appendices reproduced with no difference in any table body; **24 of 26 references verified, 0 needing
attention**.

---

## Chapter by chapter

### Chapter 1 — Introduction
**Claims:** the contributions listed, and the promise that each is delivered later.
**Validated by:** every contribution must point at a later section that contains its evidence.
This is a reading check, not a script: go down the contribution list and confirm each one names
a section, and that the section reports a measurement rather than a description.
**Status:** contributions are stated as four items; each maps to §7.1, §7.3, §7.5 and §7.8.

### Chapter 2 — Background
**Claims:** what the cited literature did, and that a gap exists.
**Validated by:** `tools/check_refs.py` confirms each entry against the Crossref record of the
publisher — title, year, journal, authors, pages. It cannot confirm that the *sentence citing it*
is a fair summary.
**Status:** 24 of 26 entries verified against the publisher by `check_refs.py`. Separately, on
2026-09-29 all 26 citations were read against the **abstract of the paper itself**: 21 were
accurate as written and **5 were corrected**. The record is `CITATION_AUDIT.md` in the thesis
repository, which gives a verdict per citation with the supporting sentence from the abstract.
One citation (`lee2025digitaltwinagv`) had been cited as supporting this thesis's method when the
paper argues the opposite; that is fixed.
**Still yours to do:** one sentence still rests on an abstract that does not confirm it — the
claim that dispatching-rule baselines are standard practice, cited to `chang2025hierarchicalagv`,
whose paywalled full text would settle it. Also decide on `schulman2017ppo` (arXiv only) and
`raffin2021sb3` (JMLR, genuinely has no DOI).

### Chapter 3 — Problem
**Claims:** the constraints (C1–C4), the objective weights, the scenario definitions.
**Validated by:** these are *configuration*, not prose. `check_claims.py` reads
`src/robofetch_factory/config/params.yaml` and compares:
- C1's "at least 15 % must remain" against `robot.battery.reserve_percent: 15.0`
- "a penalty worth twenty delivered units" against `cost_per_safety_violation: 20.0`
- the twelve scenarios against the twelve files in `config/scenarios/`
**Status:** all matched. If you ever change a weight in `params.yaml`, this check fails until
Chapter 3 is updated — which is the point.

### Chapter 4 — System
**Claims:** the architecture: which node exists, what it publishes, which file holds what.
**Validated by:** the files and nodes named in the text must exist. Every path the thesis names
was checked against the repository; the one stale path found (`tools/thesis/gen_appendix.py`,
left behind when the thesis moved to its own repo) has been fixed.
**Status:** all named paths resolve. **Note the weakness:** this proves the files *exist*, not
that the description of their behaviour is right. That part is covered by the tests in
Chapter 5's checks and by the fact that the system actually runs (`scripts/run.sh`).

### Chapter 5 — Decision models
**Claims:** the rules H1–H5, the tier system, the bounded correction, and that the bound β = 1.0
was chosen on training data only.
**Validated by:** `pytest src/robofetch_ai/test` — there is a test per hard rule asserting that
it masks the action it is supposed to mask. `check_claims.py` confirms `\maxCorrection` in the
thesis equals `mission.neurosymbolic.max_correction` in `params.yaml`, and that the
`heavy_load` numbers (54.88 bounded against 33.78 unbounded) match the episode CSV.
**Status:** 163 tests pass; both numbers match to two decimals.

### Chapter 6 — Experimental setup
**Claims:** the protocol — 9 policies × 12 scenarios × 30 seeds = 3240 shifts, test seeds
5000–5029, never used in training.
**Validated by:** `check_claims.py` compares each against `mission.evaluation` in `params.yaml`
**and against the CSV itself** — the CSV genuinely contains 3240 rows, 9 policies and 12
scenarios, and the seeds present are exactly 5000–5029.
**Status:** all matched. This is the strongest check in the thesis, because the protocol is
verified against the data it actually produced, not against its own description.

### Chapter 7 — Results
**Claims:** every number in Tables 7.1 and 7.2, and the prose around them.
**Validated by:** `check_claims.py` re-derives all of it from
`tools/ai/results/episodes_20260920_210847.csv` using the same estimators as `evaluate.py`
(2000-resample bootstrap seeded at 0, paired Wilcoxon signed-rank on identical seeds). It checks
means, both CI bounds, deltas, p-values, unsafe-shift counts, every significance star, and the
seen/unseen label of each scenario. The Gazebo section is checked against
`gazebo_20260917_150533.csv` and `gazebo_60min.csv`.
**Status:** 232 checks, all passing. One error was found this way and fixed: the `ppo_wp6`
upper confidence bound read 55.8 where the data gives 55.86.

### Chapter 8 — Discussion
**Claims:** interpretations. These are *arguments*, not facts, so they cannot be checked
mechanically — but every number an argument leans on can be.
**Validated by:** the numbers it quotes are the same ones checked in Chapter 7. What you must
check by reading: that each argument is actually supported by the number it cites, and that the
limitations section admits the things the data shows. In particular §8.4 admits that the rule
policy's energy advantage is an accounting artefact (idling is charged to the charger, not the
battery) — that admission is correct and load-bearing; do not remove it.

### Chapter 9 — Conclusion
**Claims:** a restatement. **Validated by:** it must claim nothing that Chapter 7 did not
measure. Read it against Table 7.1 once; specifically, it must not claim the NS beats PPO on
score (it does not — it beats it on *safety at comparable score*).

### Appendices A and B
**Claims:** the parameter and scenario tables.
**Validated by:** they are *generated*, so they cannot drift. `tools/gen_appendix.py` rebuilds
them from `params.yaml` and `config/scenarios/`; `git diff generated/` must be empty.
**Status:** reproduce byte for byte; only the header comment changed when the tool moved.

### Appendix C
**Claims:** the reproduction commands.
**Validated by:** running them. The evaluation, training and Gazebo commands are the ones used
to produce the results.
**Still yours to do:** the commit hash placeholder is not yet filled in.

---

## What is still open

1. **Chapter 2 citations** — confirm each cited paper says what you claim. Not automatable.
2. **Two references without a DOI** — `schulman2017ppo` is arXiv-only, which conflicts with a
   no-preprints rule if your faculty has one; `raffin2021sb3` is a real JMLR paper that simply
   has no DOI.
3. **Appendix C commit hash** — fill in the placeholder once the project repo is final.
4. **`macros.tex` is barely used.** 35 numbers are defined there as the single source of truth,
   but only 6 are actually referenced in the chapters; the rest are typed as literals. That was
   the thing `macros.tex` existed to prevent. It does not matter much now that
   `check_claims.py` verifies the literals against the data, but if you re-run any experiment,
   change the macro *and* grep for the old literal.
