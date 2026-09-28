# Thesis

LaTeX source of the bachelor thesis *Towards Autonomous Industrial Mobility: An AI Framework for
Optimal Path Planning and Resource-Efficient Navigation*.

## Layout

```
thesis/
  main.tex            the only file that is compiled; it \input's everything else
  preamble/           packages.tex · style.tex · macros.tex (every recurring number)
  front/              titlepage · declaration · abstract · acknowledgements
  chapters/           01 … 09, one file per chapter
  back/               appendices A–C
  figures/            figures; the result figures are copied from tools/ai/results/
  bib/references.bib  seeded from docs/related_work.md (20 verified papers)
```

Compile with **pdfLaTeX + Biber** (Overleaf: Menu → Compiler → pdfLaTeX).

## How to work in here

- **One chapter per file.** Write in whatever order you like; `main.tex` never changes.
- **Numbers come from `preamble/macros.tex`.** Write `\scoreNS{}`, not `53.81`. If a result is ever
  re-run, one edit updates the whole thesis and no two chapters can disagree.
- **Every chapter file starts with a comment block** saying what that chapter has to achieve and
  which numbers, figures and `HANDOVER.md` sections belong in it. Delete the comments as you write.
- **Facts live in the repository, not in memory:** `THESIS_DATA.md` (final numbers and which file
  each comes from), `HANDOVER.md` (how each result was produced and why decisions were made),
  `docs/related_work.md` (the papers, with a note on where each one belongs).
- **`docs/explained/` is out of date** (written 2026-09-17, before the final results). Its
  explanations of how the system works are still useful; its numbers are not.

## The narrative

A thesis that only reports numbers is forgettable. This one has a spine: **three times, measurement
contradicted a reasonable assumption, and each time the system got better.**

1. **The learned ranker overruled its own rules.** In an overloaded factory it had never seen, it
   kept choosing to wait while production lines stood still — the very thing its symbolic estimate
   said was pointless. Bounding its influence to $\pm 1$ score unit turned 28.5 into 52.3 and made
   it the best safe policy overall. *(Chapter 5 sets it up, Chapter 7 measures it, Chapter 8
   explains why it works.)*
2. **A benchmark measured the wrong regime.** The global planner chosen on short tours (3 % faster)
   failed in 6 of 17 full shifts, because a momentarily blocked start pose makes Theta\* refuse to
   plan at all. *(Chapter 7, §planner study.)*
3. **A learned policy was safe until it was not.** PPO matched the neuro-symbolic model on score,
   then drove the battery to 9.4 % in a full Gazebo shift while the rule-carrying policy never went
   below 17 %. *(Chapter 7, §Gazebo validation.)*

Introduce each as a question, not as a conclusion, and let the data answer. That is what makes a
thesis read like an investigation rather than a report.

## Writing order that works

1. Chapter 3 (problem) and Chapter 6 (setup) — factual, quick, and they force the terminology.
2. Chapter 7 (results) — you already have every number.
3. Chapters 4 and 5 (system, models) — now you know what has to be explained for the results to
   make sense.
4. Chapter 2 (background) — you know what you must cite.
5. Chapters 8 and 9 (discussion, conclusion).
6. Chapter 1 and the abstract **last**, when you know what the thesis actually says.

## Figures

Result figures are generated:

```bash
venv/bin/python tools/ai/plot_results.py \
    --episodes tools/ai/results/episodes_20260920_210847.csv \
    --gazebo   tools/ai/results/gazebo_20260917_150533.csv
cp tools/ai/results/fig_*.png thesis/figures/
```

Still to draw by hand (an architecture diagram beats a screenshot every time): the system
architecture, the decision pipeline of the neuro-symbolic model, and the maze layout. A screenshot
of the dashboard works well for the monitoring section.

## Overleaf

Two ways to keep Overleaf and this repository in step:

- **Overleaf → Menu → GitHub** (premium): link the Overleaf project to this repository directly.
- **Otherwise:** work here, then Menu → Upload / drag the `thesis/` folder into Overleaf; or edit in
  Overleaf and periodically Menu → Download → Source and unpack it over `thesis/`.

Keep one of the two as the master copy at any time, and commit after each Overleaf round-trip.
