# MPPI from the Beginning

**Read online: [sajad2025.github.io/mppi-review](https://sajad2025.github.io/mppi-review/)**

A short book that derives model predictive path integral control (MPPI) step by step, starting from the optimal control problem and ending with a working implementation. Seventeen short chapters, five figures, and about seventy lines of NumPy.

## Read it

- **In the reader.** [The deployed reader](https://sajad2025.github.io/mppi-review/) (`index.html`) is a single-page reader with a contents drawer, three page colours and adjustable text size. It loads the chapters from `chapters/` and renders the mathematics with KaTeX.
- **On GitHub.** Every chapter is a plain Markdown file and renders directly on github.com, formulas included. Start with [the preface](chapters/00-preface.md).

## Contents

| | Chapter |
|---|---|
| | [Preface](chapters/00-preface.md) |
| 1 | [The optimal control problem](chapters/01-optimal-control.md) |
| 2 | [Dynamic programming and the linear-quadratic regulator](chapters/02-dynamic-programming.md) |
| 3 | [Model predictive control](chapters/03-model-predictive-control.md) |
| 4 | [Monte Carlo and importance sampling](chapters/04-monte-carlo.md) |
| 5 | [Relative entropy and free energy](chapters/05-free-energy.md) |
| 6 | [Stochastic optimal control in continuous time](chapters/06-stochastic-control.md) |
| 7 | [Path integral control](chapters/07-path-integral-control.md) |
| 8 | [The information-theoretic derivation](chapters/08-information-theoretic-derivation.md) |
| 9 | [The MPPI algorithm](chapters/09-the-algorithm.md) |
| 10 | [A worked example: the linear-quadratic problem](chapters/10-linear-quadratic-example.md) |
| 11 | [Tuning and practice](chapters/11-tuning.md) |
| 12 | [MPPI as an optimiser, and its relatives](chapters/12-relatives.md) |
| 13 | [Variants and extensions](chapters/13-variants.md) |
| 14 | [What is proved about MPPI](chapters/14-theory.md) |
| 15 | [A reference implementation](chapters/15-implementation.md) |
| 16 | [References](chapters/16-references.md) |

## Publish with GitHub Pages

1. Push this folder to a repository.
2. In the repository, open Settings, then Pages, and choose "Deploy from a branch" with the root of the main branch.
3. The reader appears at `https://<user>.github.io/<repository>/`; for this repository that is <https://sajad2025.github.io/mppi-review/>.

The empty file `.nojekyll` tells GitHub Pages to serve the Markdown files as they are. Keep it.

## Preview locally

The reader fetches the chapters over HTTP, so open it through a local server, not from disk:

```text
python -m http.server
```

then visit `http://localhost:8000`.

## Code

```text
cd code
pip install -r requirements.txt
python pendulum.py          # pendulum swing-up
python point_mass.py        # point mass around an obstacle
python lq_example.py        # closed-form checks of Chapter 10
python make_figures.py      # regenerate chapters/figures
```

Every number and figure in the text comes from these scripts with the seeds in the files.

## Layout

```text
index.html          the reader
book.json           chapter list used by the reader
chapters/           one Markdown file per chapter
chapters/figures/   figures produced by code/make_figures.py
code/               reference implementation and examples
```

To add or reorder chapters, edit `book.json`.
