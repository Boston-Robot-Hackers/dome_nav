# General
* No code without a task; no task without a feature; no feature contradicting the spec.
* After creating a feature + task file, stop and present the plan — no code until approved.
* Don't close a feature until its full test suite exists and passes.
* Exception: simple bug fixes/refactors (no spec/behavior change) skip the feature/task pair — log as a chore.
* Any bug fix or regression gets a test.
* Switching between task/feature/chore: stop and ask permission first.

# chores
* One running file, `04-tasks/chores.md`: `- [ ] <what and why>` → `- [x]` when applied.

# features
* `FNN-<slug>.md` in `03-features/{notdone,done,deferred}/`; a mini-spec — scope/intent, not task detail.
* `Tasks File Created: yes` only once a matching `04-tasks/TFNN-*.md` exists. `template.md` shows the format.

# tasks
* Full task list before any design/code. `TFNN-<slug>.md` (`NN` matches the feature) in `04-tasks/{notdone,done,deferred}/`; `template.md` shows the format.
* Every step gets a test where feasible (else record why); every feature gets a dedicated test-writing task.
* Last task done → move the task file to `done/`, set the feature's Done/Tests Written/Test Passing to yes, move the feature file to `done/`.

# issues
* `05-issues/{open,closed,deferred}/`, numbered, follow the template.
* New → `open/`; resolved or absorbed into a feature/task → `closed/`; explicitly deferred → `deferred/`.

# writing .md files
* Applies to `03-features/`, `04-tasks/`, `05-issues/`, and any hand-written doc — not `01-literate/` (own prompt in `literate.md`).
* Blank line between paragraphs, always. Short paragraphs; bullets for anything enumerable; **bold** for key decisions; *italics* for emphasis or naming a pattern. Several short, headed subsections beat one block.
* Applies every time a file is rewritten, not just on first authoring.

# bootstrap
* `.claude/bootstrap.md` is the scaffold spec — follow it when bootstrapping a new project.

# github
* Literate docs: apply `.claude/literate.md`'s prompt to each changed Python module, save as `01-literate/<module>.md`.
* Run tests and regenerate literate docs before committing/pushing.
* Commit or push only when asked, with a good message.
